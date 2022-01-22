#include "X2Robot.h"

/**
 * An enum type.
 * Joint Index for the 4 joints (note, CANopen NODEID = this + 1)
 */
enum X2Joints {
    X2_LEFT_HIP = 0,   /**< Left Hip*/
    X2_LEFT_KNEE = 1,  /**< Left Knee*/
    X2_RIGHT_HIP = 2,  /**< Right Hip*/
    X2_RIGHT_KNEE = 3, /**< Right Knee*/
};
/**
 * Paramater definitions: Hip motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
JointDrivePairs hipJDP{
    250880,       // drivePosA
    0,            // drivePosB
    deg2rad(90),  //jointPosA
    deg2rad(0)    //jointPosB
};
/**
 * Paramater definitions: Knee motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
JointDrivePairs kneeJDP{
    250880,       // drivePosA
    0,            //drivePosB
    deg2rad(90),  //jointPosA
    deg2rad(0)    //jointPosB
};

/**
 * Defines the Joint Limits of the X2 Exoskeleton
 *
 */
ExoJointLimits X2JointLimits = {deg2rad(120), deg2rad(-40), deg2rad(120), deg2rad(0)};

static volatile sig_atomic_t exitHoming = 0;

#ifdef SIM
X2Robot::X2Robot(ros::NodeHandle &nodeHandle, std::string robot_name, std::string yaml_config_file)
#else
X2Robot::X2Robot(std::string robot_name, std::string yaml_config_file)
#endif
        {

    spdlog::debug("{} Created", robotName);

#ifdef NOROBOT
    simJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    simJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    simJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    simInteractionForces_ = Eigen::VectorXd::Zero(X2_NUM_FORCE_SENSORS);
    simBackPackAngleOnMedianPlane_ = 0;
#endif

    // Initializing the parameters to zero
    x2Parameters.m = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.l = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.s = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.I = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.c0 = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.c1 = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.c2 = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.cuffWeights = Eigen::VectorXd::Zero(X2_NUM_FORCE_SENSORS);
    x2Parameters.forceSensorScaleFactor = Eigen::VectorXd::Zero(X2_NUM_FORCE_SENSORS);
    x2Parameters.imuParameters.useIMU = false;
    interactionForces_ = Eigen::VectorXd::Zero(X2_NUM_FORCE_SENSORS);
    backpackAccelerations_ = Eigen::VectorXd::Zero(3);
    backpackQuaternions_ = Eigen::VectorXd::Zero(4);
    backPackAngleOnMedianPlane_ = 0;

    //Check if YAML file exists and contain robot parameters
    initialiseFromYAML(yaml_config_file);

    spdlog::debug("initialiseJoints call");
    initialiseJoints();
    initialiseInputs();
#ifdef SIM
    initialiseROS(nodeHandle);
#endif
}

X2Robot::~X2Robot() {
    if(x2Parameters.imuParameters.useIMU) technaidIMUs->exit();
    freeMemory();
    spdlog::debug("X2Robot deleted");
}

void X2Robot::signalHandler(int signum) {
    exitHoming = 1;
    std::raise(SIGTERM); //Clean exit
}

#ifdef SIM
void X2Robot::initialiseROS(ros::NodeHandle &nodeHandle) {
    controllerSwitchClient_ = nodeHandle.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

    positionCommandPublisher_ = nodeHandle.advertise<std_msgs::Float64MultiArray>("position_controller/command", 10);
    velocityCommandPublisher_ = nodeHandle.advertise<std_msgs::Float64MultiArray>("velocity_controller/command", 10);
    torqueCommandPublisher_ = nodeHandle.advertise<std_msgs::Float64MultiArray>("torque_controller/command", 10);

    jointStateSubscriber_ = nodeHandle.subscribe("joint_states", 1, &X2Robot::jointStateCallback, this);
}
#endif

void X2Robot::resetErrors() {
    spdlog::debug("Clearing errors on all motor drives ");
    for (auto p : joints) {
        // Put into ReadyToSwitchOn()
        p->resetErrors();
    }
}

bool X2Robot::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        p->enable();
    }

#ifdef SIM
    controllerSwitchMsg_.request.start_controllers = {"position_controller"};
    controllerSwitchMsg_.request.stop_controllers = {"velocity_controller", "torque_controller"};
    controllerSwitchMsg_.request.strictness = 1;
    controllerSwitchMsg_.request.start_asap = true;
    controllerSwitchMsg_.request.timeout = 0.0;

    if (controllerSwitchClient_.call(controllerSwitchMsg_)) {
        spdlog::info("Switched to position controller");
    } else {
        spdlog::error("Failed switching to position controller");
        returnValue = false;
    }
#endif

    return returnValue;
}

bool X2Robot::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_VELOCITY_CONTROL, velControlMotorProfile) != CM_VELOCITY_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(10000);
    for (auto p : joints) {
        p->enable();
    }

#ifdef SIM
    controllerSwitchMsg_.request.start_controllers = {"velocity_controller"};
    controllerSwitchMsg_.request.stop_controllers = {"position_controller", "torque_controller"};
    controllerSwitchMsg_.request.strictness = 1;
    controllerSwitchMsg_.request.start_asap = true;
    controllerSwitchMsg_.request.timeout = 0.0;

    if (controllerSwitchClient_.call(controllerSwitchMsg_)) {
        spdlog::info("Switched to velocity controller");
    } else {
        spdlog::error("Failed switching to velocity controller");
        returnValue = false;
    }
#endif

    return returnValue;
}

bool X2Robot::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        p->enable();
    }

#ifdef SIM
    controllerSwitchMsg_.request.start_controllers = {"torque_controller"};
    controllerSwitchMsg_.request.stop_controllers = {"position_controller", "velocity_controller"};
    controllerSwitchMsg_.request.strictness = 1;
    controllerSwitchMsg_.request.start_asap = true;
    controllerSwitchMsg_.request.timeout = 0.0;

    if (controllerSwitchClient_.call(controllerSwitchMsg_)) {
        spdlog::info("Switched to torque controller");
    } else {
        spdlog::error("Failed switching to torque controller");
        returnValue = false;
    }
#endif

    return returnValue;
}

setMovementReturnCode_t X2Robot::setPosition(Eigen::VectorXd positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        spdlog::debug("Joint {}, Target {}, Current {}", i, positions[i], ((X2Joint *)p)->getPosition());
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setPosition(positions[i]);
        if (setPosCode == INCORRECT_MODE) {
            spdlog::error("Joint {} is not in Position Control ", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} Unknown Error", p->getId());
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }

#ifdef SIM
    std::vector<double> positionVector(X2_NUM_JOINTS);

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        positionVector[i] = positions[i];
    }

    positionCommandMsg_.data = positionVector;
    positionCommandPublisher_.publish(positionCommandMsg_);
#elif NOROBOT
    simJointPositions_ = positions;
#endif

    return returnValue;
}

setMovementReturnCode_t X2Robot::setVelocity(Eigen::VectorXd velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setVelocity(velocities[i]);
        if (setPosCode == INCORRECT_MODE) {
            spdlog::error("Joint {} is not in Velocity Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} Unknown Error", p->getId());
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }

#ifdef SIM
    std::vector<double> velocityVector(X2_NUM_JOINTS);

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        velocityVector[i] = velocities[i];
    }

    velocityCommandMsg_.data = velocityVector;
    velocityCommandPublisher_.publish(velocityCommandMsg_);
#endif

    return returnValue;
}

setMovementReturnCode_t X2Robot::setTorque(Eigen::VectorXd torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setTorque(torques[i]);
        if (setPosCode == INCORRECT_MODE) {
            spdlog::error("Joint {} is not in Torque Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} Unknown Error", p->getId());
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }

#ifdef SIM
    std::vector<double> torqueVector(X2_NUM_JOINTS);

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        torqueVector[i] = torques[i];
    }

    torqueCommandMsg_.data = torqueVector;
    torqueCommandPublisher_.publish(torqueCommandMsg_);
#endif

    return returnValue;
}

Eigen::VectorXd &X2Robot::getPosition() {
#ifndef NOROBOT
    return Robot::getPosition();
#else
    return simJointPositions_;
#endif
}

Eigen::VectorXd &X2Robot::getVelocity() {
#ifndef NOROBOT
    return Robot::getVelocity();
#else
    return simJointVelocities_;
#endif
}

Eigen::VectorXd &X2Robot::getTorque() {
#ifndef NOROBOT
    return Robot::getTorque();
#else
    return simJointTorques_;
#endif
}

Eigen::VectorXd &X2Robot::getInteractionForce() {
#ifndef NOROBOT
    return interactionForces_;
#else
    return simInteractionForces_; // TODO: add force sensor to simulation
#endif

}

double & X2Robot::getBackPackAngleOnMedianPlane() {
#ifndef NOROBOT
    return backPackAngleOnMedianPlane_;
#else
    return simBackPackAngleOnMedianPlane_; // TODO: add IMU to simulation
#endif
}

bool X2Robot::calibrateForceSensors() {
    int numberOfSuccess = 0;
    for (int i = 0; i < X2_NUM_FORCE_SENSORS; i++) {
        if (forceSensors[i]->calibrate()) numberOfSuccess++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    if (numberOfSuccess == X2_NUM_FORCE_SENSORS) {
        spdlog::info("[X2Robot::calibrateForceSensors]: Zeroing of force sensors are successfully completed.");
        return true;
    } else {
        spdlog::error("[X2Robot::calibrateForceSensors]: Zeroing failed.");
        return false;
    }
}

Eigen::VectorXd X2Robot::getBackpackQuaternions() {

    for(int imuIndex = 0; imuIndex<numberOfIMUs_; imuIndex++){
        if(x2Parameters.imuParameters.location[imuIndex] == 'b'){
            if(technaidIMUs->getOutputMode_(imuIndex).name != "quat"){
                spdlog::warn("Backpack IMU mode is not quaternion for imu no: . Returns 0", imuIndex);
                backpackQuaternions_ = Eigen::VectorXd::Zero(4);
            }
            backpackQuaternions_ = technaidIMUs->getQuaternion().col(imuIndex);
        }
    }
    return backpackQuaternions_;
}

void X2Robot::updateBackpackAngleOnMedianPlane() {

    if(!x2Parameters.imuParameters.useIMU){ // if IMU not used set backpack angle to zero
        backPackAngleOnMedianPlane_ = 0.0;
        return;
    }

    Eigen::Quaterniond q;
    Eigen::MatrixXd quatEigen = getBackpackQuaternions();
    q.x() = quatEigen(0,0);
    q.y() = quatEigen(1,0);
    q.z() = quatEigen(2,0);
    q.w() = quatEigen(3,0);

    Eigen::Matrix3d R = q.toRotationMatrix();
    double thetaBase = -std::atan2(-R(2,2), -R(2,0));

    backPackAngleOnMedianPlane_ = thetaBase;

}

void X2Robot::updateInteractionForce() {

    Eigen::VectorXd cuffCompensation = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    cuffCompensation[0] = x2Parameters.cuffWeights[0] * sin(getBackPackAngleOnMedianPlane() - getPosition()[0]);
    cuffCompensation[1] = x2Parameters.cuffWeights[1] * sin(getBackPackAngleOnMedianPlane() - getPosition()[1] + getPosition()[0]);
    cuffCompensation[2] = x2Parameters.cuffWeights[2] * sin(getBackPackAngleOnMedianPlane() - getPosition()[2]);
    cuffCompensation[3] = x2Parameters.cuffWeights[3] * sin(getBackPackAngleOnMedianPlane() - getPosition()[3] + getPosition()[2]);

    //Update values
    for (int i = 0; i < X2_NUM_FORCE_SENSORS; i++) {
        interactionForces_[i] = forceSensors[i]->getForce() + cuffCompensation[i];
    }

}

bool X2Robot::homing(std::vector<int> homingDirection, float thresholdTorque, float delayTime,
                     float homingSpeed, float maxTime) {
    std::vector<bool> success(X2_NUM_JOINTS, false);
    std::chrono::steady_clock::time_point time0;
    signal(SIGINT, signalHandler); // check if ctrl + c is pressed

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        if (homingDirection[i] == 0) continue;  // skip the joint if it is not asked to do homing

        this->initVelocityControl();
        Eigen::VectorXd desiredVelocity;
        desiredVelocity = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        std::chrono::steady_clock::time_point firstTimeHighTorque;  // time at the first time joint exceed thresholdTorque
        bool highTorqueReached = false;

        desiredVelocity[i] = homingSpeed * homingDirection[i] / std::abs(homingDirection[i]);  // setting the desired velocity by using the direction
        time0 = std::chrono::steady_clock::now();

        spdlog::debug("Homing Joint {} ...", i);

        while (success[i] == false &&
                exitHoming == 0 &&
               std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count() < maxTime * 1000) {
            this->updateRobot();  // because this function has its own loops, updateRobot needs to be called
            this->setVelocity(desiredVelocity);
            usleep(10000);

            if (std::abs(this->getTorque()[i]) >= thresholdTorque) {  // if high torque is reached
                highTorqueReached = true;
                firstTimeHighTorque = std::chrono::steady_clock::now();
                while (std::chrono::duration_cast<std::chrono::milliseconds>  // high torque should be measured for delayTime
                       (std::chrono::steady_clock::now() - firstTimeHighTorque).count() < delayTime * 1000 &&
                        exitHoming == 0) {
                    this->updateRobot();
                    usleep(10000);

                    if (std::abs(this->getTorque()[i]) < thresholdTorque) {  // if torque value reach below thresholdTorque, goes back
                        spdlog::debug("Torque drop", this->getTorque()[i]);
                        highTorqueReached = false;
                        break;
                    }
                }
            }
            success[i] = highTorqueReached;
        }

        if (success[i]) {
            spdlog::info("Homing Succeeded for Joint {} .", i);
            usleep(10000);
            if (i == X2_LEFT_HIP || i == X2_RIGHT_HIP) {  // if it is a hip joint

                // zeroing is done depending on the limits on the homing direction
                if (homingDirection[i] > 0)
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.hipMax);
                else
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.hipMin);
            } else if (i == X2_LEFT_KNEE || i == X2_RIGHT_KNEE) {  // if it is a knee joint

                // zeroing is done depending on the limits on the homing direction
                if (homingDirection[i] > 0)
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.kneeMax);
                else
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.kneeMin);
            }
            // fell joint down from the limit
            initTorqueControl();
            sleep(2);

        } else {
            spdlog::error("Homing Failed for Joint {} .", i);
        }
    }
    // Checking if all commanded joint successfully homed
    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        if (homingDirection[i] == 0) continue;  // skip the joint if it is not asked to do homing
        if (success[i] == false) return false;
    }
    return true;  // will come here if all joints successfully homed
}

bool X2Robot::setBackpackIMUMode(IMUOutputMode imuOutputMode) {

    for(int i = 0; i<numberOfIMUs_; i++){
        if(x2Parameters.imuParameters.location[i] == 'b'){
            if(!technaidIMUs->setOutputMode(i, imuOutputMode)){
                return false;
            }
        }
    }
    return true;
}

bool X2Robot::initialiseJoints() {
    for (int id = 0; id < X2_NUM_JOINTS; id++) {
        motorDrives.push_back(new CopleyDrive(id + 1));
        // The X2 has 2 Hips and 2 Knees, by default configured as 2 hips, then 2 legs int jointID, double jointMin, double jointMax, JointDrivePairs jdp, Drive *drive
        if (id == X2_LEFT_HIP || id == X2_RIGHT_HIP) {
            joints.push_back(new X2Joint(id, X2JointLimits.hipMin, X2JointLimits.hipMax, hipJDP, motorDrives[id]));
        } else if (id == X2_LEFT_KNEE || id == X2_RIGHT_KNEE) {
            joints.push_back(new X2Joint(id, X2JointLimits.kneeMin, X2JointLimits.kneeMax, kneeJDP, motorDrives[id]));
        }
        spdlog::debug("X2Robot::initialiseJoints() loop");
    }

    return true;
}


bool X2Robot::initialiseNetwork() {
    spdlog::debug("X2Robot::initialiseNetwork()");

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }

    return true;
}
bool X2Robot::initialiseInputs() {
    inputs.push_back(keyboard = new Keyboard());

    for (int id = 0; id < X2_NUM_FORCE_SENSORS; id++) {
        forceSensors.push_back(new FourierForceSensor(id + 17, x2Parameters.forceSensorScaleFactor[id]));
        inputs.push_back(forceSensors[id]);
    }

    if(x2Parameters.imuParameters.useIMU){
        technaidIMUs = new TechnaidIMU(x2Parameters.imuParameters);
//        inputs.push_back(technaidIMUs); // commented because techIMU class starts its own thread for data update
        technaidIMUs->initialize();
    }

    return true;
}


bool X2Robot::loadParametersFromYAML(YAML::Node params) {

    if(params[robotName]["m"].size() != X2_NUM_JOINTS || params[robotName]["l"].size() != X2_NUM_JOINTS ||
       params[robotName]["s"].size() != X2_NUM_JOINTS || params[robotName]["I"].size() != X2_NUM_JOINTS ||
       params[robotName]["c0"].size() != X2_NUM_JOINTS || params[robotName]["c1"].size() != X2_NUM_JOINTS ||
       params[robotName]["c2"].size() != X2_NUM_JOINTS || params[robotName]["cuff_weights"].size() != X2_NUM_FORCE_SENSORS ||
       params[robotName]["force_sensor_scale_factor"].size() != X2_NUM_FORCE_SENSORS){

        spdlog::error("Parameter sizes are not consistent");
        spdlog::error("All parameters are zero !");

        return false;
    }

    // getting the parameters from the yaml file
    for(int i = 0; i<X2_NUM_JOINTS; i++){
        x2Parameters.m[i] = params[robotName]["m"][i].as<double>();
        x2Parameters.l[i] = params[robotName]["l"][i].as<double>();
        x2Parameters.s[i] = params[robotName]["s"][i].as<double>();
        x2Parameters.I[i] = params[robotName]["I"][i].as<double>();
        x2Parameters.c0[i] = params[robotName]["c0"][i].as<double>();
        x2Parameters.c1[i] = params[robotName]["c1"][i].as<double>();
        x2Parameters.c2[i] = params[robotName]["c2"][i].as<double>();
    }
    for(int i = 0; i<X2_NUM_FORCE_SENSORS; i++) {
        x2Parameters.cuffWeights[i] = params[robotName]["cuff_weights"][i].as<double>();
        x2Parameters.forceSensorScaleFactor[i] = params[robotName]["force_sensor_scale_factor"][i].as<double>();
    }

    if(!params[robotName]["technaid_imu"]){
        spdlog::warn("IMU parameters couldn't be found!");
        x2Parameters.imuParameters.useIMU = false;
    } else{ // there is imu params
        if(params[robotName]["technaid_imu"]["network_id"].size() != params[robotName]["technaid_imu"]["serial_no"].size()||
           params[robotName]["technaid_imu"]["location"].size() != params[robotName]["technaid_imu"]["serial_no"].size()){
            spdlog::error("IMU parameter sizes are not consistent!");
            x2Parameters.imuParameters.useIMU = false;
        }else{ // imu parameters have consistent size

            numberOfIMUs_ = params[robotName]["technaid_imu"]["serial_no"].size();
            int numberOfContactIMUs = 0;
            int numberOfBackpackIMUs = 0;

            for(int i = 0; i<numberOfIMUs_; i++) {
                x2Parameters.imuParameters.serialNo.push_back(params[robotName]["technaid_imu"]["serial_no"][i].as<int>());
                x2Parameters.imuParameters.networkId.push_back(params[robotName]["technaid_imu"]["network_id"][i].as<int>());
                x2Parameters.imuParameters.location.push_back(params[robotName]["technaid_imu"]["location"][i].as<char>());
                if(x2Parameters.imuParameters.location[i] == 'c') numberOfContactIMUs++;
                else if(x2Parameters.imuParameters.location[i] == 'b') numberOfBackpackIMUs++;
            }

            if(numberOfContactIMUs == 0 || numberOfBackpackIMUs != 1){
                spdlog::error("IMU locations are not proper");
                x2Parameters.imuParameters.useIMU = false;
            } else{
                spdlog::info("IMU parameters are succefully parsed");
                x2Parameters.imuParameters.useIMU = true;
                x2Parameters.imuParameters.canChannel = params[robotName]["technaid_imu"]["can_channel"].as<std::string>();

                contactAccelerations_ = Eigen::MatrixXd::Zero(3, numberOfContactIMUs);
                contactQuaternions_ = Eigen::MatrixXd::Zero(4, numberOfContactIMUs);
            }
        }
    }

    return true;

}

void X2Robot::freeMemory() {
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    for (auto p : motorDrives) {
        spdlog::debug("Delete Drive Node: {}", p->getNodeID());
        delete p;
    }
    for (auto p : inputs) {
        spdlog::debug("Deleting Input");
        delete p;
    }
}
void X2Robot::updateRobot() {
    //TODO: generalise sensors update
#ifndef SIM
    Robot::updateRobot();
    if(x2Parameters.imuParameters.useIMU){
        updateBackpackAngleOnMedianPlane();
    }
    updateInteractionForce();
#endif
}

bool X2Robot::setPosControlContinuousProfile(bool continuous){
    bool returnValue = true;
    for (auto p : joints) {
        if(!(p->setPosControlContinuousProfile(continuous))){
            returnValue = false;
        }
    }
    return returnValue;
}
Eigen::VectorXd X2Robot::getFeedForwardTorque(int motionIntend) {
    float coulombFriction;
    const float velTreshold = 1*M_PI/180.0; // [rad/s]

    // todo generalized 4 Dof Approach
    if(abs(jointVelocities_[1]) > velTreshold){ // if in motion
        coulombFriction = x2Parameters.c1[1]*jointVelocities_[1]/abs(jointVelocities_[1]) +
        + x2Parameters.c2[1]*sqrt(abs(jointVelocities_[1]))*jointVelocities_[1]/abs(jointVelocities_[1]);
    }else { // if static
        coulombFriction = x2Parameters.c1[1]*motionIntend/abs(motionIntend);
    }

    Eigen::VectorXd ffTorque = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    ffTorque[1] = x2Parameters.m[1]*x2Parameters.s[1]*9.81*sin(jointPositions_[1] - jointPositions_[0]) + coulombFriction + x2Parameters.c0[1]*jointVelocities_[1];

    return ffTorque;

}

void X2Robot::setRobotName(std::string robot_name) {
    robotName = robot_name;
}

std::string & X2Robot::getRobotName() {
    return robotName;
}

RobotParameters& X2Robot::getRobotParameters() {
    return x2Parameters;

}

#ifdef SIM
void X2Robot::setNodeHandle(ros::NodeHandle &nodeHandle) {
    nodeHandle_ = &nodeHandle;
}

void X2Robot::jointStateCallback(const sensor_msgs::JointState &msg) {
    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        simJointPositions_[i] = msg.position[i];
        simJointVelocities_[i] = msg.velocity[i];
        simJointTorques_[i] = msg.effort[i];
    }
}

#endif
