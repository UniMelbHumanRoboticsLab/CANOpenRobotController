#include "RobotM3.h"

using namespace Eigen;
using namespace std;

RobotM3::RobotM3(string robot_name, string yaml_config_file) :  Robot(robot_name, yaml_config_file),
                                                                endEffTool(&M3Handle),
                                                                calibrated(false),
                                                                maxEndEffVel(2),
                                                                maxEndEffForce(60),
                                                                velFilt(2, VM3::Zero()) {
    //Check if YAML file exists and contain robot parameters
    initialiseFromYAML(yaml_config_file);

    //TODO: to add joint specific parameters (reduction, torque constant) and associated YAML loading

    //Define the robot structure: each joint with limits and drive
    joints.push_back(new JointM3(0, qLimits[0], qLimits[1], qSigns[0], -dqMax, dqMax, -tauMax, tauMax, new KincoDrive(1), "q1"));
    joints.push_back(new JointM3(1, qLimits[2], qLimits[3], qSigns[1], -dqMax, dqMax, -tauMax, tauMax, new KincoDrive(2), "q2"));
    joints.push_back(new JointM3(2, qLimits[4], qLimits[5], qSigns[2], -dqMax, dqMax, -tauMax, tauMax, new KincoDrive(3), "q3"));

    //Possible inputs: keyboard and joystick
    inputs.push_back(keyboard = new Keyboard());
    inputs.push_back(joystick = new Joystick(1));

    last_update_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}
RobotM3::~RobotM3() {
    spdlog::debug("Delete RobotM3 object begins");
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    joints.clear();
    delete keyboard;
    delete joystick;
    inputs.clear();
    spdlog::debug("RobotM3 deleted");
}

bool RobotM3::loadParametersFromYAML(YAML::Node params) {
    YAML::Node params_r=params[robotName]; //Specific node corresponding to the robot

    if(params_r["dqMax"]){
        dqMax = fmin(fmax(0., params_r["dqMax"].as<double>()), 360.) * M_PI / 180.; //Hard constrained for safety
    }

    if(params["tauMax"]){
        tauMax = fmin(fmax(0., params_r["tauMax"].as<double>()), 50.); //Hard constrained for safety
    }

    if(params_r["linkLengths"]){
        for(unsigned int i=0; i<linkLengths.size(); i++)
            linkLengths[i]=params_r["linkLengths"][i].as<double>();
    }

    if(params_r["linkMasses"]){
        for(unsigned int i=0; i<linkMasses.size(); i++)
            linkMasses[i]=params_r["linkMasses"][i].as<double>();
    }

    if(params_r["frictionVis"]){
        for(unsigned int i=0; i<frictionVis.size(); i++)
            frictionVis[i]=params_r["frictionVis"][i].as<double>();
    }

    if(params_r["frictionCoul"]){
        for(unsigned int i=0; i<frictionCoul.size(); i++)
            frictionCoul[i]=params_r["frictionCoul"][i].as<double>();
    }

    if(params_r["qLimits"]){
        for(unsigned int i=0; i<qLimits.size(); i++)
            qLimits[i]=params_r["qLimits"][i].as<double>() * M_PI / 180.;
    }

    if(params_r["qSigns"]){
        for(unsigned int i=0; i<qSigns.size(); i++)
            qSigns[i]=params_r["qSigns"][i].as<double>();
    }

    if(params_r["qCalibration"]){
        for(unsigned int i=0; i<qCalibration.size(); i++)
            qCalibration[i]=params_r["qCalibration"][i].as<double>() * M_PI / 180.;
    }

    //Create and replace existing tool if one specified
    if(params_r["tool"]){
        if(params_r["tool"]["name"] && params_r["tool"]["length"] && params_r["tool"]["mass"]) {
            M3Tool *t = new M3Tool(params_r["tool"]["length"].as<double>(), params_r["tool"]["mass"].as<double>(), params_r["tool"]["name"].as<string>()); //Will be destroyed at end of app
            endEffTool = t;
        }
    }

    spdlog::info("Using YAML M3 parameters of {} (Tool: {}).", robotName, endEffTool->name);
    return true;
}

bool RobotM3::initialiseJoints() {
    return true;
}
bool RobotM3::initialiseNetwork() {
    spdlog::debug("RobotM3::initialiseNetwork()");

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }
    //Give time to drives PDO initialisation
    spdlog::debug("...");
    for (int i = 0; i < 5; i++) {
        spdlog::debug(".");
        usleep(10000);
    }
    //Start node
    for (auto joint : joints) {
        joint->start();
    }

    //Enable drive
    int n = 0;
    for (auto joint : joints) {
        bool joint_ready = false;
        for (int i = 0; (i < 10) && (!joint_ready); i++) {
            joint->readyToSwitchOn();
            usleep(10000);
            joint_ready = ((joint->getDriveStatus() & 0x01) == 0x01);
        }
        #ifndef NOROBOT
        if (!joint_ready) {
            spdlog::error("M3: Failed to enable joint {} (status: {})", n, joint->getDriveStatus());
            return false;
        }
        #endif
        n++;
    }
    updateRobot();
    return true;
}
bool RobotM3::initialiseInputs() {
    /*nothing to do*/
    return true;
}

void RobotM3::applyCalibration() {
    for (unsigned int i = 0; i < joints.size(); i++) {
        ((JointM3 *)joints[i])->setPositionOffset(qCalibration[i]);
    }
    calibrated = true;
}

void RobotM3::updateRobot() {
    Robot::updateRobot();

    //Update copies of end-effector values
    endEffPositions = directKinematic(getPosition());
    Matrix3d _J = J();
    endEffVelocities = _J * getVelocity();
    endEffAccelerations = calculateEndEffAcceleration();
    Matrix3d _Jtinv = (_J.transpose()).inverse();
    endEffForces = _Jtinv * getTorque();
    //Todo: improve by including friction compensation (dedicated calculation function...)
    interactionForces = endEffForces - _Jtinv * calculateGravityTorques();

    if (safetyCheck() != SUCCESS) {
        disable();
    }
    last_update_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}

setMovementReturnCode_t RobotM3::safetyCheck() {
    //End-effector safeties if calibrated
    if (calibrated) {
        if (getEndEffVelocity().norm() > maxEndEffVel) {
            spdlog::error("M3: Max velocity reached ({}m.s-1)!", getEndEffVelocity().norm());
            return OUTSIDE_LIMITS;
        }
        //if(getEndEffForce().norm()>maxEndEffForce) {
        //   spdlog::error("M3: Max force reached ({}N)!", getEndEffForce().norm());
        //   return OUTSIDE_LIMITS;
        //}
    }
    //otherwise basic joint safeties
    else {
        for (unsigned int i = 0; i < 3; i++) {
            if (((JointM3 *)joints[i])->safetyCheck() != SUCCESS) {
                spdlog::error("M3: Joint {} safety triggered!", i);
                return OUTSIDE_LIMITS;
            }
        }
    }
    return SUCCESS;
}

void RobotM3::printStatus() {
    std::cout << std::setprecision(3) << std::fixed << std::showpos;
    std::cout << "X=[ " << getEndEffPosition().transpose() << " ]\t";
    std::cout << "dX=[ " << getEndEffVelocity().transpose() << " ]\t";
    std::cout << "F=[ " << getEndEffForce().transpose() << " ]\t";
    std::cout <<  std::endl;
    std::cout <<  std::noshowpos;
}
void RobotM3::printJointStatus() {
    std::cout << std::setprecision(1) << std::fixed << std::showpos;
    std::cout << "q=[ " << getPosition().transpose() * 180 / M_PI << " ]\t";
    std::cout << "dq=[ " << getVelocity().transpose() * 180 / M_PI << " ]\t";
    std::cout << "tau=[ " << getTorque().transpose() << " ]\t";
    std::cout << "{";
    for (auto joint : joints)
        std::cout << "0x" << std::hex << ((JointM3 *)joint)->getDriveStatus() << "; ";
    std::cout << "}" << std::endl;
    std::cout <<  std::noshowpos;
}

bool RobotM3::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(CM_POSITION_CONTROL) != CM_POSITION_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM3 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM3 *)p)->enable();
    }

    //TODO:CHECK STATUS 0x07
    return returnValue;
}
bool RobotM3::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(CM_VELOCITY_CONTROL) != CM_VELOCITY_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM3 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM3 *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}
bool RobotM3::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM3 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM3 *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}

setMovementReturnCode_t RobotM3::applyPosition(std::vector<double> positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    if (!calibrated) {
        returnValue = NOT_CALIBRATED;
    } else {
        for (auto p : joints) {
            setMovementReturnCode_t setPosCode = ((JointM3 *)p)->setPosition(positions[i]);
            if (setPosCode == INCORRECT_MODE) {
                spdlog::error("Joint {} : is not in Position Control", p->getId());
                returnValue = INCORRECT_MODE;
            } else if (setPosCode != SUCCESS) {
                // Something bad happened
                spdlog::error("Joint {} position error : {} ", p->getId(), setMovementReturnCodeString[setPosCode]);
                returnValue = UNKNOWN_ERROR;
            }
            i++;
        }
    }
    return returnValue;
}
setMovementReturnCode_t RobotM3::applyVelocity(std::vector<double> velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setVelCode = ((JointM3 *)p)->setVelocity(velocities[i]);
        if (setVelCode == INCORRECT_MODE) {
            spdlog::error("Joint {} : is not in Velocity Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setVelCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} velocity error : {} ", p->getId(), setMovementReturnCodeString[setVelCode]);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}
setMovementReturnCode_t RobotM3::applyTorque(std::vector<double> torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setTorCode = ((JointM3 *)p)->setTorque(torques[i]);
        if (setTorCode == INCORRECT_MODE) {
            spdlog::error("Joint {} : is not in Torque Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setTorCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} torque error : {} ", p->getId(), setMovementReturnCodeString[setTorCode]);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

VM3 RobotM3::directKinematic(VM3 q) {
    VM3 X;

    std::vector<float> L = linkLengths;

    double F1 = (L[2] * sin(q[1]) + (L[3]+endEffTool->length) * cos(q[2]) + L[0]);

    X[0] = -F1 * cos(q[0]);
    X[1] = -F1 * sin(q[0]);
    X[2] = L[2] * cos(q[1]) - (L[3]+endEffTool->length) * sin(q[2]);

    return X;
}
VM3 RobotM3::inverseKinematic(VM3 X) {
    VM3 q;

    std::vector<float> L = linkLengths;

    //Check accessible workspace
    double normX = X.norm();
    if ((L[3] < L[2] && normX < L[2] - (L[3]+endEffTool->length)) || ((L[3]+endEffTool->length) > L[2] && normX < sqrt((L[4]+endEffTool->length) * (L[3]+endEffTool->length) - L[2] * L[2])) || normX > (L[2] + (L[3]+endEffTool->length) + L[0]) || X[0] > 0) {
        spdlog::error("RobotM3::inverseKinematic() error: Point not accessible. NaN returned.");
        q[0] = q[1] = q[2] = nan("");
        return q;
    }

    //Compute first joint
    q[0] = -atan2(X[1], -X[0]);

    //Project onto parallel mechanism plane
    VM3 tmpX;
    if (X[0] > 0) {
        //should never happen as outside of workspace...
        tmpX[0] = sqrt(X[0] * X[0] + X[1] * X[1]);
    } else {
        tmpX[0] = -sqrt(X[0] * X[0] + X[1] * X[1]);
    }
    //Remove offset along -x
    tmpX[0] = tmpX[0] + L[0];
    tmpX[1] = X[1];
    tmpX[2] = X[2];

    //Calculate joints 2 and 3
    double beta = acos((L[2] * L[2] + L[3] * L[3] - tmpX[0] * tmpX[0] - tmpX[2] * tmpX[2]) / (2. * (L[2] * L[3])));
    q[1] = acos(L[3] * sin(beta) / sqrt(tmpX[0] * tmpX[0] + tmpX[2] * tmpX[2])) - atan2(tmpX[2], -tmpX[0]);
    q[2] = M_PI / 2. + q[1] - beta;

    return q;
}
Matrix3d RobotM3::J() {
    Matrix3d J;
    VM3 q;
    for (unsigned int i = 0; i < 3; i++) {
        q(i) = ((JointM3 *)joints[i])->getPosition();
    }

    std::vector<float> L = linkLengths;

    //Pre calculate factors for optimisation
    double F1 = (L[3]+endEffTool->length) * sin(q[2]);
    double F2 = -L[2] * cos(q[1]);
    double F3 = (L[3]+endEffTool->length) * cos(q[2]) + L[2] * sin(q[1]) + L[0];

    //Jacobian matrix elements
    J(0, 0) = F3 * sin(q[0]);
    J(0, 1) = F2 * cos(q[0]);
    J(0, 2) = F1 * cos(q[0]);

    J(1, 0) = -F3 * cos(q[0]);
    J(1, 1) = F2 * sin(q[0]);
    J(1, 2) = F1 * sin(q[0]);

    J(2, 0) = 0;
    J(2, 1) = -L[2] * sin(q[1]);
    J(2, 2) = -(L[3]+endEffTool->length) * cos(q[2]);

    return J;
}

VM3 RobotM3::calculateGravityTorques() {
    VM3 tau_g;

    //For convenience
    std::vector<float> L = linkLengths;
    std::vector<float> M = linkMasses;

    float g = 9.81;  //Gravitational constant: remember to change it if using the robot on the Moon or another planet

    //Get current configuration
    VM3 q;
    for (unsigned int i = 0; i < 3; i++) {
        q(i) = ((JointM3 *)joints[i])->getPosition();
    }

    //Calculate gravitational torques
    tau_g[0] = 0;
    tau_g[1] = -L[2] / 2.0f * sin(q[1]) * (M[1]+M[2]+M[3]+M[4]+endEffTool->mass) * g;
    tau_g[2] = -(L[1]/2.0f*(M[0]+M[3]) + L[1]*M[2] + (L[1]+L[3]/2.0f)*M[4] + (L[3]+endEffTool->length/2.)*endEffTool->mass) * cos(q[2]) * g;

    return tau_g;
}

VM3 RobotM3::calculateEndEffAcceleration() {

    VM3 endEffVelocitiesFiltered_new = VM3::Zero();
    double dt = (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() / 1e6) - last_update_time;

    //Filter
    if(!velFilt.isInitialised()) {
        //Initialise filter at 10Hz w/ current sampling freq (Butterworth order 2)
        if(dt<1.) { //dt not reliable at startup
            velFilt.initButter2low(10.*dt);
        }
        endEffVelocitiesFiltered = VM3::Zero();
    }
    else {
        //Filter velocity
        endEffVelocitiesFiltered_new = velFilt.filt(endEffVelocities);
    }

    //Diff
    endEffAccelerations = (endEffVelocitiesFiltered_new - endEffVelocitiesFiltered) / dt;

    //Update value
    endEffVelocitiesFiltered = endEffVelocitiesFiltered_new;

    return endEffAccelerations;
}

const VX& RobotM3::getEndEffPosition() {
    return endEffPositions;
}
const VX& RobotM3::getEndEffVelocity() {
    return endEffVelocities;
}
const VX& RobotM3::getEndEffVelocityFiltered() {
    return endEffVelocitiesFiltered;
}
const VX& RobotM3::getEndEffAcceleration() {
    return endEffAccelerations;
}
const VX& RobotM3::getEndEffForce() {
    return endEffForces;
}
const VX& RobotM3::getInteractionForce() {
    return interactionForces;
}


setMovementReturnCode_t RobotM3::setJointPosition(VM3 q) {
    std::vector<double> pos{q(0), q(1), q(2)};
    return applyPosition(pos);
}
setMovementReturnCode_t RobotM3::setJointVelocity(VM3 dq) {
    std::vector<double> vel{dq(0), dq(1), dq(2)};
    return applyVelocity(vel);
}
setMovementReturnCode_t RobotM3::setJointTorque(VM3 tau) {
    std::vector<double> tor{tau(0), tau(1), tau(2)};
    return applyTorque(tor);
}
setMovementReturnCode_t RobotM3::setEndEffPosition(VM3 X) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }

    VM3 q = inverseKinematic(X);
    if (std::isnan(q[0]) || std::isnan(q[1]) || std::isnan(q[2])) {
        return OUTSIDE_LIMITS;
    } else {
        return setJointPosition(q);
    }
}
setMovementReturnCode_t RobotM3::setEndEffVelocity(VM3 dX) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }

    VM3 dq = J().inverse() * dX;
    return setJointVelocity(dq);
}
setMovementReturnCode_t RobotM3::setEndEffForce(VM3 F) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }

    VM3 tau = J().transpose() * F;
    return setJointTorque(tau);
}
setMovementReturnCode_t RobotM3::setEndEffForceWithCompensation(VM3 F, bool friction_comp) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }
    VM3 tau_g = calculateGravityTorques();  //Gravity compensation torque
    VM3 tau_f(0, 0, 0);                     //Friction compensation torque
    if (friction_comp) {
        double threshold = 0.000000;
        for (unsigned int i = 0; i < 3; i++) {
            double dq = ((JointM3 *)joints[i])->getVelocity();
            if (abs(dq) > threshold) {
                tau_f(i) = frictionCoul[i] * sign(dq) + frictionVis[i] * dq;
            } else {
                tau_f(i) = .0;
            }
        }
    }

    return setJointTorque(J().transpose() * F + tau_g + tau_f);
}
