#include "X2DemoMachineROS.h"

X2DemoMachineROS::X2DemoMachineROS(X2Robot *robot, std::shared_ptr<X2DemoState> x2DemoState, ros::NodeHandle& nodeHandle):
        robot_(robot),
        x2DemoState_(x2DemoState),
        nodeHandle_(&nodeHandle)
{

#ifndef SIM  // if simulation, these will be published by Gazebo
    jointStatePublisher_ = nodeHandle_->advertise<sensor_msgs::JointState>("joint_states", 10);
    interactionForcePublisher_ = nodeHandle_->advertise<CORC::X2Array>("interaction_forces", 10);
    for(int i = 0; i< X2_NUM_GRF_SENSORS; i++){
        groundReactionForcePublisher_[i] = nodeHandle_->advertise<geometry_msgs::WrenchStamped>
                ("grf_" + grfFramesArray_[i], 10);
    }
#endif //todo check which ones should be published if SIM

    calibrateForceSensorsService_ = nodeHandle_->advertiseService("calibrate_force_sensors", &X2DemoMachineROS::calibrateForceSensorsCallback, this);
    emergencyStopService_ = nodeHandle_->advertiseService("emergency_stop", &X2DemoMachineROS::emergencyStopCallback, this);
    startHomingService_ = nodeHandle_->advertiseService("start_homing", &X2DemoMachineROS::startHomingCallback, this);
    imuCalibrationService_ = nodeHandle_->advertiseService("calibrate_imu", &X2DemoMachineROS::calibrateIMUCallback, this);
    interactionForceCommand_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
}

X2DemoMachineROS::~X2DemoMachineROS() {
    ros::shutdown();
}

void X2DemoMachineROS::initialize() {
    spdlog::debug("X2DemoMachineROS::init()");

}

void X2DemoMachineROS::update() {
#ifndef SIM  // if simulation, these will be published by Gazebo
    publishJointStates();
    publishInteractionForces();
    publishGroundReactionForces();
#endif
}

void X2DemoMachineROS::publishJointStates() {
    Eigen::VectorXd jointPositions = robot_->getPosition();
    Eigen::VectorXd jointVelocities = robot_->getVelocity();
    Eigen::VectorXd jointTorques = robot_->getTorque();

    jointStateMsg_.header.stamp = ros::Time::now();
    jointStateMsg_.name.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.position.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.velocity.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.effort.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.name[0] = "left_hip_joint";
    jointStateMsg_.name[1] = "left_knee_joint";
    jointStateMsg_.name[2] = "right_hip_joint";
    jointStateMsg_.name[3] = "right_knee_joint";
    jointStateMsg_.name[4] = "world_to_backpack";

    for(int id = 0; id < X2_NUM_JOINTS; id++){
        jointStateMsg_.position[id] = jointPositions[id];
        jointStateMsg_.velocity[id] = jointVelocities[id];
        jointStateMsg_.effort[id] = jointTorques[id];
    }

    jointStateMsg_.position[4] = robot_->getBackPackAngleOnMedianPlane() - M_PI_2;
    jointStatePublisher_.publish(jointStateMsg_);
}

void X2DemoMachineROS::publishInteractionForces() {
    Eigen::VectorXd interactionForces = robot_->getInteractionForce();

    interactionForceMsg_.header.stamp = ros::Time::now();

    interactionForceMsg_.name.resize(X2_NUM_GENERALIZED_COORDINATES);
    interactionForceMsg_.data.resize(X2_NUM_GENERALIZED_COORDINATES);

    interactionForceMsg_.name[0] = "backpack_force";
    interactionForceMsg_.name[1] = "left_thigh_force";
    interactionForceMsg_.name[2] = "left_shank_force";
    interactionForceMsg_.name[3] = "right_thigh_force";
    interactionForceMsg_.name[4] = "right_shank_force";

    for(int id = 0; id< X2_NUM_GENERALIZED_COORDINATES; id++){
        interactionForceMsg_.data[id] = interactionForces[id];
    }

    interactionForcePublisher_.publish(interactionForceMsg_);
}

void X2DemoMachineROS::publishGroundReactionForces() {
    Eigen::VectorXd groundReactionForces = -robot_->getGroundReactionForces();

    for(int i = 0; i< X2_NUM_GRF_SENSORS; i++){
        groundReactionForceMsgArray_[i].header.stamp = ros::Time::now();
        groundReactionForceMsgArray_[i].header.frame_id = grfFramesArray_[i];
        groundReactionForceMsgArray_[i].wrench.force.z = groundReactionForces[i];
        groundReactionForcePublisher_[i].publish(groundReactionForceMsgArray_[i]);
    }
}

void X2DemoMachineROS::setNodeHandle(ros::NodeHandle &nodeHandle) {
    nodeHandle_ = &nodeHandle;
}

ros::NodeHandle & X2DemoMachineROS::getNodeHandle() {

    return *nodeHandle_;
}

bool X2DemoMachineROS::calibrateForceSensorsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    bool success = true;
    for(int id = 0; id < X2_NUM_FORCE_SENSORS + X2_NUM_GRF_SENSORS; id++){
        success = success & robot_->forceSensors[id]->sendInternalCalibrateSDOMessage();
    }

    success = success & robot_->calibrateForceSensors();
    res.success = success;

    return success;
}

bool X2DemoMachineROS::startHomingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    std::vector<int> homingDirection{1, 1, 1, 1};
    res.success = robot_->homing(homingDirection);
    return true;
}

bool X2DemoMachineROS::emergencyStopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    std::raise(SIGTERM); //Clean exit
    return true;
}

bool X2DemoMachineROS::calibrateIMUCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    res.success = robot_->setBackpackIMUMode(IMUOutputMode::QUATERNION_GYRO);

//    robot_->calibrateContactIMUAngles(2.0);
    return true;
}