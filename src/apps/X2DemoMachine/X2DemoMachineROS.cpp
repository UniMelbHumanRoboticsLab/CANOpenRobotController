#include "X2DemoMachineROS.h"

X2DemoMachineROS::X2DemoMachineROS(X2Robot *robot, ros::NodeHandle& nodeHandle):
    robot_(robot),
    nodeHandle_(&nodeHandle)
{

#ifndef SIM  // if simulation, these will be published by Gazebo
    jointStatePublisher_ = nodeHandle_->advertise<sensor_msgs::JointState>("joint_states", 10);
    leftThighForcePublisher_ = nodeHandle_->advertise<geometry_msgs::WrenchStamped>("left_thigh_wrench", 10);
    leftShankForcePublisher_ = nodeHandle_->advertise<geometry_msgs::WrenchStamped>("left_shank_wrench", 10);
    rightThighForcePublisher_ = nodeHandle_->advertise<geometry_msgs::WrenchStamped>("right_thigh_wrench", 10);
    rightShankForcePublisher_ = nodeHandle_->advertise<geometry_msgs::WrenchStamped>("right_shank_wrench", 10);
#endif
    interactionForceCommandSubscriber_ = nodeHandle_->subscribe("interaction_effort_commands", 1, &X2DemoMachineROS::interactionForceCommandCallback, this);
    startExoService_ = nodeHandle_->advertiseService("start_exo", &X2DemoMachineROS::startExoServiceCallback, this);
    calibrateForceSensorsService_ = nodeHandle_->advertiseService("calibrate_force_sensors", &X2DemoMachineROS::calibrateForceSensorsCallback, this);
    startExoTriggered_ = false;
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
#endif
}

void X2DemoMachineROS::publishJointStates() {
    Eigen::VectorXd jointPositions = robot_->getPosition();
    Eigen::VectorXd jointVelocities = robot_->getVelocity();
    Eigen::VectorXd jointTorques = robot_->getTorque();

    jointStateMsg_.header.stamp = ros::Time::now();
    jointStateMsg_.name.resize(X2_NUM_JOINTS);
    jointStateMsg_.position.resize(X2_NUM_JOINTS);
    jointStateMsg_.velocity.resize(X2_NUM_JOINTS);
    jointStateMsg_.effort.resize(X2_NUM_JOINTS);
    jointStateMsg_.name[0] = "left_hip_joint";
    jointStateMsg_.name[1] = "left_knee_joint";
    jointStateMsg_.name[2] = "right_hip_joint";
    jointStateMsg_.name[3] = "right_knee_joint";
    jointStateMsg_.position[0] = jointPositions[0];
    jointStateMsg_.position[1] = jointPositions[1];
    jointStateMsg_.position[2] = jointPositions[2];
    jointStateMsg_.position[3] = jointPositions[3];
    jointStateMsg_.velocity[0] = jointVelocities[0];
    jointStateMsg_.velocity[1] = jointVelocities[1];
    jointStateMsg_.velocity[2] = jointVelocities[2];
    jointStateMsg_.velocity[3] = jointVelocities[3];
    jointStateMsg_.effort[0] = jointTorques[0];
    jointStateMsg_.effort[1] = jointTorques[1];
    jointStateMsg_.effort[2] = jointTorques[2];
    jointStateMsg_.effort[3] = jointTorques[3];

    jointStatePublisher_.publish(jointStateMsg_);
}

void X2DemoMachineROS::publishInteractionForces() {
    Eigen::VectorXd interactionForces = robot_->getInteractionForce();
    ros::Time time = ros::Time::now();

    leftThighForceMsg_.header.stamp = time;
    leftShankForceMsg_.header.stamp = time;
    rightThighForceMsg_.header.stamp = time;
    rightShankForceMsg_.header.stamp = time;

    leftThighForceMsg_.header.frame_id = "left_upper_thigh_sensor";
    leftShankForceMsg_.header.frame_id = "left_upper_shank_sensor";
    rightThighForceMsg_.header.frame_id = "right_upper_thigh_sensor";
    rightShankForceMsg_.header.frame_id = "right_upper_shank_sensor";

    leftThighForceMsg_.wrench.force.z = interactionForces[0];
    leftShankForceMsg_.wrench.force.z = interactionForces[1];
    rightThighForceMsg_.wrench.force.z = interactionForces[2];
    rightShankForceMsg_.wrench.force.z = interactionForces[3];

    leftThighForcePublisher_.publish(leftThighForceMsg_);
    leftShankForcePublisher_.publish(leftShankForceMsg_);
    rightThighForcePublisher_.publish(rightThighForceMsg_);
    rightShankForcePublisher_.publish(rightShankForceMsg_);
}

void X2DemoMachineROS::setNodeHandle(ros::NodeHandle &nodeHandle) {
    nodeHandle_ = &nodeHandle;
}

bool X2DemoMachineROS::startExoServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    startExoTriggered_ = true;
    res.success = true;
    return true;
}

bool X2DemoMachineROS::calibrateForceSensorsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    res.success = robot_->calibrateForceSensors();
    return true;
}

void X2DemoMachineROS::interactionForceCommandCallback(const std_msgs::Float64MultiArray &msg) {
    for(int i=0; i<X2_NUM_JOINTS; i++){
        interactionForceCommand_[i] = msg.data[i];
    }
}