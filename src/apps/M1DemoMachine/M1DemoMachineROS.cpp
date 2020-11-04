#include "M1DemoMachineROS.h"

M1DemoMachineROS::M1DemoMachineROS(RobotM1 *robot) {
    robot_ = robot;
}

M1DemoMachineROS::~M1DemoMachineROS() {
    ros::shutdown();
}

void M1DemoMachineROS::initialize() {
    spdlog::debug("M1DemoMachineROS::init()");
//#ifndef SIM  // if simulation, these will be published by Gazebo
    jointStatePublisher_ = nodeHandle_->advertise<sensor_msgs::JointState>("/m1/joint_states", 10);
//    interactionWrenchPublisher_ = nodeHandle_->advertise<geometry_msgs::WrenchStamped>("/M1/interaction_wrench", 10);
//#endif
}

void M1DemoMachineROS::update() {
//#ifndef SIM  // if simulation, these will be published by Gazebo
    publishJointStates();
//    publishInteractionForces();
//#endif
}

void M1DemoMachineROS::publishJointStates() {
    Eigen::VectorXd jointPositions = robot_->getPosition();
    Eigen::VectorXd jointVelocities = robot_->getVelocity();
    Eigen::VectorXd jointTorques = robot_->getTorque();

    jointStateMsg_.header.stamp = ros::Time::now();
    jointStateMsg_.name.resize(M1_NUM_JOINTS);
    jointStateMsg_.position.resize(M1_NUM_JOINTS);
    jointStateMsg_.velocity.resize(M1_NUM_JOINTS);
    jointStateMsg_.effort.resize(M1_NUM_JOINTS);
    jointStateMsg_.name[0] = "M1_joint";
    jointStateMsg_.position[0] = jointPositions[0];
    jointStateMsg_.velocity[0] = jointVelocities[0];
    jointStateMsg_.effort[0] = jointTorques[0];

    jointStatePublisher_.publish(jointStateMsg_);
}

void M1DemoMachineROS::publishInteractionForces() {
//    Eigen::VectorXd interactionTorque = robot_->getInteractionForce();
//    ros::Time time = ros::Time::now();
//
//    interactionWrenchMsg_.header.stamp = time;
//    interactionWrenchMsg_.header.frame_id = "interaction_torque_sensor";
//    interactionWrenchMsg_.wrench.torque.z = interactionTorque[0];
//
//    interactionWrenchPublisher_.publish(interactionWrenchMsg_);
}

void M1DemoMachineROS::setNodeHandle(ros::NodeHandle &nodeHandle) {
    nodeHandle_ = &nodeHandle;
}