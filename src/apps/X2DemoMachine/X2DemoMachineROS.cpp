#include "X2DemoMachineROS.h"

X2DemoMachineROS::X2DemoMachineROS(X2Robot *robot) {
    robot_ = robot;
}

X2DemoMachineROS::~X2DemoMachineROS() {
    ros::shutdown();
}

void X2DemoMachineROS::initialize(int argc, char *argv[]) {
    ros::init(argc, argv, "x2_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle;

    jointStatePublisher_ = nodeHandle.advertise<sensor_msgs::JointState>("joint_states", 10);
}

void X2DemoMachineROS::update() {
    publishJointStates();
    ros::spinOnce();
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
    jointStateMsg_.name[0] = "left_hip";
    jointStateMsg_.name[1] = "left_knee";
    jointStateMsg_.name[2] = "right_hip";
    jointStateMsg_.name[3] = "right_knee";
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
