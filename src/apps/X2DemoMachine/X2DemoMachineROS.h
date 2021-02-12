/**
 * /file X2DemoState.h
 * /author Emek Baris Kucuktabak
 * /brief ROS part of the X2DemoMachine
 * /version 0.1
 * /date 2020-07-06
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef SRC_X2DEMOMACHINEROS_H
#define SRC_X2DEMOMACHINEROS_H

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>

#include "X2Robot.h"
#include "ros/ros.h"  // This state machine requires ROS

class X2DemoMachineROS {
   public:
    X2DemoMachineROS(X2Robot *robot, ros::NodeHandle& nodeHandle);
    ~X2DemoMachineROS();

    void update(void);
    void publishJointStates(void);
    void publishInteractionForces(void);
    void initialize();
    void setNodeHandle(ros::NodeHandle& nodeHandle);

    bool startExoTriggered_;
    Eigen::VectorXd interactionForceCommand_;

   private:
    ros::Publisher jointStatePublisher_;
    ros::Publisher leftThighForcePublisher_;
    ros::Publisher leftShankForcePublisher_;
    ros::Publisher rightThighForcePublisher_;
    ros::Publisher rightShankForcePublisher_;
    ros::ServiceServer startExoService_;
    ros::ServiceServer calibrateForceSensorsService_;
    ros::Subscriber interactionForceCommandSubscriber_;

    void interactionForceCommandCallback(const std_msgs::Float64MultiArray &msg);

    sensor_msgs::JointState jointStateMsg_;
    geometry_msgs::WrenchStamped leftThighForceMsg_;
    geometry_msgs::WrenchStamped leftShankForceMsg_;
    geometry_msgs::WrenchStamped rightThighForceMsg_;
    geometry_msgs::WrenchStamped rightShankForceMsg_;
    X2Robot *robot_;

    bool startExoServiceCallback(std_srvs::Trigger::Request& req,
                                 std_srvs::Trigger::Response& res);

    bool calibrateForceSensorsCallback(std_srvs::Trigger::Request& req,
                                 std_srvs::Trigger::Response& res);

    ros::NodeHandle* nodeHandle_;
};

#endif  //SRC_X2DEMOMACHINEROS_H
