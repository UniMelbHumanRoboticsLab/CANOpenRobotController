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

#include "X2Robot.h"
#include "ros/ros.h"  // This state machine requires ROS

class X2DemoMachineROS {
   public:
    X2DemoMachineROS(X2Robot *robot);
    ~X2DemoMachineROS();

    void update(void);
    void publishJointStates(void);
    void publishInteractionForces(void);
    void initialize();
    void setNodeHandle(ros::NodeHandle& nodeHandle);

   private:
    ros::Publisher jointStatePublisher_;
    ros::Publisher leftThighForcePublisher_;
    ros::Publisher leftShankForcePublisher_;
    ros::Publisher rightThighForcePublisher_;
    ros::Publisher rightShankForcePublisher_;

    sensor_msgs::JointState jointStateMsg_;
    geometry_msgs::WrenchStamped leftThighForceMsg_;
    geometry_msgs::WrenchStamped leftShankForceMsg_;
    geometry_msgs::WrenchStamped rightThighForceMsg_;
    geometry_msgs::WrenchStamped rightShankForceMsg_;
    X2Robot *robot_;

    ros::NodeHandle* nodeHandle_;
};

#endif  //SRC_X2DEMOMACHINEROS_H
