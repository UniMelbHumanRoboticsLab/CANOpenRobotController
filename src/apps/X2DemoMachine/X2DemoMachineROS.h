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

#include "X2DemoMachine.h"

#include "ros/ros.h" // This state machine requires ROS
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#ifndef SRC_X2DEMOMACHINEROS_H
#define SRC_X2DEMOMACHINEROS_H

class X2DemoMachineROS {

public:
    X2DemoMachineROS(ExoRobot *robot);
    void publishJointStates(void);
    void initialize(ros::NodeHandle nodeHandle);

private:
    ros::NodeHandle nodeHandle_;
    ros::Publisher jointStatePublisher_;
    sensor_msgs::JointState jointStateMsg_;
    ExoRobot *robot_;

};

#endif //SRC_X2DEMOMACHINEROS_H
