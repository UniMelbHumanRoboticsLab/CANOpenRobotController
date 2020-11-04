/**
 * /file M1DemoMachineROS.h
 * /author Emek Baris Kucuktabak
 * /brief ROS part of the M1DemoMachine
 * /version 0.1
 * /date 2020-11-03
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef SRC_M1DEMOMACHINEROS_H
#define SRC_M1DEMOMACHINEROS_H

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>

#include "RobotM1.h"
#include "ros/ros.h"  // This state machine requires ROS

class M1DemoMachineROS {
public:
    M1DemoMachineROS(RobotM1 *robot);
    ~M1DemoMachineROS();

    void update(void);
    void publishJointStates(void);
    void publishInteractionForces(void);
    void initialize();
    void setNodeHandle(ros::NodeHandle& nodeHandle);

private:
    ros::Publisher jointStatePublisher_;
//    ros::Publisher interactionWrenchPublisher_;

    sensor_msgs::JointState jointStateMsg_;
//    geometry_msgs::WrenchStamped interactionWrenchMsg_;
    RobotM1 *robot_;

    ros::NodeHandle* nodeHandle_;
};

#endif  //SRC_M1DEMOMACHINEROS_H
