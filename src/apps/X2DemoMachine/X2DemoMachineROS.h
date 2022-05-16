/**
 * /file X2DemoMachineROS.h
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
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Trigger.h>
#include <CORC/X2Array.h>
#include <CORC/X2Acceleration.h>
#include <CORC/X2AccelerationMerge.h>

#include "X2Robot.h"
#include "X2DemoState.h"
#include "ros/ros.h"  // This state machine requires ROS

class X2DemoMachineROS {
public:
    X2DemoMachineROS(X2Robot *robot, std::shared_ptr<X2DemoState> x2DemoState, ros::NodeHandle& nodeHandle);
    ~X2DemoMachineROS();

    void update(void);
    void publishJointStates(void);
    void publishInteractionForces(void);
    void publishGroundReactionForces(void);
    void initialize();
    void setNodeHandle(ros::NodeHandle& nodeHandle);
    ros::NodeHandle& getNodeHandle();

    Eigen::VectorXd interactionForceCommand_;

private:
    ros::Publisher jointStatePublisher_;
    ros::Publisher interactionForcePublisher_;
    ros::Publisher groundReactionForcePublisher_[X2_NUM_GRF_SENSORS];

    ros::ServiceServer calibrateForceSensorsService_;
    ros::ServiceServer startHomingService_;
    ros::ServiceServer emergencyStopService_;
    ros::ServiceServer imuCalibrationService_;

    sensor_msgs::JointState jointStateMsg_;
    CORC::X2Array interactionForceMsg_;
    geometry_msgs::WrenchStamped groundReactionForceMsgArray_[X2_NUM_GRF_SENSORS];

    std::string grfFramesArray_[X2_NUM_GRF_SENSORS] = {"left_lower_shank", "right_lower_shank"};

    X2Robot *robot_;
    std::shared_ptr<X2DemoState> x2DemoState_;

    bool calibrateForceSensorsCallback(std_srvs::Trigger::Request& req,
                                       std_srvs::Trigger::Response& res);

    bool startHomingCallback(std_srvs::Trigger::Request& req,
                             std_srvs::Trigger::Response& res);

    bool emergencyStopCallback(std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res);

    bool calibrateIMUCallback(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res);

    ros::NodeHandle* nodeHandle_;
};

#endif  //SRC_X2DEMOMACHINEROS_H
