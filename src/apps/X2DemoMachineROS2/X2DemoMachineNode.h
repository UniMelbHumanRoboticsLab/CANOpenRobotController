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

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "X2Robot.h"
#include "rclcpp/rclcpp.hpp"  // This state machine requires ROS

using std::placeholders::_1;

class X2DemoMachineROS {
   public:
    X2DemoMachineROS(X2Robot *robot, std::shared_ptr<rclcpp::Node> &node);
    ~X2DemoMachineROS();

    void update(void);
    void publishJointStates(void);
    void publishInteractionForces(void);
    void initialize();
    void setNodeHandle(std::shared_ptr<rclcpp::Node> &node);
    std::shared_ptr<rclcpp::Node> getNodeHandle();

    bool startExoTriggered_;
    Eigen::VectorXd interactionForceCommand_;

   private:
    auto jointStatePublisher_;
    auto leftThighForcePublisher_;
    auto leftShankForcePublisher_;
    auto rightThighForcePublisher_;
    auto rightShankForcePublisher_;
    auto startExoService_;
    auto calibrateForceSensorsService_;
    auto interactionForceCommandSubscriber_;

    void interactionForceCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    sensor_msgs::msg::JointState jointStateMsg_;
    geometry_msgs::msg::WrenchStamped leftThighForceMsg_;
    geometry_msgs::msg::WrenchStamped leftShankForceMsg_;
    geometry_msgs::msg::WrenchStamped rightThighForceMsg_;
    geometry_msgs::msg::WrenchStamped rightShankForceMsg_;
    X2Robot *robot_;

    bool startExoServiceCallback(std_srvs::srv::Trigger::Request::SharedPtr req,
                                 std_srvs::srv::Trigger::Response::SharedPtr res);

    bool calibrateForceSensorsCallback(std_srvs::srv::Trigger::Request::SharedPtr req,
                                 std_srvs::srv::Trigger::Response::SharedPtr res);

    std::shared_ptr<rclcpp::Node> node;
};

#endif  //SRC_X2DEMOMACHINEROS_H
