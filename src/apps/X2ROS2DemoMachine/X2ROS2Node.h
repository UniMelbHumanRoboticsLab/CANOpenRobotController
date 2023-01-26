/**
 * \file X2ROS2Node.h
 * \author Benjamin von Snarski
 * \version 0.1
 * \date 2022-10-24
 * \copyright Copyright (c) 2022
 * \brief An example ROS2 node that also holds a reference to the robot object.
 */
#ifndef SRC_X2ROS2NODE_H
#define SRC_X2ROS2NODE_H

#include "X2Robot.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;

class X2ROS2Node : public rclcpp::Node
{
public:
    X2ROS2Node(const std::string &name, X2Robot *robot);

    void string_callback(const std_msgs::msg::String::SharedPtr msg);
    void publish_joint_states();

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_interface();

private:
    X2Robot *m_Robot;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_Sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_Pub;
};

#endif//SRC_X2ROS2NODE_H