/**
 * \file Node.h
 * \author Benjamin von Snarski
 * \brief The robot node is a ROS2 implementation of the robot class which
 * exposes certain parameters through topics over a local network. A basic
 * example of the class subscribes to a joint command topic but is not
 * recommended to be used in practice. Exposing such low-level parameters
 * is dangerous and is instead intended to be replaced by higher level
 * parameters such as controller configurations. The specific joint commands
 * should be kept hidden in the state machine of the CORC app structure.
 *
 * @version 0.1
 * @data 2022-08-29
 *
 * @copyright Copyright (c) 2020,2021
 */
/**
 * @defgroup Robot Robot Module
 * TOOD:
 */
#ifndef NODE_H_INCLUDED
#define NODE_H_INCLUDED

#include "Robot.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;
using namespace sensor_msgs::msg;

/**
 * @ingroup Robot
 * \brief ROS2 node class of the robot.
 */
class RobotNode : public Robot, public rclcpp::Node
{
public:
	/**
	 * @name Constructors and Destructors
	 */
	//@{
	/**
	 * \brief Default node constructor.
	 * \param __node Name of the ROS2 node. Copies the name for the robot class.
	 */
	RobotNode(const std::string &__node);
	/**
	 * \brief Default node destructor.
	 */
	virtual ~RobotNode();
	//@}

private:
	/**
	 * @name ROS2 methods
	 */
	//@{
	/**
	 * \brief Publish joint states.
	 */
	void publish_joint_states();
	/**
	 * \brief Sets the joints from the message.
	 * \param __msg ROS2 message of the joint command.
	 */
	void joint_command_callback(const JointState::SharedPtr __msg);
	//@}

private:
	/**
	 * @name ROS2 publishers and subscriptions
	 */
	//@{
	/* Publisher for joint staetes */
	rclcpp::Publisher<JointState>::SharedPtr _JointStatePublisher;
	/* Subscription for joint commands */
	rclcpp::Subscription<JointState>::SharedPtr _JointCommandSubscription;
	//@}
};

#endif
