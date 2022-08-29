/**
 * \file Node.h
 * \author Benjamin von Snarski
 * \brief The robot node is a ROS2 implementation of the robot class which
 * exposes certain parameters through topics over a local network. A basic
 * example of the class subscribes to a joint command topic and copies the
 * message to a member variable. The state machine that constructs the
 * <code>Robot</code> class can then access the joint command message and
 * determine what to do with the ROS2 message. It is intended for this class
 * to interject the inheritence from X2Robot to Robot if the ROS2 compiler
 * option is set to ON.
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
	 * \param __config The YAML config file.
	 */
	RobotNode(const std::string &__node, const std::string &__config);
	/**
	 * \brief Default node destructor.
	 */
	virtual ~RobotNode();
	//@}
	/**
	 * @name Getters
	 */
	//@{
	/**
	 * \brief Gets the joint command message.
	 * \return The joint command as a joint state message type.
	 */
	const JointState &get_joint_command();
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

protected:
	/**
	 * @name ROS2 messages
	 */
	//@{
	/* Joint command message */
	JointState _JointCommand;
	//@}
};

#endif