#include "Node.h"

RobotNode::RobotNode(const std::string &__node, const std::string &__config)
	: Robot(__node, __config), rclcpp::Node(__node), _JointCommand{}
{
	_JointStatePublisher = create_publisher<JointState>("joint_states", 10);
	_JointCommandSubscription = create_subscription<JointState>(
			"joint_command", 10, std::bind(&RobotNode::joint_command_callback, _1)
	);
}

const JointState &RobotNode::get_joint_command()
{
	return _JointCommand;
}

void RobotNode::publish_joint_states()
{
	JointState msg;

	msg.header.frame_id = this->robotName;
	msg.header.stamp = now();

	msg.position.assign(
		this->jointPositions_.data(),
		this->jointPositions_.data() + this->jointPositions_.size()
	);
	msg.velocity.assign(
		this->jointVelocities_.data(),
		this->jointVelocities_.data() + this->jointVelocities_.size()
	);
	msg.effort.assign(
		this->jointTorques_.data(),
		this->jointTorques_.data() + this->jointTorques_.size()
	);

	_JointStatePublisher->publish(msg);
}

void RobotNode::joint_command_callback(const JointState::SharedPtr __msg)
{
	_JointCommand = *__msg;
}