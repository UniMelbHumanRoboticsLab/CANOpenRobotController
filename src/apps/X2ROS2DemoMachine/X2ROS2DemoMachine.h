/**
 * \file X2ROS2DemoMachine.h
 * \author Benjamin von Snarski
 * \version 0.1
 * \date 2022-10-24
 * \copyright Copyright (c) 2022
 * \brief An example CORC state machine implementing the ROS2 client library.
 */
#ifndef SRC_X2ROS2DEMOMACHINE_H
#define SRC_X2ROS2DEMOMACHINE_H

#include "X2Robot.h"
#include "X2ROS2Node.h"
#include "X2ROS2State.h"
#include "StateMachine.h"

class X2ROS2DemoMachine : public StateMachine
{
public:
	X2ROS2DemoMachine(int argc, char **argv);

	void end() override;
	void init() override;
	void hwStateUpdate() override;
	bool configureMasterPDOs() override;

	X2Robot * get_robot();
	const std::shared_ptr<X2ROS2Node> & get_node();

private:
	std::shared_ptr<X2ROS2Node> m_Node;
};

#endif//SRC_X2ROS2DEMOMACHINE_H