/**
 * \file X2ROS2State.h
 * \author Benjamin von Snarski
 * \version 0.1
 * \date 2022-10-24
 * \copyright Copyright (c) 2022
 * \brief An example state that publishes the current X2 robot joint states and subscribes to an example topic.
 * Also holds reference to the ROS2 node.
 */
#ifndef SRC_X2ROS2STATE_H
#define SRC_X2ROS2STATE_H

#include "State.h"
#include "X2Robot.h"
#include "X2ROS2Node.h"

class X2ROS2State : public State
{
public:
    X2ROS2State(X2Robot *robot, const std::shared_ptr<X2ROS2Node> node);

    void exit() override;
    void entry() override;
    void during() override;

private:
    X2Robot *m_Robot;
    const std::shared_ptr<X2ROS2Node> m_Node;
};

#endif//SRC_X2ROS2STATE_H