#include "X2ROS2State.h"

X2ROS2State::X2ROS2State(X2Robot *robot, const std::shared_ptr<X2ROS2Node> node)
    : m_Robot(robot), m_Node(node)
{
}

void
X2ROS2State::exit()
{
    spdlog::info("Exited X2ROS2State");
}

void
X2ROS2State::entry()
{
    spdlog::info("Entered X2ROS2State");
}

void
X2ROS2State::during()
{
}