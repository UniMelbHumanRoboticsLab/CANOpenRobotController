#include "Node.h"

RobotNode::RobotNode(const std::string &__node) : Robot(__node), rclcpp::Node(__node)
{

}
