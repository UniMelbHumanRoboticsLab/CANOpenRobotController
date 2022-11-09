#include "X2ROS2Node.h"

X2ROS2Node::X2ROS2Node(const std::string &name, X2Robot *robot)
    : Node(name), m_Robot(robot)
{
    // Create an example subscription
    m_Sub = create_subscription<std_msgs::msg::String>(
        "example_topic", 10, std::bind(&X2ROS2Node::string_callback, this, _1)
    );
    // Create a joint state publisher
    m_Pub = create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10
    );
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
X2ROS2Node::get_interface()
{
    // Must have this method to return base interface for spinning
    return this->get_node_base_interface();
}

void
X2ROS2Node::string_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // Example callback for subscription
    spdlog::info("Received ({}) in string callback", msg->data);
}

void
X2ROS2Node::publish_joint_states()
{
    // Instantiate joint state message
    sensor_msgs::msg::JointState msg;

    // Assign current header time stamp
    msg.header.stamp = this->now();

    // Use this naming scheme for robot state publisher to recognise
    msg.name = {
        "left_hip_joint",
        "left_knee_joint",
        "right_hip_joint",
        "right_knee_joint",
        "world_to_backpack"
    };

    // Copy position, velocity and toroque from X2 Robot
    msg.position.assign(
        m_Robot->getPosition().data(),
        m_Robot->getPosition().data() + m_Robot->getPosition().size()
    );
    msg.velocity.assign(
        m_Robot->getVelocity().data(),
        m_Robot->getVelocity().data() + m_Robot->getVelocity().size()
    );
    msg.effort.assign(
        m_Robot->getTorque().data(),
        m_Robot->getTorque().data() + m_Robot->getTorque().size()
    );

    // Add backpack angle to positions for visualisation (RViz)
    msg.position.push_back(m_Robot->getBackPackAngleOnMedianPlane() - M_PI_2);

    // Publish the joint state message
    m_Pub->publish(msg);
}