#include "X2ROS2DemoMachine.h"

X2ROS2DemoMachine::X2ROS2DemoMachine(int argc, char **argv) : StateMachine()
{
    // Create and set X2Robot object
    setRobot(std::make_unique<X2Robot>());

    // Configure ROS2 initialisation options and disable SIGINT capture (handled by CORC)
    rclcpp::InitOptions ros_init = rclcpp::InitOptions();
    ros_init.shutdown_on_signal = false;
    rclcpp::init(argc, argv, ros_init);

    // Create the ROS2 node and pass a reference to the X2 Robot object
    m_Node = std::make_shared<X2ROS2Node>("x2", get_robot());

    // Add some state that also holds reference to the ROS2 node
    addState("state", std::make_shared<X2ROS2State>(get_robot(), get_node()));

    // Set the initial state
    setInitState("state");
}

void
X2ROS2DemoMachine::end()
{
    spdlog::info("State machine end");
}

void
X2ROS2DemoMachine::init()
{
    get_robot()->initialiseNetwork();
}

void
X2ROS2DemoMachine::hwStateUpdate()
{
    // Update robot parameters
    get_robot()->updateRobot();
    // Call custom publish method for publishing joint states of X2 Robot
    get_node()->publish_joint_states();
    // Allow for the ROS2 node to execute callbacks (e.g., subscriptions)
    rclcpp::spin_some(get_node()->get_interface());
}

bool
X2ROS2DemoMachine::configureMasterPDOs()
{
    return get_robot()->configureMasterPDOs();
}

X2Robot *
X2ROS2DemoMachine::get_robot()
{
    return static_cast<X2Robot *>(_robot.get());
}

const std::shared_ptr<X2ROS2Node> &
X2ROS2DemoMachine::get_node()
{
    return m_Node;
}
