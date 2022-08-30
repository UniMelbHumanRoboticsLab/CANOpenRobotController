#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, const char *name) :
        State(m, name), robot_(exo) {
}

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

    robot_->calibrateForceSensors();
    robot_->homing();
}

void X2DemoState::during(void) {
    // Prints joint command from robot subscription and publish joint states
    auto &cmd = robot_->get_joint_command().position;
    spdlog::info("Joint command: [{}, {}, {}, {}]", cmd[0], cmd[1], cmd[2], cmd[3]);
    robot_->publish_joint_states();
    rclcpp::spin_some(robot_->get_node_base_interface());
}

void X2DemoState::exit(void) {
    robot_->initTorqueControl();
    // setting 0 torque for safety. Not required for X2(2018) but for some reason, in X2(2019), after exit() it takes around 2-3 second to drives to tunr off.
    robot_->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
    std::cout << "Example State Exited" << std::endl;
}