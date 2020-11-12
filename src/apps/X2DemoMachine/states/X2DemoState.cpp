#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, const char *name) :
        State(m, name), robot_(exo) {
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    mode_ = 0;
}

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

//      robot_->initVelocityControl();
    robot_->initTorqueControl();

//    std::cout << robot_->getInteractionForce()[1] << std::endl;
//    robot_->calibrateForceSensors();

    mode_ = 1;
    time0 = std::chrono::steady_clock::now();
}

void X2DemoState::during(void) {
    float t_final = 3.0;

    double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count() /1000.0;

    // ramp torque
    desiredJointTorques_[1] = time;

    float period = 1;
    float A = 0;
    float offset = 15.0;
    desiredJointTorques_[1] = A*std::sin(2*M_PI/period*time) + offset;

    std::cout<<"*: "<<time<<std::endl<<std::endl;
    if (time > t_final || robot_->getPosition()[1] > M_PI / 2.0) {
        mode_ = 0;
        spdlog::warn("MODE set to 0");
        desiredJointTorques_[1] = 0.0;
    }
    robot_->setTorque(desiredJointTorques_);
}

void X2DemoState::exit(void) {
    std::cout << "Example State Exited" << std::endl;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorques() {
    return desiredJointTorques_;
}
