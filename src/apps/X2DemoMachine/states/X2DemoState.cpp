#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, X2DemoMachineROS *x2DemoMachineRos, const char *name) :
        State(m, name), robot_(exo), x2DemoMachineRos_(x2DemoMachineRos) {
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

}

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

    // set dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&X2DemoState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

//    std::cout << robot_->getInteractionForce()[1] << std::endl;
//    robot_->calibrateForceSensors();

    time0 = std::chrono::steady_clock::now();
}

void X2DemoState::during(void) {

//    std::cout<<"force 1: "<<robot_->getInteractionForce()<<std::endl;

    if(controller_mode_ == 1){ // zero torque mode
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);

    } else if(controller_mode_ == 2){ // zero velocity mode
        desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setVelocity(desiredJointVelocities_);

    } else if(controller_mode_ == 3){ // system identification mode

    float t_final = 3.0;

    double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count() /1000.0;

    // ramp torque
    desiredJointTorques_[1] = time;

    float period = 1;
    float A = 0;
    float offset = 0.0;
    desiredJointTorques_[1] = A*std::sin(2*M_PI/period*time) + offset;

    std::cout<<"*: "<<time<<std::endl<<std::endl;
    if (time > t_final || robot_->getPosition()[1] > M_PI / 2.0) {
        controller_mode_ = 0;
        spdlog::warn("MODE set to 0");
        desiredJointTorques_[1] = 0.0;
    }
    robot_->setTorque(desiredJointTorques_);

    }
}

void X2DemoState::exit(void) {
    std::cout << "Example State Exited" << std::endl;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorques() {
    return desiredJointTorques_;
}

void X2DemoState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {

    controller_mode_ = config.controller_mode;

    if(controller_mode_ == 1) robot_->initTorqueControl();
    if(controller_mode_ == 2) robot_->initVelocityControl();
    if(controller_mode_ == 3) robot_->initTorqueControl();

    return;



}