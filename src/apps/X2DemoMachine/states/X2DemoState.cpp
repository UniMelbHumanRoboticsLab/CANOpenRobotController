#include "X2DemoState.h"

X2DemoState::X2DemoState(X2Robot *exo, const char *name) : State(name), robot_(exo) {
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    enableJoints = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    kTransperancy_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    amplitude_ = 0.0;
    period_ = 5.0;
    offset_ = 0.0;

}

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

    // set dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&X2DemoState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    time0 = std::chrono::steady_clock::now();

}

void X2DemoState::during(void) {

#ifndef SIM
    // GREEN BUTTON IS THE DEAD MAN SWITCH --> if it is not pressed, all motor torques are set to 0. Except controller 2 which sets 0 velocity
    if(robot_->getButtonValue(ButtonColor::GREEN) == 0 && controller_mode_ !=2){
        if(robot_->getControlMode()!=CM_TORQUE_CONTROL) robot_->initTorqueControl();
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);

        return;
    }
#endif

    if(controller_mode_ == 1){ // zero torque mode

        if(robot_->getControlMode()!=CM_TORQUE_CONTROL){
            robot_->initTorqueControl();
        }

        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);

    } else if(controller_mode_ == 2){ // zero velocity mode
        if(robot_->getControlMode()!=CM_VELOCITY_CONTROL) robot_->initVelocityControl();

        desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setVelocity(desiredJointVelocities_);

    } else if(controller_mode_ == 3){ // " a very simple (and not ideal) transparent controller"
        if(robot_->getControlMode()!=CM_TORQUE_CONTROL) robot_->initTorqueControl();

        std::cout<<"force: "<<robot_->getSmoothedInteractionForce()[2]<<std::endl;
        std::cout<<"multiplied: "<<kTransperancy_.asDiagonal()*robot_->getSmoothedInteractionForce()<<std::endl;
        desiredJointTorques_ = robot_->getPseudoInverseOfSelectionMatrixTranspose()*
                (robot_->getFeedForwardTorque() + kTransperancy_.asDiagonal()*robot_->getSmoothedInteractionForce());


        for(int id = 0; id <X2_NUM_JOINTS; id++) desiredJointTorques_[id] *= enableJoints[id];

        robot_->setTorque(desiredJointTorques_);

    } else if(controller_mode_ == 4){ // sin vel
        if(robot_->getControlMode()!=CM_VELOCITY_CONTROL) robot_->initVelocityControl();

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        for(int joint = 0; joint < X2_NUM_JOINTS; joint++)
        {
        desiredJointVelocities_[joint] = enableJoints[joint]*amplitude_*sin(2.0*M_PI/period_*time);
        }

        robot_->setVelocity(desiredJointVelocities_);

    } else if(controller_mode_ == 5){ // sin torque
        if(robot_->getControlMode()!=CM_TORQUE_CONTROL) robot_->initTorqueControl();

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        for(int joint = 0; joint < X2_NUM_JOINTS; joint++)
        {
            desiredJointTorques_[joint] = enableJoints[joint]*amplitude_*sin(2.0*M_PI/period_*time);
        }
        robot_->setTorque(desiredJointTorques_);

    }
}

void X2DemoState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {

    controller_mode_ = config.controller_mode;

    enableJoints[0] = config.left_hip;
    enableJoints[1] = config.left_knee;
    enableJoints[2] = config.right_hip;
    enableJoints[3] = config.right_knee;

    kTransperancy_[1] = config.k_left_hip;
    kTransperancy_[2] = config.k_left_knee;
    kTransperancy_[3] = config.k_right_hip;
    kTransperancy_[4] = config.k_right_knee;

    amplitude_ = config.Amplitude;
    period_ = config.Period;
    offset_ = config.offset;

    robot_->setJointVelDerivativeCutOffFrequency(config.acc_deriv_cutoff);
    robot_->setBackpackVelDerivativeCutOffFrequency(config.backpack_deriv_cutoff);
    robot_->setDynamicParametersCutOffFrequency(config.g_cutoff);

    if(controller_mode_ == 4 || controller_mode_ == 5) time0 = std::chrono::steady_clock::now();

    return;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorques() {
    return desiredJointTorques_;
}

Eigen::VectorXd & X2DemoState::getDesiredJointVelocities() {
    return desiredJointVelocities_;
}