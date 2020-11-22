#include "MultiControllerState.h"

void MultiControllerState::entry(void) {

    spdlog::info("Multi Controller State is entered.");

    // set dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&MultiControllerState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    robot_->applyCalibration();

}
void MultiControllerState::during(void) {

    if(controller_mode_ == 1){  // zero torque mode
        robot_->setJointTor(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
    }
    else if(controller_mode_ == 2){ // follow position commands
        robot_->setJointPos(multiM1MachineRos_->jointPositionCommand_);
    }
    else if(controller_mode_ == 3){ // follow torque commands
        robot_->setJointTor(multiM1MachineRos_->jointTorqueCommand_);
    }
}
void MultiControllerState::exit(void) {

}

void MultiControllerState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {

    controller_mode_ = config.controller_mode;

    if(controller_mode_ == 1) robot_->initTorqueControl();
    if(controller_mode_ == 2) robot_->initPositionControl();
    if(controller_mode_ == 3) robot_->initTorqueControl();

    return;
}


