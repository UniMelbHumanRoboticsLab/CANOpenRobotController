#include "MultiControllerState.h"

void MultiControllerState::entry(void) {

    spdlog::info("Multi Controller State is entered.");

    // set dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&MultiControllerState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    robot_->applyCalibration();
    robot_->calibrateForceSensors();

    // FOR TRANSPERANCY STUFF
    q = Eigen::VectorXd::Zero(1);
    dq = Eigen::VectorXd::Zero(1);
    tau = Eigen::VectorXd::Zero(1);
    tau_s = Eigen::VectorXd::Zero(1);
    tau_cmd = Eigen::VectorXd::Zero(1);

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
    else if(controller_mode_ == 4){ // transperancy - torque mode
        tau = robot_->getJointTor();
        tau_s = (robot_->getJointTor_s()+tau_s)/2;
        q = robot_->getJointPos();
        dq = robot_->getJointVel();

        // torque tracking with PD controller
        error = tau_s(0);  // interaction torque error, desired interaction torque is 0
        delta_error = (error-torque_error_last_time_step)*800;  // derivative of interaction torque error;
        tau_cmd(0) = error*kp_ + delta_error*kd_;  // tau_cmd = P*error + D*delta_error; 1 and 0.001
        torque_error_last_time_step = error;

        robot_->setJointTor_comp(tau_cmd, tau_s, ffRatio_);
    }
    else if(controller_mode_ == 5){ // virtual spring - torque mode
        tau = robot_->getJointTor();
        tau_s = (robot_->getJointTor_s()+tau_s)/2;
        q = robot_->getJointPos();
        dq = robot_->getJointVel();

        spring_tor = spk_*3.14*(30-q(0))/180;  //stiffness; q(0) in degree
//        spdlog::info("MultiM1MachineROS::init()");

        // torque tracking with PD controller
//        error = spring_tor;  // interaction torque error, desired interaction torque is 0
        error = tau_s(0) + spring_tor;  // interaction torque error, desired interaction torque is 0
        delta_error = (error-torque_error_last_time_step)*800;  // derivative of interaction torque error;
        tau_cmd(0) = error*kp_ + delta_error*kd_;  // tau_cmd = P*error + D*delta_error; 1 and 0.001
        torque_error_last_time_step = error;
//        tau_cmd(0) = spring_tor;
//        std::cout << "spring_tor:" << spring_tor  << "; sensor_tor: " << tau_s(0) << "; cmd_tor: " << tau_cmd(0) << "; motor_tor: " << tau(0) << std::endl;
//        "pos:" << q(0) << "; spring_tor:" <<
        robot_->setJointTor_comp(tau_cmd, tau_s, ffRatio_);
//        robot_->setJointTor(tau_cmd);
    }
}
void MultiControllerState::exit(void) {

}

void MultiControllerState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {

    controller_mode_ = config.controller_mode;
    kp_ = config.kp;
    kd_ = config.kd;
    ffRatio_ = config.ff_ratio;
    spk_ = config.spk;

    if(controller_mode_ == 1) robot_->initTorqueControl();
    if(controller_mode_ == 2) robot_->initPositionControl();
    if(controller_mode_ == 3) robot_->initTorqueControl();
    if(controller_mode_ == 4) robot_->initTorqueControl();
    if(controller_mode_ == 5) robot_->initTorqueControl();

    return;
}


