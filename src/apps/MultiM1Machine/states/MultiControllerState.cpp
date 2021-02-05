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
    tick_count = tick_count + 1;
    if(controller_mode_ == 0){  // Homing
        if(cali_stage == 1){
            // set calibration velocity
            JointVec dq_t;
            dq_t(0) = cali_velocity; // calibration velocity
            if(robot_->setJointVel(dq_t) != SUCCESS){
                std::cout << "Error: " << std::endl;
            }

            // monitor velocity and interaction torque
            dq= robot_->getJointVel();
            tau = robot_->getJointTor_s();
            if ((dq(0) <= 2) & (tau(0) >= 2)){
                cali_velocity = 0;
                robot_->applyCalibration();
                robot_->initPositionControl();
                cali_stage = 2;
            }
            else {
                robot_->printJointStatus();
            }
        }
        else if(cali_stage == 2)
        {
            // set position control: 16 is vertical for #2
            q(0) = 16;
            if(robot_->setJointPos(q) != SUCCESS){
                std::cout << "Error: " << std::endl;
            }
            JointVec tau = robot_->getJointTor_s();
            if (tau(0)>0.2 || tau(0)<-0.2)
            {
                robot_->m1ForceSensor->calibrate();

            }
            robot_->printJointStatus();
            std::cout << "Calibration done!" << std::endl;
            cali_stage = 3;
        }
    }
    else if(controller_mode_ == 1){  // zero torque mode
        robot_->setJointTor(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
    }
    else if(controller_mode_ == 2){ // follow position commands
        robot_->setJointPos(multiM1MachineRos_->jointPositionCommand_);
    }
    else if(controller_mode_ == 3){ // follow torque commands
        robot_->setJointTor(multiM1MachineRos_->jointTorqueCommand_);
    }
    else if(controller_mode_ == 4){ // virtual spring - torque mode
        tau = robot_->getJointTor();
        tau_s = (robot_->getJointTor_s()+tau_s)/2;
        q = robot_->getJointPos();
        dq = robot_->getJointVel();

        // get interaction torque from virtual spring
        spring_tor = -multiM1MachineRos_->interactionTorqueCommand_(0);
//        spring_tor = spk_*M_PIf64*(45-q(0))/180;  //stiffness; q(0) in degree

        // torque tracking with PD controller
        error = tau_s(0) + spring_tor*1.5;  // interaction torque error, desired interaction torque is spring_tor, 1.5 is ratio to achieve the desired torque
        delta_error = (error-torque_error_last_time_step)*800;  // derivative of interaction torque error;
        integral_error = integral_error + error;
        tau_cmd(0) = error*kp_ + delta_error*kd_ + integral_error*ki_;  // tau_cmd = P*error + D*delta_error; 1 and 0.001
        torque_error_last_time_step = error;
//        std::cout << "spring_tor:" << spring_tor  << "; sensor_tor: " << tau_s(0) << "; cmd_tor: " << tau_cmd(0) << "; motor_tor: " << tau(0) << std::endl;
        robot_->setJointTor_comp(tau_cmd, tau_s, ffRatio_);

        // reset integral_error every 1 mins, to be decided
        if(tick_count >= 800*60){
            integral_error = 0;
            tick_count = 0;
        }
    }
    else if(controller_mode_ == 5){ // transperancy - torque mode
        tau = robot_->getJointTor();
        tau_s = (robot_->getJointTor_s()+tau_s)/2;
        q = robot_->getJointPos();
        dq = robot_->getJointVel();

        // torque tracking with PD controller
        error = tau_s(0);  // interaction torque error, desired interaction torque is 0
        delta_error = (error-torque_error_last_time_step)*800;  // derivative of interaction torque error;
        integral_error = integral_error + error;
        tau_cmd(0) = error*kp_ + delta_error*kd_ + integral_error*ki_;  // tau_cmd = P*error + D*delta_error + ; 1 and 0.001
        torque_error_last_time_step = error;
        robot_->setJointTor_comp(tau_cmd, tau_s, ffRatio_);

        // reset integral_error every 1 mins, to be decided
        if(tick_count >= 800*tick_max_){
            integral_error = 0;
            tick_count = 0;
        }
    }
}
void MultiControllerState::exit(void) {

}

void MultiControllerState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {

    kp_ = config.kp;
    kd_ = config.kd;
    ki_ = config.ki;
    ffRatio_ = config.ff_ratio;
    spk_ = config.spk;

    if(tick_max_ != config.tick_max)
    {
        tick_max_ = config.tick_max;
        tick_count = 0;
    }

//    controller_mode_ = config.controller_mode;
    if(controller_mode_!=config.controller_mode)
    {
        if(controller_mode_ == 0) {
            robot_->initVelocityControl();
            cali_stage = 1;
            cali_velocity = -30;
        }
        if(controller_mode_ == 1) robot_->initTorqueControl();
        if(controller_mode_ == 2) robot_->initPositionControl();
        if(controller_mode_ == 3) robot_->initTorqueControl();
        if(controller_mode_ == 4) robot_->initTorqueControl();
        if(controller_mode_ == 5) robot_->initTorqueControl();
        controller_mode_ = config.controller_mode;
    }

    return;
}


