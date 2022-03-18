#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, X2DemoMachineROS *x2DemoMachineRos, const char *name) :
        State(m, name), robot_(exo), x2DemoMachineRos_(x2DemoMachineRos) {
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

}

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

    // set dynamic parameter server
    //dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    //f = boost::bind(&X2DemoState::dynReconfCallback, this, _1, _2);
    //server_.setCallback(f);

//    robot_->calibrateForceSensors();
//    robot_->homing();

    robot_->setBackpackIMUMode(IMUOutputMode::QUATERNION);
    time0 = std::chrono::steady_clock::now();

}

void X2DemoState::during(void) {
//    std::cout<<robot_->getBackPackAngleOnMedianPlane()*180/M_PI<<std::endl;
    if(controller_mode_ == 1){ // zero torque mode
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);
//        std::cout<<robot_->getInteractionForce()[1]<<std::endl<<"**********"<<std::endl;

    } else if(controller_mode_ == 2){ // zero velocity mode
        desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setVelocity(desiredJointVelocities_);
//        std::cout<<robot_->getInteractionForce()<<std::endl<<"**********"<<std::endl;

    } else if(controller_mode_ == 3){ // feedforward model compensation
        int motionIntend;
        if(robot_->getPosition()[1]>M_PI/4.0) motionIntend = -1;
        else motionIntend = 1;

        desiredJointTorques_ = robot_->getFeedForwardTorque(motionIntend);
        std::cout<<desiredJointTorques_<<std::endl;
        robot_->setTorque(desiredJointTorques_);

    } else if(controller_mode_ == 4){ // virtual mass controller

        Eigen::VectorXd feedBackTorque = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        double J = 0.1; // distance between knee joint and force sensor

        int motionIntend;
        if(robot_->getPosition()[1]>M_PI/4.0) motionIntend = -1;
        else motionIntend = 1;

        desiredInteractionForce_ = x2DemoMachineRos_->interactionForceCommand_[1]; //todo: uncomment in muliti robot control

        desiredJointAcceleration_ = (1/(robot_->getRobotParameters().m[1]*virtualMassRatio_))*J*(robot_->getInteractionForce()[1] - desiredInteractionForce_);

        feedBackTorque[1] = (1.0/virtualMassRatio_-1)*J*robot_->getInteractionForce()[1] -
                            (1.0/virtualMassRatio_)*J*desiredInteractionForce_;
        desiredJointTorques_ = robot_->getFeedForwardTorque(motionIntend) + feedBackTorque;
        robot_->setTorque(desiredJointTorques_);
//        std::cout<<"robot: "<< robot_->getRobotName()<<std::endl;
//        std::cout<<"desired: "<<desiredInteractionForce_<<std::endl;
//        std::cout<<"force: "<<robot_->getInteractionForce()[1]<<std::endl;
//        std::cout<<"ff: "<<robot_->getFeedForwardTorque(motionIntend)[1]<<std::endl;
//        std::cout<<"fb: "<<feedBackTorque[1]<<std::endl;
//        std::cout<<"total: "<<desiredJointTorques_[1]<<std::endl;
//        std::cout<<"***************"<<std::endl;
    } else if(controller_mode_ == 5){ // Admittance control

        double b0, b1, a1;
        admittanceInputHistory_[0] = robot_->getInteractionForce()[1];

        b0 = t_step_/(mAdmittance_*2.0+bAdmittance_*t_step_);
        b1 = t_step_/(mAdmittance_*2.0+bAdmittance_*t_step_);

        a1 = (mAdmittance_*-4.0)/(mAdmittance_*2.0+bAdmittance_*t_step_)+1.0;

        admittanceOutputHistory_[0] = + b0*admittanceInputHistory_[0] + b1*admittanceInputHistory_[1] - a1*admittanceOutputHistory_[1];

        for(int k =  1 ; k > 0 ; k--) {
            admittanceInputHistory_[k] = admittanceInputHistory_[k-1];
        }

        for(int k =  1 ; k > 0 ; k--) {
            admittanceOutputHistory_[k] = admittanceOutputHistory_[k-1];
        }

        desiredJointVelocities_ << 0, admittanceOutputHistory_[0], 0, 0;

        robot_->setVelocity(desiredJointVelocities_);
//        std::cout<<"Force: "<<robot_->getInteractionForce()[1]<<std::endl;
//        std::cout << "Output vel: " << desiredJointVelocities_[1] * 180.0 / M_PI << std::endl << "****************" << std::endl;

    } else if(controller_mode_ == 6){ // Parameter estimation with velocity control

        double desiredVel = 5.0*M_PI/180.0;

        if(robot_->getPosition()[1]<M_PI/2.0) {
            desiredJointVelocities_ << 0, desiredVel, 0, 0;
        }else {
            desiredJointVelocities_ << 0, 0, 0, 0;
        }

        robot_->setVelocity(desiredJointVelocities_);
//        std::cout<<"Force: "<<robot_->getInteractionForce()[1]<<std::endl;
//        std::cout << "Output vel: " << desiredJointVelocities_[1] * 180.0 / M_PI << std::endl << "****************" << std::endl;

    } else if(controller_mode_ == 7){ // Chirp torque

        double T=15; //chirp time in seconds
        double a=6.; //Amplitude in N.m
        double fi=0; //initial frequency
        double fn=5; //final frequency

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        double f = 0;
        if(time<T) {
            double f = fi + (fn-fi)*time/T;
            desiredJointTorques_[2] = a*sin(2.*M_PI*f*time);
        }
        else {
            desiredJointTorques_[0] = 0;
            std::cout<<"done"<<std::endl;
            controller_mode_ = 0;
        }
        robot_->setTorque(desiredJointTorques_);
    }
}

void X2DemoState::exit(void) {
    robot_->initTorqueControl();
    // setting 0 torque for safety. Not required for X2(2018) but for some reason, in X2(2019), after exit() it takes around 2-3 second to drives to tunr off.
    robot_->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
    std::cout << "Example State Exited" << std::endl;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorques() {
    return desiredJointTorques_;
}

//void X2DemoState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {
//
//    controller_mode_ = config.controller_mode;
//    virtualMassRatio_ = config.virtual_mass_ratio;
//    desiredInteractionForce_ = config.desired_interaction_force;
//    mAdmittance_ = config.m_admittance;
//    bAdmittance_ = config.b_admittance;
//
//    if(controller_mode_ == 1) robot_->initTorqueControl();
//    if(controller_mode_ == 2) robot_->initVelocityControl();
//    if(controller_mode_ == 3) robot_->initTorqueControl();
//    if(controller_mode_ == 4) robot_->initTorqueControl();
//    if(controller_mode_ == 5) robot_->initVelocityControl();
//    if(controller_mode_ == 6) robot_->initVelocityControl();
//    if(controller_mode_ == 7) {
//        robot_->initTorqueControl();
//        time0 = std::chrono::steady_clock::now();
//    }
//
//    return;
//
//}