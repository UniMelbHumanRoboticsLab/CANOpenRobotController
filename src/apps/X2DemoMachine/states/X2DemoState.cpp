#include "X2DemoState.h"
void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

    std::vector<int> homingDirection = {1, -1, 0, 0};
//    robot->homing(homingDirection);
    robot->calibrateForceSensors();

//    robot->initVelocityControl();
    robot->initTorqueControl();

    jointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    jointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    jointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredjointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredjointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

    initializeLogger(120000);

    time0 = std::chrono::steady_clock::now();

}
void X2DemoState::during(void) {

    if(controller_type == 1){ // Admittance control

//    double b0, b1, a1;
//    inputHistory_[0] = robot->getInteractionForce()[1];
//
//    b0 = t_step/(mass*2.0+b*t_step);
//    b1 = t_step/(mass*2.0+b*t_step);
//
//    a1 = (mass*-4.0)/(mass*2.0+b*t_step)+1.0;
//
//    outputHistory_[0] = + b0*inputHistory_[0] + b1*inputHistory_[1] - a1*outputHistory_[1];
//
//    for(int k =  1 ; k > 0 ; k--) {
//        inputHistory_[k] = inputHistory_[k-1];
//    }
//
//    for(int k =  1 ; k > 0 ; k--) {
//        outputHistory_[k] = outputHistory_[k-1];
//    }
//
//    desiredVelocity << 0, outputHistory_[0], 0, 0;
//
//    robot->setVelocity(desiredVelocity);
//    std::cout<<"Output vel: "<<desiredVelocity[1]*180.0/M_PI<<std::endl<<"****************"<<std::endl;

//    log();

    }

    else if(controller_type == 2){ // virtual mass controller

        vel_theresh = 2; // [deg/s]
        J = 0.1;
        virtMassRatio = 0.5;
        static_fric = 3;
        M = 0.5444;
        c0 = 5.7173;

        if(robot->getVelocity()[3]*180.0/M_PI >vel_theresh) static_fric = 0;
        else if(robot->getPosition()[3]*180/M_PI <45) static_fric = static_fric;
        else static_fric = -static_fric;
        feedForwardTorque = 9.81*M*std::sin(robot->getPosition()[3] - robot->getPosition()[2]) + c0*robot->getVelocity()[3] + static_fric;
        feedBackTorque = 1.0/virtMassRatio*J*robot->getInteractionForce()[3];

        desiredjointTorques_[3] = feedForwardTorque + feedBackTorque;

        std::cout<<"Command Torque: "<<desiredjointTorques_[3]<<std::endl;
        std::cout<<"Force: "<<robot->getInteractionForce()[3]<<std::endl;

        robot->setTorque(desiredjointTorques_);

    }else if(controller_type == 3){

        float t_final = 3.0;
        float period = 2.0; //3.0
//        float A[3] = {9.0, 8.0, 7.0};
//        float offset[3] = {4.0, 3.0, 2.0};

        float A[3] = {10.0};
        float offset[3] = {5.0};

        time = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count())/1000.0;
//        int index = 0*(int)time%27/9;
//        desiredjointTorques_[3] = A[index]*std::sin(2*M_PI/period*time) + offset[index];

        desiredjointTorques_[3] = 11; // t = 3
//        desiredjointTorques_[3] = 5.0*time; // t = 3

//         desiredjointTorques_[3] = A[index]*std::sin(2*M_PI/period*time) + offset[index];

        robot->setTorque(desiredjointTorques_);

        if(time > t_final){
            desiredjointTorques_[3] = 0;
            robot->setTorque(desiredjointTorques_);
            this->exit();

        }

    }

    updateLogElements();

}
void X2DemoState::exit(void) {
    std::cout << "Example State Exited" << std::endl;
    robot->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
    signal_logger::logger->saveLoggerData(  {signal_logger::LogFileType::BINARY}  );
    signal_logger::logger->cleanup();
//    signal_logger::logger->stopAndSaveLoggerData();
}

void X2DemoState::log(){
    logJoint
            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0
            << ", ";
    logJoint
            << robot->getPosition()[3]
            << ", ";
    logJoint
            << robot->getVelocity()[3]
            << ", ";
    logJoint
            << robot->getTorque()[3]
            << "\n";

}

void X2DemoState::initializeLogger(int bufferSize) {

    signal_logger::setSignalLoggerStd();
    signal_logger::SignalLoggerOptions options;
    options.updateFrequency_ = 1.0/t_step;
    signal_logger::logger->initLogger(options);
    signal_logger::add(jointPositions_, "position", "joint", "rad", 1,
                       signal_logger::LogElementAction::SAVE, bufferSize);
    signal_logger::add(jointVelocities_, "velocity", "joint", "rad/s", 1,
                       signal_logger::LogElementAction::SAVE, bufferSize);
    signal_logger::add(jointTorques_, "torque", "joint", "N.m", 1,
                       signal_logger::LogElementAction::SAVE, bufferSize);
    signal_logger::add(desiredjointTorques_, "desired_torque", "joint", "N.m", 1,
                       signal_logger::LogElementAction::SAVE, bufferSize);
    signal_logger::add(time, "time", "time", "s", 1,
                       signal_logger::LogElementAction::SAVE, bufferSize);

    signal_logger::logger->updateLogger();
    signal_logger::logger->startLogger();

}

void X2DemoState::updateLogElements() {

    time = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count())/1000.0;
    jointPositions_ = robot->getPosition();
    jointVelocities_ = robot->getVelocity();
    jointTorques_ = robot->getTorque();

    signal_logger::logger->collectLoggerData();
}
