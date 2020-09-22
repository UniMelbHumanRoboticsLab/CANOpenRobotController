#include "X2DemoState.h"
void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

    std::vector<int> homingDirection = {1, -1, 0, 0};
//    robot->homing(homingDirection);
//    robot->calibrateForceSensors();

    robot->initVelocityControl();
//    robot->initTorqueControl();

    jointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    jointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    jointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    interactionForces_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

//    initializeLogger(120000);

    time0 = std::chrono::steady_clock::now();
}
void X2DemoState::during(void) {

    if(controller_type == 1){ // Admittance control

    double b0, b1, a1;
    inputHistory_[0] = robot->getInteractionForce()[3];

    b0 = t_step/(m*2.0+b*t_step);
    b1 = t_step/(m*2.0+b*t_step);

    a1 = (m*-4.0)/(m*2.0+b*t_step)+1.0;

    outputHistory_[0] = + b0*inputHistory_[0] + b1*inputHistory_[1] - a1*outputHistory_[1];

    for(int k =  1 ; k > 0 ; k--) {
        inputHistory_[k] = inputHistory_[k-1];
    }

    for(int k =  1 ; k > 0 ; k--) {
        outputHistory_[k] = outputHistory_[k-1];
    }

    desiredJointVelocities_ << 0, 0, 0, outputHistory_[0];

    robot->setVelocity(desiredJointVelocities_);
    std::cout<<"Force: "<<interactionForces_[3]<<std::endl;
    std::cout << "Output vel: " << desiredJointVelocities_[3] * 180.0 / M_PI << std::endl << "****************" << std::endl;

    }

    else if(controller_type == 2){ // virtual mass controller

        vel_theresh = 2; // [deg/s]
        J = 0.1;
        virtMassRatio = 10.4;
        static_fric = 2.0;

        M = 0.9055; //0.5444
        c0 = 2.2051; //5.7173

        c1 = 3.7910;

        if(std::abs(robot->getVelocity()[3]*180.0/M_PI) > vel_theresh) static_fric = 0;
        else if(robot->getPosition()[3]*180/M_PI <45) static_fric = static_fric;
        else static_fric = -static_fric;

        if(std::abs(robot->getVelocity()[3]*180.0/M_PI) <vel_theresh) dynamic_const_fric = 0.0;
        else if(robot->getVelocity()[3] > 0) dynamic_const_fric = c1;
        else dynamic_const_fric = -c1;

        feedForwardTorque = 9.81*M*std::sin(robot->getPosition()[3] - robot->getPosition()[2]) + c0*robot->getVelocity()[3]
                + dynamic_const_fric + static_fric;
        feedBackTorque = (1.0/virtMassRatio-1)*J*robot->getInteractionForce()[3];

        desiredJointTorques_[3] = feedForwardTorque + feedBackTorque;

        std::cout<<"Force: "<<robot->getInteractionForce()[3]<<std::endl;
        std::cout << "Command Torque: " << desiredJointTorques_[3] << std::endl<<"**********"<<std::endl;
//        std::cout<<"dynamic_const_fric fric: "<<dynamic_const_fric<<std::endl;


        robot->setTorque(desiredJointTorques_);

    }else if(controller_type == 3){ // torque commands for parameter identification

//        float A[3] = {9.0, 8.0, 7.0};
//        float offset[3] = {4.0, 3.0, 2.0};

//        float t_final = 6.0;
//        float period = 3.0;
//        float A[3] = {8.0};
//        float offset[3] = {5.0};

//        float t_final = 6.0;
//        float period = 2.0;
//        float A[3] = {10.0};
//        float offset[3] = {5.0};

//        float t_final = 6.0;
//        float period = 1.0;
//        float A[3] = {12.0};
//        float offset[3] = {5.0};

//        float t_final = 3.0;
//        desiredJointTorques_[3] = 12; // t = 3

        float t_final = 7.0;
        desiredJointTorques_[3] = 1.0 * time; // t = 3

        time = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count())/1000.0;
//        int index = 0*(int)time%27/9;
        int index = 0;
//        desiredJointTorques_[3] = A[index]*std::sin(2*M_PI/period*time) + offset[index];





        robot->setTorque(desiredJointTorques_);

        if(time > t_final){
            desiredJointTorques_[3] = 0;
            robot->setTorque(desiredJointTorques_);
//            if(logSaved == false) {
//
//                signal_logger::logger->saveLoggerData({signal_logger::LogFileType::BINARY});
//                signal_logger::logger->cleanup();
//                logSaved = true;
//            }
        }
    }else if(controller_type == 4) {

        float t_final = 2.0;

        time = (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - time0).count()) / 1000000.0;

//        desiredJointVelocities_ << 0.0, 0.0, 0.0, 80.0 / t_final * M_PI / 180.0;
        desiredJointVelocities_ << 0.0, 00.0* M_PI / 180.0, 0.0, 30.0* M_PI / 180.0;

        if (time > t_final) {
            if (logSaved == false) {
//                signal_logger::logger->saveLoggerData({signal_logger::LogFileType::BINARY});
//                signal_logger::logger->cleanup();
                logSaved = true;
                robot->initTorqueControl();
                robot->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
            }
        }
        else robot->setVelocity(desiredJointVelocities_);
    }

//    updateLogElements();
}
void X2DemoState::exit(void) {
    std::cout << "Example State Exited" << std::endl;
    robot->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
//    signal_logger::logger->saveLoggerData({signal_logger::LogFileType::BINARY});
//    signal_logger::logger->cleanup();

}

//void X2DemoState::log(){
//    logJoint
//            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0
//            << ", ";
//    logJoint
//            << robot->getPosition()[3]
//            << ", ";
//    logJoint
//            << robot->getVelocity()[3]
//            << ", ";
//    logJoint
//            << robot->getTorque()[3]
//            << "\n";
//
//}

//void X2DemoState::initializeLogger(int bufferSize) {
//
//    signal_logger::setSignalLoggerStd();
//    signal_logger::SignalLoggerOptions options;
//    options.updateFrequency_ = 1.0/t_step;
//    signal_logger::logger->initLogger(options);
//    signal_logger::add(jointPositions_, "position", "joint", "rad", 1,
//                       signal_logger::LogElementAction::SAVE, bufferSize);
//    signal_logger::add(jointVelocities_, "velocity", "joint", "rad/s", 1,
//                       signal_logger::LogElementAction::SAVE, bufferSize);
//    signal_logger::add(jointTorques_, "torque", "joint", "N.m", 1,
//                       signal_logger::LogElementAction::SAVE, bufferSize);
//    signal_logger::add(desiredJointTorques_, "desired_torque", "joint", "N.m", 1,
//                       signal_logger::LogElementAction::SAVE, bufferSize);
//    signal_logger::add(interactionForces_, "interaction_force", "sensor", "N", 1,
//                       signal_logger::LogElementAction::SAVE, bufferSize);
//    signal_logger::add(time, "time", "time", "s", 1,
//                       signal_logger::LogElementAction::SAVE, bufferSize);
//
//    signal_logger::logger->updateLogger();
//    signal_logger::logger->startLogger();
//
//}

//void X2DemoState::updateLogElements() {
//
//    time = (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time0).count())/1000000.0;
//    jointPositions_ = robot->getPosition();
//    jointVelocities_ = robot->getVelocity();
//    jointTorques_ = robot->getTorque();
//    interactionForces_ = robot->getInteractionForce();
//
//    signal_logger::logger->collectLoggerData();
//}
