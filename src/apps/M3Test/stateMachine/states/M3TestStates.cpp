#include "M3TestStates.h"

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}



void M3TestState::entryCode(void) {
    robot->applyCalibration();
    //robot->initPositionControl();
    robot->initVelocityControl();
}

void M3TestState::duringCode(void) {
    //std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
    robot->printStatus();
    /*Eigen::Vector3d q = robot->getJointPos();
    q(1)=68*M_PI/180.-0.1*elapsedTime;*/
    //std::cout << q.transpose() <<std::endl;
    //robot->setJointPos(q);
    //double v=-sin(2*M_PI*1./10*elapsedTime);
    //double v=-0.1;
    //robot->setJointVel(Eigen::Vector3d(0,0,0));
    Eigen::Vector3d dX(0,0,.1);
    robot->setEndEffVel(dX);
}

void M3TestState::exitCode(void) {
    robot->setJointVel(Eigen::Vector3d(0,0,0));
}






void M3CalibState::entryCode(void) {
    robot->initVelocityControl();
}

void M3CalibState::duringCode(void) {
    //Move slowly on each joint until max force detected
    robot->setVelocity(vel);


    std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
}

void M3CalibState::exitCode(void) {

    //Set joint init values


    std::cout << "Exit "<< getName() << std::endl;
}

