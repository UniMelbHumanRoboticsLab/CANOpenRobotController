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
    //robot->printStatus();
    //Eigen::Vector3d q = robot->getJointPos();
    //robot->setJointPos(q);
    r*=1.000001;
    double v=sin(2*M_PI*1/5.*elapsedTime);
    robot->setJointVel(Eigen::Vector3d(v,0,0));
    //robot->J();
}

void M3TestState::exitCode(void) {

}






void M3CalibState::entryCode(void) {
    std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "==================================" << std::endl
        << std::endl;

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

