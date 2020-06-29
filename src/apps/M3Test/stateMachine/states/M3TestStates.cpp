#include "M3TestStates.h"

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

static Eigen::Vector3d qi, Xi;
double spd=0;

void M3TestState::entryCode(void) {
    robot->applyCalibration();
    //robot->initPositionControl();
    robot->initVelocityControl();
    qi=robot->getJointPos();
    Xi=robot->getEndEffPos();
}

void M3TestState::duringCode(void) {
    if(iterations%10==1) {
        //std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
        robot->printJointStatus();
        //robot->printStatus();
    }
    /*Eigen::Vector3d q = robot->getJointPos();
    q(1)=68*M_PI/180.-0.1*elapsedTime;*/
    //std::cout << q.transpose() <<std::endl;
    //robot->setJointPos(qi-Eigen::Vector3d(0.1,0.1,0.1));
    //double v=-sin(2*M_PI*1./10*elapsedTime);
    //double v=-0.1;
    //robot->setJointVel(Eigen::Vector3d(0,0,0));

    //robot->printStatus();
    Eigen::Vector3d dX(0,0,0.1);
    //std::cout << (robot->J().inverse()*dX).transpose() <<std::endl;
    if(robot->getEndEffPos()(2)<0) {
        robot->setEndEffVel(dX);
    }
    else {
        std::cout << std::setprecision(5) << ((Xi-robot->getEndEffPos()).norm())/elapsedTime <<std::endl;
        robot->setEndEffVel(Eigen::Vector3d(0,0,0));
    }


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

