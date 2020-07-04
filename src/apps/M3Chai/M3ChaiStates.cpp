#include "M3ChaiStates.h"

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

void M3CalibState::entryCode(void) {
    calibDone=false;
    for(unsigned int i=0; i<3; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
    std::cout << "Calibrating (keep clear)...";
}
//Move slowly on each joint until max force detected
void M3CalibState::duringCode(void) {
    Eigen::Vector3d tau(0, 0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    Eigen::Vector3d vel=robot->getJointVel();
    double b = 3.;
    for(unsigned int i=0; i<3; i++) {
        tau(i) = std::min(std::max(2 - b * vel(i), .0), 2.);
        if(stop_reached_time(i)>0.5) {
            at_stop[i]=true;
        }
        if(vel(i)<0.01) {
            stop_reached_time(i) += dt;
        }
    }

    //Switch to gravity control when done
    if(robot->isCalibrated()) {
        robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
        robot->printJointStatus();
        calibDone=true; //Trigger event
    }
    else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1] && at_stop[2]) {
            robot->applyCalibration();
            std::cout << "OK." << std::endl;
        }
        else {
            robot->setJointTor(tau);
        }
    }
}
void M3CalibState::exitCode(void) {
    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
}





void M3ChaiCommunication::entryCode(void) {

    std::cout << "Press S to decrease mass (-100g), W to increase (+100g)." << mass << std::endl;
}
void M3ChaiCommunication::duringCode(void) {

    //Smooth transition in case a mass is set at startup
    double settling_time = 3.0;
    double t=elapsedTime>settling_time?1.0:elapsedTime/settling_time;

    //Bound mass to +-5kg
    if(mass>5.0) {
        mass = 5;
    }
    if(mass<-5) {
        mass = -5;
    }

    //Apply corresponding force
    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,t*mass*9.8));

    //Mass controllable through keyboard inputs
    if(robot->keyboard.getS()) {
        mass -=0.1;
        std::cout << "Mass: " << mass << std::endl;
    }
    if(robot->keyboard.getW()) {
        mass +=0.1;
        std::cout << "Mass: " << mass << std::endl;
    }
}
void M3ChaiCommunication::exitCode(void) {
    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
}
