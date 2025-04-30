#include "FITHVExoDemoStates.h"
#include "FITHVExoDemoMachine.h"

using namespace std;


void CalibState::entry(void) {
    calibDone=false;
    stop_reached_time = .0;
    at_stop = false;

    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void CalibState::during(void) {
    //TODO
}
void CalibState::exit(void) {
    //TODO
    //apply zero force/torque
}


void StandbyState::entry(void) {
    //robot->initVelocityControl();
    robot->initTorqueControl();
    cmd=V2::Zero();
}
void StandbyState::during(void) {
    //Apply corresponding force
    //TODO
    robot->setJointTorque(cmd);
    //robot->setJointVelocity(cmd);

    //Keyboard inputs
    if(robot->keyboard->getS()) {
        cmd[1]-=0.1;///180.*M_PI;
        std::cout << cmd.transpose() << "\n";
    }
    if(robot->keyboard->getW()) {
        cmd[1]+=0.1;///180.*M_PI;
        std::cout << cmd.transpose() << "\n";
    }

    //Regular display status
    if(iterations()%200==1) {
        robot->printJointStatus();
    }
}
void StandbyState::exit(void) {
    //TODO
    //apply zero force/torque
    robot->initTorqueControl();
    robot->setJointTorque(V2::Zero());
}

