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
}
void StandbyState::during(void) {
    //Apply corresponding force
    //TODO
    //robot->setJointTorque(tau);

    //Keyboard inputs
    if(robot->keyboard->getS()) {
    //TODO
    }
    if(robot->keyboard->getW()) {
    //TODO
    }

    //Regular display status
    if(iterations()%1000==1) {
        robot->printJointStatus();
    }
}
void StandbyState::exit(void) {
    //TODO
    //apply zero force/torque
}

