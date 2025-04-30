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
    //Apply cmd
    //robot->setJointTorque(cmd);
    robot->setJointTorqueWithCompensation(cmd);
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

    /*V2 tau_f;
    double threshold = 0.05;
    for (unsigned int i = 0; i < 2; i++) {
        double dq = robot->getVelocity()[i];
        if (abs(dq) > threshold) {
            tau_f(i) = a * sign(dq) + b * dq;
        }
        else {
            tau_f(i) = .0;
        }
    }
    robot->setJointTorque(tau_f);

    //Keyboard inputs
    if(robot->keyboard->getS()) {
        //cmd[1]-=0.1;///180.*M_PI;
        //std::cout << cmd.transpose() << "\n";
        a-=0.1; std:: cout << "a=" << a << "\n";
    }
    if(robot->keyboard->getW()) {
        //cmd[1]+=0.1;///180.*M_PI;
        //std::cout << cmd.transpose() << "\n";
        a+=0.1; std:: cout << "a=" << a << "\n";
    }

    if(robot->keyboard->getA()) {
        //cmd[1]-=0.1;///180.*M_PI;
        //std::cout << cmd.transpose() << "\n";
        b-=0.1; std:: cout << "b=" << b << "\n";
    }
    if(robot->keyboard->getQ()) {
        //cmd[1]+=0.1;///180.*M_PI;
        //std::cout << cmd.transpose() << "\n";
        b+=0.1; std:: cout << "b=" << b << "\n";
    }*/



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

