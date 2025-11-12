#include "FITHVExoDemoStates.h"
#include "FITHVExoDemoMachine.h"

using namespace std;


void CalibState::entry(void) {
    robot->decalibrate();
    calibDone=false;
    robot->initTorqueControl();
    robot->setJointTorque(V2::Zero());
    robot->printJointStatus();
}
void CalibState::during(void) {
    robot->applyCalibration();
    if(robot->isCalibrated()) {
        calibDone=true;
        std::cout << "Calibrated.\n";
    }
    robot->printJointStatus();
}
void CalibState::exit(void) {
    robot->setJointTorque(V2::Zero());
}




void StandbyState::entry(void) {
    robot->initTorqueControl();
    tau=V2::Zero();
}
void StandbyState::during(void) {

    //Apply viscous field and friction comp
    V2 dq=robot->getVelocity();
    for(unsigned int i=0; i<2; i++) {
        tau[i] = b * dq[i];
    }
    robot->setJointTorqueWithCompensation(tau);

    //Keyboard inputs
    if(robot->keyboard->getS()) {
        b-=0.1;
        std::cout << "b=" << b << "\n";
    }
    if(robot->keyboard->getW()) {
        b+=0.1;
        std::cout << "b=" << b << "\n";
    }

    //Regular display status
    if(iterations()%500==1) {
        robot->printJointStatus();
    }
}
void StandbyState::exit(void) {
    //apply zero force/torque
    robot->initTorqueControl();
    robot->setJointTorque(V2::Zero());
}




void WallAssistState::entry(void) {
    //If already within wall at start, setup a smooth transition
    V2 q = robot->getPosition();
    for(unsigned int i=0; i<2; i++) {
        if(q[i]>=q0[i]) {
            q0[i] = q[i];
        }
        else {
            q0[i] = q0t;
        }
    }
}
void WallAssistState::during(void) {

    //Keyboard inputs
    if(robot->keyboard->getKeyUC()=='Z') {
        q0t-=1.*M_PI/180.; std:: cout << "q0=" << q0t/M_PI*180. << "[deg] \n";
    }
    if(robot->keyboard->getKeyUC()=='A') {
        q0t+=1.*M_PI/180.; std:: cout << "q0=" << q0t/M_PI*180. << "[deg] \n";
    }

    if(robot->keyboard->getKeyUC()=='X') {
        k-=1.;
        k = fmin(fmax(k, 0), maxk);
        std:: cout << "k=" << k << " [Nm/rad] \n";
    }
    if(robot->keyboard->getKeyUC()=='S') {
        k+=1.;
        k = fmin(fmax(k, 0), maxk);
        std:: cout << "k=" << k << " [Nm/rad] \n";
    }

    if(robot->keyboard->getKeyUC()=='C') {
        b-=.1;
        std:: cout << "b=" << b << " [Nm/rad.s] \n";
    }
    if(robot->keyboard->getKeyUC()=='D') {
        b+=.1;
        std:: cout << "b=" << b << " [Nm/rad.s] \n";
    }

    V2 tau = V2::Zero();
    V2 q = robot->getPosition();
    V2 dq = robot->getVelocity();
    //Apply impedance wall on each axis
    for(unsigned int i=0; i<2; i++) {
        //Calculate effective applied ref of spring based on desired and change rate to avoid abrupt changes
        q0[i] += sign(q0t - q0[i])*M_PI/5.*dt();
        if(q[i]>=q0[i]) {
            tau[i] = -k*(q[i]-q0[i]);
        }
        else {
            //Some little damping assistance outside the wall in up direction
            if(dq[i]<0) {
                tau[i] = -b * dq[i];
            }
            //Assistance to go downwards
            else {
                tau[i] = b * dq[i];
            }
        }
    }

    //Add some gravity compensation. Assumes thighs vertical and angle is average both hips angles
    double q_mean=(q[0]+q[1])/2.;
    double tau_g=-mgl*sin(q_mean);
    for(unsigned int i=0; i<2; i++) {
        tau[i]+=tau_g;
    }

    //TODO: To change and apply compensation here ONLY if not in wall? Seem ok with compensation
    robot->setJointTorqueWithCompensation(tau);

    //Regular display status
    if(iterations()%500==1) {
        //std::cout << q_mean*180./M_PI << "[deg]\t" << tau_g << "[Nm]\t" << tau[1] << "[Nm]\n";
        robot->printJointStatus();
    }
}
void WallAssistState::exit(void) {
    robot->setJointTorque(V2::Zero());
}




void AmplificationState::entry(void) {
}
void AmplificationState::during(void) {

    //Keyboard inputs
    if(robot->keyboard->getKeyUC()=='C') {
        b-=.1;
        std:: cout << "b=" << b << " [Nm/rad.s] \n";
    }
    if(robot->keyboard->getKeyUC()=='D') {
        b+=.1;
        std:: cout << "b=" << b << " [Nm/rad.s] \n";
    }

    V2 tau = V2::Zero();
    V2 dq=robot->getVelocity();
    //Apply impedance wall on each axis
    for(unsigned int i=0; i<2; i++) {
        //Damping assistance
        tau[i] = b * dq[i];
    }

    robot->setJointTorqueWithCompensation(tau);

    //Regular display status
    if(iterations()%500==1) {
        robot->printJointStatus();
    }
}
void AmplificationState::exit(void) {
    robot->setJointTorque(V2::Zero());
}
