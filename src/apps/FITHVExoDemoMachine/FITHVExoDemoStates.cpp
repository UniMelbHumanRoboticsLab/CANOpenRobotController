#include "FITHVExoDemoStates.h"
#include "FITHVExoDemoMachine.h"

using namespace std;


VM2 impedance(Eigen::Matrix2d K, Eigen::Matrix2d D, V2 X0, V2 X, V2 dX, V2 dXd=V2::Zero()) {
    return K*(X0-X) + D*(dXd-dX);
}

double JerkIt(V2 X0, V2 Xf, double T, double t, V2 &Xd, V2 &dXd) {
    t = std::max(std::min(t, T), .0); //Bound time
    double tn=std::max(std::min(t/T, 1.0), .0);//Normalised time bounded 0-1
    double tn3=pow(tn,3.);
    double tn4=tn*tn3;
    double tn5=tn*tn4;
    Xd = X0 + ( (X0-Xf) * (15.*tn4-6.*tn5-10.*tn3) );
    dXd = (X0-Xf) * (4.*15.*tn4-5.*6.*tn5-10.*3*tn3)/t;
    return tn;
}


void CalibState::entry(void) {
    robot->decalibrate();
    calibDone=false;
    robot->initTorqueControl();
    robot->setJointTorque(V2::Zero());
    robot->printJointStatus();
}
//Move slowly on each joint until max force detected
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




void TestState::entry(void) {
    //robot->initVelocityControl();
    robot->initTorqueControl();
    cmd=V2::Zero();
}
void TestState::during(void) {
    //Apply cmd
    robot->setJointTorque(cmd);
    //robot->setJointTorqueWithCompensation(cmd);
    //robot->setJointVelocity(cmd);

    robot->applyCalibration();

    //Keyboard inputs
    if(robot->keyboard->getS()) {
        cmd[1]-=1.0*M_PI/180.;
        std::cout << cmd.transpose()*180.*M_PI << "\n";
    }
    if(robot->keyboard->getW()) {
        cmd[1]+=1.0*M_PI/180.;
        std::cout << cmd.transpose()*180.*M_PI << "\n";
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
    if(robot->keyboard->getS()) {;
        a-=0.1; std:: cout << "a=" << a << "\n";
    }
    if(robot->keyboard->getW()) {
        a+=0.1; std:: cout << "a=" << a << "\n";
    }

    if(robot->keyboard->getA()) {
        b-=0.1; std:: cout << "b=" << b << "\n";
    }
    if(robot->keyboard->getD()) {
        b+=0.1; std:: cout << "b=" << b << "\n";
    }*/



    //Regular display status
    if(iterations()%200==1) {
        robot->printJointStatus();
    }
}
void TestState::exit(void) {
    //apply zero force/torque
    robot->initTorqueControl();
    robot->setJointTorque(V2::Zero());
}




void WallAssistState::entry(void) {
    robot->initTorqueControl();
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
    spdlog::debug("----- q0: {},    q0t: {}", q0[1], q0t);
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
    V2 dq=robot->getVelocity();
    //Apply impedance wall on each axis
    for(unsigned int i=0; i<2; i++) {
        //Calculate effective applied ref of spring based on desired and ahcnge rate to avoid abrupt changes
        q0[i] += sign(q0t - q0[i])*M_PI/5.*dt();
        if(q[i]>=q0[i]) {
            tau[i] = -k*(q[i]-q0[i]);
        }
        else {
            //Some little damping assistance outside the wall
            tau[i] = b * dq[i];
        }
    }
    //TODO: To change and apply compensation here ONLY if not in wall
    robot->setJointTorqueWithCompensation(tau);

    //Regular display status
    if(iterations()%500==1) {
        robot->printJointStatus();
    }
}
void WallAssistState::exit(void) {
    robot->setJointTorque(V2::Zero());
}

