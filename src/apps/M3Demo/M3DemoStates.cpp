#include "M3DemoStates.h"

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}


//minJerk(X0, Xf, T, t, &X, &dX)

Eigen::Vector3d impedance(Eigen::Matrix3d K, Eigen::Matrix3d D, Eigen::Vector3d X0, Eigen::Vector3d X, Eigen::Vector3d dX) {
    return K*(X0-X) - D*dX;
}


void M3DemoState::entryCode(void) {
    //robot->applyCalibration();
    //robot->initPositionControl();
    //robot->initVelocityControl();
    robot->initTorqueControl();
    qi=robot->getJointPos();
    Xi=robot->getEndEffPos();
}
void M3DemoState::duringCode(void) {
    if(iterations%100==1) {
        //std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
        robot->printJointStatus();
        //robot->printStatus();
    }
    /*Eigen::Vector3d q = robot->getJointPos();
    q(1)=68*M_PI/180.-0.1*elapsedTime;*/
    //std::cout << q.transpose() <<std::endl;
    //robot->setJointPos(qi-Eigen::Vector3d(0.03,0.03,0.03));
    //double v=-sin(2*M_PI*1./10*elapsedTime);
    //double v=-0.1;
    //robot->setJointVel(Eigen::Vector3d(0,0,0));

    //robot->printStatus();

    /*Eigen::Vector3d dX(-0.02,0.05,0.1);
    if(robot->getEndEffPos()(2)<0) {
        robot->setEndEffVel(dX);
    }
    else {
        robot->setEndEffVel(Eigen::Vector3d(0,0,0));
    }*/


    /*Eigen::Vector3d Dq;
    if(elapsedTime<5)
        Dq={0,0.015*elapsedTime,0.015*elapsedTime};
    else
        Dq={0,0.015*5.,0.015*5.};
    robot->setJointPos(qi-Dq);*/

    /*Eigen::Vector3d tau(0,-5.0,0);*/
    //robot->setJointTor(robot->calculateGravityTorques());
    /*Eigen::Vector3d F(0,0,0);
    robot->setEndEffForWithCompensation(F);*/
}
void M3DemoState::exitCode(void) {
    robot->setJointVel(Eigen::Vector3d::Zero());
    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
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





void M3MassCompensation::entryCode(void) {

    std::cout << "Press S to decrease mass (-100g), W to increase (+100g)." << mass << std::endl;
}
void M3MassCompensation::duringCode(void) {

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
void M3MassCompensation::exitCode(void) {
    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
}





void M3EndEffDemo::entryCode(void) {
    robot->initVelocityControl();
}
void M3EndEffDemo::duringCode(void) {
    Eigen::Vector3d dX(0,0,0);

    for(unsigned int i=0; i<3; i++) {
        dX(i)=robot->joystick.getAxis(i)/2.;
    }

    //Apply
    robot->setEndEffVel(dX);

    if(iterations%20==1) {
        robot->printStatus();
    }
}
void M3EndEffDemo::exitCode(void) {
    robot->setEndEffVel(Eigen::Vector3d(0,0,0));
}






void M3DemoImpedanceState::entryCode(void) {
    robot->initTorqueControl();
    std::cout << "Press Q to select reference point, S/W to tune K gain and A/D for D gain" << std::endl;
}
void M3DemoImpedanceState::duringCode(void) {

    //Select start point
    if(robot->keyboard.getQ()) {
        Xi=robot->getEndEffPos();
        init=true;
    }

    //K tuning
    if(robot->keyboard.getS()) {
        k -= 5;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    if(robot->keyboard.getW()) {
        k += 5;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    Eigen::Matrix3d K = k*Eigen::Matrix3d::Identity();

    //D tuning
    if(robot->keyboard.getD()) {
        d -= 1;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    if(robot->keyboard.getA()) {
        d += 1;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    Eigen::Matrix3d D = d*Eigen::Matrix3d::Identity();

    //Apply impedance control
    if(init) {
        std::cout << "K=" << k << " D=" << d << " => F=" << impedance(K, Eigen::Matrix3d::Zero(), Xi, robot->getEndEffPos(), robot->getEndEffVel()).transpose() << " N" <<std::endl;
        robot->setEndEffForWithCompensation(impedance(K, D, Xi, robot->getEndEffPos(), robot->getEndEffVel()));
    }
    else {
        robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
    }
}
void M3DemoImpedanceState::exitCode(void) {
    robot->setEndEffForWithCompensation(Eigen::Vector3d::Zero());
}





void M3SamplingEstimationState::entryCode(void) {
    robot->initTorqueControl();
    robot->setEndEffForWithCompensation(Eigen::Vector3d::Zero());
    std::cout << "Move robot around while estimating time" << std::endl;
}
void M3SamplingEstimationState::duringCode(void) {
    //Apply gravity compensation
    robot->setEndEffForWithCompensation(Eigen::Vector3d::Zero());

    //Save dt
    if(iterations<nb_samples) {
        //Do some math for fun
        Eigen::Matrix3d K = 2.36*Eigen::Matrix3d::Identity();
        Eigen::Matrix3d D = 0.235*Eigen::Matrix3d::Identity();
        impedance(K, Eigen::Matrix3d::Zero(), Eigen::Vector3d(-0.5, 0.23, 0.65), robot->getEndEffPos(), robot->getEndEffVel()).transpose();
        robot->J().inverse()*Eigen::Vector3d::Zero()+2*robot->inverseKinematic(Eigen::Vector3d(-0.5, 0, 0));

        //Get time and actual value read from CAN to get sampling rate
        dts[iterations] = dt;
        dX[iterations] = robot->getEndEffVel().norm();
        if(dX[iterations-1]!=dX[iterations]){ //Value has actually been updated
            new_value++;
        }
    }
    else if(iterations==nb_samples) {
        std::cout << "Done." <<std::endl;
        double dt_avg=0;
        double dt_max=0;
        double dt_min=50000;
        for(unsigned int i=0; i<nb_samples; i++) {
            dt_avg+=dts[i]/(double)nb_samples;
            if(dts[i]>dt_max)
                dt_max=dts[i];
            if(i>1 && dts[i]<dt_min)
                dt_min=dts[i];
        }
        std::cout<< std::dec<<std::setprecision(3) << "Loop time (min, avg, max): " << dt_min*1000 << " < " << dt_avg*1000 << " < " << dt_max*1000 << " (ms). Actual CAN sampling: " << nb_samples*dt_avg / (double) new_value*1000 <<  std::endl;
    }
}
void M3SamplingEstimationState::exitCode(void) {
    robot->setEndEffForWithCompensation(Eigen::Vector3d::Zero());

}

