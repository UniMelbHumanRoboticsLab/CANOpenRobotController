#include "M3TestStates.h"

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}


//minJerk(X0, Xf, T, t, &X, &dX)

Eigen::Vector3d impedance(Eigen::Matrix3d K, Eigen::Matrix3d D, Eigen::Vector3d X0, Eigen::Vector3d X, Eigen::Vector3d dX) {
    return K*(X-X0) - D*dX;
}




void M3TestState::entryCode(void) {
    robot->applyCalibration();
    //robot->initPositionControl();
    //robot->initVelocityControl();
    robot->initTorqueControl();
    qi=robot->getJointPos();
    Xi=robot->getEndEffPos();
}

void M3TestState::duringCode(void) {
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
    Eigen::Vector3d F(0,0,0);
    robot->setEndEffForWithCompensation(F);
}

void M3TestState::exitCode(void) {
    robot->initTorqueControl();
    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
}


//void M3CalibState::entryCode(void) {
//    for(unsigned int i=0; i<3; i++) {
//        stop_reached_time[i] = .0;
//        at_stop[i] = false;
//    }
//    robot->decalibrate();
//    robot->initVelocityControl();
//    std::cout << "Calibrating...";
//}
//
////Move slowly on each joint until max force detected
//void M3CalibState::duringCode(void) {
//    Eigen::Vector3d vel(0, 0, 0);
//
//    //Apply velocity unless stop has been detected for more than 0.5s
//    for(unsigned int i=0; i<3; i++) {
//        if(stop_reached_time(i)<0.5) {
//            vel(i)=0.1;
//        } else {
//            vel(i)=0;
//            at_stop[i]=true;
//        }
//
//        Eigen::Vector3d tau=robot->getJointTor();
//        if(tau(i)>tau_threshold) {
//            stop_reached_time(i) += dt;
//        }
//    }
//
//    //Switch to gravity control when done
//    if(robot->isCalibrated()) {
//        robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
//    }
//    else {
//        //If all joints are calibrated
//        if(at_stop[0] && at_stop[1] && at_stop[2]) {
//            robot->applyCalibration();
//            robot->initTorqueControl();
//            std::cout << "OK." << std::endl;
//        }
//        else {
//            robot->setJointVel(vel);
//        }
//    }
//
//    //robot->printJointStatus();
//}

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

    //Apply constant torque unless stop has been detected for more than 0.5s
    for(unsigned int i=0; i<3; i++) {
        if(stop_reached_time(i)<0.5) {
            tau(i)=2;
        } else {
            tau(i)=0;
            at_stop[i]=true;
        }

        Eigen::Vector3d vel=robot->getJointVel();
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
            //robot->initTorqueControl();
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



