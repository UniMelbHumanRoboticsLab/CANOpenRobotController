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

    if(chaiServer.Connect("192.168.6.2")!=0) {
        std::cerr << "M3ChaiCommunication: Unable to initialise socket... Quitting." <<std::endl;
        raise(SIGTERM); //Clean exit
    }
}
void M3ChaiCommunication::duringCode(void) {

    //Retrive values from chai client if exist
    if(chaiServer.IsConnected()) {
        if(chaiServer.IsReceivedValues()) {
            double *force = new double[3];
            force = chaiServer.GetReceivedValues();
            lastReceivedTime = elapsedTime;
            F=Eigen::Vector3d(-force[0], -force[1], force[2]);//Chai representation frame is: X towards the operator when facing device, Y towards right hand side and Z up
            std::cout << F.transpose() << std::endl;
        }

        //Watchdog: If no fresh values for more than 10ms, fallback
        if(elapsedTime-lastReceivedTime>watchDogTime) {
             F=Eigen::Vector3d::Zero();
             std::cerr << "M3ChaiCommunication: No new value received from client (in last " << watchDogTime*1000. << "ms): fallback."  << std::endl;
        }

        //Anyway send values
        X=robot->getEndEffPos();
        double *x = new double[3]{-X(0),-X(1),X(2)}; //Chai representation frame is: X towards the operator when facing device, Y towards right hand side and Z up
        chaiServer.Send(x);
    }
    else {
        F=Eigen::Vector3d::Zero();
    }

    //Apply requested force on top of device gravity compensation
    robot->setEndEffForWithCompensation(F);
}
void M3ChaiCommunication::exitCode(void) {
    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
}
