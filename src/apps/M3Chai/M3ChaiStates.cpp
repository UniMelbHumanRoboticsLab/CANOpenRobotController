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
    spdlog::info("Calibrating (keep clear)...");
}
//Move slowly on each joint until max force detected
void M3CalibState::duringCode(void) {
    Eigen::Vector3d tau(0, 0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    VM3 vel=robot->getVelocity();
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
        robot->setEndEffForceWithCompensation(Eigen::Vector3d(0,0,0));
        robot->printJointStatus();
        calibDone=true; //Trigger event
    }
    else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1] && at_stop[2]) {
            robot->applyCalibration();
            spdlog::info("Calibrating: OK.");
        }
        else {
            robot->setJointTorque(tau);
        }
    }
}
void M3CalibState::exitCode(void) {
    robot->setEndEffForceWithCompensation(Eigen::Vector3d(0,0,0));
}




void M3ChaiWaitForCommunication::entryCode(void) {
    robot->setEndEffForceWithCompensation(VM3::Zero());
}
void M3ChaiWaitForCommunication::duringCode(void) {
    //Simply transparent mode while not connected
    robot->setEndEffForceWithCompensation(VM3(0,0,0));

    //Attempt to reconnect (wait for client incoming connection)
    chaiServer->Reconnect();

    if(iterations%100==0)
        spdlog::info("M3ChaiCommunication: Wait for connection");
}
void M3ChaiWaitForCommunication::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM3(0,0,0));
}




void M3ChaiCommunication::entryCode(void) {
    F=Eigen::Vector3d::Zero();
    robot->setEndEffForceWithCompensation(F);
}
void M3ChaiCommunication::duringCode(void) {

    //Retrieve values from chai client if exist
    if(chaiServer->IsConnected()) {
        if(chaiServer->IsReceivedValues()) {
            double force[3];
            chaiServer->GetReceivedValues(force);
            lastReceivedTime = elapsedTime;
            F=VM3(-force[0], force[1], force[2]);//Chai representation frame is: X towards the operator when facing device, Y towards right hand side and Z up
        } else if(elapsedTime-lastReceivedTime>watchDogTime) {
            //Watchdog: If no fresh values for more than 10ms, fallback
            F=VM3::Zero();
            spdlog::warn("M3ChaiCommunication: No new value received from client in last {} ms: fallback.", watchDogTime*1000.);
        }

        //Anyway send values
        X=robot->getEndEffPosition();
        dX=robot->getEndEffVelocity();
        double x[6]={-X(0),X(1),X(2),-dX(0),dX(1),dX(2)};  //Chai representation frame is: X towards the operator when facing device, Y towards right hand side and Z up
        chaiServer->Send(x);
    }
    else {
        if(iterations%100==0)
            spdlog::warn("M3ChaiCommunication: Reconnecting");
        //Simply transparent mode while not connected
        F=VM3::Zero();
        //Attempt to reconnect (wait for client incoming connection)
        chaiServer->Reconnect();
    }

    //Apply requested force on top of device gravity compensation
    if(robot->setEndEffForceWithCompensation(F)!=SUCCESS) {
         spdlog::error("M3ChaiCommunication: Error applying force");
    }

    if(iterations%100==0)
        std::cout << F.transpose() << " X=" << X.transpose() << std::endl;
}
void M3ChaiCommunication::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM3(0,0,0));
    chaiServer->Disconnect();
}
