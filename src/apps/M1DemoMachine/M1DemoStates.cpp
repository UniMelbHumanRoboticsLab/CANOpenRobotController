#include "M1DemoStates.h"


double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

void IdleState::entry(void) {
    std::cout
            << "==================================" << std::endl
            << " WELCOME TO THE TEST STATE MACHINE" << std::endl
            << "==================================" << std::endl
            << std::endl
            << "========================" << std::endl
            << " PRESS X to start zeroing" << std::endl
            << " PRESS S to start demo state" << std::endl
            << " PRESS A to start position control" << std::endl
            << "========================" << std::endl;
}

void IdleState::during(void) {
//    if (chaiServer->IsConnected()) {
//        double *x = new double[3]{0, 1, 2}; //Chai representation frame is: X towards the operator when facing device, Y towards right hand side and Z up
//        chaiServer->Send(x);
//    }
}

void IdleState::exit(void) {
//    delete chaiServer;
//    robot->stop();
    std::cout << "Idle State Exited" << std::endl;
}

//******************************* Calibration **************************
void Calibration::entry(void) {
    std::cout << "Enter calibration ... " << std::endl;
    robot->applyCalibration();
    robot->initVelocityControl();
    robot->m1ForceSensor->calibrate();
    cal_velocity = -15;   // degree per second
}

void Calibration::during(void) {
    dq=robot->getJointVel();
    tau = robot->getJointTor_s();
    JointVec dq_t;
    dq_t(0) = cal_velocity;
    if(robot->setJointVel(dq_t) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
    if (dq(0) <= 5 & tau(0) >= 10){
        cal_velocity = 0;
//        std::cout << "Calibration done!" << std::endl;
    }
    else
    {
        robot->printJointStatus();
    }
}

void Calibration::exit(void) {
    robot->applyCalibration();
    robot->initPositionControl();
    q(0) = 16;
    if(robot->setJointPos(q) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
    std::cout << "Calibration state exited" << std::endl;
}

//******************************* Monitoring **************************
void Monitoring::entry(void) {
    std::cout << "Enter monitoring ... " << std::endl;
    robot->applyCalibration();
    //robot->initMonitoring();
    robot->m1ForceSensor->calibrate();
    robot->initPositionControl();
    q(0) = 90;
    robot->setJointPos(q);
}

void Monitoring::during(void) {
    // display the information at lower frequency
    if(iterations++%100==1) {
        robot->printJointStatus();
        JointVec tau = robot->getJointTor_s();
        std::cout << std::dec << iterations << ": " << std::setprecision(2) << tau(0) << std::endl;
    }
}

void Monitoring::exit(void) {
    robot->stop();
    std::cout << "Monitoring State Exited" << std::endl;
}

//******************************* Demo state **************************
void M1PositionTracking::entryCode(void) {
    std::cout << "Enter Position tracking!" << std::endl;
    mode = 1; // Set mode to 1 for position control test: move from 0 to 90 degree
    // set mode to 2 for velocity control test
    // set mode to 3 for torque control test
    // set mode to 4 for admittance control
    cycle = 0;
    counter = 0;
    robot->applyCalibration();
    initMode(mode);
}

void M1PositionTracking::duringCode(void) {
    if(iterations%100==1) {
        //std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
        robot->printJointStatus();
    }
    counter = counter + 1;

    if(robot->status != R_SUCCESS){
        status = false;
        std::cout << "Robot error !" << std::endl;
    }
    control(mode);
    if(cycle == 5)
    {
        mode = mode + 1;
        cycle = 0;
        counter = 0;
        initMode(mode);
    }
    if(mode > 4)
    {
        mode = 4;
    }
}

void M1PositionTracking::exitCode(void) {
    std::cout << "Exit Position tracking!" << std::endl;
    switch(mode) {
        case 1:
            robot->setJointPos(JointVec::Zero());
            break;
        case 2:
            robot->setJointVel(JointVec::Zero());
            break;
        case 3:
            robot->setJointTor(JointVec::Zero());
            break;
        case 4:
            robot->setJointVel(JointVec::Zero());
            break;
        default:
            std::cout << "Wrong mode !" << std::endl;
    }
    robot->stop();
}

void M1PositionTracking::initMode(int mode_t){
    mode = mode_t;
    switch(mode_t){
        case 1:
            robot->initPositionControl();
            freq = 0.5;
            counter = 1;
            magnitude = 20;
            break;
        case 2:
            robot->initVelocityControl();
//            robot->admittanceControl();
            freq = 0.2;
            magnitude = 20;   // degree per second
            break;
        case 3:
            robot->initTorqueControl();
            freq = 0.2;
            magnitude = 0.7;   // torque
            break;
        case 4:
            robot->initVelocityControl();
            robot->m1ForceSensor->calibrate();
            Ks = 0;
            B = 0.01;
            dt = 0.01;
            Mass = 0.01;
            gain = 1;
            break;
        default:
            std::cout << "Wrong mode !" << std::endl;
    }
}

void M1PositionTracking::control(int mode_t){
    switch(mode_t){
        case 1:
            positionControl();
            break;
        case 2:
            velocityControl();
            break;
        case 3:
            torqueControl();
            break;
        case 4:
            admittanceControl();
            break;
        default:
            std::cout << "Wrong mode !" << std::endl;
    }
}

void M1PositionTracking::positionControl(void){
    q=robot->getJointPos();
    std::cout << q(0) << " <-> ";
//    q(0) = magnitude*sin(2*M_PI*freq*iterations/100);
    if(dir) // positive direction
    {
        if(q(0)<90)
        {
            q(0) = 90;
        }
        else
        {
            dir = false;
        }
    }
    else // negative direction
    {
        if(q(0)>0)
        {
            q(0) = 0;
        }
        else
        {
            dir = true;
            cycle = cycle + 1;
        }
    }

    std::cout << q(0) << std::endl;
    // set joint position
    if(robot->setJointPos(q) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
}

void M1PositionTracking::velocityControl(void){
    dq=robot->getJointVel();
//    dq(0) = magnitude*sin(2*M_PI*freq*iterations/100);
//     velocity control, differential velocity and command velocity
    q=robot->getJointPos();

    if(dir) // positive direction
    {
        if(q(0) < 90)
        {
            dq(0) = magnitude;
        }
        else
        {
            dq(0) = 0;
            dir = false;
        }
    }
    else // negative direction
    {
        if(q(0) > 0)
        {
            dq(0) = -magnitude;
        }
        else
        {
            dq(0) = 0;
            dir = true;
            magnitude = magnitude + 10;
            cycle = cycle + 1;
        }
    }

    // give velocity command for 5 s and check the position changes
//    if ((iterations > 0) & (iterations <= 500)) {
//        dq(0) = magnitude;
//    }
//    else
//    {
//        dq(0) = 0;
//    }

    // set joint velocity
    if(robot->setJointVel(dq) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
}

void M1PositionTracking::torqueControl(void){
    tau=robot->getJointTor();
    std::cout << std::dec << counter << ": current tau " << tau(0);
//    tau(0) = magnitude*sin(2*M_PI*freq*counter/100);
    q=robot->getJointPos();
    std::cout << " - command tau " << tau(0) << " - theta " << q(0) << std::endl;

    if(dir) // positive direction
    {
        if(q(0) < 90)
        {
            tau(0) = tau(0) + 0.1;
        }
        else
        {
            tau(0) = 0;
            dir = false;
        }
    }
    else // negative direction
    {
        if(q(0) > 0)
        {
            tau(0) = tau(0) - 0.1;
        }
        else
        {
            tau(0) = 0;
            dir = true;
            cycle = cycle + 1;
        }
    }

    // set max tau
    if(tau(0) > 3)
    {
        tau(0) = 3;
    }
    else if (tau(0) < -3)
    {
        tau(0) = -3;
    }

//    tau(0) = magnitude;
//    if (iterations == 6000){
//        magnitude = 0;
//    }
    if(robot->setJointTor(tau) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
}

void M1PositionTracking::admittanceControl(void){
    tau = (2*tau+robot->getJointTor_s())/3;
    if (tau(0) > 100 || tau(0) < -100){
        std::cout << "Torque sensor:: reading out of limits " << std::endl;
    }
    else
    {
        B = 0.01;
        Mass = 1;
        q = robot->getJointPos();
        dq = robot->getJointVel();
        B = 0;
        net_tau = tau(0) - Ks*q(0) - B*dq(0);
        acc = net_tau/Mass;
        dq(0) = dq(0) + gain*acc*dt;

        if (iterations%5==0){
            std::cout << std::dec << iterations << ": " << std::setprecision(2) << tau(0) << " ~ " << q(0) << " ~ " << dq(0)<< std::endl;
        }
        if (q(0) > 45 or q(0) <-45){
            dq(0) = dq(0)*0.5;
        }
        if(robot->setJointVel(dq) != SUCCESS){
            std::cout << "Error: " << std::endl;
        }
    }
}

//******************************* Demo state **************************
//
void M1DemoState::entryCode(void) {
    std::cout << "Enter Demo tracking!" << std::endl;

    robot->applyCalibration();
    robot->initPositionControl();
    freq = 0.5;
    counter = 1;
//    qi(0) = 45;
//    robot->setJointPos(qi);
}

void M1DemoState::duringCode(void) {
    if(iterations%100==1) {
        robot->printJointStatus();
    }

    if(robot->status == R_SUCCESS && iterations%4==0) {
        // control frequency is 400 hz
        uint sample = iterations/4;
//        std::cout << qi(0) << std::endl;
        qi=robot->getJointPos();
        std::cout << qi(0) << " <-> ";
        qi(0) = 20*sin(2*M_PI*freq*sample/100);
        std::cout << qi(0) << std::endl;
        JointVec dq_t;
        dq_t(0) = 1;
        if(robot->setJointPos(qi) != SUCCESS){
            std::cout << "Error: " << std::endl;
        }
//        std::cout << "Velocity _1: " << dq_t(0) << std::endl;
//        robot->setJointVel(dq_t);
    }
}

void M1DemoState::exitCode(void) {
//    robot->setJointVel(JointVec::Zero());
//    robot->setJointTor(JointVec::Zero());
    robot->setJointPos(JointVec::Zero());
}