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
            << " PRESS S to start zeroing/calibration" << std::endl
            << " PRESS X to start monitoring state" << std::endl
            << " PRESS A to start torque control" << std::endl
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
    cal_velocity = -20;   // degree per second
    stages = 1;
}

void Calibration::during(void) {
    if(stages == 1){
        dq=robot->getJointVel();
        tau = robot->getJointTor_s();
        JointVec dq_t;
        dq_t(0) = cal_velocity;
        if(robot->setJointVel(dq_t) != SUCCESS){
            std::cout << "Error: " << std::endl;
        }
        if ((dq(0) <= 2) & (tau(0) >= 1.5)){
            cal_velocity = 0;
            robot->applyCalibration();
            robot->initPositionControl();
            stages = 2;
//        std::cout << "Calibration done!" << std::endl;
        }
        else {
//        std::cout << "Calibration velocity :" << cal_velocity<< std::endl;
            robot->printJointStatus();
        }
    }
    else
    {
        q(0) = 16; // 16 is vertical for #2
        if(robot->setJointPos(q) != SUCCESS){
            std::cout << "Error: " << std::endl;
        }
        JointVec tau = robot->getJointTor_s();
        if (tau(0)>0.2 || tau(0)<-0.2)
        {
            robot->m1ForceSensor->calibrate();
        }
        robot->printJointStatus();
    }
}

void Calibration::exit(void) {
    std::cout << "Calibration state exited" << std::endl;
}

//******************************* Monitoring **************************
void Monitoring::entry(void) {
    std::cout << "Enter monitoring ... " << std::endl;
    robot->applyCalibration();
    robot->initMonitoring();
//    robot->m1ForceSensor->calibrate();
//    robot->initPositionControl();
//    q(0) = 90;
//    robot->setJointPos(q);
//    sleep(1);
//    robot->initTorqueControl();
//    robot->initMonitoring();
}

void Monitoring::during(void) {
    // display the information at lower frequency
    JointVec tau;
    if(iterations++%100==1) {
        tau = robot->getJointTor_s();
        robot->printJointStatus();
        std::cout << std::dec << iterations << ": " << std::setprecision(2) << tau(0) << std::endl;
    }

//    tau = robot->getJointTor_s();
//    tau(0) = tau(0)*10;
////        robot->setJointTor(tau);
////    robot->setJointTor_comp(tau);
//    if(robot->setJointTor_comp(tau) != SUCCESS){
//        std::cout << "Error: " << std::endl;
//    }
}

void Monitoring::exit(void) {
//    robot->stop();
    std::cout << "Monitoring State Exited" << std::endl;
}

//******************************* Demo state **************************
void M1PositionTracking::entryCode(void) {
    std::cout << "Enter Position tracking!" << std::endl;
    cfreq = 800; // set control loop frequency
    mode = 3; // Set mode to 1 for position control test: move from 0 to 90 degree
    // set mode to 2 for velocity control test
    // set mode to 3 for torque control test
    // set mode to 4 for admittance control
    sub_mode = 3;
    // sub_mode 1 for sine wave tracking
    // sub_mode 2 for ramp tracking
    // sub_mode 3 for torque control only
    robot->applyCalibration();
//    robot->initPositionControl();
//    q(0) = 50;
//    robot->setJointPos(q);
//    sleep(1);
    cycle = 0;
    counter = 0;
    sflag = 0;
//    robot->applyCalibration();
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
    if(cycle >= 4)
    {
        //mode = mode + 1;
        cycle = 0;
        counter = 0;
        step = step + 0.05;             // slope torque
        magnitude = magnitude + 0.2;    //sine wave torque
        freq = freq + 0.1;              //sine wave torque
        //initMode(mode);
//        if(magnitude > 5)
//        {
//            mode = 4;
//        }
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
//    robot->stop();
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
            freq = 0.1;
            magnitude = 3;   // magnitude for sine wave without compensation
//            magnitude = 0.6;   // magnitude for sine wave
            step = 0.1;
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
    // position control;
    q=robot->getJointPos();
    std::cout << q(0) << " <-> ";
    switch(sub_mode){
        case 1:     // sine wave position command
            q(0) = magnitude*sin(2*M_PI*freq*iterations/100);
            break;
        case 2:
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
            break;
        default:
            std::cout << "Wrong sub mode !" << std::endl;
    }

    // display and set joint position
    std::cout << q(0) << std::endl;
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
    double spring_tor;
    double error;
    double delta_error;
    tau = robot->getJointTor();
    tau_s = (robot->getJointTor_s()+tau_s)/2;
    q = robot->getJointPos();
    dq = robot->getJointVel();
    switch(sub_mode) {
        case 1:
            sflag = sin(2*M_PI*freq*counter/cfreq);
            tau_cmd(0) = magnitude*sflag+0.8;
            if(abs(sflag)<10e-8 && dir==true)
            {
                dir = false;
            }
            else if(abs(sflag)<10e-8 && dir==false)
            {
                dir = true;
                cycle = cycle + 1;
                std::cout << std::setprecision(2)  << "torque magnitude: " << magnitude << "; frequency: " << freq << "; Cycle: "<< cycle << std::endl;
            }
//            if( abs(tau_cmd(0) - 0.8) <= sin(2*M_PI*freq/cfreq))
////            if((abs(sflag) < 10e-8) && (sin(2*M_PI*freq*(counter+1)/cfreq)>0))
//            {
//                std::cout << std::setprecision(2)  << "torque magnitude: " << magnitude << "; frequency: " << freq << "; Cycle: "<< cycle << std::endl;
//                cycle = cycle + 1;
//            }
            break;
        case 2:
            if(dir) // positive direction
            {
                if(q(0) < 80)
                {
                    tau_cmd(0) = tau_cmd(0) + step;
                }
                else
                {
//                    tau_cmd(0) = tau_cmd(0) - step;
                    dir = false;
                }
            }
            else // negative direction
            {
                if(q(0) > 10)
                {
                    tau_cmd(0) = tau_cmd(0) - step;
                }
                else
                {
//                    tau_cmd(0) = 0;
                    dir = true;
                    cycle = cycle + 1;
                }
            }
            // set max tau
            if(tau_cmd(0) > 4)
            {
                tau_cmd(0) = 4;
            }
            else if (tau_cmd(0) < -4)
            {
                tau_cmd(0) = -4;
            }
            break;
        case 3: // feedback torque compensation
            error = tau_s(0);  // interaction torque error, desired interaction torque is 0
            delta_error = (error-torque_error_last_time_step)*cfreq;  // derivative of interaction torque error;
            tau_cmd(0) = error*1 + delta_error*0.001;  // tau_cmd = P*error + D*delta_error; 1 and 0.001
            torque_error_last_time_step = error;
            break;
        case 4:
            spring_tor = 7*3.14*(30-q(0))/180;  //stiffness
//            if(abs(tau_s(0))>=0.3)
//                error = spring_tor+tau_s(0);
//            else
//                error = spring_tor;
            error = spring_tor+tau_s(0);
            delta_error = (error-torque_error_last_time_step)*cfreq;  // derivative of interaction torque error;
            tau_cmd(0) = error*1 + delta_error*0.001;  // tau_cmd = P*error + D*delta_error
            torque_error_last_time_step = error;
//            tau_cmd(0) = 3*error;
            std::cout << "; spring_tor: " << spring_tor <<  ":sensor tau " << tau_s(0) <<  ":command tau " << tau_cmd(0) <<  "; theta error: " << (30-q(0)) << std::endl;
            break;
        default:
            std::cout << "Wrong sub mode !" << std::endl;
    }

//    std::cout << std::setprecision(2) << ";     cmd tau :"<< tau(0) << ";      q: " << q(0) << ";     dq: " << dq(0)<< std::endl;
//    std::cout << " - command tau " << tau(0) << " - theta " << q(0) << std::endl;
//    std::cout << std::setprecision(2)  << "command tau " << tau(0) << std::endl;
//    tau(0) = magnitude;
//    if (iterations == 6000){
//        magnitude = 0;
//    }
//    tau_cmd(0) = 0;
    if(robot->setJointTor_comp(tau_cmd, tau_cmd) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
//    if(robot->setJointTor(tau_cmd) != SUCCESS){
//        std::cout << "Error: " << std::endl;
//    }
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