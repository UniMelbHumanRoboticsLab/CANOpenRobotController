#include "M1DemoStates.h"

using namespace std;

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
            << " PRESS A to start demon machines" << std::endl
            << " PRESS Q to return idle state" << std::endl
            << "========================" << std::endl;
}

void IdleState::during(void) {
}

void IdleState::exit(void) {
    std::cout << "Idle State Exited" << std::endl;
}

//******************************* Calibration **************************
void Calibration::entry(void) {
    spdlog::info("Starting Calibration: Position and Force Zeroing");
    robot->applyCalibration();
    robot->initVelocityControl();
    robot->calibrateForceSensors();
    cal_velocity = -20;   // degree per second
    stages = 1;
}

void Calibration::during(void) {
    // Two stage calibration
    if(stages == 1){
        // Stage 1: Position Homing (zeroing)
        dq=robot->getJointVel();
        tau = robot->getJointTor_s();
        JointVec dq_t;
        dq_t(0) = cal_velocity;

        setMovementReturnCode_t result = robot->setJointVel(dq_t);
        if (result != SUCCESS) {
            spdlog::error(setMovementReturnCodeString[result]);
        }

        if ((dq(0) <= 2) & (tau(0) >= 1.5)){
            cal_velocity = 0;
            robot->applyCalibration();
            robot->initPositionControl();
            stages = 2;
            spdlog::info("Position Homing Complete. Starting Force Sensor Calibration");
        }
        else {
            robot->printJointStatus();
        }
    }
    else {
        // Stage 2: Force Sensor Zeroing (calibration)
        q(0) = 16; // 16 is vertical for #2
        setMovementReturnCode_t result = robot->setJointPos(q);
        if (result != SUCCESS) {
            spdlog::error(setMovementReturnCodeString[result]);
        }
        JointVec tau = robot->getJointTor_s();
        if (tau(0)>0.2 || tau(0)<-0.2) {
            if(robot->m1ForceSensor->calibrate()){
                spdlog::info("Force Sensor Calibration Complete, Press Q to exit calibration");
            }
        }
        robot->printJointStatus();
    }
}

void Calibration::exit(void) {
    std::cout << "Calibration state exited" << std::endl;
}

//******************************* Monitoring **************************
void Monitoring::entry(void) {
    std::cout << "Enter torque monitoring ... " << std::endl;
    robot->applyCalibration();
    robot->initMonitoring();
}

void Monitoring::during(void) {
    // display the information at lower frequency
    JointVec tau;
    if(iterations++%100==1) {
        tau = robot->getJointTor_s();
        robot->printJointStatus();
        std::cout << std::dec << iterations << ": " << std::setprecision(2) << tau(0) << std::endl;
    }
}

void Monitoring::exit(void) {
//    robot->stop();
    std::cout << "Monitoring State Exited" << std::endl;
}

//******************************* Demo state **************************
void M1DemoState::entryCode(void) {
    std::cout << "Enter demos!" << std::endl;
    cfreq = 800; // set control loop frequency
    mode = 1; // Set mode to 1 for position control test: move from 0 to 90 degree
    // set mode to 2 for velocity control test
    // set mode to 3 for torque control test
    // set mode to 4 for admittance control
    sub_mode = 1;
    // sub_mode 1 for sine wave tracking
    // sub_mode 2 for ramp tracking
    // sub_mode 3 for torque control only
    robot->applyCalibration();
    sleep(1);
    cycle = 0;
    counter = 0;
    sflag = 0;
    initMode(mode);
}

void M1DemoState::duringCode(void) {
    if(iterations()%500==1) {
        robot->printJointStatus();
    }
    counter = counter + 1;

    if(robot->status != R_SUCCESS){
        status = false;
        std::cout << "Robot error !" << std::endl;
    }
    control(mode);

    // switch through four mode
    if(cycle >= 4)
    {
        exitCode();
        std::cout << "End demo!" << std::endl;
        mode = mode + 1;
        cycle = 0;
        counter = 0;
        sflag = 0;
        initMode(mode);
    }
    if(mode > 4)
    {
        mode = 4;
    }
}

void M1DemoState::exitCode(void) {
    switch(mode) {
        case 1:
            robot->setJointPos(JointVec::Zero());
            std::cout << "Exit position tracking!" << std::endl;
            break;
        case 2:
            robot->setJointVel(JointVec::Zero());
            std::cout << "Exit velocity tracking!" << std::endl;
            break;
        case 3:
            robot->setJointTor(JointVec::Zero());
            std::cout << "Exit torque tracking!" << std::endl;
            break;
        case 4:
            robot->setJointVel(JointVec::Zero());
            std::cout << "Exit admittance control!" << std::endl;
            break;
        default:
            std::cout << "Wrong mode !" << std::endl;
    }
    robot->stop();
}

void M1DemoState::initMode(int mode_t){
    mode = mode_t;
    switch(mode_t){
        case 1:
            std::cout << "Demo position tracking!" << std::endl;
            robot->initPositionControl();
            freq = 0.5;
            counter = 1;
            magnitude = 20;
            break;
        case 2:
            std::cout << "Demo velocity tracking!" << std::endl;
            robot->initVelocityControl();
            freq = 0.5;
            magnitude = 20;   // degree per second
            break;
        case 3:
            std::cout << "Demo torque tracking!" << std::endl;
            robot->initTorqueControl();
            freq = 0.5;
            magnitude = 1.0;   // torque magnitude
            step = 0.1;
            break;
        case 4:
            std::cout << "Demo admittance control!" << std::endl;
            robot->initVelocityControl();
            robot->m1ForceSensor->calibrate();
            Ks = 0;
            B = 0.01;
            dt = 0.01;
            Mass = 0.01;
            gain = 10;
            break;
        default:
            std::cout << "Wrong mode !" << std::endl;
    }
}

void M1DemoState::control(int mode_t){
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

void M1DemoState::positionControl(void){
    // position control;
    q=robot->getJointPos();
//    std::cout << q(0) << " <-> ";
    switch(sub_mode){
        case 1:     // sine wave position command
            sflag = sin(2*M_PI*freq*iterations()/100);
            q(0) = magnitude*sflag+magnitude;
            if(abs(sflag)<10e-8 && dir==true)
            {
                dir = false;
            }
            else if(abs(sflag)<10e-8 && dir==false)
            {
                dir = true;
                cycle = cycle + 1;
                std::cout << std::setprecision(2)  << "position magnitude: " << magnitude << "; frequency: " << freq << "; Cycle: "<< cycle << std::endl;
            }
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
//    std::cout << q(0) << std::endl;
    if(robot->setJointPos(q) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
}

void M1DemoState::velocityControl(void){
    dq=robot->getJointVel();
    q=robot->getJointPos();

    switch(sub_mode){
        case 1:     // sine wave velocity command
            sflag = sin(2*M_PI*freq*iterations()/100);
            dq(0) = magnitude*sflag;
            if(abs(sflag)<10e-8 && dir==true)
            {
                dir = false;
            }
            else if(abs(sflag)<10e-8 && dir==false)
            {
                dir = true;
                cycle = cycle + 1;
                std::cout << std::setprecision(2)  << "velocity magnitude: " << magnitude << "; frequency: " << freq << "; Cycle: "<< cycle << std::endl;
            }
            break;
        case 2:
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
            break;
        default:
            std::cout << "Wrong sub mode !" << std::endl;
    }

    // display and set joint velocity
//    std::cout << dq(0) << std::endl;
    if(robot->setJointVel(dq) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
}

void M1DemoState::torqueControl(void){
    double spring_tor;
    double error;
    double delta_error;
    double ff_ratio = 0.7;   // default feedforward ratio
    tau = robot->getJointTor();
    tau_s = (robot->getJointTor_s()+tau_s)/2;
    q = robot->getJointPos();
    dq = robot->getJointVel();
    switch(sub_mode) {
        case 1:
            sflag = sin(2*M_PI*freq*iterations()/100);
            tau_cmd(0) = magnitude*sflag + 0.2;
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
            std::cout << "; spring_tor: " << spring_tor <<  ":sensor tau " << tau_s(0) <<  ":command tau " << tau_cmd(0) <<  "; theta error: " << (30-q(0)) << std::endl;
            break;
        default:
            std::cout << "Wrong sub mode !" << std::endl;
    }

    if(robot->setJointTor_comp(tau_cmd, tau_cmd, ff_ratio) != SUCCESS){
        std::cout << "Error: " << tau_cmd << std::endl;
    }
}

void M1DemoState::admittanceControl(void){
    tau = (2*tau+robot->getJointTor_s())/3;
    if (tau(0) > 100 || tau(0) < -100){
        std::cout << "Torque sensor:: reading out of limits " << std::endl;
    }
    else
    {
        B = 0.01;
        Mass = 0.3;
        q = robot->getJointPos();
        dq = robot->getJointVel();
        B = 0;
        net_tau = tau(0) - Ks*q(0) - B*dq(0);
        acc = net_tau/Mass;
        dq(0) = dq(0) + gain*acc*dt;

        if (iterations()%100==0){
            std::cout << std::dec << iterations() << "tau sensor : " << std::setprecision(2) << tau(0) << "; q: " << q(0) << "; dq: " << dq(0)<< std::endl;
        }

        if (q(0) > 50 or q(0) <-45){
            dq(0) = dq(0)*0.5;
        }
        if(robot->setJointVel(dq) != SUCCESS){
            std::cout << "Error: " << std::endl;
        }
    }
}
