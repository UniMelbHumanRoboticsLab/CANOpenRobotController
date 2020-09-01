/**
 * @file JointM1.cpp
 * @author Yue Wen, Tim Haswell borrowing heavily from Vincent Crocher
 * @brief
 * @version 0.1
 * @date 2020-07-08
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "JointM1.h"

#include <iostream>

#include "DebugMacro.h"

JointM1::JointM1(int jointID, double q_min, double q_max, short int sign_, double dq_min, double dq_max, double tau_min, double tau_max): ActuatedJoint(jointID, q_min, q_max, nullptr),
                                                                                                                                          sign(sign_), qMin(q_min), qMax(q_max), dqMin(dq_min), dqMax(dq_max), tauMin(tau_min), tauMax(tau_max){
    drive = new KincoDrive(jointID+1);
    d2r = M_PIf64 / 180.;
    r2d = 180. / M_PIf64;

    // Define unchanging unit conversion properties
    encoderCounts = 10000;          //Encoder counts per turn
    reductionRatio = 1;         // Reduction ratio due to gear head, seems right, but not sure yet
    Ipeak = 45.0;                   //Kinco FD123 peak current
    motorTorqueConstant = 0.132;    //SMC60S-0020 motor torque constant
//    d2j_Pos = sign / (double)encoderCounts / reductionRatio;   // Drive to Joint unit conversion for position
    d2j_Pos = 5.14/(encoderCounts);         // count to degree - Drive to Joint unit conversion for position
//    j2d_Pos = sign * (double)encoderCounts * reductionRatio;   // Joint to Drive unit conversion for position
    j2d_Pos = encoderCounts/5.14;           // degree to count - Joint to Drive unit conversion for position
    d2j_Vel = sign / 60. / 512. / (double)encoderCounts * 1875 / reductionRatio;   // Drive to Joint unit conversion for velocity
//    d2j_Vel = sign / 512. / (double)encoderCounts * 1875;   // Drive to Joint unit conversion for velocity
    j2d_Vel = sign * 60. * 512. * (double)encoderCounts / 1875 * reductionRatio;   // Joint to Drive unit conversion for velocity
//    j2d_Vel = sign * 512. * (double)encoderCounts / 1875;   // Joint to Drive unit conversion for velocity

    d2j_Trq = sign / Ipeak / 1.414 * motorTorqueConstant * reductionRatio;   // Drive to Joint unit conversion for torque
    j2d_Trq = sign * Ipeak * 1.414 / motorTorqueConstant / reductionRatio;   // Joint to Drive unit conversion for torque

    DEBUG_OUT("MY JOINT ID: " << this->id)
}

JointM1::~JointM1() {
    delete drive;
}

bool JointM1::initNetwork() {
    DEBUG_OUT("JointM1::initNetwork()")
    return drive->Init();
}

/***************************************************************************************/
/****************** implementation of virtual function of ActuatedJoint class **********/
/***************************************************************************************/
// convert from driver unit to joint unit for reading command
double JointM1::driveUnitToJointPosition(int driveValue)  {
    return d2j_Pos * (double)driveValue;
}

double JointM1::driveUnitToJointVelocity(int driveValue) {
    return d2j_Vel * driveValue;
}

double JointM1::driveUnitToJointTorque(int driveValue) {
    return d2j_Trq * driveValue;
}

// covert from joint unit to driver unit for control command
int JointM1::jointPositionToDriveUnit(double jointValue) {
//    int count = int(round(j2d_Pos * jointValue));
//    DEBUG_OUT("jointPositionToDriveUnit " << count);
    return int(round(j2d_Pos * jointValue));
}

int JointM1::jointVelocityToDriveUnit(double jointValue) {
    return int(round(j2d_Vel * jointValue));
}

int JointM1::jointTorqueToDriveUnit(double jointValue) {
    return int(round(j2d_Trq * jointValue));
}

bool JointM1::updateValue() {
    position = driveUnitToJointPosition(drive->getPos()) - q0;
    velocity = driveUnitToJointVelocity(drive->getVel());
    torque = driveUnitToJointTorque(drive->getTorque());
    return true;
}

setMovementReturnCode_t JointM1::safetyCheck() {
    if(velocity > dqMax  ||  velocity < dqMin) {
        DEBUG_OUT(" Velocity out of bound: " << velocity);
        return OUTSIDE_LIMITS;
    }
    if(torque > tauMax  ||  torque < tauMin) {
        DEBUG_OUT(" Torque out of bound: " << torque);
        return OUTSIDE_LIMITS;
    }
    return SUCCESS;
}

/***************************************************************************************/
/****************** Check command safety range and send command to driver **************/
/***************************************************************************************/
// including position command, velocity command, torque command
setMovementReturnCode_t JointM1::setPosition(double qd) {
//    DEBUG_OUT("JointM1::setPosition: " << qd)
    if(calibrated) {
        if(qd >= qMin  &&  qd <= qMax) {
            return ActuatedJoint::setPosition(qd);
//            drive->setPos(jointPositionToDriveUnit(qd+q0));
//            return SUCCESS;
        }
        else {
            DEBUG_OUT(" Position out of bound: " << qd);
            return OUTSIDE_LIMITS;
        }
    }
    else {
        return NOT_CALIBRATED;
    }
}

setMovementReturnCode_t JointM1::setVelocity(double dqd) {
    //Position protection first only if calibrated
    if(calibrated) {
        if(position <= qMin  &&  dqd < 0) {
            dqd = 0;
        }
        if(position >= qMax  &&  dqd > 0) {
            dqd = 0;
        }
    }
    //Capped velocity
    if(dqd>=dqMin && dqd<=dqMax) {
        return ActuatedJoint::setVelocity(dqd);
//        drive->setVel(jointVelocityToDriveUnit(dqd));
        return SUCCESS;
    }
    else {
        return OUTSIDE_LIMITS;
    }
}

setMovementReturnCode_t JointM1::setTorque(double taud) {
    //Position protection first only if calibrated
    if(calibrated) {
        if(position <= qMin  &&  taud < 0) {
            taud = 0;
        }
        if(position >= qMax  &&  taud > 0) {
            taud = 0;
        }
    }
    //Capped torque
    if(taud >= tauMin  &&  taud <= tauMax) {
        return ActuatedJoint::setTorque(taud);
    }
    else {
        return OUTSIDE_LIMITS;
    }
}

void JointM1::errorMessage(setMovementReturnCode_t errorCode){
    switch(errorCode) {
        case SUCCESS:
            DEBUG_OUT(" ActuatedJoint::Success");
            break; //optional
        case OUTSIDE_LIMITS:
            DEBUG_OUT(" ActuatedJoint::Outside of limitations");
            break; //optional
        case INCORRECT_MODE:
            DEBUG_OUT(" ActuatedJoint::Incorrect mode");
            break; //optional
        case NOT_CALIBRATED:
            DEBUG_OUT(" ActuatedJoint::Not calibrated");
            break; //optional
        default : //Optional
            DEBUG_OUT(" ActuatedJoint::Unknown error");
    }
}

