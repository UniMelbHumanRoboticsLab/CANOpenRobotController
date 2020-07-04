/**
 * @file JointM3.cpp
 * @author Vincent Crocher
 * @brief
 * @version 0.1
 * @date 2020-06-16
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "JointM3.h"

#include <iostream>

#include "DebugMacro.h"

JointM3::JointM3(int jointID, double q_min, double q_max, short int sign_, double dq_min, double dq_max, double tau_min, double tau_max):ActuatedJoint(jointID, q_min, q_max, NULL),
                                                                                                                                        sign(sign_), qMin(q_min), qMax(q_max), dqMin(dq_min), dqMax(dq_max), tauMin(tau_min), tauMax(tau_max){
    drive = new KincoDrive(jointID+1);
    DEBUG_OUT("MY JOINT ID: " << this->id)
}

JointM3::~JointM3() {
    delete drive;
}


bool JointM3::updateValue() {
    q=driveUnitToJointPosition(drive->getPos())-q0;
    dq=driveUnitToJointVelocity(drive->getVel());
    tau=driveUnitToJointTorque(drive->getTorque());
    return true;
}

setMovementReturnCode_t JointM3::safetyCheck() {
    if(dq>dqMax || dq<dqMin) {
        return OUTSIDE_LIMITS;
    }
    if(tau>tauMax || tau<tauMin) {
        return OUTSIDE_LIMITS;
    }
    return SUCCESS;
}



setMovementReturnCode_t JointM3::setPosition(double qd) {
    if(calibrated) {
        if(qd>=qMin && qd<=qMax) {
            return ActuatedJoint::setPosition(qd);
        }
        else {
            return OUTSIDE_LIMITS;
        }
    }
    else {
        return NOT_CALIBRATED;
    }
}

setMovementReturnCode_t JointM3::setVelocity(double dqd) {
    //Position protection first only if calibrated
    if(calibrated)
    {
        if(q<=qMin && dqd<0) {
        dqd=0;
        }
        if(q>=qMax && dqd>0) {
            dqd=0;
        }
    }
    //Caped velocity
    if(dqd>=dqMin && dqd<=dqMax) {
        return ActuatedJoint::setVelocity(dqd);
    }
    else {
        return OUTSIDE_LIMITS;
    }
}

setMovementReturnCode_t JointM3::setTorque(double taud) {
    //Position protection first only if calibrated
    if(calibrated)
    {
        if(q<=qMin && taud<0) {
            taud=0;
        }
        if(q>=qMax && taud>0) {
            taud=0;
        }
    }
    //Caped torque
    if(taud>=tauMin && taud<=tauMax) {
        return ActuatedJoint::setTorque(taud);
    }
    else {
        return OUTSIDE_LIMITS;
    }
}


bool JointM3::initNetwork() {
    DEBUG_OUT("JointM3::initNetwork()")
    if (drive->Init()) {
        return true;
    } else {
        return false;
    }
}

