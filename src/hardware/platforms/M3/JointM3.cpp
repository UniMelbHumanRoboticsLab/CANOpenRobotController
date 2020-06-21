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

JointM3::JointM3(int jointID, double q_min, double q_max, double dq_min, double dq_max, double tau_min, double tau_max):ActuatedJoint(jointID, qMin, qMax, NULL), dqMin(dq_min), dqMax(dq_max), tauMin(tau_min), tauMax(tau_max) {
    drive = new KincoDrive(jointID+1);

    DEBUG_OUT("MY JOINT ID: " << this->id)

    // Do nothing else
}

JointM3::~JointM3() {
    delete drive;
}


bool JointM3::updateValue() {
    std::cout<<drive->getPos()<<std::endl;
    q=jointPositionToDriveUnit(drive->getPos())-q0;
    dq=jointVelocityToDriveUnit(drive->getVel());
    tau=jointTorqueToDriveUnit(drive->getTorque());

    return true;
}


setMovementReturnCode_t JointM3::setPosition(double qd) {
    if(qd>=qMin && qd<=qMax) {
        return ActuatedJoint::setPosition(qd);
    }
    else {
        return OUTSIDE_LIMITS;
    }
}

setMovementReturnCode_t JointM3::setVelocity(double dqd) {
    if(dqd>=dqMin && dqd<=dqMax) {
        return ActuatedJoint::setPosition(dqd);
    }
    else {
        return OUTSIDE_LIMITS;
    }
}

setMovementReturnCode_t JointM3::setTorque(double taud) {
    if(taud>=tauMin && taud<=tauMax) {
        return ActuatedJoint::setPosition(taud);
    }
    else {
        return OUTSIDE_LIMITS;
    }
}

void JointM3::setCurrentOffset(double qcalib) {
    q0=jointPositionToDriveUnit(drive->getPos())-qcalib;
}


bool JointM3::initNetwork() {
    DEBUG_OUT("JointM3::initNetwork()")
    if (drive->Init()) {
        return true;
    } else {
        return false;
    }
}
double JointM3::getQ() {
    return q;
}
double JointM3::getDq() {
    return dq;
}
double JointM3::getTau() {
    return tau;
}
