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

JointM3::JointM3(int jointID, double q_min, double q_max, short int sign_, double dq_min, double dq_max, double tau_min, double tau_max):ActuatedJoint(jointID, qMin, qMax, NULL),
                                                                                                                                        sign(sign_), dqMin(dq_min), dqMax(dq_max), tauMin(tau_min), tauMax(tau_max), q0(0) {
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
    //Vincent
    //std::cout <<"Status: 0x" << std::hex <<  drive->updateDriveStatus() << std::endl;

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
        return ActuatedJoint::setVelocity(dqd);
    }
    else {
        return OUTSIDE_LIMITS;
    }
}

setMovementReturnCode_t JointM3::setTorque(double taud) {
    if(taud>=tauMin && taud<=tauMax) {
        return ActuatedJoint::setTorque(taud);
    }
    else {
        return OUTSIDE_LIMITS;
    }
}

void JointM3::setPositionOffset(double qcalib) {
    q0=driveUnitToJointPosition(drive->getPos())-qcalib;
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
