/**
 * @file X2Joint.cpp
 * @author Justin Fong
 * @brief 
 * @version 0.1
 * @date 2020-04-09
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "X2Joint.h"

#include <iostream>

#include "DebugMacro.h"

X2Joint::X2Joint(int jointID, double jointMin, double jointMax, JointDrivePairs jdp, Drive *drive) : ActuatedJoint(jointID, jointMin, jointMax, drive) {
    DEBUG_OUT("Hip Joint Created, JOINT ID: " << this->id)
    // Do nothing else
    JDSlope = (jdp.drivePosB - jdp.drivePosA) / (jdp.jointPosB - jdp.jointPosA);
    JDIntercept = jdp.drivePosA - JDSlope * jdp.jointPosA;
}

int X2Joint::jointPositionToDriveUnit(double jointPosition) {
    return JDSlope * jointPosition + JDIntercept;
}

double X2Joint::driveUnitToJointPosition(int driveValue) {
    return (driveValue - JDIntercept) / JDSlope;
}

int X2Joint::jointVelocityToDriveUnit(double jointVelocity) {
    return (JDSlope * jointVelocity) * 10;
}

double X2Joint::driveUnitToJointVelocity(int driveValue) {
    return ((driveValue) / (JDSlope * 10));
}

int X2Joint::jointTorqueToDriveUnit(double jointTorque) {
    return jointTorque / (MOTOR_RATED_TORQUE * REDUCTION_RATIO * 1000);
}

double X2Joint::driveUnitToJointTorque(int driveValue) {
    return MOTOR_RATED_TORQUE * REDUCTION_RATIO * 1000 * driveValue;
}

bool X2Joint::updateValue() {
    position = ActuatedJoint::getPosition();
    velocity = ActuatedJoint::getVelocity();
    torque = ActuatedJoint::getTorque();

    return true;
}

setMovementReturnCode_t X2Joint::setPosition(double desiredPosition) {
    return ActuatedJoint::setPosition(desiredPosition);
}

setMovementReturnCode_t X2Joint::setVelocity(double desiredVelocity) {
    return ActuatedJoint::setVelocity(desiredVelocity);
}

setMovementReturnCode_t X2Joint::setTorque(double desiredTorque) {
    return ActuatedJoint::setTorque(desiredTorque);
}

bool X2Joint::initNetwork() {
    DEBUG_OUT("Joint::initNetwork()")
    drive->start();
    if (drive->initPDOs()) {
        return true;
    } else {
        return false;
    }
}
double X2Joint::getPosition() {
    return position;
}
double X2Joint::getVelocity() {
    return velocity;
}
double X2Joint::getTorque() {
    return torque;
}
