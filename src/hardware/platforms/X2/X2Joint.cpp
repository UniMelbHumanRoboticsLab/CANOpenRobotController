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


X2Joint::X2Joint(int jointID, double jointMin, double jointMax, JointDrivePairs jdp, Drive *drive) : Joint(jointID, jointMin, jointMax, drive) {
    spdlog::debug("Joint Created, JOINT ID: {}", this->id);
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
    return jointTorque / (MOTOR_RATED_TORQUE * REDUCTION_RATIO / 1000.0);
}

double X2Joint::driveUnitToJointTorque(int driveValue) {
    return driveValue * (MOTOR_RATED_TORQUE * REDUCTION_RATIO / 1000.0);
}

bool X2Joint::initNetwork() {
    spdlog::debug("Joint::initNetwork()");
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

void X2Joint::setPositionOffset(double offset) {
    ((CopleyDrive *)drive)->setPositionOffset(jointPositionToDriveUnit(-offset));
}
