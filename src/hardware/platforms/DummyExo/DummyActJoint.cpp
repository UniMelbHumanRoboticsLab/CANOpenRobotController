/**
 * @file DummyActJoint.cpp
 * @author Justin Fong
 * @brief
 * @version 0.1
 * @date 2020-04-09
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "DummyActJoint.h"


DummyActJoint::DummyActJoint(int jointID, double jointMin, double jointMax, Drive *drive) : Joint(jointID, jointMin, jointMax, drive) {
   spdlog::debug("MY JOINT ID: {}", id);
    // Do nothing else
}

bool DummyActJoint::updateValue() {
    position = Joint::updatePosition();
    velocity = Joint::updateVelocity();
    torque = Joint::updateTorque();

    return true;
}

setMovementReturnCode_t DummyActJoint::setPosition(double desiredPosition) {
    lastPositionCommand = desiredPosition;
    return Joint::setPosition(desiredPosition);
}

setMovementReturnCode_t DummyActJoint::setVelocity(double desiredVelocity) {
    lastVelocityCommand = desiredVelocity;
    return Joint::setVelocity(desiredVelocity);
}

setMovementReturnCode_t DummyActJoint::setTorque(double desiredTorque) {
    lastTorqueCommand = desiredTorque;
    return Joint::setTorque(desiredTorque);
}

bool DummyActJoint::initNetwork() {
    spdlog::debug("Joint::initNetwork()");
    drive->start();
    if (drive->initPDOs()) {
        return true;
    } else {
        return false;
    }
}
double DummyActJoint::getPosition() {
    return position;
}
double DummyActJoint::getVelocity() {
    return velocity;
}
double DummyActJoint::getTorque() {
    return torque;
}
