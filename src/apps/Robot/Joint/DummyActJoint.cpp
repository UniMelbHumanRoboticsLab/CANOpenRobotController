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

#include <iostream>

#include "DebugMacro.h"

DummyActJoint::DummyActJoint(int jointID, double jointMin, double jointMax, Drive *drive) : ActuatedJoint(jointID, jointMin, jointMax, drive) {
    DEBUG_OUT("MY JOINT ID: " << this->id)
    // Do nothing else
}

bool DummyActJoint::updateValue() {
    drive->getPos();
    q = lastQCommand;

    return true;
}

setMovementReturnCode_t DummyActJoint::setPosition(double desQ) {
    lastQCommand = desQ;
    return ActuatedJoint::setPosition(desQ);
}

bool DummyActJoint::initNetwork() {
    DEBUG_OUT("Joint::initNetwork()")
    if (drive->initPDOs()) {
        return true;
    } else {
        return false;
    }
}
double DummyActJoint::getQ() {
    return q;
}