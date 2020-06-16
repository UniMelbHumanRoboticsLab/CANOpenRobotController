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

JointM3::JointM3(int jointID, double jointMin, double jointMax):ActuatedJoint(jointID, jointMin, jointMax, NULL) {
    drive = new KincoDrive(jointID);
    
    DEBUG_OUT("MY JOINT ID: " << this->id)
    
    // Do nothing else
}

JointM3::~JointM3() {
    delete drive;
}
    
bool JointM3::updateValue() {
    drive->getPos();
    q = lastQCommand;

    return true;
}

setMovementReturnCode_t JointM3::setPosition(double desQ) {
    lastQCommand = desQ;
    return ActuatedJoint::setPosition(desQ);
}

bool JointM3::initNetwork() {
    DEBUG_OUT("Joint::initNetwork()")
    if (drive->initPDOs()) {
        return true;
    } else {
        return false;
    }
}
double JointM3::getQ() {
    return q;
}