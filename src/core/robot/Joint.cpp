/**
 * The <code>Joint</code> class is a abstract class which represents a joint in a
 * <code>Robot</code> objec. This class can be used to represent all types of joints,
 * including actuated, non-actuated, revolute, prismatic, etc.
 *
 *
 * Version 0.1
 * Date: 07/04/2020
 *
 */
#include "Joint.h"

#include <iostream>

#include "DebugMacro.h"

Joint::Joint(int jointID, double jointMin, double jointMax) : id(jointID), qMin(jointMin), qMax(jointMax), actuated(false) {
    position = 0;
    velocity = 0;
    torque = 0;
}

Joint::Joint(int jointID, double jointMin, double jointMax, double q0) : id(jointID), qMin(jointMin), qMax(jointMax), actuated(false) {
    position = q0;
    velocity = 0;
    torque = 0;
}

Joint::Joint(int jointID, double jointMin, double jointMax, Drive *jointDrive) : id(jointID), qMin(jointMin), qMax(jointMax), actuated(true) {
    position = 0;
    velocity = 0;
    torque = 0;
    this->drive = jointDrive;
    calibrated = false;
}

Joint::Joint(int jointID, double jointMin, double jointMax, double q0, Drive *jointDrive) : id(jointID), qMin(jointMin), qMax(jointMax), actuated(true) {
    position = q0;
    velocity = 0;
    torque = 0;
    this->drive = jointDrive;
    calibrated = false;
}

Joint::~Joint() {
    DEBUG_OUT(" Joint object deleted")
}
int Joint::getId() {
    return id;
}

double Joint::getPosition() {
    position = updatePosition();
    return position;
}
double Joint::getVelocity() {
    velocity = updateVelocity();
    return velocity;
}
double Joint::getTorque() {
    torque = updateTorque();
    return torque;
}

void Joint::getStatus() {
    std::cout << "Joint ID " << id << " @ pos " << getPosition() << " deg" << std::endl;
}

// Methods if joint is actuated
ControlMode Joint::setMode(ControlMode driveMode_, motorProfile profile) {
    if (actuated) {
        if (driveMode_ == CM_POSITION_CONTROL) {
            if (drive->initPosControl(profile)) {
                driveMode = driveMode_;
                return CM_POSITION_CONTROL;
            }
        } else if (driveMode_ == CM_VELOCITY_CONTROL) {
            if (drive->initVelControl(profile)) {
                driveMode = driveMode_;
                return CM_VELOCITY_CONTROL;
            }
        } else if (driveMode_ == CM_TORQUE_CONTROL) {
            if (drive->initTorqueControl()) {
                driveMode = driveMode_;
                return CM_TORQUE_CONTROL;
            }
        }
    }
    return CM_UNACTUATED_JOINT;
}

setMovementReturnCode_t Joint::setPosition(double desQ) {
    if (actuated) {
        if (driveMode == CM_POSITION_CONTROL) {
            drive->setPos(jointPositionToDriveUnit(desQ + q0));
            drive->posControlConfirmSP();
            return SUCCESS;
        } else {
            // Replace once complete
            return INCORRECT_MODE;
        }
    }
    return UNACTUATED_JOINT;
}

setMovementReturnCode_t Joint::setVelocity(double velocity) {
    if (actuated) {
        if (driveMode == CM_VELOCITY_CONTROL) {
            drive->setVel(jointVelocityToDriveUnit(velocity));
            return SUCCESS;
        } else {
            // Replace once complete
            return INCORRECT_MODE;
        }
    }
    return UNACTUATED_JOINT;
}

setMovementReturnCode_t Joint::setTorque(double torque) {
    if (actuated) {
        if (driveMode == CM_TORQUE_CONTROL) {
            drive->setTorque(jointTorqueToDriveUnit(torque));
            return SUCCESS;
        }
        return INCORRECT_MODE;
    }
    return UNACTUATED_JOINT;
}

// Updating functions to access joint-level commands
void Joint::setPositionOffset(double qcalib = 0) {
    q0 = driveUnitToJointPosition(drive->getPos()) - qcalib;
    calibrated = true;
}

double Joint::updatePosition() {
    return driveUnitToJointPosition(drive->getPos()) - q0;
}

double Joint::updateVelocity() {
    return driveUnitToJointVelocity(drive->getVel());
}

double Joint::updateTorque() {
    return driveUnitToJointTorque(drive->getTorque());
}

// Drive configuration methods
void Joint::readyToSwitchOn() {
    drive->readyToSwitchOn();
}

bool Joint::enable() {
    if (drive->getState() == READY_TO_SWITCH_ON) {
        drive->enable();
        return true;
    }
    return false;
}

bool Joint::disable() {
    drive->readyToSwitchOn();  //Ready to switch on is also power off state
}
