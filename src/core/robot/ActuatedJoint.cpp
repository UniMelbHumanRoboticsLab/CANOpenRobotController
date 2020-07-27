/**
 * The <code>ActuatedJoint</code> class is a abstract class which represents a joint in a
 * <code>Robot</code> objec. This class implements the Joint class, and specifically
 * represents a joint which is actuated. This therefore requires a Drive object
 * which will be used to interact with the physical hardware.
 *
 *
 * Version 0.1
 * Date: 09/04/2020
 */

#include "ActuatedJoint.h"

#include "DebugMacro.h"

ActuatedJoint::ActuatedJoint(int jointID, double jointMin, double jointMax, Drive *drive) : Joint(jointID, jointMin, jointMax), calibrated(false) {
    this->drive = drive;
}

bool ActuatedJoint::start() {
    drive->start();
}

//TODO: add in check
ControlMode ActuatedJoint::setMode(ControlMode driveMode_, motorProfile profile) {
    if (driveMode_ == POSITION_CONTROL) {
        if (drive->initPosControl(profile)) {
            driveMode = driveMode_;
            return POSITION_CONTROL;
        }
    }
    else if (driveMode_ == VELOCITY_CONTROL) {
        if (drive->initVelControl(profile)) {
            driveMode = driveMode_;
            return VELOCITY_CONTROL;
        }
    }

    else if (driveMode_ == TORQUE_CONTROL) {
        if (drive->initTorqueControl()) {
            driveMode = driveMode_;
            return TORQUE_CONTROL;
        }
    }
    return ERROR;
}

ControlMode ActuatedJoint::setMode(ControlMode driveMode_) {
    if (driveMode_ == POSITION_CONTROL) {
        if (drive->initPosControl()) {
            driveMode = driveMode_;
            return POSITION_CONTROL;
        }
    }
    else if (driveMode_ == VELOCITY_CONTROL) {
        if (drive->initVelControl()) {
            driveMode = driveMode_;
            return VELOCITY_CONTROL;
        }
    }

    else if (driveMode_ == TORQUE_CONTROL) {
        if (drive->initTorqueControl()) {
            driveMode = driveMode_;
            return TORQUE_CONTROL;
        }
    }
    return ERROR;
}

setMovementReturnCode_t ActuatedJoint::setPosition(double desQ) {
    if (driveMode == POSITION_CONTROL) {
        drive->setPos(jointPositionToDriveUnit(desQ+q0));
        drive->posControlConfirmSP();
        return SUCCESS;
    } else {
        // Replace once complete
        return INCORRECT_MODE;
    }
}

setMovementReturnCode_t ActuatedJoint::setVelocity(double velocity) {
    if (driveMode == VELOCITY_CONTROL) {
        drive->setVel(jointVelocityToDriveUnit(velocity));
        return SUCCESS;
    } else {
        // Replace once complete
        return INCORRECT_MODE;
    }
}

setMovementReturnCode_t ActuatedJoint::setTorque(double torque) {
    if (driveMode == TORQUE_CONTROL) {
        drive->setTorque(jointTorqueToDriveUnit(torque));
        return SUCCESS;
    }
    return INCORRECT_MODE;
}

void ActuatedJoint::setPositionOffset(double qcalib=0) {
    q0=driveUnitToJointPosition(drive->getPos())-qcalib;
    calibrated=true;
}

double ActuatedJoint::getPosition() {
    return driveUnitToJointPosition(drive->getPos())-q0;
}

double ActuatedJoint::getVelocity() {
    return driveUnitToJointVelocity(drive->getVel());
}

double ActuatedJoint::getTorque() {
    return driveUnitToJointTorque(drive->getTorque());
}


void ActuatedJoint::readyToSwitchOn() {
    drive->readyToSwitchOn();
}

bool ActuatedJoint::enable() {
    if (drive->getState() == READY_TO_SWITCH_ON) {
        drive->enable();
        return true;
    }
    return false;
}

bool ActuatedJoint::disable() {
    drive->readyToSwitchOn(); //Ready to switch on is also power off state
}
