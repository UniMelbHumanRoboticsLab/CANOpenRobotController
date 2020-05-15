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

ActuatedJoint::ActuatedJoint(int jointID, double jointMin, double jointMax, Drive *drive) : Joint(jointID, jointMin, jointMax) {
    this->drive = drive;
}
//TODO: add in check
ControlMode ActuatedJoint::setMode(ControlMode driveMode_, motorProfile profile) {
    if (driveMode_ == POSITION_CONTROL) {
        if (drive->initPosControl(profile)) {
            driveMode = driveMode_;
            return POSITION_CONTROL;
        }
    }
    if (driveMode_ == VELOCITY_CONTROL) {
        if (drive->initVelControl(profile)) {
            driveMode = driveMode_;
            return VELOCITY_CONTROL;
        }
    }
    // \todo Implement Torque
    return ERROR;
}

setMovementReturnCode_t ActuatedJoint::setPosition(double desQ) {
    if (driveMode == POSITION_CONTROL) {
        drive->setPos(toDriveUnits(desQ));
        drive->posControlConfirmSP();
        return SUCCESS;
    } else {
        // Replace once complete
        return INCORRECT_MODE;
    }
}

setMovementReturnCode_t ActuatedJoint::setVelocity(double velocity) {
    if (driveMode == VELOCITY_CONTROL) {
        drive->setVel(toDriveUnits(velocity));
        return SUCCESS;
    } else {
        // Replace once complete
        return INCORRECT_MODE;
    }
}

setMovementReturnCode_t ActuatedJoint::setTorque(double torque) {
    if (driveMode == TORQUE_CONTROL) {
        /*\todo A conversion to the drive value for torque */
        drive->setTorque(torque)
    }
    return UNKNOWN_ERROR;
}

void ActuatedJoint::readyToSwitchOn() {
    drive->readyToSwitchOn();
}

bool ActuatedJoint::enable() {
    if (drive->getDriveState() == READY_TO_SWITCH_ON) {
        drive->enable();
        return true;
    }
    return false;
}