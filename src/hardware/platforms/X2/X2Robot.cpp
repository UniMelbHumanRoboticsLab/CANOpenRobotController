#include "X2Robot.h"

#include "DebugMacro.h"

X2Robot::X2Robot() : Robot() {
}

X2Robot::~X2Robot() {
    freeMemory();
    DEBUG_OUT("X2Robot deleted")
}

bool X2Robot::initPositionControl() {
    DEBUG_OUT("Initialising Position Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((ActuatedJoint *)p)->setMode(POSITION_CONTROL, posControlMotorProfile) != POSITION_CONTROL) {
            // Something back happened if were are here
            DEBUG_OUT("Something bad happened")
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((ActuatedJoint *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((ActuatedJoint *)p)->enable();
    }
    return returnValue;
}

bool X2Robot::initVelocityControl() {
    DEBUG_OUT("Initialising Velocity Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((ActuatedJoint *)p)->setMode(VELOCITY_CONTROL) != VELOCITY_CONTROL) {
            // Something back happened if were are here
            DEBUG_OUT("Something bad happened")
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((ActuatedJoint *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((ActuatedJoint *)p)->enable();
    }
    return returnValue;
}

bool X2Robot::initTorqueControl() {
    DEBUG_OUT("Initialising Torque Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((ActuatedJoint *)p)->setMode(TORQUE_CONTROL) != TORQUE_CONTROL) {
            // Something back happened if were are here
            DEBUG_OUT("Something bad happened")
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((ActuatedJoint *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((ActuatedJoint *)p)->enable();
    }
    return returnValue;
}

setMovementReturnCode_t X2Robot::setPosition(std::vector<double> positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((ActuatedJoint *)p)->setPosition(positions[i]);
        if (setPosCode == INCORRECT_MODE) {
            std::cout << "Joint ID " << p->getId() << ": is not in Position Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

setMovementReturnCode_t X2Robot::setVelocity(std::vector<double> velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((ActuatedJoint *)p)->setVelocity(velocities[i]);
        if (setPosCode == INCORRECT_MODE) {
            std::cout << "Joint ID " << p->getId() << ": is not in Velocity Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

setMovementReturnCode_t X2Robot::setTorque(std::vector<double> torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((ActuatedJoint *)p)->setTorque(torques[i]);
        if (setPosCode == INCORRECT_MODE) {
            std::cout << "Joint ID " << p->getId() << ": is not in Torque Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

std::vector<double> X2Robot::getPosition() {
    int i = 0;
    std::vector<double> actualJointPositions(joints.size());
    for (auto p : joints) {
        actualJointPositions[i] = ((ActuatedJoint *)p)->getPosition();
        i++;
    }
    return actualJointPositions;
}

std::vector<double> X2Robot::getVelocity() {
    int i = 0;
    std::vector<double> actualJointVelocities(joints.size());
    for (auto p : joints) {
        actualJointVelocities[i] = ((ActuatedJoint *)p)->getVelocity();
        i++;
    }
    return actualJointVelocities;
}

std::vector<double> X2Robot::getTorque() {
    int i = 0;
    std::vector<double> actualJointTorques(joints.size());
    for (auto p : joints) {
        actualJointTorques[i] = ((ActuatedJoint *)p)->getTorque();
        i++;
    }
    return actualJointTorques;
}

bool X2Robot::initialiseJoints() {
    for (int id = 0; id < X2_NUM_JOINTS; id++) {
        motorDrives.push_back(new CopleyDrive(id + 1));
        // The X2 has 2 Hips and 2 Knees, by default configured as 2 hips, then 2 legs int jointID, double jointMin, double jointMax, JointDrivePairs jdp, Drive *drive
        if (id == X2_LEFT_HIP || id == X2_RIGHT_HIP) {
            joints.push_back(new X2Joint(id, X2JointLimits.hipMin, X2JointLimits.hipMax, hipJDP, motorDrives[id]));
        } else if (id == X2_LEFT_KNEE || id == X2_RIGHT_KNEE) {
            joints.push_back(new X2Joint(id, X2JointLimits.kneeMin, X2JointLimits.kneeMax, kneeJDP, motorDrives[id]));
        }
    }

    return true;
}

bool X2Robot::initialiseNetwork() {
    DEBUG_OUT("X2Robot::initialiseNetwork()");

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }

    return true;
}
bool X2Robot::initialiseInputs() {
    inputs.push_back(&keyboard);
    return true;
}
void X2Robot::freeMemory() {
    for (auto p : joints) {
        DEBUG_OUT("Delete Joint ID: " << p->getId())
        delete p;
    }
    for (auto p : motorDrives) {
        DEBUG_OUT("Delete Drive Node: " << p->getNodeID())
        delete p;
    }
    for (auto p : inputs) {
        DEBUG_OUT("Deleting Input")
        delete p;
    }
    keyboard.~Keyboard();
}
void X2Robot::updateRobot() {
    Robot::updateRobot();
}