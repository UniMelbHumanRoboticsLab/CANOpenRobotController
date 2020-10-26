#include "ExoRobot.h"


ExoRobot::ExoRobot() : Robot() {
}

ExoRobot::~ExoRobot() {
    spdlog::debug("Delete ExoRobot object begins");
    freeMemory();

    //    joints.clear();
    //    copleyDrives.clear();
    spdlog::debug("ExoRobot deleted");
}

bool ExoRobot::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        p->enable();
    }
    return returnValue;
}

bool ExoRobot::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_VELOCITY_CONTROL) != CM_VELOCITY_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        p->enable();
    }
    return returnValue;
}

bool ExoRobot::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        p->enable();
    }
    return returnValue;
}

setMovementReturnCode_t ExoRobot::setPosition(std::vector<double> positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((DummyActJoint *)p)->setPosition(positions[i]);
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

setMovementReturnCode_t ExoRobot::setVelocity(std::vector<double> velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((DummyActJoint *)p)->setVelocity(velocities[i]);
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

setMovementReturnCode_t ExoRobot::setTorque(std::vector<double> torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((DummyActJoint *)p)->setTorque(torques[i]);
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

std::vector<double> ExoRobot::getPosition() {
    int i = 0;
    std::vector<double> actualJointPositions(joints.size());
    for (auto p : joints) {
        actualJointPositions[i] = ((DummyActJoint *)p)->getPosition();
        i++;
    }
    return actualJointPositions;
}

std::vector<double> ExoRobot::getVelocity() {
    int i = 0;
    std::vector<double> actualJointVelocities(joints.size());
    for (auto p : joints) {
        actualJointVelocities[i] = ((DummyActJoint *)p)->getVelocity();
        i++;
    }
    return actualJointVelocities;
}

std::vector<double> ExoRobot::getTorque() {
    int i = 0;
    std::vector<double> actualJointTorques(joints.size());
    for (auto p : joints) {
        actualJointTorques[i] = ((DummyActJoint *)p)->getTorque();
        i++;
    }
    return actualJointTorques;
}

bool ExoRobot::initialiseJoints() {
    for (int id = 0; id < NUM_JOINTS; id++) {
        copleyDrives.push_back(new CopleyDrive(id + 1));
        joints.push_back(new DummyActJoint(id, jointMinMap[id], jointMaxMap[id], copleyDrives[id]));
    }
    return true;
}

bool ExoRobot::initialiseNetwork() {
    spdlog::debug("ExoRobot::initialiseNetwork()");

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }

    return true;
}
bool ExoRobot::initialiseInputs() {
    inputs.push_back(keyboard = new Keyboard());
    return true;
}

void ExoRobot::freeMemory() {
    for (auto p : copleyDrives) {
        spdlog::debug("Delete Drive Node: {}", p->getNodeID());
        delete p;
    }
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    for (auto p : inputs) {
        spdlog::debug("Deleting Input");
        delete p;
    }
}
void ExoRobot::updateRobot() {
    Robot::updateRobot();
}
