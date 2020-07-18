#include "X2Robot.h"

#include "DebugMacro.h"

X2Robot::X2Robot() : Robot() {
}

X2Robot::~X2Robot() {
    freeMemory();
    DEBUG_OUT("X2Robot deleted")
}

/**
 * An enum type.
 * Joint Index for the 4 joints (note, CANopen NODEID = this + 1)
 */
enum X2Joints {
    X2_LEFT_HIP = 0,   /**< Left Hip*/
    X2_LEFT_KNEE = 1,  /**< Left Knee*/
    X2_RIGHT_HIP = 2,  /**< Right Hip*/
    X2_RIGHT_KNEE = 3, /**< Right Knee*/
};
/**
 * Paramater definitions: Hip motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
JointDrivePairs hipJDP{
        250880,       // drivePosA
        0,            // drivePosB
        deg2rad(90),  //jointPosA
        deg2rad(0)  //jointPosB
};
/**
 * Paramater definitions: Knee motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
JointDrivePairs kneeJDP{
        250880,       // drivePosA
        0,            //drivePosB
        deg2rad(90),  //jointPosA
        deg2rad(0)    //jointPosB
};

/**
 * Defines the Joint Limits of the X2 Exoskeleton
 *
 */
ExoJointLimits X2JointLimits = {deg2rad(210), deg2rad(70), deg2rad(120), deg2rad(0)};

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

setMovementReturnCode_t X2Robot::setPosition(Eigen::VectorXd positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setPosition(positions[i]);
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

setMovementReturnCode_t X2Robot::setVelocity(Eigen::VectorXd velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setVelocity(velocities[i]);
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

setMovementReturnCode_t X2Robot::setTorque(Eigen::VectorXd torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setTorque(torques[i]);
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

Eigen::VectorXd X2Robot::getPosition() {
    int i = 0;
    Eigen::VectorXd actualJointPositions(joints.size());
    for (auto p : joints) {
        actualJointPositions[i] = ((X2Joint *)p)->getPosition();
        i++;
    }
    return actualJointPositions;
}

Eigen::VectorXd X2Robot::getVelocity() {
    int i = 0;
    Eigen::VectorXd actualJointVelocities(joints.size());
    for (auto p : joints) {
        actualJointVelocities[i] = ((X2Joint *)p)->getVelocity();
        i++;
    }
    return actualJointVelocities;
}

Eigen::VectorXd X2Robot::getTorque() {
    int i = 0;
    Eigen::VectorXd actualJointTorques(joints.size());
    for (auto p : joints) {
        actualJointTorques[i] = ((X2Joint *)p)->getTorque();
        i++;
    }
    return actualJointTorques;
}

Eigen::VectorXd X2Robot::getInteractionForce() {
    Eigen::VectorXd actualInteractionForces(X2_NUM_FORCE_SENSORS);
    for (int i = 0; i< X2_NUM_FORCE_SENSORS; i++) {
        actualInteractionForces[i] = forceSensors[i]->getForce();
    }
    return actualInteractionForces;
}

bool X2Robot::calibrateForceSensors() {
    int numberOfSuccess = 0;
    for (int i = 0; i< X2_NUM_FORCE_SENSORS; i++) {
        if(forceSensors[i]->calibrate()) numberOfSuccess++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    if (numberOfSuccess == X2_NUM_FORCE_SENSORS){
        DEBUG_OUT("[X2Robot::calibrateForceSensors]: Zeroing of force sensors are successfully completed.")
    } else{
        DEBUG_OUT("[X2Robot::calibrateForceSensors]: Zeroing failed.")
    }
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

    for (int id = 0; id < X2_NUM_FORCE_SENSORS; id++) {
        forceSensors.push_back(new X2ForceSensor(id));
        inputs.push_back(forceSensors[id]);
    }

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