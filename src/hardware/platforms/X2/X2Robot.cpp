#include "X2Robot.h"

#include "DebugMacro.h"

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
    deg2rad(0)    //jointPosB
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
ExoJointLimits X2JointLimits = {deg2rad(120), deg2rad(-30), deg2rad(120), deg2rad(0)};

X2Robot::X2Robot() : Robot() {
    jointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    jointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    jointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    interactionForces_ = Eigen::VectorXd::Zero(X2_NUM_FORCE_SENSORS);
}

X2Robot::~X2Robot() {
    freeMemory();
    DEBUG_OUT("X2Robot deleted")
}

bool X2Robot::initPositionControl() {
    DEBUG_OUT("Initialising Position Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something back happened if were are here
            DEBUG_OUT("Something bad happened")
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

bool X2Robot::initVelocityControl() {
    DEBUG_OUT("Initialising Velocity Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_VELOCITY_CONTROL, velControlMotorProfile) != CM_VELOCITY_CONTROL) {
            // Something back happened if were are here
            DEBUG_OUT("Something bad happened")
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

bool X2Robot::initTorqueControl() {
    DEBUG_OUT("Initialising Torque Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something back happened if were are here
            DEBUG_OUT("Something bad happened")
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

Eigen::VectorXd& X2Robot::getPosition() {
    int i = 0;
    for (auto p : joints) {
        jointPositions_[i] = ((X2Joint *)p)->getPosition();
        i++;
    }
    return jointPositions_;
}

Eigen::VectorXd& X2Robot::getVelocity() {
    int i = 0;
    for (auto p : joints) {
        jointVelocities_[i] = ((X2Joint *)p)->getVelocity();
        i++;
    }
    return jointVelocities_;
}

Eigen::VectorXd& X2Robot::getTorque() {
    int i = 0;
    for (auto p : joints) {
        jointTorques_[i] = ((X2Joint *)p)->getTorque();
        i++;
    }
    return jointTorques_;
}

Eigen::VectorXd& X2Robot::getInteractionForce() {
    for (int i = 0; i < X2_NUM_FORCE_SENSORS; i++) {
        interactionForces_[i] = forceSensors[i]->getForce();
    }
    return interactionForces_;
}

bool X2Robot::calibrateForceSensors() {
    int numberOfSuccess = 0;
    for (int i = 0; i < X2_NUM_FORCE_SENSORS; i++) {
        if (forceSensors[i]->calibrate()) numberOfSuccess++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    if (numberOfSuccess == X2_NUM_FORCE_SENSORS) {
        DEBUG_OUT("[X2Robot::calibrateForceSensors]: Zeroing of force sensors are successfully completed.")
    } else {
        DEBUG_OUT("[X2Robot::calibrateForceSensors]: Zeroing failed.")
    }
}

bool X2Robot::homing(std::vector<int> homingDirection, float thresholdTorque, float delayTime,
                     float homingSpeed, float maxTime) {
    std::vector<bool> success(X2_NUM_JOINTS, false);
    std::chrono::steady_clock::time_point time0;
    this->initVelocityControl();

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        if (homingDirection[i] == 0) continue;  // skip the joint if it is not asked to do homing

        Eigen::VectorXd desiredVelocity(X2_NUM_JOINTS);
        std::chrono::steady_clock::time_point firstTimeHighTorque;  // time at the first time joint exceed thresholdTorque
        bool highTorqueReached = false;

        desiredVelocity[i] = homingSpeed * homingDirection[i] / std::abs(homingDirection[i]);  // setting the desired velocity by using the direction
        time0 = std::chrono::steady_clock::now();

        DEBUG_OUT("Homing Joint " << i << "...")

        while (success[i] == false &&
               std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count() < maxTime * 1000) {
            this->updateRobot();  // because this function has its own loops, updateRobot needs to be called
            this->setVelocity(desiredVelocity);
            if (std::abs(this->getTorque()[i]) >= thresholdTorque) {  // if high torque is reached
                highTorqueReached = true;
                firstTimeHighTorque = std::chrono::steady_clock::now();
                while (std::chrono::duration_cast<std::chrono::milliseconds>  // high torque should be measured for delayTime
                       (std::chrono::steady_clock::now() - firstTimeHighTorque)
                           .count() < delayTime * 1000) {
                    this->updateRobot();
                    if (std::abs(this->getTorque()[i]) < thresholdTorque) {  // if torque value reach below thresholdTorque, goes back
                        highTorqueReached = false;
                        break;
                    }
                }
            }
            success[i] = highTorqueReached;
        }

        if (success[i]) {
            DEBUG_OUT("Homing Succeeded for Joint " << i << ".")
            if (i == X2_LEFT_HIP || i == X2_RIGHT_HIP) {  // if it is a hip joint

                // zeroing is done depending on the limits on the homing direction
                if (homingDirection[i] > 0)
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.hipMax);
                else
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.hipMin);
            } else if (i == X2_LEFT_KNEE || i == X2_RIGHT_KNEE) {  // if it is a knee joint

                // zeroing is done depending on the limits on the homing direction
                if (homingDirection[i] > 0)
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.kneeMax);
                else
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.kneeMin);
            }

        } else {
            DEBUG_OUT("Homing Failed for Joint " << i << ".")
        }
    }
    // Checking if all commanded joint successfully homed
    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        if (homingDirection[i] == 0) continue;  // skip the joint if it is not asked to do homing
        if (success[i] == false) return false;
    }
    return true;  // will come here if all hoints successfully homed
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
}
void X2Robot::updateRobot() {
    Robot::updateRobot();
}