#include "RobotFITHVExo.h"

using namespace Eigen;
using namespace std;

//TODO: update all max values

RobotFITHVExo::RobotFITHVExo(string robot_name, string yaml_config_file) :  Robot(robot_name, yaml_config_file),
                                                                            calibrated(false) {
    //Check if YAML file exists and contain robot parameters
    initialiseFromYAML(yaml_config_file);

    //Define the robot structure: each joint with limits and drive
    joints.push_back(new JointFITHVExo(0, qLimits[0], qLimits[1], qSigns[0], -dqMax, dqMax, -tauMax, tauMax, new CopleyDrive(1), "R"));
    joints.push_back(new JointFITHVExo(1, qLimits[2], qLimits[3], qSigns[1], -dqMax, dqMax, -tauMax, tauMax, new CopleyDrive(2), "L"));
    absEncoders.push_back(new FITAbsEncoder(3, 2.*M_PI/524288.));
    absEncoders.push_back(new FITAbsEncoder(4, 2.*M_PI/524288.));

    inputs.push_back(keyboard = new Keyboard());
    inputs.push_back(joystick = new Joystick());

    //TODO: add IMU(s)??
}
RobotFITHVExo::~RobotFITHVExo() {
    spdlog::debug("Delete RobotFITHVExo object begins");
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    joints.clear();
    delete keyboard;
    delete joystick;
    inputs.clear();
    spdlog::debug("RobotFITHVExo deleted");
}

bool RobotFITHVExo::initialiseJoints() {
    return true;
}
bool RobotFITHVExo::initialiseNetwork() {
    spdlog::debug("RobotFITHVExo::initialiseNetwork()");

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }
    //Give time to drives PDO initialisation
    spdlog::debug("...");
    for (int i = 0; i < 5; i++) {
        spdlog::debug(".");
        usleep(10000);
    }
    //Start node
    for (auto joint : joints) {
        joint->start();
    }

    //Enable drive
    int n = 0;
    for (auto joint : joints) {
        bool joint_ready = false;
        for (int i = 0; (i < 10) && (!joint_ready); i++) {
            joint->readyToSwitchOn();
            usleep(10000);
            joint_ready = ((joint->getDriveStatus() & 0x01) == 0x01);
        }
        #ifndef NOROBOT
        if (!joint_ready) {
            spdlog::error("FITHVExo: Failed to enable joint {} (status: {})", n, joint->getDriveStatus());
            return false;
        }
        #endif
        n++;
    }
    updateRobot();
    return true;
}
bool RobotFITHVExo::initialiseInputs() {
    spdlog::debug("RobotFITHVExo::initialiseInputs()");

    return true;
}

void RobotFITHVExo::applyCalibration() {
    double abs[2];
    abs[0] = absEncoders[0]->readValue();
    abs[1] = absEncoders[1]->readValue();

    if( isnan(abs[0]) || isnan(abs[1]) )
        return;

    ((JointFITHVExo *)joints[0])->setPositionOffset(2.*M_PI-abs[0]+qCalibration[0]);
    ((JointFITHVExo *)joints[1])->setPositionOffset(abs[1]+qCalibration[1]);
    spdlog::debug("RobotFITHVExo calibrated from abs. enc. at {} {}.", absEncoders[0]->readValue()*180./M_PI, absEncoders[1]->readValue()*180./M_PI);
    calibrated = true;
}

void RobotFITHVExo::updateRobot() {
    Robot::updateRobot();
    if (safetyCheck() != SUCCESS) {
        disable();
    }
}

setMovementReturnCode_t RobotFITHVExo::safetyCheck() {
    for (unsigned int i = 0; i < joints.size(); i++) {
        if (((JointFITHVExo *)joints[i])->safetyCheck() != SUCCESS) {
            spdlog::error("FITHVExo: Joint {} safety triggered!", i);
            return OUTSIDE_LIMITS;
        }
    }
    return SUCCESS;
}

void RobotFITHVExo::printJointStatus() {
    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "q=[ " << getPosition().transpose() * 180/M_PI << " ] [deg]\t";
    std::cout << "dq=[ " << getVelocity().transpose() * 180/M_PI << " ] [deg/s]\t";
    std::cout << "tau=[ " << getTorque().transpose() << " ] [Nm]\t";
    std::cout << "{";
    for (auto joint : joints)
        std::cout << "0x" << std::hex << ((JointFITHVExo *)joint)->getDriveStatus() << "; ";
    std::cout << "}" << std::endl;
}

bool RobotFITHVExo::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointFITHVExo *)p)->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointFITHVExo *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointFITHVExo *)p)->enable();
    }

    //TODO:CHECK STATUS 0x07
    return returnValue;
}
bool RobotFITHVExo::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointFITHVExo *)p)->setMode(CM_VELOCITY_CONTROL, posControlMotorProfile) != CM_VELOCITY_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointFITHVExo *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointFITHVExo *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}
bool RobotFITHVExo::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointFITHVExo *)p)->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointFITHVExo *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointFITHVExo *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}

setMovementReturnCode_t RobotFITHVExo::applyPosition(std::vector<double> positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    if (!calibrated) {
        returnValue = NOT_CALIBRATED;
    } else {
        for (auto p : joints) {
            setMovementReturnCode_t setPosCode = ((JointFITHVExo *)p)->setPosition(positions[i]);
            if (setPosCode == INCORRECT_MODE) {
                spdlog::error("Joint {} : is not in Position Control", p->getId());
                returnValue = INCORRECT_MODE;
            } else if (setPosCode != SUCCESS) {
                // Something bad happened
                spdlog::error("Joint {} position error : {} ", p->getId(), setMovementReturnCodeString[setPosCode]);
                returnValue = UNKNOWN_ERROR;
            }
            i++;
        }
    }
    return returnValue;
}
setMovementReturnCode_t RobotFITHVExo::applyVelocity(std::vector<double> velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setVelCode = ((JointFITHVExo *)p)->setVelocity(velocities[i]);
        if (setVelCode == INCORRECT_MODE) {
            spdlog::error("Joint {} : is not in Velocity Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setVelCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} velocity error : {} ", p->getId(), setMovementReturnCodeString[setVelCode]);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}
setMovementReturnCode_t RobotFITHVExo::applyTorque(std::vector<double> torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setTorCode = ((JointFITHVExo *)p)->setTorque(torques[i]);
        if (setTorCode == INCORRECT_MODE) {
            spdlog::error("Joint {} : is not in Torque Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setTorCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} torque error : {} ", p->getId(), setMovementReturnCodeString[setTorCode]);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}



setMovementReturnCode_t RobotFITHVExo::setJointPosition(V2 q) {
    std::vector<double> pos{q(0), q(1)};
    return applyPosition(pos);
}
setMovementReturnCode_t RobotFITHVExo::setJointVelocity(V2 dq) {
    std::vector<double> vel{dq(0), dq(1)};
    return applyVelocity(vel);
}
setMovementReturnCode_t RobotFITHVExo::setJointTorque(V2 tau) {
    std::vector<double> tor{tau(0), tau(1)};
    return applyTorque(tor);
}


setMovementReturnCode_t RobotFITHVExo::setJointTorqueWithCompensation(V2 tau) {
    std::vector<double> tor{0, 0};

    V2 tau_f(0, 0); //Friction compensation torque
    double threshold = 0.04; //Threshold of velocity under which no compensation (deadzone)
    for (unsigned int i = 0; i < joints.size(); i++) {
        double dq = ((JointFITHVExo *)joints[i])->getVelocity();
        if (abs(dq) > threshold) {
            tau_f(i) = frictionCoul[i] * sign(dq) + frictionVis[i] * dq;
        }
        else {
            tau_f(i) = .0;
        }
        tor[i] = tau(i)+tau_f(i);
    }

    return applyTorque(tor);
}

//TODO: see .h TODOs
/*setMovementReturnCode_t RobotFITHVExo::setEndEffForceWithCompensation(V2 F, bool friction_comp) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }
    V2 tau_f(0, 0);                     //Friction compensation torque
    if (friction_comp) {
        double alpha = 8, beta = 1, threshold = 0.05;
        for (unsigned int i = 0; i < joints.size(); i++) {
            double dq = ((JointFITHVExo *)joints[i])->getVelocity();
            if (abs(dq) > threshold) {
                tau_f(i) = alpha * sign(dq) + beta * dq;
            } else {
                tau_f(i) = .0;
            }
        }
    }

    return setJointTorque(J().transpose() * F + tau_f);
}*/
