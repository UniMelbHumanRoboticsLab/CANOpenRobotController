#include "RobotM2.h"

using namespace Eigen;
using namespace std;

RobotM2::RobotM2(string robot_name, string yaml_config_file) :  Robot(robot_name, yaml_config_file),
                                                                calibrated(false),
                                                                maxEndEffVel(3),
                                                                maxEndEffForce(80) {
    //Check if YAML file exists and contain robot parameters
    initialiseFromYAML(yaml_config_file);

    //Define the robot structure: each joint with limits and drive
    double tau_max = 1.9 * 166;
    joints.push_back(new JointM2(0, 0, 0.625, 1, -maxEndEffVel, maxEndEffVel, -tau_max, tau_max, new KincoDrive(1), "x"));
    joints.push_back(new JointM2(1, 0, 0.440, 1, -maxEndEffVel, maxEndEffVel, -tau_max, tau_max, new KincoDrive(2), "y"));

    forceSensors.push_back(new FourierForceSensor(3, 4.0)); //TODO: to calibrate!
    forceSensors.push_back(new FourierForceSensor(4, 4.0));
    for(unsigned int i=0; i<forceSensors.size(); i++)
        inputs.push_back(forceSensors[i]);

    inputs.push_back(keyboard = new Keyboard());
    inputs.push_back(joystick = new Joystick());
}
RobotM2::~RobotM2() {
    spdlog::debug("Delete RobotM2 object begins");
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    joints.clear();
    delete keyboard;
    delete joystick;
    inputs.clear();
    spdlog::debug("RobotM2 deleted");
}

bool RobotM2::initialiseJoints() {
    return true;
}
bool RobotM2::initialiseNetwork() {
    spdlog::debug("RobotM2::initialiseNetwork()");

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
            spdlog::error("M2: Failed to enable joint {} (status: {})", n, joint->getDriveStatus());
            return false;
        }
        #endif
        n++;
    }
    updateRobot();
    return true;
}
bool RobotM2::initialiseInputs() {
    spdlog::debug("RobotM2::initialiseInputs()");
    for (auto sensor : forceSensors) {
        sensor->calibrate();
    }

    return true;
}

void RobotM2::applyCalibration() {
    for (unsigned int i = 0; i < joints.size(); i++) {
        ((JointM2 *)joints[i])->setPositionOffset(qCalibration[i]);
    }
    for (unsigned int i = 0; i < forceSensors.size(); i++) {
        forceSensors[i]->calibrate();
    }

    calibrated = true;
}

void RobotM2::updateRobot() {
    Robot::updateRobot();

    //Update copies of end-effector values
    endEffPositions = directKinematic(getPosition());
    Matrix2d _J = J();
    endEffVelocities = _J * getVelocity();
    endEffForces = getEndEffForce();
    interactionForces = getInteractionForce();

    if (safetyCheck() != SUCCESS) {
        disable();
    }
}

setMovementReturnCode_t RobotM2::safetyCheck() {
    //End-effector safeties if calibrated
    if (calibrated) {
        if (getEndEffVelocity().norm() > maxEndEffVel) {
            spdlog::error("M2: Max velocity reached ({}m.s-1)!", getEndEffVelocity().norm());
            return OUTSIDE_LIMITS;
        }
        if(interactionForces.norm()>maxEndEffForce) {
           spdlog::error("M2: Max force reached ({}N)!", interactionForces.norm());
           return OUTSIDE_LIMITS;
        }
    }
    //otherwise basic joint safeties
    else {
        for (unsigned int i = 0; i < joints.size(); i++) {
            if (((JointM2 *)joints[i])->safetyCheck() != SUCCESS) {
                spdlog::error("M2: Joint {} safety triggered!", i);
                return OUTSIDE_LIMITS;
            }
        }
    }
    return SUCCESS;
}

void RobotM2::printStatus() {
    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "X=[ " << getEndEffPosition().transpose() << " ]\t";
    std::cout << "dX=[ " << getEndEffVelocity().transpose() << " ]\t";
    std::cout << "Fm=[ " << getEndEffForce().transpose() << " ]\t";
    std::cout << "Fs=[ " << interactionForces.transpose() << " ]\t";
    std::cout << std::endl;
}
void RobotM2::printJointStatus() {
    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "q=[ " << getPosition().transpose() << " ]\t";
    std::cout << "dq=[ " << getVelocity().transpose() << " ]\t";
    std::cout << "tau=[ " << getTorque().transpose() << " ]\t";
    std::cout << "{";
    for (auto joint : joints)
        std::cout << "0x" << std::hex << ((JointM2 *)joint)->getDriveStatus() << "; ";
    std::cout << "}" << std::endl;
}

bool RobotM2::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM2 *)p)->setMode(CM_POSITION_CONTROL) != CM_POSITION_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM2 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM2 *)p)->enable();
    }

    //TODO:CHECK STATUS 0x07
    return returnValue;
}
bool RobotM2::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM2 *)p)->setMode(CM_VELOCITY_CONTROL) != CM_VELOCITY_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM2 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM2 *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}
bool RobotM2::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM2 *)p)->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM2 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM2 *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}

setMovementReturnCode_t RobotM2::applyPosition(std::vector<double> positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    if (!calibrated) {
        returnValue = NOT_CALIBRATED;
    } else {
        for (auto p : joints) {
            setMovementReturnCode_t setPosCode = ((JointM2 *)p)->setPosition(positions[i]);
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
setMovementReturnCode_t RobotM2::applyVelocity(std::vector<double> velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setVelCode = ((JointM2 *)p)->setVelocity(velocities[i]);
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
setMovementReturnCode_t RobotM2::applyTorque(std::vector<double> torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setTorCode = ((JointM2 *)p)->setTorque(torques[i]);
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

VM2 RobotM2::directKinematic(VM2 q) {
    VM2 X = q;

    return X;
}
VM2 RobotM2::inverseKinematic(VM2 X) {
    VM2 q = X;

    return q;
}
Matrix2d RobotM2::J() {
    Matrix2d J = Eigen::Matrix2d::Identity();

    return J;
}


const VX& RobotM2::getEndEffPosition() {
    //Update values
    endEffPositions = directKinematic(getPosition());
    return endEffPositions;
}
const VX& RobotM2::getEndEffVelocity() {
    //Update values
    endEffVelocities = J() * getVelocity();
    return endEffVelocities;
}
const VX& RobotM2::getEndEffForce() {
    //Update values
    endEffForces = (J().transpose()).inverse() * getTorque();
    return endEffForces;
}
const VX& RobotM2::getInteractionForce() {
    if((unsigned int)interactionForces.size()!=forceSensors.size()) {
        interactionForces = Eigen::VectorXd::Zero(forceSensors.size());
    }

    //Update values from force sensors
    for(unsigned int i=0; i< forceSensors.size(); i++) {
        if(forceSensors[i]->isCalibrated())
            interactionForces[i]=forceSensors[i]->getForce();
        else
            interactionForces[i]=std::nan("Not calibrated");
    }
    return interactionForces;
}

setMovementReturnCode_t RobotM2::setJointPosition(VM2 q) {
    std::vector<double> pos{q(0), q(1)};
    return applyPosition(pos);
}
setMovementReturnCode_t RobotM2::setJointVelocity(VM2 dq) {
    std::vector<double> vel{dq(0), dq(1)};
    return applyVelocity(vel);
}
setMovementReturnCode_t RobotM2::setJointTorque(VM2 tau) {
    std::vector<double> tor{tau(0), tau(1)};
    return applyTorque(tor);
}
setMovementReturnCode_t RobotM2::setEndEffPosition(VM2 X) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }

    VM2 q = inverseKinematic(X);
    if (std::isnan(q[0]) || std::isnan(q[1])) {
        return OUTSIDE_LIMITS;
    } else {
        return setJointPosition(q);
    }
}
setMovementReturnCode_t RobotM2::setEndEffVelocity(VM2 dX) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }

    VM2 dq = J().inverse() * dX;
    return setJointVelocity(dq);
}
setMovementReturnCode_t RobotM2::setEndEffForce(VM2 F) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }

    VM2 tau = J().transpose() * F;
    return setJointTorque(tau);
}

//TODO: turn to force control
setMovementReturnCode_t RobotM2::setEndEffForceWithCompensation(VM2 F, bool friction_comp) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }
    VM2 tau_f(0, 0);                     //Friction compensation torque
    if (friction_comp) {
        double alpha = 8, beta = 1, threshold = 0.05;
        for (unsigned int i = 0; i < joints.size(); i++) {
            double dq = ((JointM2 *)joints[i])->getVelocity();
            if (abs(dq) > threshold) {
                tau_f(i) = alpha * sign(dq) + beta * dq;
            } else {
                tau_f(i) = .0;
            }
        }
    }

    return setJointTorque(J().transpose() * F + tau_f);
}
