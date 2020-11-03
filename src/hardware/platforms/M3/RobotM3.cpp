#include "RobotM3.h"

using namespace Eigen;

short int sign(double val) { return (val > 0) ? 1 : ((val < 0) ? -1 : 0); }

RobotM3::RobotM3() : Robot(),
                     endEffTool(&M3Handle),
                     calibrated(false),
                     maxEndEffVel(2),
                     maxEndEffForce(60) {
    //Define the robot structure: each joint with limits and drive: should be in constructor
    double max_speed = 360 * M_PI / 180.;
    double tau_max = 1.9 * 23;
    joints.push_back(new JointM3(0, -45 * M_PI / 180., 45 * M_PI / 180., 1, -max_speed, max_speed, -tau_max, tau_max, new KincoDrive(1), "q1"));
    joints.push_back(new JointM3(1, -15 * M_PI / 180., 70 * M_PI / 180., 1, -max_speed, max_speed, -tau_max, tau_max, new KincoDrive(2), "q2"));
    joints.push_back(new JointM3(2, 0 * M_PI / 180., 95 * M_PI / 180., -1, -max_speed, max_speed, -tau_max, tau_max, new KincoDrive(3), "q3"));

    inputs.push_back(keyboard = new Keyboard());
    inputs.push_back(joystick = new Joystick());
}
RobotM3::~RobotM3() {
    spdlog::debug("Delete RobotM3 object begins");
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    joints.clear();
    delete keyboard;
    delete joystick;
    inputs.clear();
    spdlog::debug("RobotM3 deleted");
}

bool RobotM3::initialiseJoints() {
    return true;
}
bool RobotM3::initialiseNetwork() {
    spdlog::debug("RobotM3::initialiseNetwork()");

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
            spdlog::error("M3: Failed to enable joint {} (status: {})", n, joint->getDriveStatus());
            return false;
        }
        #endif
        n++;
    }
    printJointStatus();
    return true;
}
bool RobotM3::initialiseInputs() {
    /*nothing to do*/
    return true;
}

void RobotM3::applyCalibration() {
    for (unsigned int i = 0; i < joints.size(); i++) {
        ((JointM3 *)joints[i])->setPositionOffset(qCalibration[i]);
    }
    calibrated = true;
}

void RobotM3::updateRobot() {
    Robot::updateRobot();
    if (safetyCheck() != SUCCESS) {
        disable();
    }
}

setMovementReturnCode_t RobotM3::safetyCheck() {
    //End-effector safeties if calibrated
    if (calibrated) {
        if (getEndEffVelocity().norm() > maxEndEffVel) {
            spdlog::error("M3: Max velocity reached ({}m.s-1)!", getEndEffVelocity().norm());
            return OUTSIDE_LIMITS;
        }
        //if(getEndEffForce().norm()>maxEndEffForce) {
        //   spdlog::error("M3: Max force reached ({}N)!", getEndEffForce().norm());
        //   return OUTSIDE_LIMITS;
        //}
    }
    //otherwise basic joint safeties
    else {
        for (unsigned int i = 0; i < 3; i++) {
            if (((JointM3 *)joints[i])->safetyCheck() != SUCCESS) {
                spdlog::error("M3: Joint {} safety triggered!", i);
                return OUTSIDE_LIMITS;
            }
        }
    }
    return SUCCESS;
}

void RobotM3::printStatus() {
    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "X=[ " << getEndEffPosition().transpose() << " ]\t";
    std::cout << "dX=[ " << getEndEffVelocity().transpose() << " ]\t";
    std::cout << "F=[ " << getEndEffForce().transpose() << " ]\t";
    std::cout << std::endl;
}
void RobotM3::printJointStatus() {
    std::cout << std::setprecision(1) << std::fixed;
    std::cout << "q=[ " << getPosition().transpose() * 180 / M_PI << " ]\t";
    std::cout << "dq=[ " << getVelocity().transpose() * 180 / M_PI << " ]\t";
    std::cout << "tau=[ " << getTorque().transpose() << " ]\t";
    std::cout << "{";
    for (auto joint : joints)
        std::cout << "0x" << std::hex << ((JointM3 *)joint)->getDriveStatus() << "; ";
    std::cout << "}" << std::endl;
}

bool RobotM3::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(CM_POSITION_CONTROL) != CM_POSITION_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM3 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM3 *)p)->enable();
    }

    //TODO:CHECK STATUS 0x07
    return returnValue;
}
bool RobotM3::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(CM_VELOCITY_CONTROL) != CM_VELOCITY_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM3 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM3 *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}
bool RobotM3::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM3 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM3 *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}

setMovementReturnCode_t RobotM3::applyPosition(std::vector<double> positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    if (!calibrated) {
        returnValue = NOT_CALIBRATED;
    } else {
        for (auto p : joints) {
            setMovementReturnCode_t setPosCode = ((JointM3 *)p)->setPosition(positions[i]);
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
setMovementReturnCode_t RobotM3::applyVelocity(std::vector<double> velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setVelCode = ((JointM3 *)p)->setVelocity(velocities[i]);
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
setMovementReturnCode_t RobotM3::applyTorque(std::vector<double> torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setTorCode = ((JointM3 *)p)->setTorque(torques[i]);
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

VM3 RobotM3::directKinematic(VM3 q) {
    VM3 X;

    std::vector<float> L = LinkLengths;

    double F1 = (L[2] * sin(q[1]) + (L[3]+endEffTool->length) * cos(q[2]) + L[0]);

    X[0] = -F1 * cos(q[0]);
    X[1] = -F1 * sin(q[0]);
    X[2] = L[2] * cos(q[1]) - (L[3]+endEffTool->length) * sin(q[2]);

    return X;
}
VM3 RobotM3::inverseKinematic(VM3 X) {
    VM3 q;

    std::vector<float> L = LinkLengths;

    //Check accessible workspace
    double normX = X.norm();
    if ((L[3] < L[2] && normX < L[2] - (L[3]+endEffTool->length)) || ((L[3]+endEffTool->length) > L[2] && normX < sqrt((L[4]+endEffTool->length) * (L[3]+endEffTool->length) - L[2] * L[2])) || normX > (L[2] + (L[3]+endEffTool->length) + L[0]) || X[0] > 0) {
        spdlog::error("RobotM3::inverseKinematic() error: Point not accessible. NaN returned.");
        q[0] = q[1] = q[2] = nan("");
        return q;
    }

    //Compute first joint
    q[0] = -atan2(X[1], -X[0]);

    //Project onto parallel mechanism plane
    VM3 tmpX;
    if (X[0] > 0) {
        //should never happen as outside of workspace...
        tmpX[0] = sqrt(X[0] * X[0] + X[1] * X[1]);
    } else {
        tmpX[0] = -sqrt(X[0] * X[0] + X[1] * X[1]);
    }
    //Remove offset along -x
    tmpX[0] = tmpX[0] + L[0];
    tmpX[1] = X[1];
    tmpX[2] = X[2];

    //Calculate joints 2 and 3
    double beta = acos((L[2] * L[2] + L[3] * L[3] - tmpX[0] * tmpX[0] - tmpX[2] * tmpX[2]) / (2. * (L[2] * L[3])));
    q[1] = acos(L[3] * sin(beta) / sqrt(tmpX[0] * tmpX[0] + tmpX[2] * tmpX[2])) - atan2(tmpX[2], -tmpX[0]);
    q[2] = M_PI / 2. + q[1] - beta;

    return q;
}
Matrix3d RobotM3::J() {
    Matrix3d J;
    VM3 q;
    for (unsigned int i = 0; i < 3; i++) {
        q(i) = ((JointM3 *)joints[i])->getPosition();
    }

    std::vector<float> L = LinkLengths;

    //Pre calculate factors for optimisation
    double F1 = (L[3]+endEffTool->length) * sin(q[2]);
    double F2 = -L[2] * cos(q[1]);
    double F3 = (L[3]+endEffTool->length) * cos(q[2]) + L[2] * sin(q[1]) + L[0];

    //Jacobian matrix elements
    J(0, 0) = F3 * sin(q[0]);
    J(0, 1) = F2 * cos(q[0]);
    J(0, 2) = F1 * cos(q[0]);

    J(1, 0) = -F3 * cos(q[0]);
    J(1, 1) = F2 * sin(q[0]);
    J(1, 2) = F1 * sin(q[0]);

    J(2, 0) = 0;
    J(2, 1) = -L[2] * sin(q[1]);
    J(2, 2) = -(L[3]+endEffTool->length) * cos(q[2]);

    return J;
}

VM3 RobotM3::calculateGravityTorques() {
    VM3 tau_g;

    //For convenience
    std::vector<float> L = LinkLengths;
    std::vector<float> M = LinkMasses;

    float g = 9.81;  //Gravitational constant: remember to change it if using the robot on the Moon or another planet

    //Get current configuration
    VM3 q;
    for (unsigned int i = 0; i < 3; i++) {
        q(i) = ((JointM3 *)joints[i])->getPosition();
    }

    //Calculate gravitational torques
    tau_g[0] = 0;
    tau_g[1] = -L[2] / 2.0f * sin(q[1]) * (M[1] + M[2] + M[3] + M[4] + endEffTool->mass) * g;
    tau_g[2] = -(L[1] / 2.0f * (M[0] + M[3]) + L[1] * M[2] + L[3]/2.0f*M[4] + (L[3]+endEffTool->length/2.)*endEffTool->mass) * cos(q[2]) * g;

    return tau_g;
}


VM3 RobotM3::getEndEffPosition() {
    return directKinematic(getPosition());
}
VM3 RobotM3::getEndEffVelocity() {
    return J() * getVelocity();
}
VM3 RobotM3::getEndEffForce() {
    return (J().transpose()).inverse() * getTorque();
}

setMovementReturnCode_t RobotM3::setJointPosition(VM3 q) {
    std::vector<double> pos{q(0), q(1), q(2)};
    return applyPosition(pos);
}
setMovementReturnCode_t RobotM3::setJointVelocity(VM3 dq) {
    std::vector<double> vel{dq(0), dq(1), dq(2)};
    return applyVelocity(vel);
}
setMovementReturnCode_t RobotM3::setJointTorque(VM3 tau) {
    std::vector<double> tor{tau(0), tau(1), tau(2)};
    return applyTorque(tor);
}
setMovementReturnCode_t RobotM3::setEndEffPosition(VM3 X) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }

    VM3 q = inverseKinematic(X);
    if (std::isnan(q[0]) || std::isnan(q[1]) || std::isnan(q[2])) {
        return OUTSIDE_LIMITS;
    } else {
        return setJointPosition(q);
    }
}
setMovementReturnCode_t RobotM3::setEndEffVelocity(VM3 dX) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }

    VM3 dq = J().inverse() * dX;
    return setJointVelocity(dq);
}
setMovementReturnCode_t RobotM3::setEndEffForce(VM3 F) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }

    VM3 tau = J().transpose() * F;
    return setJointTorque(tau);
}
setMovementReturnCode_t RobotM3::setEndEffForceWithCompensation(VM3 F, bool friction_comp) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }
    VM3 tau_g = calculateGravityTorques();  //Gravity compensation torque
    VM3 tau_f(0, 0, 0);                     //Friction compensation torque
    if (friction_comp) {
        double alpha = 0.5, beta = 0.2, threshold = 0.000000;
        for (unsigned int i = 0; i < 3; i++) {
            double dq = ((JointM3 *)joints[i])->getVelocity();
            if (abs(dq) > threshold) {
                tau_f(i) = alpha * sign(dq) + beta * dq;
            } else {
                tau_f(i) = .0;
            }
        }
    }

    return setJointTorque(J().transpose() * F + tau_g + tau_f);
}
