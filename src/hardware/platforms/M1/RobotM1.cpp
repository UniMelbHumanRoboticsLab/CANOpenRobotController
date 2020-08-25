#include "RobotM3.h"

#include "DebugMacro.h"

using namespace Eigen;

// Moved to Robot.h - TMH
//short int sign(double val) { return (val > 0) ? 1 : ((val < 0) ? -1 : 0); }

RobotM3::RobotM3() : Robot(), calibrated(false), maxEndEffVel(2), maxEndEffForce(60) {
    // Conversion factors between degrees and radians
    d2r = M_PIf64 / 180.;
    r2d = 180. / M_PIf64;

    //Define the robot structure: each joint with limits and drive - TMH
    // JOINT 0 - the only joint in the case of M1
    max_speed(0) = 360 * d2r; // {radians}
    tau_max(0) = 1.9 * 23;  // {Nm}
    LinkLengths(0) = 0.1;   // Link lengths used for kinematic models (in m)
    LinkMasses(0) = 0.5;    // Link masses used for gravity compensation (in kg)
    CoGLengths(0) = 0.08;    // Length along link(s) to the Center og Gravity
    Zero2GravityAngle(0) = 0;    // Angle from q=0 to the direction of gravitational force
    // g is the gravitational constant affecting that link, if its motion isn't subject to gravity set to 0
    g(0) = 9.81;  //Gravitational constant: remember to change it if using the robot on the Moon or another planet

    // Calibration configuration: posture in which the robot is when using the calibration procedure
    qCalibration(0) = 0 * d2r;

    joints.push_back(new JointM3(0, -45*d2r, 45*d2r, 1, -max_speed(0), max_speed(0), -tau_max(0), tau_max(0)));

    inputs.push_back(keyboard = new Keyboard());
    inputs.push_back(joystick = new Joystick());
}
RobotM3::~RobotM3() {
    DEBUG_OUT("Delete RobotM3 object begins")
    for (auto p : joints) {
        DEBUG_OUT("Delete Joint ID: " << p->getId())
        delete p;
    }
    joints.clear();
    inputs.clear();
    delete keyboard;
    delete joystick;
    DEBUG_OUT("RobotM3 deleted")
}

bool RobotM3::initialiseJoints() {
    return true;
}
bool RobotM3::initialiseNetwork() {
    DEBUG_OUT("RobotM3::initialiseNetwork()");

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }
    //Give time to drives PDO initialisation
    //TODO: Parameterize the number of PDOs for situations like the one below
    DEBUG_OUT("...");
    for (int i = 0; i < 5; i++) {
        DEBUG_OUT(".");
        usleep(10000);
    }

    return true;
}
bool RobotM3::initialiseInputs() {
    /*nothing to do*/
    return true;
}

bool RobotM3::stop() {
    std::cout << "Stopping M1 robot..." << std::endl;
    for (auto p : joints) {
        ((JointM3 *)p)->disable();
    }
    return true;
}

void RobotM3::applyCalibration() {
    for (int i = 0; i < joints.size(); i++) {
        ((JointM3 *)joints[i])->setPositionOffset(qCalibration(i));
    }
    calibrated = true;
}

void RobotM3::updateRobot() {
    Robot::updateRobot();   // Trigger RT data update at the joint level
    // Gather joint data at the Robot level
    //TODO: we should probably do this with all PDO data, if we are setting it up
    // to be delivered in real-time we should make it available and use it or stop
    // sending it in real-time to conserve bandwidth. It would also be good to break
    // down the status word into a vector of booleans and have descriptive indices to
    // be able to clearly access the bits in a meaningful and readable way. -TMH
    for(uint i = 0; i < nJoints; i++) {
        q(i) = ((JointM3 *)joints[i])->getPosition();
        dq(i) = ((JointM3 *)joints[i])->getVelocity();
        tau(i) = ((JointM3 *)joints[i])->getTorque();
    }
    if (safetyCheck() != SUCCESS) {
        stop();
    }
}

setMovementReturnCode_t RobotM3::safetyCheck() {
    //End-effector safeties if calibrated
    if (calibrated) {
        if (getEndEffVel().norm() > maxEndEffVel) {
            std::cout /*cerr is banned*/ << "M1: Max velocity reached (" << getEndEffVel().norm() << "m.s-1)!" << std::endl;
            return OUTSIDE_LIMITS;
        }
        //if(getEndEffFor().norm()>maxEndEffForce) {
        //   std::cout /*cerr is banned*/ << "M1: Max force reached (" << getEndEffFor().norm() << "N)!" << std::endl;
        //   return OUTSIDE_LIMITS;
        //}
    }
    //otherwise basic joint safeties
    else {
        for (unsigned int i = 0; i < 3; i++) {
            if (((JointM3 *)joints[i])->safetyCheck() != SUCCESS) {
                std::cout /*cerr is banned*/ << "M1: Joint " << i << " safety triggered!" << std::endl;
                return OUTSIDE_LIMITS;
            }
        }
    }
    return SUCCESS;
}

void RobotM3::printStatus() {
    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "X=[ " << getEndEffPos().transpose() << " ]\t";
    std::cout << "dX=[ " << getEndEffVel().transpose() << " ]\t";
    std::cout << "F=[ " << getEndEffFor().transpose() << " ]\t";
    std::cout << std::endl;
}

void RobotM3::printJointStatus() {
    std::cout << std::setprecision(1) << std::fixed;
    std::cout << "q=[ " << getJointPos().transpose() * r2d << " ]\t";
    std::cout << "dq=[ " << getJointVel().transpose() * r2d << " ]\t";
    std::cout << "tau=[ " << getJointTor().transpose() << " ]\t";
    std::cout << "{";
    for (auto joint : joints)
        std::cout << "0x" << std::hex << ((JointM3 *)joint)->getDriveStatus() << "; ";
    std::cout << "}" << std::endl;
}

bool RobotM3::initPositionControl() {
    DEBUG_OUT("Initialising Position Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(POSITION_CONTROL, posControlMotorProfile) != POSITION_CONTROL) {
            // Something bad happened if were are here
            DEBUG_OUT("Something bad happened")
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
    return returnValue;
}
bool RobotM3::initVelocityControl() {
    DEBUG_OUT("Initialising Velocity Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(VELOCITY_CONTROL, posControlMotorProfile) != VELOCITY_CONTROL) {
            // Something bad happened if were are here
            DEBUG_OUT("Something bad happened")
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
    return returnValue;
}
bool RobotM3::initTorqueControl() {
    DEBUG_OUT("Initialising Torque Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(TORQUE_CONTROL) != TORQUE_CONTROL) {
            // Something bad happened if were are here
            DEBUG_OUT("Something bad happened")
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
    return returnValue;
}

setMovementReturnCode_t RobotM3::applyPosition(JointVec positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    if (!calibrated) {
        returnValue = NOT_CALIBRATED;
    } else {
        for (auto p : joints) {
            setMovementReturnCode_t setPosCode = ((JointM3 *)p)->setPosition(positions(i));
            if (setPosCode == INCORRECT_MODE) {
                std::cout << "Joint " << p->getId() << ": is not in Position Control " << std::endl;
                returnValue = INCORRECT_MODE;
            } else if (setPosCode != SUCCESS) {
                // Something bad happened
                std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
                returnValue = UNKNOWN_ERROR;
            }
            i++;
        }
    }
    return returnValue;
}

setMovementReturnCode_t RobotM3::applyVelocity(JointVec velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setVelCode = ((JointM3 *)p)->setVelocity(velocities(i));
        if (setVelCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Velocity Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setVelCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

setMovementReturnCode_t RobotM3::applyTorque(JointVec torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setTorCode = ((JointM3 *)p)->setTorque(torques(i));
        if (setTorCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Torque Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setTorCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

RobotM3::EndEffVec RobotM3::directKinematic(JointVec q_given) {
    Vector2d X;

    X(0) = LinkLengths(0) * cos(q_given(0));
    X(1) = LinkLengths(0) * sin(q_given(0));

    return X;
}

RobotM3::JointVec RobotM3::inverseKinematic(EndEffVec X) {
    // Calculate joint angle from (X,Y)
    JointVec q_exp;
    q_exp(0) = -atan2(X(1), -X(0));
    return q_exp;
}

RobotM3::JacMtx RobotM3::J() {
    JacMtx J;

    //Jacobian matrix elements
    //TODO: Check Jacobian calculations for M1 - what angle will be considered 0
    J(0) = LinkLengths(0) * sin(q(0));
    J(1) = LinkLengths(0) * cos(q(0));

    return J;
}

RobotM3::JointVec RobotM3::calculateGravityTorques() {
    JointVec tau_g;

    //Calculate gravitational torques
    tau_g(0) = LinkMasses(0) * g(0) * CoGLengths(0) * sin(q(0) - Zero2GravityAngle(0));

    return tau_g;
}

RobotM3::JointVec RobotM3::getJointPos() {
    return q;
}
RobotM3::JointVec RobotM3::getJointVel() {
    return dq;
}
RobotM3::JointVec RobotM3::getJointTor() {
    return tau;
}
RobotM3::EndEffVec RobotM3::getEndEffPos() {
    return directKinematic(getJointPos());
}
RobotM3::EndEffVec RobotM3::getEndEffVel() {
    return J() * getJointVel();
}
RobotM3::EndEffVec RobotM3::getEndEffFor() {
    return (J().transpose()).inverse() * getJointTor();
}

setMovementReturnCode_t RobotM3::setJointPos(JointVec pos_d) {
    return applyPosition(pos_d);
}
setMovementReturnCode_t RobotM3::setJointVel(JointVec vel_d) {
    return applyVelocity(vel_d);
}
setMovementReturnCode_t RobotM3::setJointTor(JointVec tor_d) {
    return applyTorque(tor_d);
}
setMovementReturnCode_t RobotM3::setEndEffPos(EndEffVec X_d) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (false) {
        return OUTSIDE_LIMITS;
    }

    JointVec q_d = inverseKinematic(X_d);
    if (isnan(q_d)) {
        return OUTSIDE_LIMITS;
    } else {
        return setJointPos(q_d);
    }
}
setMovementReturnCode_t RobotM3::setEndEffVel(EndEffVec dX_d) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (false) {
        return OUTSIDE_LIMITS;
    }

    JointVec dq_d = J().inverse() * dX_d;
    return setJointVel(dq_d);
}
setMovementReturnCode_t RobotM3::setEndEffFor(EndEffVec F_d) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (false) {
        return OUTSIDE_LIMITS;
    }

    JointVec tau_d = J().transpose() * F_d;
    return setJointTor(tau_d);
}
setMovementReturnCode_t RobotM3::setEndEffForWithCompensation(EndEffVec F_d) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (false) {
        return OUTSIDE_LIMITS;
    }
    JointVec tau_g = calculateGravityTorques();  //Gravity compensation torque
    JointVec tau_f;                              //Friction compensation torque
    //TODO: how are these values determined? Do they need to be tuned for each device? Joint?
    double alpha = 0.5, beta = 0.03, threshold = 0.000000;
    for (unsigned int i = 0; i < nJoints; i++) {
        //double dq = ((JointM3 *)joints[i])->getVelocity();
        if (abs(dq(i)) > threshold) {
            tau_f(i) = alpha * sign(dq(i)) + beta * dq(i);
        }
    }

    JointVec tau_d = J().transpose() * F_d + tau_g + tau_f;
    return setJointTor(tau_d);
}
