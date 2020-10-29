#include "RobotM1.h"

using namespace Eigen;

// Moved to Robot.h - TMH
short int sign(double val) { return (val > 0) ? 1 : ((val < 0) ? -1 : 0); }

RobotM1::RobotM1() : Robot(), calibrated(false), maxEndEffVel(2), maxEndEffForce(60) {
    // Conversion factors between degrees and radians
    d2r = M_PIf64 / 180.;
    r2d = 180. / M_PIf64;

    //Define the robot structure: each joint with limits and drive - TMH
    // JOINT 0 - the only joint in the case of M1
    max_speed(0) = 360; // {radians}
    tau_max(0) = 1.9 * 23;  // {Nm}
    LinkLengths(0) = 0.1;   // Link lengths used for kinematic models (in m)
    LinkMasses(0) = 0.5;    // Link masses used for gravity compensation (in kg)
    CoGLengths(0) = 0.08;    // Length along link(s) to the Center og Gravity
    Zero2GravityAngle(0) = 0;    // Angle from q=0 to the direction of gravitational force
    // g is the gravitational constant affecting that link, if its motion isn't subject to gravity set to 0
    g(0) = 9.81;  //Gravitational constant: remember to change it if using the robot on the Moon or another planet

    // Calibration configuration: posture in which the robot is when using the calibration procedure
    qCalibration(0) = 0 * d2r;

    posControlMotorProfile.profileVelocity = 600.*512*10000/1875;
    posControlMotorProfile.profileAcceleration = 500.*65535*10000/4000000;
    posControlMotorProfile.profileDeceleration = 500.*65535*10000/4000000;

    joints.push_back(new JointM1(0, -100, 100, 1, -max_speed(0), max_speed(0), -tau_max(0), tau_max(0), new KincoDrive(1), "q1"));

    inputs.push_back(keyboard = new Keyboard());
    inputs.push_back(joystick = new Joystick());
//    inputs.push_back(m1ForceSensor = new M1ForceSensor(1));

    status = R_SUCCESS;
}

RobotM1::~RobotM1() {
    spdlog::debug(" Delete RobotM1 object begins");

    for (auto p : joints) {
        spdlog::debug(" Delete Joint ID: {}.", p->getId());
        delete p;
    }
    joints.clear();
    inputs.clear();
    delete keyboard;
    delete joystick;
    spdlog::debug("RobotM1 deleted");
}

bool RobotM1::initialiseJoints() {
    return true;
}
bool RobotM1::initialiseNetwork() {
    std::cout << "RobotM1::initialiseNetwork()" << std::endl;

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }
    //Give time to drives PDO initialisation
    //TODO: Parameterize the number of PDOs for situations like the one below
    spdlog::debug("...");
    for (uint i = 0; i < 5; i++) {
        spdlog::debug(".");
        usleep(10000);
    }
    std::cout << "RobotM1::initialiseNetwork() end" << std::endl;

    return true;
}
bool RobotM1::initialiseInputs() {
    /*nothing to do*/
//    inputs.push_back(&keyboard);
//
//    for (int id = 0; id < X2_NUM_FORCE_SENSORS; id++) {
//        forceSensors.push_back(new X2ForceSensor(id));
//        inputs.push_back(forceSensors[id]);
//    }
    return true;
}

bool RobotM1::stop() {
    std::cout << "Stopping M1 robot..." << std::endl;
    for (auto p : joints) {
        ((JointM1 *)p)->disable();
    }
    return true;
}

void RobotM1::applyCalibration() {
    for (uint i = 0; i < joints.size(); i++) {
        ((JointM1 *)joints[i])->setPositionOffset(qCalibration(i));
    }
    calibrated = true;
}

//bool RobotM1::calibrateForceSensors() {
//    if(m1ForceSensor->calibrate()){
//        DEBUG_OUT("[RobotM1::calibrateForceSensors]: Zeroing of force sensors are successfully completed.");
//        return true;
//    } else{
//        DEBUG_OUT("[RobotM1::calibrateForceSensors]: Zeroing failed.");
//        return false;
//    }
//}

//Eigen::VectorXd X2Robot::getInteractionForce() {
//    Eigen::VectorXd actualInteractionForces(X2_NUM_FORCE_SENSORS);
//    for (int i = 0; i< X2_NUM_FORCE_SENSORS; i++) {
//        actualInteractionForces[i] = forceSensors[i]->getForce();
//    }
//    return actualInteractionForces;
//}

void RobotM1::updateRobot() {
//    std::cout << "RobotM1::updateRobot()" << std::endl;
    Robot::updateRobot();   // Trigger RT data update at the joint level
    // Gather joint data at the Robot level
    //TODO: we should probably do this with all PDO data, if we are setting it up
    // to be delivered in real-time we should make it available and use it or stop
    // sending it in real-time to conserve bandwidth. It would also be good to break
    // down the status word into a vector of booleans and have descriptive indices to
    // be able to clearly access the bits in a meaningful and readable way. -TMH
//    std::cout << "RobotM1::updateRobot" << std::endl; //YW debug
    for(uint i = 0; i < nJoints; i++) {
//        std::cout << "update values" << std::endl;  //YW debug
        q(i) = ((JointM1 *)joints[i])->getPosition();
        dq(i) = ((JointM1 *)joints[i])->getVelocity();
        tau(i) = ((JointM1 *)joints[i])->getTorque();
//        tau_s(i) = m1ForceSensor[i].getForce();
    }
//    std::cout << "safety check" << std::endl; // YW debug
    if (safetyCheck() != SUCCESS) {
        status = R_OUTSIDE_LIMITS;
        stop();
    }
//    std::cout << "RobotM1::updateRobot() end" << std::endl;
//    std::cout << "safety check done" << std::endl; // YW debug
}

setMovementReturnCode_t RobotM1::safetyCheck() {
    //End-effector safeties if calibrated
    for (uint i = 0; i < nJoints; i++) {    // Error found, YW
        if (((JointM1 *)joints[i])->safetyCheck() != SUCCESS) {
            std::cout /*cerr is banned*/ << "M1: Joint " << i << " safety triggered!" << std::endl;
            return OUTSIDE_LIMITS;
        }
    }
    return SUCCESS;
}

void RobotM1::printStatus() {
    std::cout << std::setprecision(3) << std::fixed;
//    std::cout << "X=[ " << getEndEffPos().transpose() << " ]\t";
//    std::cout << "dX=[ " << getEndEffVel().transpose() << " ]\t";
//    std::cout << "F=[ " << getEndEffFor().transpose() << " ]\t";
    std::cout << std::endl;
}

void RobotM1::printJointStatus() {
    std::cout << std::setprecision(1) << std::fixed;
    std::cout << "q=[ " << getJointPos().transpose() << " ]\t";
    std::cout << "dq=[ " << getJointVel().transpose() << " ]\t";
    std::cout << "tau=[ " << getJointTor().transpose() << " ]\t";
    std::cout << "tau_s=[ " << getJointTor_s().transpose() << " ]\t";
    std::cout << "{";
    for (auto joint : joints)
        std::cout << "0x" << std::hex << ((JointM1 *)joint)->getDriveStatus() << "; ";
    std::cout << "}" << std::endl;
}

bool RobotM1::initMonitoring() {
    spdlog::debug("Initialising monitoring all joints.");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something bad happened if were are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM1 *)p)->enable();
    }
    return returnValue;
}

bool RobotM1::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints.");
    bool returnValue = true;

    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something bad happened if were are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
//        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
//    usleep(2000);
//    for (auto p : joints) {
//        ((JointM1 *)p)->enable();
//    }
    return returnValue;
}

bool RobotM1::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints.");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_VELOCITY_CONTROL, posControlMotorProfile) != CM_VELOCITY_CONTROL) {
            // Something bad happened if were are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM1 *)p)->enable();
    }
    return returnValue;
}

bool RobotM1::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints.");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something bad happened if were are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM1 *)p)->enable();
    }
    return returnValue;
}

setMovementReturnCode_t RobotM1::applyPosition(JointVec positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((JointM1 *)p)->setPosition(positions(i));
        if (setPosCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Position Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": " << std::endl;
            ((JointM1 *)p)->errorMessage(setPosCode);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

setMovementReturnCode_t RobotM1::applyVelocity(JointVec velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
//        std::cout << "Joint velocity 2: " << velocities(0) << std::endl;
        setMovementReturnCode_t setVelCode = ((JointM1 *)p)->setVelocity(velocities(i));
        if (setVelCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Velocity Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setVelCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << " velocity : " << std::endl;
            ((JointM1 *)p)->errorMessage(setVelCode);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

setMovementReturnCode_t RobotM1::applyTorque(JointVec torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setTorCode = ((JointM1 *)p)->setTorque(torques(i));
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

EndEffVec RobotM1::directKinematic(JointVec q_given) {
    Vector2d X;

    X(0) = LinkLengths(0) * cos(q_given(0));
    X(1) = LinkLengths(0) * sin(q_given(0));

    return X;
}

JointVec RobotM1::inverseKinematic(EndEffVec X) {
    // Calculate joint angle from (X,Y)
    JointVec q_exp;
    q_exp(0) = -atan2(X(1), -X(0));
    return q_exp;
}

JacMtx RobotM1::J() {
    JacMtx J;

    //Jacobian matrix elements
    //TODO: Check Jacobian calculations for M1 - what angle will be considered 0
    J(0) = LinkLengths(0) * sin(q(0));
    J(1) = LinkLengths(0) * cos(q(0));

    return J;
}
//*
JointVec RobotM1::calculateGravityTorques() {
    JointVec tau_g;

    //Calculate gravitational torques
    tau_g(0) = LinkMasses(0) * g(0) * CoGLengths(0) * sin(q(0) - Zero2GravityAngle(0));

    return tau_g;
}

JointVec RobotM1::getJointPos() {
    return q*r2d;
}

JointVec RobotM1::getJointVel() {
    return dq*r2d;
}

JointVec RobotM1::getJointTor() {
    return tau;
}

JointVec RobotM1::getJointTor_s() {
    return tau_s;
}

setMovementReturnCode_t RobotM1::setJointPos(JointVec pos_d) {
    return applyPosition(pos_d*d2r);
}

setMovementReturnCode_t RobotM1::setJointVel(JointVec vel_d) {
    return applyVelocity(vel_d*d2r);
}

setMovementReturnCode_t RobotM1::setJointTor(JointVec tor_d) {
    return applyTorque(tor_d);
}

/*
EndEffVec RobotM1::getEndEffPos() {
    //return directKinematic(getJointPos());
    EndEffVec s;
    return s;
}

EndEffVec RobotM1::getEndEffVel() {
    // return J() * getJointVel();
    EndEffVec s;
    return s;

}

EndEffVec RobotM1::getEndEffFor() {
    //return (J().transpose()).inverse() * getJointTor();
    EndEffVec s;
    return s;

}

setMovementReturnCode_t RobotM1::setEndEffPos(EndEffVec X_d) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (false) {
        return OUTSIDE_LIMITS;
    }

    Vector3d q_d = inverseKinematic(X_d);
    if (std::isnan(q_d[0]) || std::isnan(q_d[1]) || std::isnan(q_d[2])) {
        return OUTSIDE_LIMITS;
    } else {
        return setJointPos(q_d);
    }
}

setMovementReturnCode_t RobotM1::setEndEffVel(EndEffVec dX_d) {
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

setMovementReturnCode_t RobotM1::setEndEffFor(EndEffVec F_d) {
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

setMovementReturnCode_t RobotM1::setEndEffForWithCompensation(EndEffVec F_d) {
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
    for (uint i = 0; i < nJoints; i++) {
        //double dq = ((JointM1 *)joints[i])->getVelocity();
	double dq_t = dq(i);
        if (abs(dq_t) > threshold) {
            tau_f(i) = alpha * sign(dq_t) + beta * dq_t;
        }
    }

    JointVec tau_d = J().transpose() * F_d + tau_g + tau_f;
    return setJointTor(tau_d);
}//*/
