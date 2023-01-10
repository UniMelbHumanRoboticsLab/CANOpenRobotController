#include "RobotM1.h"

using namespace Eigen;
using namespace std;

RobotM1::RobotM1(string robot_name, string yaml_config_file):   Robot(robot_name, yaml_config_file),
                                                                calibrated(false),
                                                                maxEndEffVel(2),
                                                                maxEndEffForce(60) {

    // Conversion factors between degrees and radians
    d2r = M_PI / 180.;
    r2d = 180. / M_PI;

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
    tau_motor(0) = 0;
    q_pre(0) = 0;
    tau_s_pre(0) = 0;

    // Set up the motor profile
    posControlMotorProfile.profileVelocity = 600.*512*10000/1875;
    posControlMotorProfile.profileAcceleration = 500.*65535*10000/4000000;
    posControlMotorProfile.profileDeceleration = 500.*65535*10000/4000000;

    //Check if YAML file exists and contain robot parameters
    robotName_ = robot_name;
    initialiseFromYAML(yaml_config_file);

    initialiseJoints();
    initialiseInputs();

    mode = 0;

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
    spdlog::debug("RobotM1 deleted");
}

bool RobotM1::initialiseJoints() {
    joints.push_back(new JointM1(0, -100, 100, 1, -max_speed(0), max_speed(0), -tau_max(0), tau_max(0), new KincoDrive(1), "q1"));
    return true;
}

bool RobotM1::initialiseNetwork() {
    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }
    //Give time to drives PDO initialisation
    //TODO: Parameterize the number of PDOs for situations like the one below
    // spdlog::debug("...");
    // for (uint i = 0; i < 5; i++) {
    //     spdlog::debug(".");
    //     usleep(10000);
    // }

    return true;
}

bool RobotM1::initialiseInputs() {
    inputs.push_back(keyboard = new Keyboard());
    inputs.push_back(m1ForceSensor = new FourierForceSensor(17, 0.1, 1));
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

bool RobotM1::calibrateForceSensors() {
    if(m1ForceSensor->calibrate()){
        spdlog::debug("[RobotM1::calibrateForceSensors]: Zeroing of force sensors are successfully completed.");
        return true;
    } else{
        spdlog::debug("[RobotM1::calibrateForceSensors]: Zeroing failed.");
        return false;
    }
}

void RobotM1::updateRobot() {
//    std::cout << "RobotM1::updateRobot()" << std::endl;
    Robot::updateRobot();   // Trigger RT data update at the joint level
    // Gather joint data at the Robot level
    //TODO: we should probably do this with all PDO data, if we are setting it up
    // to be delivered in real-time we should make it available and use it or stop
    // sending it in real-time to conserve bandwidth. It would also be good to break
    // down the status word into a vector of booleans and have descriptive indices to
    // be able to clearly access the bits in a meaningful and readable way. -TMH

    for(uint i = 0; i < nJoints; i++) {
        q(i) = ((JointM1 *)joints[i])->getPosition();
        dq(i) = ((JointM1 *)joints[i])->getVelocity();
        tau(i) = ((JointM1 *)joints[i])->getTorque();
        tau_s(i) = m1ForceSensor[i].getForce();
        // compensate inertia, move it later
        double inertia_s = 1.0592; // m*s*g =
        double inertia_c = 0.3258;
        double theta_bias = 0.1604;
        tau_s(i) =  tau_s(i) - inertia_s*sin(q(i)+theta_bias) - inertia_c*cos(q(i)+theta_bias);
    }
    if (safetyCheck() != SUCCESS) {
        status = R_OUTSIDE_LIMITS;
        stop();
    }
}

bool RobotM1::setDigitalOut(int digital_out) {
    return ((JointM1 *)joints[0])->setDigitalOut(digital_out);
}

int RobotM1::getDigitalIn() {

    return ((JointM1 *)joints[0])->getDigitalIn();
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
//        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM1 *)p)->disable();
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
   usleep(2000);
   for (auto p : joints) {
       ((JointM1 *)p)->enable();
   }
    mode = 1;
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
    mode = 2;
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
    mode = 3;
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

void RobotM1::filter_q(double alpha_q){
    q(0) = alpha_q*q(0)+(1-alpha_q)*q_pre(0);
    q_pre(0) = q(0);
}

void RobotM1::filter_tau(double alpha_tau){
    tau_s(0) = alpha_tau*tau_s(0)+(1-alpha_tau)*tau_s_pre(0);
    tau_s_pre(0) = tau_s(0);
}

JointVec& RobotM1::getJointTor_s() {
    double inertia_s = 1.0592; // m*s*g =
    double inertia_c = 0.3258;
    double theta_bias = 0.1604;
    tau_sc(0) =  tau_s(0); //+ inertia_s*sin(q(0)+theta_bias) + inertia_c*cos(q(0)+theta_bias);
    return tau_sc;
}

setMovementReturnCode_t RobotM1::setJointPos(JointVec pos_d) {
    return applyPosition(pos_d*d2r);
}

setMovementReturnCode_t RobotM1::setJointVel(JointVec vel_d) {
    return applyVelocity(vel_d*d2r);
}

setMovementReturnCode_t RobotM1::setJointTor(JointVec tor_d) {
//    tor_d = compensateJointTor(tor_d);
    tau_motor(0) = (tor_d(0)+tau_motor(0)*2.0)/3.0;
    return applyTorque(tau_motor);
}

JointVec RobotM1::compensateJointTor(JointVec tor){
    double f_s = 1.4;
    double f_d = 0.5;
    double inertia_c = 0.12;
    if(abs(dq(0))<0.05)
    {
        tor(0) = tor(0) + f_s*sign(tor(0)) + f_d*dq(0)+inertia_c*sin(q(0));
//        tor(0) = tor(0) + f_s*sign(tor(0))+ f_d*dq(0)+inertia_c*sin(q(0));

    }
    else
    {
        tor(0) = tor(0) + f_s*sign(dq(0))+ f_d*dq(0)+inertia_c*sin(q(0));
    }
    return tor;
}

setMovementReturnCode_t RobotM1::setJointTor_comp(JointVec tor, JointVec tor_s, double ffRatio) {

    double f_s = 1.2302;
    double f_d = 1.2723;   //1.2
    double inertia_s = 1.0592; // m*s*g =
    double inertia_c = 0.3258;
    double theta_bias = 0.1604;
    double c2 = 1.2532;
    double tor_ff = 0;
    double vel = 0;
    vel = dq(0);

    if(abs(vel)<0.1)
    {
//         tor_ff = f_s*sign(tor_s(0)) + f_d*dq(0)+inertia_c*sin(q(0));
        if(abs(tor_s(0))>0.2)
        {
            tor_ff = f_s*sign(tor_s(0)) + inertia_s*sin(q(0)+theta_bias) + inertia_c*cos(q(0)+theta_bias);
        }
        else
        {
            tor_ff = inertia_s*sin(q(0)+theta_bias) + inertia_c*cos(q(0)+theta_bias);
        }
    }
    else
    {
        vel = vel-sign(vel)*0.1;
//        tor_ff = f_s*sign(dq(0))+ f_d*dq(0)+inertia_c*sin(q(0));
        tor_ff = f_s*sign(vel) + f_d*vel + inertia_s*sin(q(0)+theta_bias) + inertia_c*cos(q(0)+theta_bias) + c2*sqrt(abs(vel))*sign(vel);
    }
    tor(0) = tor(0) + tor_ff*ffRatio;
    return setJointTor(tor);
}

std::string & RobotM1::getRobotName() {
    return robotName_;
}

short RobotM1::sign(double val) { return (val > 0) ? 1 : ((val < 0) ? -1 : 0); }
