
/**
 *
 * \file RobotM1.h
 * \author Tim Haswell borrowing heavily from Vincent Crocher
 * \version 0.1
 * \date 2020-07-08
 * \copyright Copyright (c) 2020
 *
 * \brief  The<code> RobotM1</ code> class represents an M1 Robot.
 *
 */

#ifndef RobotM1_H_INCLUDED
#define RobotM1_H_INCLUDED

#include <map>
#include <Eigen/Dense>

#include "JointM1.h"
#include "Keyboard.h"
#include "Joystick.h"
#include "Robot.h"
#include "FourierForceSensor.h"

#define M1_NUM_JOINTS 1
#define M1_NUM_INTERACTION 1

//typedef unsigned int uint;

// System description constants
static const uint nJoints = 1;  // Number of joints in the system
static const uint nEndEff = 2;  // Number of dimensions for the end effector data
typedef Eigen::Matrix<double, nJoints, 1> JointVec;
typedef Eigen::Matrix<double, nEndEff, 1> EndEffVec;
typedef Eigen::Matrix<double, nJoints, nEndEff> JacMtx;


/**
 * An enum type.
 * Constants representing the Drives State
 */
enum RobotState{
    R_SUCCESS = 1,
    R_OUTSIDE_LIMITS = -1,
    R_UNKNOWN_ERROR = -100
};

/**
     * \todo Load in parameters and dictionary entries from JSON file.
     *
     */

/**
 * \brief Implementation of the M1 robot class, representing an M1 using 1 JointM1 (and so Kinco drive).
 * model reference:
 */
class RobotM1 : public Robot {
   private:
    /**
     * \brief motor drive position control profile parameters, user defined.
     *
     */
    motorProfile posControlMotorProfile{2000000, 80000, 80000};

    JointVec LinkLengths;   // Link lengths used for kinematic models (in m)
    JointVec LinkMasses;    // Link masses used for gravity compensation (in kg)
    JointVec CoGLengths;    // Length along link(s) to the Center og Gravity
    JointVec Zero2GravityAngle;    // Angle from q=0 to the direction of gravitational force
    JointVec g;  //Gravitational constant: remember to change it if using the robot on the Moon or another planet
    JointVec max_speed; // {radians}
    JointVec tau_max;  // {Nm}
    /*!< Conversion factors between degrees and radians */
    double d2r, r2d;

    // Storage variables for real-time updated values from CANopn
    JointVec q, dq, tau, tau_s, tau_sc, tau_cmd;
    JointVec q_pre, tau_s_pre;

    JointVec qCalibration;  // Calibration configuration: posture in which the robot is when using the calibration procedure

    bool calibrated;
    double maxEndEffVel; /*!< Maximal end-effector allowable velocity. Used in checkSafety when robot is calibrated.*/
    double maxEndEffForce; /*!< Maximal end-effector allowable force. Used in checkSafety when robot is calibrated. */

    std::string robotName_;

    short int sign(double val);

public:
    /**
      * \brief Default <code>RobotM1</code> constructor.
      * Initialize memory for the <code>Joint</code> + sensor.
      * Load in parameters to  <code>TrajectoryGenerator.</code>.
      */
    RobotM1(std::string robot_name="", std::string yaml_config_file="");
    ~RobotM1();
    JointVec tau_motor;
    Keyboard *keyboard;
    FourierForceSensor *m1ForceSensor;
    RobotState status;
    int mode;

    JointVec tau_spring;

    bool initMonitoring();
    /**
       * \brief Initializes joint to position control mode.
       *
       * \return true If joint is successfully configured
       * \return false  If joint fails the configuration
       */
    bool initPositionControl();

    /**
       * \brief Initialises joint to velocity control mode.
       *
       * \return true If joint is successfully configured
       * \return false  If joint fails the configuration
       */
    bool initVelocityControl();

    /**
       * \brief Initializes joint to torque control mode.
       *
       * \return true If joint is successfully configured
       * \return false  If joint fails the configuration
       */
    bool initTorqueControl();

    /**
       * \brief Send a stop command to joint drive.
       *
       * \return true If joint is stopped
       * \return false  Otherwise
       */
    bool stop();

    /**
    * \brief Set the target position for the joint
    *
    * \param positions a target position - applicable for the actuated joint
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t applyPosition(JointVec positions);

    /**
    * \brief Set the target velocity for the joint
    *
    * \param velocities a target velocity - applicable for the actuated joint
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t applyVelocity(JointVec velocities);

    /**
    * \brief Set the target torque for the joint
    *
    * \param torques a target torque - applicable for the actuated joint
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t applyTorque(JointVec torques);

    /**
    * \brief Apply current configuration as calibration configuration using qcalibration such that:
    *  q=qcalibration in current configuration.
    */
    void applyCalibration();
    bool calibrateForceSensors();

    bool isCalibrated() {return calibrated;}
    void decalibrate() {calibrated = false;}


    /**
       * \brief Implementation of Pure Virtual function from <code>Robot</code> Base class.
       * Create designed <code>Joint</code> and <code>Driver</code> objects and load into
       * Robot joint vector.
       */
    bool initialiseJoints();
    /**
       * \brief Implementation of Pure Virtual function from <code>Robot</code> Base class.
       * Initialize each <code>Drive</code> Objects underlying CANOpen Networking.

      */
    bool initialiseNetwork();
    /**
       * \brief Implementation of Pure Virtual function from <code>Robot</code> Base class.
       * Initialize each <code>Input</code> Object.

      */
    bool initialiseInputs();
    /**
       * \brief update current state of the robot, including input and output devices.
       * Overloaded Method from the Robot Class.
       * Example. for a keyboard input this would poll the keyboard for any button presses at this moment in time.
       */
    void updateRobot();

    /**
       * Writes the desired digital out value to the drive
       *
       * \return true if successful
       * \return false if not
       */
    bool setDigitalOut(int digital_out);

    /**
           * Returns the value of digital IN
           * \return Digital in state from the motor drive
           */
    virtual int getDigitalIn();

    /**
     * \brief Check if current end effector force and velocities are within limits (if calibrated, otherwise
     *  check that joints velocity and torque are within limits).
     *
     * \return OUTSIDE_LIMITS if outside the limits (!), SUCCESS otherwise
     */
    setMovementReturnCode_t safetyCheck();


    /**
     * \brief get the name of the robot that is obtained from node name
     *
     * \return std::string name of the robot
     */
    std::string & getRobotName();

    void printStatus();
    void printJointStatus();


    JacMtx J();
    EndEffVec directKinematic(JointVec q);
    JointVec inverseKinematic(EndEffVec X);
    JointVec calculateGravityTorques();

    JointVec getJointPos();
    JointVec getJointVel();
    JointVec getJointTor();
    JointVec& getJointTor_s();
    JointVec& getJointTor_cmd();

    void filter_q(double alpha_q);
    void filter_tau(double alpha_tau);
//    EndEffVec getEndEffPos();
//    EndEffVec getEndEffVel();
//    EndEffVec getEndEffFor();

    setMovementReturnCode_t setJointPos(JointVec pos);
    setMovementReturnCode_t setJointVel(JointVec vel);
    setMovementReturnCode_t setJointTor(JointVec tor);
    setMovementReturnCode_t setJointTor_comp(JointVec tor, JointVec tor_s, double ffRatio);
    JointVec compensateJointTor(JointVec tor);
};
#endif /*RobotM1_H*/
