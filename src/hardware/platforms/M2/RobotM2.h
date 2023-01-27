/**
 *
 * \file RobotM2.h
 * \author Vincent Crocher
 * \version 0.2
 * \date 2020-12-09
 * \copyright Copyright (c) 2020
 *
 * \brief  The<code> RobotM2</ code> class represents an M2 Robot.
 *
 */

#ifndef RobotM2_H_INCLUDED
#define RobotM2_H_INCLUDED


#include "JointM2.h"
#include "FourierForceSensor.h"
#include "Keyboard.h"
#include "Joystick.h"
#include "Robot.h"


typedef Eigen::Vector2d VM2; //! Convenience alias for double  Vector of length 3
typedef Eigen::VectorXd VX; //!< Generic (dynamic) size version required for compatibility w/ other libraries (FLNL)

/**
 * \brief Implementation of the M2 robot class, representing an M2 using 2 JointM2
 */
class RobotM2: public Robot {
   private:
    VM2 qCalibration = {0, 0.};  //!< Calibration configuration: posture in which the robot is when using the calibration procedure

    bool calibrated;
    double maxEndEffVel; //!< Maximal end-effector allowable velocity. Used in checkSafety when robot is calibrated.
    double maxEndEffForce; //!< Maximal end-effector allowable force. Used in checkSafety when robot is calibrated.

    VX endEffPositions;
    VX endEffVelocities;
    VX endEffForces;
    VX interactionForces;

   public:
    /**
      * \brief Default <code>RobotM2</code> constructor.
      * Initialize memory for the Exoskelton <code>Joint</code> + sensors.
      * Load in exoskeleton paramaters to  <code>TrajectoryGenerator.</code>.
      */
    RobotM2(std::string robot_name="", std::string yaml_config_file="");
    ~RobotM2();

    std::vector<FourierForceSensor*> forceSensors;

    Keyboard *keyboard;
    Joystick *joystick;

    /**
       * \brief Initialises all joints to position control mode.
       *
       * \return true If all joints are successfully configured
       * \return false  If some or all joints fail the configuration
       */
    bool initPositionControl();

    /**
       * \brief Initialises all joints to velocity control mode.
       *
       * \return true If all joints are successfully configured
       * \return false  If some or all joints fail the configuration
       */
    bool initVelocityControl();

    /**
       * \brief Initialises all joints to torque control mode.
       *
       * \return true If all joints are successfully configured
       * \return false  If some or all joints fail the configuration
       */
    bool initTorqueControl();

    /**
    * \brief Set the target positions for each of the joints
    *
    * \param positions a vector of target positions - applicable for each of the actauted joints
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t applyPosition(std::vector<double> positions);

    /**
    * \brief Set the target velocities for each of the joints
    *
    * \param velocities a vector of target velocities - applicable for each of the actuated joints
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t applyVelocity(std::vector<double> velocities);

    /**
    * \brief Set the target torque for each of the joints
    *
    * \param torques a vector of target torques - applicable for each of the actuated joints
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t applyTorque(std::vector<double> torques);

    /**
    * \brief Apply current configuration as calibration configuration using qcalibration such that:
    *  q=qcalibration in current configuration; AND request force sensor zeroing.
    */
    void applyCalibration();

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
     * \brief Check if current end effector force and velocities are within limits (if calibrated, otherwise
     *  check that joints velocity and torque are within limits).
     *
     * \return OUTSIDE_LIMITS if outside the limits (!), SUCCESS otherwise
     */
    setMovementReturnCode_t safetyCheck();

    void printStatus();
    void printJointStatus();


    Eigen::Matrix2d J();
    VM2 directKinematic(VM2 q);
    VM2 inverseKinematic(VM2 X);

    const VX& getEndEffPosition();       //!< Return vector containing end-effector position (in m)
    const VX& getEndEffVelocity();       //!< Return vector containing end-effector velocity (in m.s-1)
    const VX& getEndEffForce();          //!< Return vector containing end-effector (motors) force (in N)
    const VX& getInteractionForce();  //!< Return vector reference containing end-effector interaction force (as per force sensors measurement) (in N)

    setMovementReturnCode_t setJointPosition(VM2 q);
    setMovementReturnCode_t setJointVelocity(VM2 dq);
    setMovementReturnCode_t setJointTorque(VM2 tau);
    setMovementReturnCode_t setEndEffPosition(VM2 X);
    setMovementReturnCode_t setEndEffVelocity(VM2 dX);
    setMovementReturnCode_t setEndEffForce(VM2 F);
    setMovementReturnCode_t setEndEffForceWithCompensation(VM2 F, bool friction_comp=true);
};
#endif /*RobotM2_H*/
