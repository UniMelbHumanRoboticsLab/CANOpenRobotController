/**
 *
 * \file RobotM3.h
 * \author Vincent Crocher
 * \version 0.2
 * \date 2020-07-27
 * \copyright Copyright (c) 2020
 *
 * \brief  The<code> RobotM3</ code> class represents an M3 Robot.
 *
 */

#ifndef RobotM3_H_INCLUDED
#define RobotM3_H_INCLUDED

#include <map>
#include <Eigen/Dense>

#include "JointM3.h"
#include "Keyboard.h"
#include "Joystick.h"
#include "Robot.h"


typedef Eigen::Vector3d VM3; //! Convenience alias for double  Vector of length 3

/**
     * \todo Load in paramaters and dictionary entries from JSON file.
     *
     */

typedef struct M3Tool
{
    M3Tool(double l, double m, std::string n="tool"):length(l),mass(m),name(n) {};
    const double length; //Tool length from attachment
    const double mass; //In kg
    const std::string name;
} M3Tool;

//Classic tools attached to M3
static M3Tool M3NoTool(0, .0, "No Tool"); //! Default handle with 3 rotational DoFs
static M3Tool M3Handle(0.140, 0.720, "Handle"); //! Default handle with 3 rotational DoFs 0.465
static M3Tool M3MachiningTool(0.060, 0.100, "Machining tool"); //!

/**
 * \brief Implementation of the M3 robot class, representing an M3 using 3 JointM3 (and so Kinco drives).
 * model reference:
 *
 *                              /\
 *                            /-  \
 *                          /-     \
 *                        /-        \
 *              (L4)    /-           \
 *                    /-  \           \
 *                  /-     \           \
 *                 /        \           \ (L2)
 *               /-          \           \
 *           M3/-             \        M1 \
 *           /-                \           \
 *         /-                   \           \
 *       /-                   M2 \           \
 *     /-                         \           \
 *+------+                         \           \
 *| MTool|                          \           \
 *+------+                           \         q1-  (L0)
 *                                    \          -------
 *                                     \    (L1)      q0
 *                                    q2              |
 *                                                    |
 *                                                    |
 *                                                    |
 *
 *
 */

class RobotM3 : public Robot {
   private:
    const std::vector<float> LinkLengths = {0.056, 0.15-0.015, 0.5, 0.325+0.15-0.015};   /*!< Link lengths used for kinematic models (in m), excluding tool*/
    const std::vector<float> LinkMasses = {0, 0.450, 0.400, 0.100, .0};                  /*!< Link masses used for gravity compensation (in kg), excluding tool*/

    M3Tool *endEffTool; /*!< End-effector representation (transformation and mass) */

    VM3 qCalibration = {38*M_PI/180., 70*M_PI/180., 95*M_PI/180.};  /*!< Calibration configuration: posture in which the robot is when using the calibration procedure */

    bool calibrated;
    double maxEndEffVel; /*!< Maximal end-effector allowable velocity. Used in checkSafety when robot is calibrated.*/
    double maxEndEffForce; /*!< Maximal end-effector allowable force. Used in checkSafety when robot is calibrated. */

   public:
    /**
      * \brief Default <code>RobotM3</code> constructor.
      * Initialize memory for the Exoskelton <code>Joint</code> + sensors.
      * Load in exoskeleton paramaters to  <code>TrajectoryGenerator.</code>.
      */
    RobotM3();
    ~RobotM3();

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
    *  q=qcalibration in current configuration.
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


    Eigen::Matrix3d J();
    VM3 directKinematic(VM3 q);
    VM3 inverseKinematic(VM3 X);
    VM3 calculateGravityTorques();

    VM3 getEndEffPosition();
    VM3 getEndEffVelocity();
    VM3 getEndEffForce();

    setMovementReturnCode_t setJointPosition(VM3 q);
    setMovementReturnCode_t setJointVelocity(VM3 q);
    setMovementReturnCode_t setJointTorque(VM3 tau);
    setMovementReturnCode_t setEndEffPosition(VM3 X);
    setMovementReturnCode_t setEndEffVelocity(VM3 dX);
    setMovementReturnCode_t setEndEffForce(VM3 F);
    setMovementReturnCode_t setEndEffForceWithCompensation(VM3 F, bool friction_comp=true);

    void changeTool(M3Tool *new_tool) {endEffTool=new_tool; std::cout << "RobotM3::changeTool: new tool: " << endEffTool->name << std::endl;}
};
#endif /*RobotM3_H*/
