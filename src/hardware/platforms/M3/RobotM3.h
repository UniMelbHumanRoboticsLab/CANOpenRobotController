/**
 * \file RobotM3.h
 * \author Vincent Crocher
 * \version 0.3
 * \date 2021-06-30
 * \copyright Copyright (c) 2020 - 2021
 *
 * \brief  The RobotM3 class represents an M3 Robot.
 *
 */

#ifndef RobotM3_H_INCLUDED
#define RobotM3_H_INCLUDED


#include "JointM3.h"
#include "Keyboard.h"
#include "Joystick.h"
#include "Robot.h"
#include "SignalProcessing.h"


typedef Eigen::Vector3d VM3; //!< Convenience alias for double  Vector of length 3
typedef Eigen::VectorXd VX; //!< Generic (dynamic) size version required for compatibility w/ other libraries (FLNL)

typedef struct M3Tool
{
    M3Tool(double l, double m, std::string n="tool"):length(l),mass(m),name(n) {};
    const double length; //Tool length from attachment
    const double mass; //In kg
    const std::string name;
} M3Tool;

//Classic tools attached to M3
static M3Tool M3NoTool(0, .0, "No Tool"); //!< No Tool
static M3Tool M3Handle(0.140, 0.800, "Handle"); //!< Default handle with 3 rotational DoFs 0.465 //TODO YAML

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
    /** @name Kinematic and dynamic parameters
    *  These constant parameters have a default value which may be overwritten by a YAML configuration file
    *  if one is provided to the constructor and the parameter is defined in the configuration file.
    */
    /* @{ */
    double dqMax = 360 * M_PI / 180.;                                           //!< Max joint speed (rad.s-1)
    double tauMax = 1.9 * 22;                                                   //!< Max joint torque (Nm)
    std::vector<float> qSigns = {1, 1, -1};                                     //!< Joint direction (as compared to built-in drives direction)
    std::vector<float> linkLengths = {0.056, 0.15-0.015, 0.5, 0.325+0.15-0.015};//!< Link lengths used for kinematic models (in m), excluding tool
    std::vector<float> linkMasses = {0, 0.450, 0.400, 0.100, .0};               //!< Link masses used for gravity compensation (in kg), excluding tool
    std::vector<double> frictionVis = {0.2, 0.2, 0.2};                          //!< Joint viscous friction compensation coefficients
    std::vector<double> frictionCoul = {0.5, 0.5, 0.5};                         //!< Joint Coulomb (static) friction compensation coefficients

    std::vector<double> qLimits = {/*q1_min*/ -45 * M_PI / 180.,/*q1_max*/ 45 * M_PI / 180., /*q2_min*/ -15 * M_PI / 180., /*q2_max*/70 * M_PI / 180., /*q3_min*/ 0 * M_PI / 180., /*q3_max*/ 95 * M_PI / 180.}; //!< Joints limits (in rad)
    VM3 qCalibration = {-38*M_PI/180., 70*M_PI/180., 95*M_PI/180.};             //!< Calibration configuration: posture in which the robot is when using the calibration procedure
    /*@}*/

    M3Tool *endEffTool; //!< End-effector representation (transformation and mass)

    bool calibrated;
    double maxEndEffVel; //!< Maximal end-effector allowable velocity. Used in checkSafety when robot is calibrated.
    double maxEndEffForce; //!< Maximal end-effector allowable force. Used in checkSafety when robot is calibrated.

    Filter velFilt;
    double last_update_time; //!< Last time updateRobot has been called (in s)

    VX endEffPositions;
    VX endEffVelocities;
    VX endEffAccelerations;
    VX endEffForces;
    VX interactionForces;
    VX endEffVelocitiesFiltered;

   public:
    /**
      * \brief Default RobotM3 constructor.
      * Creates joints and inputs.
      * \param yaml_config_file the name of a valide YAML file describing kinematic and dynamic parameters of the M3. If absent or incomplete default parameters are used instead.
      */
    RobotM3(std::string robot_name="", std::string yaml_config_file="");
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

   private:
    /**
    * \brief Load parameters from YAML file if valid one specified in constructor.
    * If absent or incomplete (some parameters only) default parameters are used instead.
    * \param params a valid YAML robot parameters node loaded by initialiseFromYAML() method.
    * \return true
    */
    bool loadParametersFromYAML(YAML::Node params);

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

   public:
    /**
    * \brief Apply current configuration as calibration configuration using qcalibration such that:
    *  q=qcalibration in current configuration.
    */
    void applyCalibration();

    bool isCalibrated() {return calibrated;}
    void decalibrate() {calibrated = false;}


    /**
       * \brief Implementation of Pure Virtual function from Robot Base class.
       * Create designed Joint and Driver objects and load into
       * Robot joint vector.
       */
    bool initialiseJoints();
    /**
       * \brief Implementation of Pure Virtual function from Robot Base class.
       * Initialize each Drive Objects underlying CANOpen Networking.

      */
    bool initialiseNetwork();
    /**
       * \brief Implementation of Pure Virtual function from Robot Base class.
       * Initialize each Input Object.

      */
    bool initialiseInputs();
    /**
       * \brief update current state of the robot, including input and output devices.
       * Overloaded Method from the Robot Class.
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


    Eigen::Matrix3d J();                        //!< Robot Jacobian matrix at current configuration
    VM3 directKinematic(VM3 q);                 //!< Apply robot direct kinematic model at configuration q (rad) and return end-effector position X (m)
    VM3 inverseKinematic(VM3 X);                //!< Apply robot inverse kinematic model at position X (m) and return corresponding configuration q (rad)
    VM3 calculateGravityTorques();              //!< Conpute gravity compensation torques for current configuration
    VM3 calculateEndEffAcceleration();          //!< Calculate end effector acceleration through differentiation of velocity filtered at 1Hz cutoff

    const VX& getEndEffPosition();             //!< Return vector containing end-effector position (in m)
    const VX& getEndEffVelocity();             //!< Return vector containing end-effector velocity (in m.s-1)
    const VX& getEndEffVelocityFiltered();     //!< Return vector containing end-effector velocity filtered (in m.s-1). Used to compute acceleration
    const VX& getEndEffAcceleration();         //!< Return vector containing end-effector acceleration (in m.s-2), calculated through differentiation of velocity filtered at 1Hz cutoff
    const VX& getEndEffForce();                //!< Return vector containing end-effector (motors) force (in N)
    const VX& getInteractionForce();           //!< Return vector containing end-effector interaction force (using model substracting gravity and friction force to motor torque) (in N)

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
