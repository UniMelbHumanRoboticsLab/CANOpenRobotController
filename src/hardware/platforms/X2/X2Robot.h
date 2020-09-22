
/**
 * 
 * \file X2Robot.h
 * \author Justin Fong 
 * \version 0.1
 * \date 2020-05-17
 * \copyright Copyright (c) 2020
 * 
 * \breif  The<code> X2Robot</ code> class is defined to interface with Fourier Intelligence's X2 (or H4)
 * ExoMotus products. 
 *
 */

#ifndef X2ROBOT_H_INCLUDED
#define X2ROBOT_H_INCLUDED

#include <map>

#include "CopleyDrive.h"
#include "Keyboard.h"
#include "Robot.h"
//#include "RobotParams.h"
#include "X2Joint.h"
#include "X2ForceSensor.h"
#include <Eigen/Dense>

#include <chrono>
#include <thread>
/**
     * \todo Load in paramaters and dictionary entries from JSON file.
     *
     */

#define X2_NUM_JOINTS 4
#define X2_NUM_FORCE_SENSORS 4

// Macros
#define M_PI 3.14159265358979323846264338327950288
#define deg2rad(deg) ((deg)*M_PI / 180.0)
#define rad2deg(rad) ((rad)*180.0 / M_PI)

/**
 * Structure which is used for joint limits. Defines minimum and maximum limits of the each joint
 *
 */
struct ExoJointLimits {
    double hipMax;
    double hipMin;
    double kneeMax;
    double kneeMin;
};

/**
 * \brief Example implementation of the Robot class, representing an X2 Exoskeleton.
 *
 */
class X2Robot : public Robot {
   private:
    /**
     * \brief motor drive position control profile paramaters, user defined.
     *
     */
    motorProfile posControlMotorProfile{4000000, 240000, 240000};
    motorProfile velControlMotorProfile{0, 240000, 240000};

   public:
    /**
      * \brief Default <code>ExoRobot</code> constructor.
      * Initialize memory for the Exoskelton <code>Joint</code> + sensors.
      * Load in exoskeleton paramaters to  <code>TrajectoryGenerator.</code>.
      */
    X2Robot();
    ~X2Robot();
    Keyboard keyboard;
    std::vector<Drive *> motorDrives;
    std::vector<X2ForceSensor *> forceSensors;

    // /**
    //  * \brief Timer Variables for moving through trajectories
    //  *
    //  */
    struct timeval tv, tv_diff, moving_tv, tv_changed, stationary_tv, start_traj, last_tv;

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
      * \brief For each joint, move through(send appropriate commands to joints) the currently
      * generated trajectory of the TrajectoryGenerator object - this assumes the trajectory and robot is in position control.
      *
      * \return true if successful
      * \return false if not successful (e.g. any joint not in position control.)
      */
    bool moveThroughTraj();

    /**
    * \brief Set the target positions for each of the joints
    *
    * \param positions a vector of target positions - applicable for each of the actuated joints
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t setPosition(Eigen::VectorXd positions);

    /**
    * \brief Set the target velocities for each of the joints
    *
    * \param velocities a vector of target velocities - applicable for each of the actuated joints
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t setVelocity(Eigen::VectorXd velocities);

    /**
    * \brief Set the target torque for each of the joints
    *
    * \param torques a vector of target torques - applicable for each of the actuated joints
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t setTorque(Eigen::VectorXd torques);

    /**
    * \brief Get the actual position of each joint
    *
    * \return Eigen::VectorXd a vector of actual joint positions
    */
    Eigen::VectorXd getPosition();

    /**
    * \brief Get the actual velocity of each joint
    *
    * \return Eigen::VectorXd a vector of actual joint positions
    */
    Eigen::VectorXd getVelocity();

    /**
    * \brief Get the actual torque of each joint
    *
    * \return Eigen::VectorXd a vector of actual joint positions
    */
    Eigen::VectorXd getTorque();

    /**
    * \brief Get the interaction force from each force sensor
    *
    * \return Eigen::VectorXd a vector of interaction forces
    */
    Eigen::VectorXd getInteractionForce();

    /**
    * \brief Calibrate force sensors
    *
    * \return bool success of calibration
    */
    bool calibrateForceSensors();

    /**
    * \brief Homing procedure of joint
    *
    * \param homingDirection a vector of int whose sign indicate homing direction. If 0 skips that joint
    * \param thresholdTorque torque to understand [Nm]
    * \param delayTime time required for the actual torque being larger than thresholdTorque to identify hardstops [s]
    * \param homingSpeed velocity used during homing [rad/s]
    * \param maxTime maximum time to complete the homing [s]
    * \return bool success of homing
    */
    bool homing(std::vector<int> homingDirection = std::vector<int>(X2_NUM_JOINTS, 1), float thresholdTorque = 45.0,
                float delayTime = 0.2, float homingSpeed = 5*M_PI/180.0, float maxTime = 30.0);

    /**
   * Determine if the currently generated trajectory is complete.
   * \return bool
   */
    bool isTrajFinished();

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
       * \brief Free robot objects vector pointer memory.
       */
    void freeMemory();
    /**
       * \brief update current state of the robot, including input and output devices. 
       * Overloaded Method from the Robot Class. 
       * Example. for a keyboard input this would poll the keyboard for any button presses at this moment in time.
       */
    void updateRobot();
    /**
       * \brief disable the drives of the X2 robot
       * \return bool
       */
    bool disable();

};
#endif /*EXOROBOT_H*/