
/**
 * 
 * \file AlexRobot.h
 * \author William Campbell 
 * \version 0.1
 * \date 2020-06-10
 * \copyright Copyright (c) 2019
 * 
 * \breif  The<code> AlexRobot</ code> class represents an ExoSkeleton Robot in terms of its
 * representation of the Alex exoskeleton hardware whose memory is managed in this class.
 *
 *
 * Version 0.1
 * Date: 07/04/2020
 */

#ifndef AlexRobot_H_INCLUDED
#define AlexRobot_H_INCLUDED

#include <time.h>

#include <csignal>
#include <map>
#include <thread>

#include "AlexJoint.h"
#include "AlexTrajectoryGenerator.h"
#include "Buttons.h"
#include "CopleyDrive.h"
#include "Keyboard.h"
#include "Robot.h"
#include "RobotParams.h"
//#include "SchneiderDrive.h"
#include "ALEXCrutchController.h"

// Logger
#include "spdlog/helper/LogHelper.h"
// yaml-parser
#include <fstream>
#include "yaml-cpp/yaml.h"

// These are used to access the MACRO: BASE_DIRECTORY
#define XSTR(x) STR(x)
#define STR(x) #x

#ifdef SIM
#include "controller_manager_msgs/SwitchController.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#endif


// robot name is used to access the properties of the correct robot version
#define X2_NAME X2_ALEX

// Macros
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

struct RobotParameters {
    Eigen::VectorXd m;                       // masses of left thigh, left shank+foot, right thigh, right shank+foot [kg]
    Eigen::VectorXd l;                       // length of left thigh, left shank, right thigh, right shank [kg]
    Eigen::VectorXd s;                       // length from previous joint to CoM [m]
    Eigen::VectorXd I;                       // mass moment of inertia of left thigh, left shank+foot, right thigh, right shank+foot [kg.m^2]
    Eigen::VectorXd c0;                      // viscous fric constant of joints [N.s]
    Eigen::VectorXd c1;                      // coulomb friction const of joints [N.m]
    Eigen::VectorXd c2;                      // friction const related to sqrt of vel
    Eigen::VectorXd cuffWeights;             // cuff Weights [N]
    Eigen::VectorXd forceSensorScaleFactor;  // scale factor of force sensors [N/sensor output]
};

/**
 * Paramater definitions: Hip motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
JointDrivePairs hipJDP{
    250880,       // drivePosA
    0,            // drivePosB
    deg2rad(90),  //jointPosA
    deg2rad(0)    //jointPosB
};
/**
 * Paramater definitions: Knee motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
JointDrivePairs kneeJDP{
    250880,       // drivePosA
    0,            //drivePosB
    deg2rad(90),  //jointPosA
    deg2rad(0)    //jointPosB
};

/**
 * Defines the Joint Limits of the X2 Exoskeleton
 *
 */
ExoJointLimits AlexJointLimits = {deg2rad(120), deg2rad(-30), deg2rad(120), deg2rad(0)};

/**
     * \todo Load in paramaters and dictionary entries from JSON file.
     * 
     */


/**
 * \brief Example implementation of the Robot class, representing an X2 Exoskeleton, using DummyActuatedJoint and AlexTrajectoryGenerator.
 * 
 */
class AlexRobot : public Robot
{
private:
   /** Parameters associated with Trajectory Progression */
   double currTrajProgress = 0;
   double currTrajTime; /*currently loaded trajectories total time of execution, must be set before begining a trajectory*/
   timespec prevTime;
   /*Flag for loading in new trajectories only after a green button release*/
   bool resetTrajectory;
   /**
     * \brief motor drive position control profile paramaters, user defined.
     * 
     */

   motorProfile posControlMotorProfile{4000000, 190000, 190000};
   motorProfile velControlMotorProfile{0, 240000, 240000};

   std::string robotName_;
   RobotParameters x2Parameters;

  UNSIGNED8 currentState; // Static Cast to AlexState
  UNSIGNED8 currentMovement; // Static Cast to RobotMode
  
  static void signalHandler(int signum);

  public:
   AlexRobot();
   /**
      * \brief Default <code>AlexRobot</code> constructor.
      * Initialize memory for the Exoskelton <code>Joint</code> + sensors. 
      * Load in exoskeleton paramaters to  <code>TrajectoryGenerator.</code>.
      */
   AlexRobot(AlexTrajectoryGenerator *tj);
   ~AlexRobot();
   AlexTrajectoryGenerator *trajectoryGenerator;
   Keyboard *keyboard;
   ALEXCrutchController *pb;
   Buttons buttons;

   // Base class drive pointer: can be any type of derived driver class.
   std::vector<Drive *> motorDrives;

   // /**
   //  * \brief Timer Variables for moving through trajectories
   //  *
   //  */
   struct timeval tv, tv_diff, moving_tv, tv_changed, stationary_tv, start_traj, last_tv;
   
   /**
       * \brief Clears (or attempts to clear) errors on the motor drives.
       */
   void resetErrors();

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
       * \brief Sets the position control profile to be continuous (i.e. movements do not to complete before a new command is issued) or not
       * 
       * \return true if successful
       * \return false if not (joints/drive not enabled or in correct mode)
       */
   bool setPosControlContinuousProfile(bool continuous);

   /** 
      *  \brief Begin a new trajectory with the currently loaded trajectory paramaters. 
      * Using the <code>AlexRobot</code> current configuration (read in from joint objects) 
      * and the trajecotry generator object, generate and save a spline to move from current 
      * to desired position.
      * 
      */
   void startNewTraj();

   /** 
      * /brief For each joint, move through(send appropriate commands to joints) the currently 
      * generated trajectory of the TrajectoryGenerator object - this assumes the trajectory and robot is in position control. 
      *
      * /return true if successful
      * /return false if not successful (e.g. any joint not in position control.)
      */
   bool moveThroughTraj();

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
   bool homing(std::vector<int> homingDirection = std::vector<int>(ALEX_NUM_JOINTS, 1), float thresholdTorque = 50.0,
               float delayTime = 0.2, float homingSpeed = 5 * M_PI / 180.0, float maxTime = 30.0);

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
    * \brief Get the latest joints position
    *
    * \return Eigen::VectorXd a reference to the vector of actual joint positions
    */
   Eigen::VectorXd &getPosition();

   /**
    * \brief Get the latest joints velocity
    *
    * \return Eigen::VectorXd a reference to the vector of actual joint positions
    */
   Eigen::VectorXd &getVelocity();

   /**
    * \brief Get the latest joints torque
    *
    * \return Eigen::VectorXd a reference to the vector of actual joint positions
    */
   Eigen::VectorXd &getTorque();

   
#ifdef VIRTUAL
   /**
    * \brief Virtual variant to 'moveThroughTraj' function, used to manually set target motor positions when running the exo in simulation
    * 
    * \param angle 
    */
   void setVirtualPosition(double angle);
#endif

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
       * \brief getter method for currentTrajectory progress variable.
       *
       * \return double currentTrajProgress
       */
   double getCurrTrajProgress();

   /**
    * \brief Get each joints state (angle) and return in a vector
    * 
    * \return std::vector<double> 
    */
   std::vector<double> getJointStates();
   /**
 * \brief Set the Next Motion object
 * 
 * @param nextMotion 
 */
   void setNextMotion(RobotMode nextMotion);
   /**
 * \brief Get the Next Motion OD entry
 * 
 * \return RobotMode 
 */
   RobotMode getNextMotion();
   /**
 * \brief Set the Current Motion object from the od.nextMotion entry
 * 
 * \return current RobotMode
 * 
 */
   void setCurrentMotion(RobotMode nextMotion);
   /**
 * \brief Get the Current Motion OD entry
 * 
 * \return RobotMode 
 */
   RobotMode getCurrentMotion();
   /**
     * \brief Get the Go OD entry
     * 
     * \return int 
     */
   bool getGo();
   /**
 * \brief Set the Current State object
 * 
 * @param state 
 */
   void setCurrentState(AlexState state);
   /**
 * \brief set Entry flag value
 * 
 * @param value 
 */
   void setResetFlag(bool value);
   /**
 * \brief get Entry Flag value
 * 
 * \return true 
 * \return false 
 */
   bool getResetFlag();
   /**
    * \todo Move jointMinMap and jointMaxMap to RobotParams.h
    * 
    */

   /**
    * \brief disable all joints of the robot, returns true if successful
    * 
    */
   bool disableJoints();


  bool initializeRobotParams(std::string robotName);

};
#endif /*AlexRobot_H*/