
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

#include <map>

#include "AlexJoint.h"
#include "AlexTrajectoryGenerator.h"
#include "CopleyDrive.h"
#include "Keyboard.h"
#include "Buttons.h"
#include "Robot.h"
#include "RobotParams.h"
//#include "SchneiderDrive.h"
#include "ALEXCrutchController.h"



#define ALEX_NUM_JOINTS 4

/**
 * An enum type.
 * Joint Index for the 4 joints (note, CANopen NODEID = this + 1)
 */
enum X2Joints {
    ALEX_LEFT_HIP = 0,   /**< Left Hip*/
    ALEX_LEFT_KNEE = 1,  /**< Left Knee*/
    ALEX_RIGHT_HIP = 2,  /**< Right Hip*/
    ALEX_RIGHT_KNEE = 3, /**< Right Knee*/
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
//ExoJointLimits X2JointLimits = {deg2rad(120), deg2rad(-30), deg2rad(120), deg2rad(0)};


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
   ALEXCrutchController pb;
   Buttons buttons;

   // Base class drive pointer: can be any type of derived driver class.
   std::vector<Drive *> motorDrives;

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
       * \brief Initialises all joints to torque control mode.
       *
       * \return true If all joints are successfully configured
       * \return false  If some or all joints fail the configuration
   */
   bool initTorqueControl();

   /** 
      * /brief For each joint, move through(send appropriate commands to joints) the currently 
      * generated trajectory of the TrajectoryGenerator object - this assumes the trajectory and robot is in position control. 
      *
      * /return true if successful
      * /return false if not successful (e.g. any joint not in position control.)
      */
   bool moveThroughTraj();

   /** 
      *  \brief Begin a new trajectory with the currently loaded trajectory paramaters. 
      * Using the <code>AlexRobot</code> current configuration (read in from joint objects) 
      * and the trajecotry generator object, generate and save a spline to move from current 
      * to desired position.
      * 
      */
   void startNewTraj();
   
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
   void
   freeMemory();
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