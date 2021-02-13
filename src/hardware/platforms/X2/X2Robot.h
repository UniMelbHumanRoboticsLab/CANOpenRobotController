
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

#include <Eigen/Dense>
#include <chrono>
#include <map>
#include <thread>
#include <csignal>

#include "CopleyDrive.h"
#include "Keyboard.h"
#include "Robot.h"
#include "FourierForceSensor.h"
#include "X2Joint.h"

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
/**
     * \todo Load in paramaters and dictionary entries from JSON file.
     *
     */

#define X2_NUM_JOINTS 4
#define X2_NUM_FORCE_SENSORS 4

// robot name is used to access the properties of the correct robot version
#define X2_NAME X2_MELB_A

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
    Eigen::VectorXd m; // masses of left thigh, left shank+foot, right thigh, right shank+foot [kg]
    Eigen::VectorXd l; // length of left thigh, left shank, right thigh, right shank [kg]
    Eigen::VectorXd s; // length from previous joint to CoM [m]
    Eigen::VectorXd I; // mass moment of inertia of left thigh, left shank+foot, right thigh, right shank+foot [kg.m^2]
    Eigen::VectorXd c0; // viscous fric constant of joints [N.s]
    Eigen::VectorXd c1; // coulomb friction const of joints [N.m]
    Eigen::VectorXd c2; // friction const related to sqrt of vel
    Eigen::VectorXd cuffWeights; // cuff Weights [N]
    Eigen::VectorXd forceSensorScaleFactor; // scale factor of force sensors [N/sensor output]
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

    RobotParameters x2Parameters;

    //Todo: generalise sensors
    Eigen::VectorXd interactionForces_;

    std::string robotName_;

    bool initializeRobotParams(std::string robotName);

    static void signalHandler(int signum);

#ifdef SIM
    ros::NodeHandle* nodeHandle_;

    ros::Publisher positionCommandPublisher_;
    ros::Publisher velocityCommandPublisher_;
    ros::Publisher torqueCommandPublisher_;
    ros::Subscriber jointStateSubscriber_;
    ros::ServiceClient controllerSwitchClient_;

    std_msgs::Float64MultiArray positionCommandMsg_;
    std_msgs::Float64MultiArray velocityCommandMsg_;
    std_msgs::Float64MultiArray torqueCommandMsg_;
    controller_manager_msgs::SwitchController controllerSwitchMsg_;

    void jointStateCallback(const sensor_msgs::JointState& msg);
#endif
#ifdef NOROBOT
    Eigen::VectorXd simJointPositions_;
    Eigen::VectorXd simJointVelocities_;
    Eigen::VectorXd simJointTorques_;
#endif

   public:
    /**
      * \brief Default <code>ExoRobot</code> constructor.
      * Initialize memory for the Exoskelton <code>Joint</code> + sensors.
      * Load in exoskeleton paramaters to  <code>TrajectoryGenerator.</code>.
      */
    X2Robot(std::string robotName = XSTR(X2_NAME));
    ~X2Robot();
    Keyboard* keyboard;
    std::vector<Drive*> motorDrives;
    std::vector<FourierForceSensor*> forceSensors;

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
    * \brief Get the latest joints position
    *
    * \return Eigen::VectorXd a reference to the vector of actual joint positions
    */
    Eigen::VectorXd& getPosition();

    /**
    * \brief Get the latest joints velocity
    *
    * \return Eigen::VectorXd a reference to the vector of actual joint positions
    */
    Eigen::VectorXd& getVelocity();

    /**
    * \brief Get the latest joints torque
    *
    * \return Eigen::VectorXd a reference to the vector of actual joint positions
    */
    Eigen::VectorXd& getTorque();

    /**
    * \brief Get the interaction force from each force sensor
    *
    * \return Eigen::VectorXd a vector of interaction forces
    */
    Eigen::VectorXd& getInteractionForce();

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
    bool homing(std::vector<int> homingDirection = std::vector<int>(X2_NUM_JOINTS, 1), float thresholdTorque = 50.0,
                float delayTime = 0.2, float homingSpeed = 5 * M_PI / 180.0, float maxTime = 30.0);

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
       * \brief Changes the mode of
       *
       * \return true if successful
       * \return false if not (joints/drive not enabled or in correct mode)
       */
    bool setPosControlContinuousProfile(bool continuous);
    /**
       * \brief returns the feedforward torque to compensate for the gravitational and frictional elements
       *
       * \param motionIntend intended direction of the motion. Used to calculate the static friction
       */
    Eigen::VectorXd getFeedForwardTorque(int motionIntend);

    /**
       * \brief sets the Robot name. This name is used to choose the proper set of parameters
       *
       * \param std::string robotName name of the robot
       */
    void setRobotName(std::string robotName);

    /**
       * \brief get the robot name
       */
    std::string& getRobotName();

    /**
       * \brief returns the parameters of the robot
       */
    RobotParameters& getRobotParameters();

#ifdef SIM
    /**
       * \brief method to pass the nodeHandle. Only available in SIM mode
       */
    void setNodeHandle(ros::NodeHandle& nodeHandle);
    /**
       * \brief Initialize ROS services, publisher ans subscribers
      */
    void initialiseROS();
#endif
};
#endif /*EXOROBOT_H*/
