/**
 *
 * \file X2Robot.h
 * \author Emek Baris Kucuktabak, Justin Fong
 * \version 1.1
 * \date 2022-02-22
 * \copyright Copyright (c) 2022
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
#include "TechnaidIMU.h"
#include "FourierHandle.h"

// Logger
#include "LogHelper.h"
// yaml-parser
#include <fstream>
#include "yaml-cpp/yaml.h"

// These are used to access the MACRO: BASE_DIRECTORY
#define XSTR(x) STR(x)
#define STR(x) #x

#ifdef SIM
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#endif
/**
     * \todo Load in paramaters and dictionary entries from JSON file.
     *
     */

#define X2_NUM_JOINTS 4
#define X2_NUM_GENERALIZED_COORDINATES 5
#define X2_NUM_FORCE_SENSORS 4
#define X2_NUM_GRF_SENSORS 2
#define X2_REDUCTION_RATIO 122.5

// robot name is used to access the properties of the correct robot version
#define X2_NAME_DEFAULT X2_MELB_A

// Macros
#define deg2rad(deg) ((deg)*M_PI / 180.0)
#define rad2deg(rad) ((rad)*180.0 / M_PI)

using std::placeholders::_1;

/**
 * Structure which is used for joint limits. Defines minimum and maximum limits of the each joint
 *
 */
struct JointPositionLimits {
    double hipMax;
    double hipMin;
    double kneeMax;
    double kneeMin;
};

struct RobotParameters {
    Eigen::VectorXd m; // masses of upper thigh, lower thigh, upper shank, lower shank, foot [kg]
    Eigen::VectorXd l; // length of left thigh, left shank, right thigh, right shank [m]
    Eigen::VectorXd s; // hip to upper thigh COM, knee to lower thigh COM, knee to upper shank COM, ankle to lower shank COM, ankle to foot COM [m]
    Eigen::VectorXd I; // mass moment of inertia of upper thigh, lower thigh, upper shank, lower shank, foot [kg.m^2]
    Eigen::VectorXd G; // apparent rotor inertias [kg. m^2]
    Eigen::VectorXd c0; // viscous fric constant of joints [N.s]
    Eigen::VectorXd c1; // coulomb friction const of joints [N.m]
    Eigen::VectorXd c2; // friction const related to sqrt of vel
    Eigen::VectorXd cuffWeights; // cuff Weights [N]
    Eigen::VectorXd forceSensorScaleFactor; // scale factor of force sensors [N/sensor output]
    Eigen::VectorXd grfSensorScaleFactor; // scale factor of GRF sensors [N/sensor output]
    Eigen::VectorXd grfSensorThreshold; // scale factor of GRF sensors [N/sensor output]
    IMUParameters imuParameters;
    double maxTorque;
//    double maxPower;
    double maxVelocity;
    Eigen::VectorXd imuDistance;
    JointPositionLimits jointPositionLimits;
    // below parameters are calculated by using the given above parameters.
    double mThigh; // mass of thigh [kg]
    double mShank; // mass of shank [kg]
    double mBackpack;  // mass of backpack and hip drives [kg]
    double sThighLeft; // CoM of left thigh measured from the hip joint [m]
    double sThighRight; // CoM of right thigh measured from the hip joint [m]
    double sShankLeft; // CoM of left shank measured from the knee joint [m]
    double sShankRight; // CoM of right shank measured from the knee joint [m]
    double LThighLeft; // mass moment of inertia of left thigh at COM
    double LThighRight; // mass moment of inertia of right thigh at COM
    double LShankLeft;  // mass moment of inertia of left shank at COM
    double LShankRight;  // mass moment of inertia of right shank at COM
    double LBackpack; // mass moment of inertia of backpack

};

struct AccelerationFuseData {
    Eigen::VectorXd jointAccByDerivative;
    Eigen::VectorXd filteredJointAccByDerivative;
    Eigen::VectorXd jointAccByIMU;
    Eigen::VectorXd filteredJointAccByImu;
    Eigen::VectorXd filteredBias;
    Eigen::VectorXd mergedJointAcc;
};

enum GaitState {
    UNDEFINED = 0,
    LEFT_STANCE = 1,
    RIGHT_STANCE = 2,
    DOUBLE_STANCE = 3,
    FLYING = 4,
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
    ControlMode controlMode;

    double dt_ = 0.003; // 0.003 todo: pass this information from main

    //Todo: generalise sensors
    Eigen::VectorXd jointTorquesViaStrainGauges_; // measured joint torques from strain gauges

    Eigen::VectorXd interactionForces_;
    Eigen::VectorXd smoothedInteractionForces_;
    Eigen::VectorXd previousSmoothedInteractionForces_;

    Eigen::VectorXd groundReactionForces_;
    Eigen::VectorXd backpackQuaternions_; // x y z w
    Eigen::VectorXd backpackGyroData_; // x y z
    Eigen::VectorXd contactAnglesOnMedianPlane_; // angles of each link wrt gravity vector
    double backPackAngleOnMedianPlane_; // backpack angle wrt gravity vector. leaning front is positive [rad]
    double backPackAngularVelocityOnMedianPlane_;
    double previousBackPackAngularVelocityOnMedianPlane_;

    Eigen::VectorXd generalizedAccByDerivative_;
    Eigen::VectorXd filteredGeneralizedAccByDerivative_; // filtered joint acc that are estimated by taking derivative of joint velocity
    Eigen::VectorXd previousFilteredGeneralizedAccByDerivative_; // filtered joint acc that are estimated by taking derivative of joint velocity
    Eigen::VectorXd estimatedGeneralizedAcceleration_;
    Eigen::VectorXd previousJointVelocities_;
    double jointVelDerivativeCutOffFreq_;
    double backpackVelDerivativeCutOffFreq_;
    double dynamicParametersCutOffFreq_;

    GaitState gaitState_;

    Eigen::MatrixXd massMatrix_;

    Eigen::VectorXd gravitationTorque_;

    Eigen::VectorXd corriolisTorque_;
    Eigen::VectorXd frictionTorque_;
    Eigen::VectorXd feedForwardTorque_; // t_ff = g + b + friction
    Eigen::MatrixXd selectionMatrix_, pseudoInverseOfSelectionMatrixTranspose_;

    int numberOfIMUs_;

    std::chrono::steady_clock::time_point time0;

    bool loadParametersFromYAML(YAML::Node params);

    static void signalHandler(int signum);

    /**
    * \brief Get backpack quaternion
    *
    * \return Eigen::VectorXd qx, qy, qz, qw
    */
    Eigen::VectorXd getBackpackQuaternions();

    /**
    * \brief Get backpack gyro data
    *
    * \return Eigen::VectorXd wx, wy, wz
    */
    Eigen::VectorXd getBackpackGyroData();

    /**
    * \brief updates the angle of back pack and cuffs with respect to - gravity vector on median plane. leaning front is positive
    */
    void updateBackpackAndContactAnglesOnMedianPlane();

    /**
    * \brief updates the angular velocity of backpack along the axes paralles with joint axes
    */
    void updateBackpackAngularVelocity();

    /**
    * \brief updates the joint torque strain gauge and grf measurements
    */
    void updateForceMeasurements();

    /**
    * \brief updates the interaction force measurements
    */
    void updateInteractionForce();

    /**
       * \brief update the estimate of generalized accelerations by filtering the time derivative of the joint velocties
       */
    void updateGeneralizedAcceleration();

    /**
       * \brief update Mass matrix, gravity vector and Coriollis vector
       */
    void updateDynamicTerms();

    /**
       * \brief update friction torque
       */
    void updateFrictionTorque(Eigen::VectorXd motionIntend);

    /**
       * \brief update feedForwardTorque
       */
    void updateFeedforwardTorque();


#ifdef SIM
    std::shared_ptr<rclcpp::Node> node;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr positionCommandPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocityCommandPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torqueCommandPublisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controllerSwitchClient_;

    std_msgs::msg::Float64MultiArray positionCommandMsg_;
    std_msgs::msg::Float64MultiArray velocityCommandMsg_;
    std_msgs::msg::Float64MultiArray torqueCommandMsg_;
    std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> controllerSwitchMsg_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
#endif
#ifdef NOROBOT
    Eigen::VectorXd simJointPositions_;
    Eigen::VectorXd simJointVelocities_;
    Eigen::VectorXd simJointTorques_;
    Eigen::VectorXd simJointTorquesViaStrainGauges_;
    Eigen::VectorXd simInteractionForces_;
    Eigen::VectorXd simGroundReactionForces_;
    double simBackPackAngleOnMedianPlane_;
    double simBackPackAngularVelocityOnMedianPlane_;
    Eigen::VectorXd simContactAnglesOnMedianPlane_;
#endif

public:
    /**
      * \brief Default <code>ExoRobot</code> constructor.
      * Initialize memory for the Exoskelton <code>Joint</code> + sensors.
      * Load in exoskeleton paramaters to  <code>TrajectoryGenerator.</code>.
      */

#ifdef SIM
    X2Robot(std::shared_ptr<rclcpp::Node> &node, std::string robotName = XSTR(X2_NAME_DEFAULT), std::string yaml_config_file="x2_params.yaml");
#else
    X2Robot(std::string robotName = XSTR(X2_NAME_DEFAULT), std::string yaml_config_file="x2_params.yaml");
#endif
    ~X2Robot();
    Keyboard* keyboard;
    std::vector<Drive*> motorDrives;
    std::vector<FourierForceSensor*> forceSensors;
    FourierHandle* buttons;
    TechnaidIMU* technaidIMUs;

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
    * \brief Get the latest
    *
    * \return Eigen::VectorXd a reference to the vector of strain gauge measurements
    */
    Eigen::VectorXd& getJointTorquesViaStrainGauges();

    /**
    * \brief Get the interaction force estimation
    *
    * \return Eigen::VectorXd a vector of interaction force estimation
    */
    Eigen::VectorXd& getInteractionForce();

    /**
    * \brief Get the smoothed interaction force estimation
    *
    * \return Eigen::VectorXd a vector of smoothed interaction force estimation
    */
    Eigen::VectorXd& getSmoothedInteractionForce();

    /**
    * \brief Get ground reaction forces from each force sensor
    *
    * \return Eigen::VectorXd a vector of ground reaction forces
    */
    Eigen::VectorXd& getGroundReactionForces();

    /**
    * \brief Get estimated joint acceleration
    *
    * \return Eigen::VectorXd a vector of estimated joint Accelerations
    */
    Eigen::VectorXd& getEstimatedGeneralizedAcceleration();

    /**
    * \brief Get mass matrix
    */
    Eigen::MatrixXd& getMassMatrix();

    /**
    * \brief Get selection matrix
    */
    Eigen::MatrixXd& getSelectionMatrix();

    /**
    * \brief Get gravitation Torque
    */
    Eigen::VectorXd& getGravitationTorque();

    /**
    * \brief Get corriolis Torque
    */
    Eigen::VectorXd& getCorriolisTorque();

    /**
    * \brief  get friction Torque
    * \param motionIntend vector of direction of motion intend to calculate coulomb friction at static case
    */
    Eigen::VectorXd& getFrictionTorque();

    /**
    * \brief get feedforward Torque
    */
    Eigen::VectorXd& getFeedForwardTorque();

    /**
    * \brief Get the backpack angle on median plane with respect to - gravity axes
    *
    * \return double& reference to backpack angle
    */
    double& getBackPackAngleOnMedianPlane();

    /**
    * \brief Get the backpack angle on median plane with respect to - gravity axes
    *
    * \return double& reference to backpack angular velocity on median plane
    */
    double& getBackPackAngularVelocityOnMedianPlane();

    /**
* \brief Get the contact angle on median plane with respect to - gravity axes
*
* \return double& reference to contact angles
*/
    Eigen::VectorXd& getContactAnglesOnMedianPlane();

    /**
    * Returns the lastest updated button reading.
    *
    */
    double& getButtonValue(ButtonColor buttonColor);

    /**
    * Returns the pseudo inverse of selection matrix transpose
    *
    */
    Eigen::MatrixXd getPseudoInverseOfSelectionMatrixTranspose();

    /**
    * Returns the control mode
    *
    */
    ControlMode& getControlMode();

    /**
    * \brief Get the variables related to the acceleration merge
    *
    * \return AcelerationFuseData
    */
    AccelerationFuseData& getAccelerationFuseData();

    /**
       * \brief returns the parameters of the robot
       */
    RobotParameters& getRobotParameters();

    /**
    * Returns sensor tresholds
    */
    Eigen::VectorXd& getGRFSensorThresholds();

    /**
    * Returns joint velocity derivative cut off frequency
    */
    double& getJointVelDerivativeCutOffFrequency();

    /**
    * Returns backpack angular velocity derivative cut off frequency
    */
    double& getBackpackVelDerivativeCutOffFrequency();

    /**
    * Returns dynamic parameters cut off frequency cut off frequency
    */
    double& getDynamicParametersCutOffFrequency();

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
    * \brief Set the backpack IMU Mode
    *
    */
    bool setBackpackIMUMode(IMUOutputMode imuOutputMode);

    /**
    * \brief Set the backpack IMU Mode
    *
    */
    bool setContactIMUMode(IMUOutputMode imuOutputMode);


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
    void updateRobot(bool duringHoming = false);

    /**
       * \brief Changes the mode of
       *
       * \return true if successful
       * \return false if not (joints/drive not enabled or in correct mode)
       */
    bool setPosControlContinuousProfile(bool continuous);

    /**
       * \brief sets the Robot name. This name is used to choose the proper set of parameters
       *
       * \param std::string robotName name of the robot
       */
    void setRobotName(std::string robotName);

    /**
       * \brief set the cut off frequency of the low pass filtered joint vel derivative
       *
       * \param double cutOffFrequency in Hz
       */
    void setJointVelDerivativeCutOffFrequency(double cutOffFrequency);

    /**
       * \brief set the cut off frequency of the low pass filtered backpack angular vel derivative
       *
       * \param double cutOffFrequency in Hz
       */
    void setBackpackVelDerivativeCutOffFrequency(double cutOffFrequency);

    /**
       * \brief set the cut off frequency of the estimated bias between IMU and joint acceleration
       *
       * \param double cutOffFrequency in Hz
       */
    void setAccBiasCutOffFrequency(double cutOffFrequency);

    /**
       * \brief set the cut off frequency of the low pass filtered gravity corrected IMU values
       *
       * \param double cutOffFrequency in Hz
       */
    void setIMUCutOffFrequency(double cutOffFrequency);

    /**
       * \brief set the cut off frequency of the low pass filtered dynamic parameters such as g vector or M matrix
       *
       * \param double cutOffFrequency in Hz
       */
    void setDynamicParametersCutOffFrequency(double cutOffFrequency);

    /**
   * \brief set the boolean variable to track if the gait state is set manually from the GUI or automatically decided
   *
   * \param double isGaitManuallySet
   */
    void setGRFSensorsThreshold(Eigen::VectorXd thresholds);

    /**
       * \brief check if the current state of the robot is safe or if emergency button is pressed
       */
    bool safetyCheck(bool duringHoming);

    /**
       * \brief get the robot name
       */
    std::string& getRobotName();

#ifdef SIM
    /**
       * \brief method to pass the nodeHandle. Only available in SIM mode
       */
    void setNodeHandle(std::shared_ptr<rclcpp::Node> &node);
    /**
       * \brief Initialize ROS services, publisher ans subscribers
      */
    void initialiseROS(std::shared_ptr<rclcpp::Node> &node);
#endif
};
#endif /*EXOROBOT_H*/
