
/**
 *
 * \file RobotM3.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-06-16
 * \copyright Copyright (c) 2020
 *
 * \breif  The<code> RobotM3</ code> class represents an M3 Robot.
 *
 */

#ifndef RobotM3_H_INCLUDED
#define RobotM3_H_INCLUDED

#include <map>
#include <Eigen/Dense>

#include "JointM3.h"
#include "Keyboard.h"
#include "Robot.h"


/**
     * \todo Load in paramaters and dictionary entries from JSON file.
     *
     */

/**
 * \brief Implementation of the M3 robot class, representing an M3 using 3 JointM3 (and so Kinco drives).
 * model reference:
 *             2
 *      3       \
 *      /\       \(L2)
 * (L3)/  \       \
 *    /    \       \
 *   4      \      .\0
 *           \   .
 *          1\.  (L1)
 *
 */
class RobotM3 : public Robot {
   private:
    /**
     * \brief motor drive position control profile paramaters, user defined.
     *
     */
    motorProfile posControlMotorProfile{4000000, 240000, 240000};
    float LinkLengths[5] = {0.056, 0.15-0.015, 0.5, 0.465, 0.465+0.15-0.015}; /*!< Link lengths used for kniematic models */
    //float LinkMasses[4]= //TODO

    Eigen::Vector3d qCalibration = {38*M_PI/180., 70*M_PI/180., 95*M_PI/180.}; /*!< Calibration configuration: posture in which the robot is when using the calibration procedure */

    bool calibrated;

   public:
    /**
      * \brief Default <code>RobotM3</code> constructor.
      * Initialize memory for the Exoskelton <code>Joint</code> + sensors.
      * Load in exoskeleton paramaters to  <code>TrajectoryGenerator.</code>.
      */
    RobotM3();
    ~RobotM3();

    //Keyboard keyboard;

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
       * \brief Send a stop command to all joint drives.
       *
       * \return true If all joints are stopped
       * \return false  Otherwise
       */
    bool stop();

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
    * \param positions a vector of target positions - applicable for each of the actauted joints
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t applyVelocity(std::vector<double> velocity);


    /**
    * \brief Apply current configuration as calibration configuration using qcalibration such that:
    *  q=qcalibration in current configuration.
    */
    void applyCalibration();


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

    void printStatus();


    Eigen::Matrix3d J();
    Eigen::Vector3d directKinematic(Eigen::Vector3d q);
    Eigen::Vector3d inverseKinematic(Eigen::Vector3d X);

    Eigen::Vector3d getJointPos();
    Eigen::Vector3d getJointVel();
    Eigen::Vector3d getJointTor();
    Eigen::Vector3d getEndEffPos();
    Eigen::Vector3d getEndEffVel();
    Eigen::Vector3d getEndEffFor();

    setMovementReturnCode_t setJointPos(Eigen::Vector3d q);
    setMovementReturnCode_t setJointVel(Eigen::Vector3d q);
    setMovementReturnCode_t setJointTor(Eigen::Vector3d tau);
    setMovementReturnCode_t setEndEffPos(Eigen::Vector3d X);
    setMovementReturnCode_t setEndEffVel(Eigen::Vector3d dX);
    setMovementReturnCode_t setEndEffFor(Eigen::Vector3d F);



};
#endif /*RobotM3_H*/
