/**
 *
 * \file RobotFITHVExo.h
 * \author Vincent Crocher
 * \version 0.2
 * \date 2025-04-16
 * \copyright Copyright (c) 2020
 *
 * \brief Class representing an FIT-HV exoskeleton with two dof for hip flex/extension
 *
 */

#ifndef RobotFITHV_H
#define RobotFITHV_H


#include "JointFITHVExo.h"
#include "FourierForceSensor.h"
#include "Keyboard.h"
#include "Joystick.h"
#include "Robot.h"


typedef Eigen::Vector2d V2; //! Convenience alias for double  Vector of length 2
typedef Eigen::VectorXd VX; //!< Generic (dynamic) size version required for compatibility w/ other libraries (FLNL)

/**
 * \brief Class representing an FIT-HV exoskeleton with two dof for hip flex/extension
 */
class RobotFITHVExo: public Robot
{
private:
    //TODO: update all max values
    double dqMax = 360 * M_PI / 180.;                     //!< Max joint speed (rad.s-1)
    double tauMax = 100;                                  //!< Max joint torque (Nm)
    //std::vector<double> iPeakDrives = {42.0, 42.0};     //!< Drive max current
    //std::vector<double> motorCstt = {0.132, 0.132};     //!< Motor constants
    std::vector<double> qSigns = {-1., 1.};               //!< Joint direction (as compared to built-in drives direction). Set for extension +
    std::vector<double> linkLengths = {0.27, 0.27};       //!< Link lengths used for kinematic models (in m) and mass compensation (i.e. center of mass pos). Distance from hip center to passive joint center.
    std::vector<double> massCoeff = {0.0, 0.0};           //!< Mass coefficients (identified) used for gravity compensation (in kg). Equivalent mass at distance linkLengths from hip.
    std::vector<double> frictionVis = {0., 0.};           //!< Joint viscous friction compensation coefficients
    std::vector<double> frictionCoul = {0., 0.};          //!< Joint Coulomb (static) friction compensation coefficients

    std::vector<double> qLimits = { /*q1_min*/ -40 * M_PI / 180.,/*q1_max*/ 135 * M_PI / 180.,
                                    /*q2_min*/ -40 * M_PI / 180., /*q2_max*/135 * M_PI / 180. }; //!< Joints limits (in rad)
    std::vector<double> qCalibration = {0, 0.};  //!< Calibration configuration: posture in which the robot is when using the calibration procedure
    bool calibrated;

    /**
     * \brief motor drive position control profile paramaters, user defined. This should not be her but in JointFITHVExo at some point.
     *
     */
    motorProfile posControlMotorProfile{4000000, 240000, 240000};

public:
    /**
        * \brief Default RobotFITHVExo constructor.
        * Initialize memory for the Exoskelton <code>Joint</code> + sensors.
        * Load in exoskeleton paramaters to  <code>TrajectoryGenerator.</code>.
        */
    RobotFITHVExo(std::string robot_name="", std::string yaml_config_file="");
    ~RobotFITHVExo();

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

    bool isCalibrated()
    {
        return calibrated;
    }
    void decalibrate()
    {
        calibrated = false;
    }


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

    void printJointStatus();

    setMovementReturnCode_t setJointPosition(V2 q);
    setMovementReturnCode_t setJointVelocity(V2 dq);
    setMovementReturnCode_t setJointTorque(V2 tau);
    //TODO convenience torque function with friction comp
    //TODO: grav compensation based on IMU??
    //setMovementReturnCode_t setEndEffForceWithCompensation(V2 F, bool friction_comp=true);
};
#endif
