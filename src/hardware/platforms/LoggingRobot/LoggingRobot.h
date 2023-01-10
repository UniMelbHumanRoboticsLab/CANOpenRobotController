/**
 *
 * \file LogginRobot.h
 * \author Justin Fong
 * \version 0.1
 * \date 2020-12-01
 * \copyright Copyright (c) 2020
 *
 * \brief  The LoggingRobot class represents a logging robot. This Robot doesn't actually do anything, except give options to act as a logger.
 * 
 * It has two functions: 1) To trigger data acquision devices not associate with the operation of the robot, and 2) to log data from existing devices
 *
 */

#ifndef LOGGINGROBOT_H_INCLUDED
#define LOGGINGROBOT_H_INCLUDED

#include "Keyboard.h"
#include "Robot.h"
#include "RobotousRFT.h"
#include "HX711.h"

class LoggingRobot : public Robot {
   private:
    int numJoints = 4;
    // -- Variables assocaited with standalone sensors -- //
    std::vector<RobotousRFT *> crutchSensors;
    Eigen::VectorXd crutchReadings;  //6xN Vector containing all crutch readings

    // -- Variables assoacited with parameters already transmitted from the robot -- //
    std::vector<RPDO *> rpdos;

    // Motor Positions, Velocity
    // Motor Torques
    // Target Motor Positions
    // Status Words
    Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1> motorPositions;
    Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1> motorVelocities;
    Eigen::Matrix<INTEGER16, Eigen::Dynamic, 1> motorTorques;
    Eigen::Matrix<INTEGER16, Eigen::Dynamic, 1> motorStatusWords;

    // Variables from the ALEX Crutch Controller
    // Go Button
    // Next Motion (current Motion)
    INTEGER16 goButton;
    INTEGER16 nextMotion;

    // Variables from the ALEX Robot
    // Current State
    // Current Motion
    // Might need an additional about trajectory progress? (or could use target position)
    INTEGER8 state;
    INTEGER8 currentMotion;

    bool sensorsOn = false;

   public:
    Keyboard *keyboard;

    LoggingRobot(std::string robot_name="", std::string yaml_config_file="");
    ~LoggingRobot();

    // Functions which are needed for the Robot Class - they don't do anything at the moment
    bool initialiseJoints() { return true; };
    bool initialiseInputs();
    bool initialiseNetwork() { return true; };  // this one might need to be changed

    /**
     * @brief Updates local copy of forces, and returns them
     *
     * @return Eigen::VectorXd& a 6xN (N is number of crutches) of crutch sensor readings
     */
    Eigen::VectorXd &getCrutchReadings();
    Eigen::VectorXi &getForcePlateReadings();
    Eigen::VectorXi &getFootSensorReadings();

    Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1> &getMotorPositions();
    Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1> &getMotorVelocities();
    Eigen::Matrix<INTEGER16, Eigen::Dynamic, 1> &getMotorTorques();

    INTEGER16& getGoButton();
    INTEGER8&  getCurrentState();
    INTEGER8&  getCurrentMovement();


    /**
     * @brief Takes the forces from the crutches and updates a local copy
     *
     */
    void updateCrutchReadings();

    void setCrutchOffsets(Eigen::VectorXd offsets);

    bool startSensors();
    bool stopSensors();

    bool startCrutchSensors();
    bool stopCrutchSensors();
    bool configureMasterPDOs();
    };

#endif /*LoggingRobot.h*/
