/**
 *
 * \file UBORobot.h
 * \author Jia Quan Loh
 * \version 0.1
 * \date 2025-10-29
 * \copyright Copyright (c) 2020
 *
 * \brief  The UBORobot class represents a logging robot. This Robot doesn't actually do anything, except give options to act as a logger.
 * 
 * It has two functions: 1) To trigger data acquision devices not associate with the operation of the robot, and 2) to log data from existing devices
 *
 */

#ifndef UBOROBOT_H_INCLUDED
#define UBOROBOT_H_INCLUDED

#include "Keyboard.h"
#include "Robot.h"
#include "RobotousRFT.h"

class UBORobot : public Robot {
   private:
    int numJoints = 4;
    // -- Variables assocaited with standalone sensors -- //
    std::vector<RobotousRFT *> UBO_FTSensors;
    Eigen::VectorXd UBO_readings;  //6xN Vector containing all UBO readings

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

    UBORobot(std::string robot_name="", std::string yaml_config_file="");
    ~UBORobot();

    // Functions which are needed for the Robot Class - they don't do anything at the moment
    bool initialiseJoints() { return true; };
    bool initialiseInputs();
    bool initialiseNetwork() { return true; };  // this one might need to be changed

    Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1> &getMotorPositions();
    Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1> &getMotorVelocities();
    Eigen::Matrix<INTEGER16, Eigen::Dynamic, 1> &getMotorTorques();

    INTEGER16& getGoButton();
    INTEGER8&  getCurrentState();
    INTEGER8&  getCurrentMovement();

    /**
     * @brief Updates local copy of forces, and returns them
     *
     * @return Eigen::VectorXd& a 6xN (N is number of crutches) of crutch sensor readings
     */
    Eigen::VectorXd &getUBO_readings();

    /**
     * @brief Takes the forces from the FT sensors and updates a local copy
     *
     */
    void updateUBO_readings();

    /**
     * @brief prints the forces from the FT sensors
     *
     */
    void printUBO_readings(Eigen::VectorXd readings);

    /**
     * @brief Updates and corrects the local copy of forces, and returns them
     *
     * @return Eigen::VectorXd& a 6xN (N is number of crutches) of crutch sensor readings
     */
    Eigen::VectorXd &getCorrectedUBO_readings();

    void setUBOOffsets(Eigen::VectorXd offsets);
    bool startUBO_FTSensors();
    bool stopUBO_FTSensors();
    bool setUBO_FTSensorsFilter();
    bool configureMasterPDOs();
    };

#endif /*UBORobot.h*/
