/**
 *
 * \file LogginRobot.h
 * \author Justin Fong
 * \version 0.1
 * \date 2020-12-01
 * \copyright Copyright (c) 2020
 *
 * \brief  The<code> LogginRobot</ code> class represents a logging robot. This Robot doesn't actually do anything, except give options to act as a logger.
 *
 */

#ifndef LOGGINGROBOT_H_INCLUDED
#define LOGGINGROBOT_H_INCLUDED

#include "Keyboard.h"
#include "Robot.h"
#include "RobotousRFT.h"

class LoggingRobot : public Robot {
   private:
    std::vector<RobotousRFT *> crutchSensors;
    Eigen::VectorXd crutchReadings;  //6xN Vector containing all crutch readings

    bool sensorsOn =  false;

   public:
    Keyboard *keyboard;

    LoggingRobot();
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

    /**
     * @brief Takes the forces from the crutches and updates a local copy
     * 
     */
    void updateCrutchReadings();

    void setCrutchOffsets(Eigen::VectorXd offsets);

    bool startSensors();
    bool stopSensors();
};

#endif /*LoggingRobot.h*/
