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

class LoggingRobot : public Robot {
   private:
   public:
    Keyboard *keyboard;
    LoggingRobot();
    ~LoggingRobot();

    // Functions which are needed for the Robot Class - they don't do anything at the moment
    bool initialiseJoints() { return true; };
    bool initialiseInputs() { return true; };
    bool initialiseNetwork() { return true; };  // this one might need to be changed
};

#endif /*LoggingRobot.h*/
