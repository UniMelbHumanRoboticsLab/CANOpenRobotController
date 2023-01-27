/**
 * /file InitState.h
 * /author Justin Fong
 * /brief Virtual Class to include all required classes for Logging Robot
 * /version 0.1
 * /date 2020-12-1
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef LR_INITSTATE_H
#define LR_INITSTATE_H

#include "LoggingRobot.h"
#include "State.h"

extern CO_NMT_reset_cmd_t reset_local;

class InitState : public State {
   public:
    LoggingRobot *robot;
    InitState(LoggingRobot *robot, const char *name = "");

    void entry(void);
    void during(void);
    void exit(void);
};
#endif