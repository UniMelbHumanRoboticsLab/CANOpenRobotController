/**
 * /file IdleState.h
 * /author Justin Fong
 * /brief Idle state - logger is doing nothing
 * /version 0.1
 * /date 2021-1-21
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef LR_IDLESTATE_H
#define LR_IDLESTATE_H

#include "LoggingRobot.h"
#include "State.h"

class IdleState : public State {
   public:
    LoggingRobot *robot;
    IdleState(LoggingRobot *robot, const char *name = "");

    void entry(void);
    void during(void);
    void exit(void);
};
#endif