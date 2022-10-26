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

#ifndef LR_RECORDSTATE_H
#define LR_RECORDSTATE_H

#include "LoggingRobot.h"
#include "State.h"

class RecordState : public State {
    private:
    int ticker = 0;
    Eigen::VectorXd lastCrutchForce;

   public:
    LoggingRobot *robot;
    RecordState(LoggingRobot *robot, const char *name = "");

    void entry(void);
    void during(void);
    void exit(void);
};
#endif
