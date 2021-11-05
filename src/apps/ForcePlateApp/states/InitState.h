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

#ifndef FORCEPLATE_INITSTATE_H
#define FORCEPLATE_INITSTATE_H

#include "ForcePlate.h"
//#include "ForcePlate4.h"
#include "State.h"

extern CO_NMT_reset_cmd_t reset_local;

class InitState : public State {
   public:
    ForcePlate *robot;
    InitState(StateMachine *m, ForcePlate *robot, const char *name = NULL);

    void entry(void);
    void during(void);
    void exit(void);
};
#endif