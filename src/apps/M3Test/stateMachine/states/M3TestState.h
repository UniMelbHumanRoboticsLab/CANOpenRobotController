/**
 * /file M3TestState.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-06-16
 * \copyright Copyright (c) 2020
 * 
 * 
 */

#ifndef M3TESTSTATE_H_DEF
#define M3TESTSTATE_H_DEF

#include <time.h>

#include <iostream>

#include "DebugMacro.h"
#include "RobotM3.h"
#include "State.h"

/**
 * \brief Example Implementation of State Class. Used with M3TestMachine
 * 
 */
class M3TestState : public State {
   protected:
    /**
    *  \todo Might be good to make these Const
    * 
    */
    RobotM3 *robot;                               /*<!Pointer to state machines robot object*/

   public:
    void entry(void);
    void during(void);
    void exit(void);
    M3TestState(StateMachine *m, RobotM3 *M3, const char *name = NULL);
};

#endif