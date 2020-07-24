/**
 * /file Sitting.h
 * /author Justin Fong
 * /version 0.1
 * /date 2020-05-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef SITTING_H_INCLUDED
#define SITTING_H_INCLUDED

#include <time.h>

#include "State.h"
#include "X2Robot.h"
/**
 * @brief State for the ExoTestMachine (implementing ExoTestState) - representing when the exoskeleton is sitting down (stationary)
 * 
 * State machines enters this state when the sitting down trajectory is finished, and waits here for input
 */
class Sitting : public State {
   public:
    void entry(void);
    void during(void);
    void exit(void);
    Sitting(StateMachine *m, X2Robot *exo, const char *name = NULL) : State(m, name), robot(exo){};

   private:
    X2Robot *robot;
};

#endif