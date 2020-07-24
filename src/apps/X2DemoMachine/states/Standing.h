/**
 * /file Standing.h
 * /author Justin Fong
 * /version 0.1
 * /date 2020-05-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef STANDING_H_INCLUDED
#define STANDING_H_INCLUDED

#include "State.h"
#include "X2Robot.h"
/**
 * @brief State for the ExoTestMachine (implementing ExoTestState) - representing when the exoskeleton is standing up (stationary)
 * 
 * State machines enters this state when the standing up trajectory is finished, and waits here for input
 */
class Standing : public State {
   public:
    void entry(void);
    void during(void);
    void exit(void);
    Standing(StateMachine *m, X2Robot *exo, const char *name = NULL) : State(m, name), robot(exo){};

   private:
    X2Robot *robot;
};

#endif