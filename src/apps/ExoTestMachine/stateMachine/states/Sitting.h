/**
 * /file Sitting.h
 * /author Justin Fong
 * /version 0.2
 * /date 2020-11-3
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef SITTING_H_INCLUDED
#define SITTING_H_INCLUDED

#include <time.h>

#include "ExoTestState.h"

/**
 * @brief State for the ExoTestMachine (implementing ExoTestState) - representing when the exoskeleton is sitting down (stationary)
 * 
 * State machines enters this state when the sitting down trajectory is finished, and waits here for input
 */
class Sitting : public ExoTestState {
   public:
    void entry(void);
    void during(void);
    void exit(void);
    Sitting(StateMachine *m, X2Robot *exo, DummyTrajectoryGenerator *tg, const char *name = NULL) : ExoTestState(m, exo, tg, name){};
};

#endif