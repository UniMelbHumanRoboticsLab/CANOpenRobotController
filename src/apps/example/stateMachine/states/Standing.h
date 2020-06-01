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

#include "ExoTestState.h"

/**
 * @brief State for the ExoTestMachine (implementing ExoTestState) - representing when the exoskeleton is standing up (stationary)
 * 
 * State machines enters this state when the standing up trajectory is finished, and waits here for input
 */
class Standing : public ExoTestState {
   public:
    void entry(void);
    void during(void);
    void exit(void);
    Standing(StateMachine *m, ExoRobot *exo, DummyTrajectoryGenerator *tg, const char *name = NULL) : ExoTestState(m, exo, tg, name){};
};

#endif