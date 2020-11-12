/**
 * /file InitState.h
 * /author Justin Fong
 * /brief Concrete implementation of ExoTestState
 * /version 0.2
 * /date 2020-11-3
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef INITSTATE_H_INCLUDED
#define INITSTATE_H_INCLUDED

#include "ExoTestState.h"

/**
 * \brief Initialisation State for the ExoTestMachine (implementing ExoTestState)
 * 
 * State holds until event triggers its exit, and runs initPositionControl on exit 
 * Control of transition is independent of this class and is defined in ExoTestMachine.
 * 
 */
class InitState : public ExoTestState {
   public:
    void entry(void);
    void during(void);
    void exit(void);
    InitState(StateMachine *m, X2Robot *exo, DummyTrajectoryGenerator *tg, const char *name = NULL) : ExoTestState(m, exo, tg, name){};
};

#endif