/**
 * /file ExoTestState.h
 * /author Justin Fong
 * /brief Virtual Class to include all required classes for ExoTestStates
 * /version 0.2
 * /date 2020-11-3
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef EXOTESTSTATE_H_DEF
#define EXOTESTSTATE_H_DEF

#include <time.h>

#include <iostream>

#include "DummyTrajectoryGenerator.h"
#include "State.h"
#include "X2Robot.h"

/**
 * \brief Example Implementation of State Class. Used with ExoTestMachine
 *
 * Note: This is used to ensure that all states here have an ExoRobot, and a DummyTrajectoryGenerator (as opposed to more generic Robot and TrajectoryGenerator)
 *
 */
class ExoTestState : public State {
   protected:
    /**
    *  \todo Might be good to make these Const
    *
    */
    X2Robot *robot;                                /*<!Pointer to state machines robot object*/
    DummyTrajectoryGenerator *trajectoryGenerator; /*<!Pointer to state machines trajectoryGenerator object*/

   public:
    virtual void entry() = 0;
    virtual void during() = 0;
    virtual void exit() = 0;
    ExoTestState(StateMachine *m, X2Robot *exo, DummyTrajectoryGenerator *tg, const char *name = NULL);
};

#endif
