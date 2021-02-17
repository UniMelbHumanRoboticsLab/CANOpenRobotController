/**
 * /file ExoTestState.h
 * /author Justin Fong
 * /brief Virtual Class to include all required classes for ExoTestStates
 * /version 0.1
 * /date 2020-05-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef EXOTESTSTATE_H_DEF
#define EXOTESTSTATE_H_DEF

#include <time.h>

#include <iostream>

#include "AlexRobot.h"
#include "AlexTrajectoryGenerator.h"
#include "State.h"

/* \brief Example Implementation of State Class.Used with ExoTestMachine
*
*  *Note : This is used to ensure that all states here have an ExoRobot,
    and a DummyTrajectoryGenerator(as opposed to more generic Robot and TrajectoryGenerator) *
*/
class ExoTestState : public State {
   protected:
    /**
    *  \todo Might be good to make these Const
    * 
    */
    AlexRobot *robot;                             /*<!Pointer to state machines robot object*/
    AlexTrajectoryGenerator *trajectoryGenerator; /*<!Pointer to state machines trajectoryGenerator object*/

   public:
    virtual void entry() = 0;
    virtual void during() = 0;
    virtual void exit() = 0;
    ExoTestState(StateMachine *m, AlexRobot *exo, AlexTrajectoryGenerator *tg, const char *name = NULL);
    void updateCrutch();
    void updateFlag();

   private:
    // bool entryFlag;
};

#endif