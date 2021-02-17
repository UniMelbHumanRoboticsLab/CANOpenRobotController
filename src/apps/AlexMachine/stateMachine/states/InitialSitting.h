/**
 * /file InitialSitting.h
 * /author William Campbell
 * /brief Concrete implementation of ExoTestState
 * /version 0.1
 * /date 2020-07-02
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef InitialSitting_H_INCLUDED
#define InitialSitting_H_INCLUDED

#include "ExoTestState.h"

/**
 * \brief Initialisation State for the ExoTestMachine (implementing ExoTestState)
 * 
 * State holds until event triggers its exit, and runs initPositionControl on exit 
 * Control of transition is independent of this class and is defined in ExoTestMachine.
 * 
 */
class InitialSitting : public ExoTestState {
   public:
    void entry(void);
    void during(void);
    void exit(void);
    InitialSitting(StateMachine *m, AlexRobot *exo, AlexTrajectoryGenerator *tg, const char *name = NULL) : ExoTestState(m, exo, tg, name){};
};

#endif