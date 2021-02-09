/**
 * /file ErrorState.h
 * /author William Campbell
 * /brief Error state
 * /version 0.1
 * /date 2020-06-23
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef ErrorState_H_INCLUDED
#define ErrorState_H_INCLUDED

#include "ExoTestState.h"
#include "AlexJoint.h"

/**
 * \brief Initialisation State for the ExoTestMachine (implementing ExoTestState)
 * 
 * State holds until event triggers its exit, and runs initPositionControl on exit 
 * Control of transition is independent of this class and is defined in ExoTestMachine.
 * 
 */
class ErrorState : public ExoTestState {
   public:
    void entry(void);
    void during(void);
    void exit(void);
    ErrorState(StateMachine *m, AlexRobot *exo, AlexTrajectoryGenerator *tg, const char *name = NULL) : ExoTestState(m, exo, tg, name){};
};

#endif