#ifndef SITTINGDWN_H_INCLUDED
#define SITTINGDWN_H_INCLUDED
#include "ExoTestState.h"

/**
 * /brief State for the ExoTestMachine (implementing ExoTestState) - representing when the exo is sitting down (moving)
 * 
 * Starts the Sitting Down trajectory on entry, executes in during, and exits when trajectory is complete
 */
class SittingDwn : public ExoTestState {
   public:
    void entry(void);
    void during(void);
    void exit(void);
    SittingDwn(StateMachine *m, ExoRobot *exo, DummyTrajectoryGenerator *tg, const char *name = NULL) : ExoTestState(m, exo, tg, name){};
};

#endif