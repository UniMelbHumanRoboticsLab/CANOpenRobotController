/**
 * \file SittingDwn.h
 * \author Justin Fong
 * \version 0.1
 * \date 2020-05-07
 * 
 * \copyright Copyright (c) 2020
 * 
 */

#ifndef SITTINGDWN_H_INCLUDED
#define SITTINGDWN_H_INCLUDED
#include "ExoTestState.h"

/**
 * \brief State for the ExoTestMachine (implementing ExoTestState) - representing when the exo is sitting down (moving)
 * 
 * Starts the Sitting Down trajectory on entry, executes in during, and exits when trajectory is complete
 */
class SittingDwn : public ExoTestState {
   public:
    /**
    * \brief Prepare Robot and Trajectory Generator objects to tigger a sit motion
    * loads SIT paramaters into the Trajectory Generator object and runs robot startNewTrajectory function.
    * 
    */
    void entry(void);
    /**
     * \brief run the robot objectsmoveThroughtrajecoty function using the loaded trajectory
     * dictated by the state machines Trajectory Generator object.
     * 
     */
    void during(void);
    void exit(void);
    SittingDwn(StateMachine *m, ExoRobot *exo, DummyTrajectoryGenerator *tg, const char *name = NULL) : ExoTestState(m, exo, tg, name){};
};

#endif