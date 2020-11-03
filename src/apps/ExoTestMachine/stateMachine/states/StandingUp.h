/**
 * /file StandingUp.h
 * /author Justin Fong
 * /version 0.2
 * /date 2020-11-3
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef STANDINGUP_H_INCLUDED
#define STANDINGUP_H_INCLUDED

#include "ExoTestState.h"

/**
 * @brief State for the ExoTestMachine (implementing ExoTestState) - representing when the exo is standing up (moving)
 * 
 * Starts the Standing Up trajectory on entry, executes in during, and exits when trajectory is complete
 */
class StandingUp : public ExoTestState {
   private:
    /** 
    * Parameters associated with progression through a trajectory
    */
    double currTrajProgress = 0;
    timespec prevTime;

   public:
    /**
    * \brief Prepare Robot and Trajectory Generator objects to tigger a stand motion
    * loads STAND paramaters into the Trajectory Generator object and runs robot startNewTrajectory function.
    * 
    */
    void entry(void);
    /**
     * \brief run the robot objects moveThroughtrajecoty function using the loaded trajectory
     * dictated by the state machines Trajectory Generator object.
     * 
     */
    void during(void);
    void exit(void);
    StandingUp(StateMachine *m, X2Robot *exo, DummyTrajectoryGenerator *tg, const char *name = NULL) : ExoTestState(m, exo, tg, name){};
};

#endif