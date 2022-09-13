/**
 * \file ExoTestMachine.h
 * \author William Campbell
 * \version 0.1
 * \date 2019-09-24
 * \copyright Copyright (c) 2020
 *
 * /brief The <code>ExoTestMachine</code> class represents an example implementation of an exoskeleton state machine
 * with five states. Initialisation, sitting, standing, standing up and sitting down. The test machine
 * is made as example for developers to structure their specific use cases with.
 * For more detail on the architecture and mechanics of the state machine class see:https://embeded.readthedocs.io/en/latest/StaeMachines/.
 *
 *  State transition Diagram.
 *
 *         startExo             startStand
 *  initState +-----> sitting +---------> standingUp
 *                      ^                  +
 *           EndTraj    |                  | EndTraj
 *                      |                  |
 *                      +                  |
 *                 sittingDwn <---------+ standing
 *                              startSit
 *
 * Version 0.1
 * Date: 07/04/2020
 *
 */
#ifndef EXO_SM_H
#define EXO_SM_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "ExoTestState.h"
#include "StateMachine.h"
#include "X2Robot.h"

// State Classes
#include "InitState.h"
#include "Sitting.h"
#include "SittingDwn.h"
#include "Standing.h"
#include "StandingUp.h"

/**
 * @brief Example implementation of a StateMachine for the ExoRobot class. States should implemented ExoTestState
 *
 */
class ExoTestMachine : public StateMachine {
   public:
    //bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    ExoTestMachine();
    void init();
    void end();

    void hwStateUpdate();

    State *gettCurState();

    bool trajComplete;
    DummyTrajectoryGenerator *trajectoryGenerator;

    X2Robot *robot() { return static_cast<X2Robot*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)
};

#endif /*EXO_SM_H*/
