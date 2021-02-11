/**
 * \file AlexMachine.h
 * \author William Campbell
 * \version 0.1
 * \date 2020-06-01
 * \copyright Copyright (c) 2020
 *
 * /brief The <code>AlexMachine</code> class represents the Alex teamsimplementation of an exoskeleton state machine
 * with five states.
 * Initialisation, sitting, standing, standing up and sitting down./todo: add walking states.
 * The machine uses the AlexTrajectory generator to perform position control opperations using an X2 hardware platform.
 *
 *
 * For more detail on the architecture, mechanics and design of the project please go to embedded.readthedocs.io
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
#ifndef ALEX_SM_H
#define ALEX_SM_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "AlexRobot.h"
#include "ExoTestState.h"
#include "StateMachine.h"

// State Classes
#include "BackStepLeft.h"
#include "BackStepRight.h"
#include "DebugState.h"
#include "ErrorState.h"
#include "InitState.h"
#include "InitialSitting.h"
#include "LeftForward.h"
#include "RightForward.h"
#include "Sitting.h"
#include "SittingDwn.h"
#include "Standing.h"
#include "StandingUp.h"
#include "SteppingFirstLeft.h"
#include "SteppingLastLeft.h"
#include "SteppingLastRight.h"
#include "SteppingLeft.h"
#include "SteppingRight.h"
#include "SteppingRightStair.h"
#include "SteppingLeftStair.h"
#include "SteppingRightStairDown.h"
#include "SteppingLeftStairDown.h"
/**
 * @brief Example implementation of a StateMachine for the ExoRobot class. States should implemented ExoTestState
 *
 */
class AlexMachine : public StateMachine
{
public:
    bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    AlexMachine();
    void init();
    void activate();
    void hwStateUpdate();
    void configureMasterPDOs();

    void end();
    State *gettCurState();
    void initRobot(AlexRobot *rb);
    bool trajComplete;
    AlexTrajectoryGenerator *trajectoryGenerator;

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    InitState *initState;
    InitialSitting *initialSitting;
    SittingDwn *sittingDwn;
    StandingUp *standingUp;
    Sitting *sitting;
    Standing *standing;
    SteppingFirstLeft *steppingFirstLeft;
    LeftForward *leftForward;
    SteppingRight *steppingRight;
    RightForward *rightForward;
    SteppingLeft *steppingLeft;
    SteppingLastRight *steppingLastRight;
    SteppingLastLeft *steppingLastLeft;
    ErrorState *errorState;
    BackStepLeft *backStepLeft;
    BackStepRight *backStepRight;
    SteppingLeftStair *steppingLeftStair;
    SteppingRightStair *steppingRightStair;
    SteppingLeftStairDown *steppingLeftStairDown;
    SteppingRightStairDown *steppingRightStairDown;

#ifdef VIRTUAL
    DebugState *debug;
#endif

protected:
    AlexRobot *robot; /*<!Pointer to the Robot*/

private:
    /**
     *
     * \brief Event Objects defined using Macro defined in StateMachine.h
     * Defines the Class itself, as well as initialises an object of that class
     * An events check function are defined in the .cpp file.
    */
    EventObject(EndTraj) * endTraj;
    EventObject(IsAPressed) * isAPressed;
    EventObject(StartButtonsPressed) * startButtonsPressed;
    EventObject(StartExo) * startExo;
    EventObject(StartSit) * startSit;
    EventObject(StartStand) * startStand;
    EventObject(StartWalk) * startWalk;
    EventObject(FeetTogether) * feetTogether;
    EventObject(IsRPressed) * isRPressed;
    EventObject(ResetButtons) * resetButtonsPressed;
    EventObject(StandSelect) * standSelect;
    EventObject(SitSelect) * sitSelect;
    EventObject(WalkSelect) * walkSelect;
    EventObject(BackStep) * backStep;
    EventObject(UpStairSelect) * upStairSelect;
    EventObject(DownStairSelect) * downStairSelect;

#ifdef VIRTUAL
    EventObject(DebugTransition) * debugTransition;
#endif
};

#endif /*EXO_SM_H*/
