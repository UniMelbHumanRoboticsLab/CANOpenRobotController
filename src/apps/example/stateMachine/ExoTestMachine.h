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

#include "ExoRobot.h"
#include "ExoTestState.h"
#include "StateMachine.h"

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
    bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     * 
     */
    ExoTestMachine();
    void init();
    void activate();
    void deactivate();

    void hwStateUpdate();
    State *gettCurState();
    void initRobot(ExoRobot *rb);
    bool trajComplete;
    DummyTrajectoryGenerator *trajectoryGenerator;

    /**
     * Pointers to the relevant states - initialised in init 
     * 
     */
    InitState *initState;
    SittingDwn *sittingDwn;
    StandingUp *standingUp;
    Sitting *sitting;
    Standing *standing;

   protected:
    ExoRobot *robot; /*<!Pointer to the Robot*/

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
};

#endif /*EXO_SM_H*/
