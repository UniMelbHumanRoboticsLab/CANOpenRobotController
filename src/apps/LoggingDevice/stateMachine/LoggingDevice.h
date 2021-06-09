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
#ifndef LOGGINGDEVICE_SM_H
#define LOGGINGDEVICE_SM_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "LoggingRobot.h"
#include "StateMachine.h"

// State Classes
#include "InitState.h"
#include "IdleState.h"
#include "CalibrateState.h"
#include "RecordState.h"

// Logger
#include "spdlog/helper/LogHelper.h"

/**
 * @brief Example implementation of a StateMachine for the ExoRobot class. States should implemented ExoTestState
 *
 */
class LoggingDevice : public StateMachine {
   public:
    bool running = false;

    // Timing variables for logging
    std::chrono::steady_clock::time_point time0;  // initial time that machine started
    double time;                                  // time passed after time0 in [s]

    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    LoggingDevice();
    void init();
    void end();
    void update();
    void hwStateUpdate();
    void configureMasterPDOs();

    State *gettCurState();
    bool trajComplete;


    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    InitState *initState;
    IdleState *idleState;
    CalibrateState *calibrateState;
    RecordState *recordState;

   protected:
    LoggingRobot *robot;   /*<!Pointer to the Robot*/
    LogHelper dataLogger;  // Logger

   private:
    /**
     *
     * \brief Event Objects defined using Macro defined in StateMachine.h
     * Defines the Class itself, as well as initialises an object of that class
     * An events check function are defined in the .cpp file.
    */
    EventObject(IsAPressed) * isAPressed;
    EventObject(IsSPressed) * isSPressed;
    EventObject(IsCalibrationFinished) * isCalibrationFinished;
};

#endif
