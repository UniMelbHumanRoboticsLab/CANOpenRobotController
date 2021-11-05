/**
 * \file ForcePlateApp.h
 * \author Justin Fong
 * \version 0.1
 * \date 2021-02-23
 * \copyright Copyright (c) 2021
 *
 * /brief 
 *
 */
#ifndef FORCEPLATEAPP_H
#define FORCEPLATEAPP_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "ForcePlate.h"
//#include "ForcePlate4.h"
#include "StateMachine.h"

// State Classes
#include "InitState.h"
#include "IdleState.h"
#include "CalibrateState.h"
#include "RecordState.h"

// Logger
#include "LogHelper.h"

/**
 * @brief Example implementation of a StateMachine for the ExoRobot class. States should implemented ExoTestState
 *
 */
class ForcePlateApp : public StateMachine {
   public:
    bool running = false;

    // Timing variables for logging
    std::chrono::steady_clock::time_point time0;  // initial time that machine started
    double time;                                  // time passed after time0 in [s]

    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    ForcePlateApp();
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
    ForcePlate *robot;   /*<!Pointer to the Robot*/
    LogHelper dataLogger;  // Logger

   private:
    /**
     *
     * \brief Event Objects defined using Macro defined in StateMachine.h
     * Defines the Class itself, as well as initialises an object of that class
     * An events check function are defined in the .cpp file.
    */
    EventObject(StartCalibrate) * startCalibrate;
    EventObject(StartRecord) * startRecord;
    EventObject(StopRecord) * stopRecord;
    EventObject(IsCalibrationFinished) * isCalibrationFinished;
    EventObject(CompleteInit) * completeInit;
};

#endif
