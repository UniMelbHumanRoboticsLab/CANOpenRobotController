/**
 * \file M2Spasticity.h
 * \author Vincent Crocher
 * /brief The <code>M2Spasticity</code> class represents an example implementation of an M2 state machine.
 * \version 0.1
 * \date 2020-12-09
 *
 * \copyright Copyright (c) 2020
 *
 */
#ifndef M2_SM_H
#define M2_SM_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <csignal> //For raise()

#include "RobotM2.h"
#include "StateMachine.h"
#include "FLNLHelper.h"

// State Classes
#include "M2SpasticityStates.h"

/**
 * @brief Example implementation of a StateMachine for the M2Robot class. States should implemented M2DemoState
 *
 */
class M2Spasticity : public StateMachine {
   public:
    bool running = false;
    std::chrono::steady_clock::time_point time_init; // initial time that machine started
    double time_running; // time passed after initialisation in [s]
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M2Spasticity();
    ~M2Spasticity();
    void init();
    void end();

    void hwStateUpdate();

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    M2Calib *calibState;
    M2Transparent *standbyState;
    M2Recording *recordingState;
    M2ArcCircle *testingState;
    M2MinJerkPosition* minJerkState;

   protected:
    RobotM2 *robot;         /*!< Pointer to the Robot*/
    FLNLHelper *UIserver;   /*!< Pointer to communication server*/

   private:
    EventObject(EndCalib) * endCalib;
    EventObject(GoToNextState) * goToNextState;
};

#endif /*M2_SM_H*/
