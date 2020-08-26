/**
 * \file M1DemoMachine.h
 * \author Yue Wen adapted from Vincent Crocher
 * \version 0.1
 * \date 2020-08-25
 * \copyright Copyright (c) 2020
 *
 * /brief The <code>M1DemoMachine</code> class represents an example implementation of an M1 state machine.
 *
 */
#ifndef M1_SM_H
#define M1_SM_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <csignal> //For raise()

#include "RobotM1.h"
#include "StateMachine.h"

// State Classes
#include "M1DemoStates.h"

/**
 * @brief Example implementation of a StateMachine for the M1Robot class. States should implemented M1DemoState
 *
 */
class M1DemoMachine : public StateMachine {
   public:
    bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M1DemoMachine();
    ~M1DemoMachine();
    void init();
    void end();

    void hwStateUpdate();

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    IdleState *idleState;
    M1DemoState *demoState;
    Monitoring *monitorState;
//    M1CalibState *calibState;
//    M1MassCompensation *standbyState;
//    M1EndEffDemo *endEffDemoState;
//    M1DemoImpedanceState *impedanceState;
//    M1SamplingEstimationState *timingState;

   protected:
    RobotM1 *robot; /*<!Pointer to the Robot*/

   private:
    EventObject(StartExo) * startExo;
    EventObject(MonitorExo) * monitorExo;

//    EventObject(EndCalib) * endCalib;
};

#endif /*M1_SM_H*/
