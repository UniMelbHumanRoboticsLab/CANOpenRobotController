/**
 * \file M3DemoMachine.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-06-16
 * \copyright Copyright (c) 2020
 *
 * /brief The <code>M3DemoMachine</code> class represents an example implementation of an M3 state machine.
 *
 */
#ifndef M3_SM_H
#define M3_SM_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <csignal> //For raise()

#include "RobotM3.h"
#include "StateMachine.h"

// State Classes
#include "M3DemoStates.h"

/**
 * @brief Example implementation of a StateMachine for the M3Robot class. States should implemented M3DemoState
 *
 */
class M3DemoMachine : public StateMachine {
   public:
    bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M3DemoMachine();
    ~M3DemoMachine();
    void init();
    void end();

    void hwStateUpdate();

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    M3CalibState *calibState;
    M3MassCompensation *standbyState;
    M3DemoState *testState;
    M3EndEffDemo *endEffDemoState;
    M3DemoImpedanceState *impedanceState;
    M3SamplingEstimationState *timingState;

   protected:
    RobotM3 *robot; /*<!Pointer to the Robot*/

   private:
    EventObject(EndCalib) * endCalib;
    EventObject(GoToState1) * goToState1;
    EventObject(GoToState2) * goToState2;
    EventObject(GoToState3) * goToState3;
    EventObject(GoToState4) * goToState4;
};

#endif /*M3_SM_H*/
