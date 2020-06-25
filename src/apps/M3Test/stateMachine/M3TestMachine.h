/**
 * \file M3TestMachine.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-06-16
 * \copyright Copyright (c) 2020
 *
 * /brief The <code>M3TestMachine</code> class represents an example implementation of an M3 state machine.
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
#include "M3TestStates.h"

/**
 * @brief Example implementation of a StateMachine for the M3Robot class. States should implemented M3TestState
 *
 */
class M3TestMachine : public StateMachine {
   public:
    bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M3TestMachine();
    void init();
    void activate();
    void deactivate();

    void hwStateUpdate();
    State *gettCurState();
    void initRobot(RobotM3 *rb);

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    M3TestState *testState;

   protected:
    RobotM3 *robot; /*<!Pointer to the Robot*/

   private:
};

#endif /*M3_SM_H*/
