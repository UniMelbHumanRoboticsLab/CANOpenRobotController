/**
 * \file X2DemoMachine.h
 * \author Emek Baris KUcukabak
 * \version 0.1
 * \date 2020-07-06
 * \copyright Copyright (c) 2020
 *
 * /brief The <code>X2DemoMachine</code> class represents an example implementation of an X2 state machine.
 *
 */

#ifndef SRC_X2DEMOMACHINE_H
#define SRC_X2DEMOMACHINE_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "DebugMacro.h"

#include "ExoRobot.h"
#include "StateMachine.h"

// State Classes
#include "states/IdleState.h"
#include "states/X2DemoState.h"

class X2DemoMachine : public StateMachine {

public:
    bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    X2DemoMachine();
    void init();
    void end();

    void hwStateUpdate();
    void initRobot(ExoRobot *rb);

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    IdleState *idleState;
    X2DemoState *x2DemoState;

    ExoRobot *robot; /*<!Pointer to the Robot*/

private:
    /**
     *
     * \brief Event Objects defined using Macro defined in StateMachine.h
     * Defines the Class itself, as well as initialises an object of that class
     * An events check function are defined in the .cpp file.
    */
    EventObject(StartExo) * startExo;

};


#endif //SRC_X2DEMOMACHINE_H
