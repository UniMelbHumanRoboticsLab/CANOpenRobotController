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

#include "X2Robot.h"
#include "StateMachine.h"

// State Classes
#include "states/IdleState.h"
#include "states/X2DemoState.h"

#include "X2DemoMachineROS.h"

// Logger
#include "spdlog/helper/LogHelper.h"

class X2DemoMachine : public StateMachine {

public:
    bool running = false;
    std::chrono::steady_clock::time_point time0; // initial time that machine started
    double time; // time passed after tim0 in [s]
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    X2DemoMachine();
    void init(int argc, char *argv[]);
    void end();

    void update();
    void hwStateUpdate();
    void initRobot(X2Robot *rb);

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    IdleState *idleState;
    X2DemoState *x2DemoState;

//    X2DemoMachineROS *x2DemoMachineRos_;

private:
    /**
     *
     * \brief Event Objects defined using Macro defined in StateMachine.h
     * Defines the Class itself, as well as initialises an object of that class
     * An events check function are defined in the .cpp file.
    */
    EventObject(StartExo) * startExo;

    LogHelper logHelper_;

    X2DemoMachineROS *x2DemoMachineRos_; /*<!Pointer to the ROS Class*/
    X2Robot *robot_; /*<!Pointer to the Robot*/

};


#endif //SRC_X2DEMOMACHINE_H
