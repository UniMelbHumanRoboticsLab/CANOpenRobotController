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
#include "LogHelper.h"

class X2DemoMachine : public StateMachine {

public:
    X2DemoMachine(int argc, char *argv[]);

    X2DemoMachineROS *x2DemoMachineRos_; /*<!Pointer to the ROS Class*/
    X2Robot *robot_; /*<!Pointer to the Robot*/ // NOTE: For some reason; if this is defined later, it doesn't publish

    bool running = false;

    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */

    void init();
    void end();

    void update();
    void hwStateUpdate();
    void initRobot(X2Robot *rb);
    bool configureMasterPDOs();

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    IdleState *idleState_;
    X2DemoState *x2DemoState_;

private:
    /**
     *
     * \brief Event Objects defined using Macro defined in StateMachine.h
     * Defines the Class itself, as well as initialises an object of that class
     * An events check function are defined in the .cpp file.
    */
    EventObject(StartExo) * startExo;

    std::string robotName_; // robot name(obtained from node name)

    std::chrono::steady_clock::time_point time0; // initial time that machine started
    double time; // time passed after time0 in [s]

};


#endif //SRC_X2DEMOMACHINE_H
