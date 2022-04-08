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

//// State Classes
#include "states/X2DemoState.h"

#include "X2DemoMachineROS.h"

// Logger
#include "LogHelper.h"

class X2DemoMachine : public StateMachine {

public:
    X2DemoMachine(int argc, char *argv[]);

    X2DemoMachineROS *x2DemoMachineRos_; /*<!Pointer to the ROS Class*/

    void init();
    void end();

    void update();
    void hwStateUpdate();


protected:
    X2Robot *robot() { return static_cast<X2Robot*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)

private:

    std::string robotName_; // robot name(obtained from node name)

    std::chrono::steady_clock::time_point time0; // initial time that machine started
    double time; // time passed after time0 in [s]

};


#endif //SRC_X2DEMOMACHINE_H
