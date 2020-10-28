//
// Created by ubuntu on 10/28/20.
//

#ifndef CORC_M1TESTMACHINE_H
#define CORC_M1TESTMACHINE_H

/**
 * \file M1TestMachine.h
 * \author Yue Wen adapted from Vincent Crocher
 * \version 0.1
 * \date 2020-08-25
 * \copyright Copyright (c) 2020
 *
 * /brief The <code>M1TestMachine</code> class represents an example implementation of an M1 state machine.
 *
 */

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <csignal> //For raise()

#include "RobotM1.h"
#include "StateMachine.h"

/**
 * @brief Example implementation of a StateMachine for the M1Robot class. States should implemented M1DemoState
 *
 */
class M1TestMachine : public StateMachine {
public:
    bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M1TestMachine();
    ~M1TestMachine();
    void init();
    void end();

    void hwStateUpdate();

    /**
     * Pointers to the relevant states - initialised in init
     *
     */

protected:
    RobotM1 *robot; /*<!Pointer to the Robot*/

};

#endif //CORC_M1TESTMACHINE_H
