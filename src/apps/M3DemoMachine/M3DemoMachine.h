/**
 * \file M3DemoMachine.h
 * \author Vincent Crocher
 * /brief The <code>M3DemoMachine</code> class represents an example implementation of an M3 state machine.
 * \version 0.2
 * \date 2020-07-27
 *
 * \copyright Copyright (c) 2020
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

#include "RobotM3.h"
#include "StateMachine.h"
#include "FLNLHelper.h"

// State Classes
#include "M3DemoStates.h"


/**
 * @brief Example implementation of a StateMachine for the M3Robot class. States should implemented M3DemoState
 *
 */
class M3DemoMachine : public StateMachine {

   public:
    std::chrono::steady_clock::time_point time_init; // initial time that machine started
    double time_running; // time passed after initialisation in [s]
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M3DemoMachine();
    ~M3DemoMachine();
    void init();
    void end();

    void hwStateUpdate();
    bool configureMasterPDOs();

    std::shared_ptr<RobotM3> robot() { return static_pointer_cast<RobotM3>(_robot); } //!< Robot getter with specialised type

    FLNLHelper *UIserver = nullptr;     //!< Pointer to communication server //TODO: use unique_ptr or shared_ptr
};

#endif /*M3_SM_H*/
