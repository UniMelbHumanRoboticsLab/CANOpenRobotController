/**
 * \file M1DemoMachine.h
 * \author Yue Wen adapted from Vincent Crocher
 * \version 0.2
 * \date 2022-10-26
 * \copyright Copyright (c) 2020 - 2022
 *
 * /brief The <code>M1DemoMachine</code> class represents an example implementation of an M1 state machine.
 *
 */
#ifndef M1_SM_H
#define M1_SM_H

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
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M1DemoMachine();
    ~M1DemoMachine();
    void init();

    RobotM1 *robot() { return static_cast<RobotM1*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)
};

#endif /*M1_SM_H*/
