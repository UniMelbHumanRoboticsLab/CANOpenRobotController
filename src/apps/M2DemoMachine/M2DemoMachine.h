/**
 * \file M2DemoMachine.h
 * \author Vincent Crocher
 * /brief The M2DemoMachine class represents an example implementation of an M2 state machine.
 * \version 0.2
 * \date 2021-02-05
 *
 * \copyright Copyright (c) 2020 - 2021
 *
 */
#ifndef M2_SM_H
#define M2_SM_H

#include "RobotM2.h"
#include "StateMachine.h"
#include "FLNLHelper.h"

// State Classes
#include "M2DemoStates.h"

/**
 * @brief Example implementation of a StateMachine for the M2Robot class. States should implemented M2DemoState
 *
 */
class M2DemoMachine : public StateMachine {

   public:
    M2DemoMachine();
    ~M2DemoMachine();
    void init();
    void end();

    void hwStateUpdate();

    RobotM2 *robot() { return static_cast<RobotM2*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)

    std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server
};

#endif /*M2_SM_H*/
