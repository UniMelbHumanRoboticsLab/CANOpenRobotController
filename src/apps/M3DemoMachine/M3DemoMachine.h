/**
 * \file M3DemoMachine.h
 * \author Vincent Crocher
 * /brief The M3DemoMachine class represents an example implementation of an M3 state machine.
 * \version 0.3
 * \date 2021-12-05
 *
 * \copyright Copyright (c) 2020 - 2021
 *
 */
#ifndef M3_SM_H
#define M3_SM_H


#include "StateMachine.h"
#include "RobotM3.h"
#include "FLNLHelper.h"

// State Classes
#include "M3DemoStates.h"


/**
 * @brief Example implementation of a StateMachine for the M3Robot class. States should implemented M3DemoState
 *
 */
class M3DemoMachine : public StateMachine {

   public:
    M3DemoMachine();
    ~M3DemoMachine();
    void init();
    void end();

    void hwStateUpdate();

    RobotM3 *robot() { return static_cast<RobotM3*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)

    std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server
};

#endif /*M3_SM_H*/
