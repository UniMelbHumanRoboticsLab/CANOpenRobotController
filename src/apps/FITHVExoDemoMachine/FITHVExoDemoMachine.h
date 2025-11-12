/**
 * \file FITHVExoDemoMachine.h
 * \author Vincent Crocher
 * \brief The FITHVExoDemoMachine class represents an example implementation of a state machine.
 * \version 0.1
 * \date 2025-04-14
 *
 * \copyright Copyright (c) 2025
 *
 */
#ifndef FITHVEXODEMOMACHINE_H
#define FITHVEXODEMOMACHINE_H


#include "StateMachine.h"
#include "RobotFITHVExo.h"
#include "FLNLHelper.h"

// State Classes
#include "FITHVExoDemoStates.h"


/**
 * Example implementation of a StateMachine class. States should implemented FITHVExoDemo
 *
 */
class FITHVExoDemoMachine : public StateMachine {

   public:
    FITHVExoDemoMachine();
    ~FITHVExoDemoMachine();
    void init();
    void end();

    void hwStateUpdate();

    RobotFITHVExo *robot() { return static_cast<RobotFITHVExo*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)

    std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server
};

#endif
