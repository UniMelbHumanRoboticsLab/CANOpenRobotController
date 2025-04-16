/**
 * \file StateMachineTemplate.h
 * \author Vincent Crocher
 * \brief The StateMachineTemplate class represents an example implementation of a state machine.
 * \version 0.1
 * \date 2025-04-14
 *
 * \copyright Copyright (c) 2025
 *
 */
#ifndef STATEMACHINETEMPLATE_H
#define STATEMACHINETEMPLATE_H


#include "StateMachine.h"
#include "PlatformName.h"
#include "FLNLHelper.h"

// State Classes
#include "StatesTemplate.h"


/**
 * Example implementation of a StateMachine class. States should implemented StatesTemplate 
 *
 */
class StateMachineTemplate : public StateMachine {

   public:
    StateMachineTemplate();
    ~StateMachineTemplate();
    void init();
    void end();

    void hwStateUpdate();

    PlatformName *robot() { return static_cast<PlatformName*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)

    std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server
};

#endif
