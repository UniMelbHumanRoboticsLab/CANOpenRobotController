//
// Created by William Campbell on 2019-09-24.
//
#include "StateMachine.h"

#include <stdio.h>

#include <iostream>

#include "DebugMacro.h"
//State machine constructors
StateMachine::StateMachine(void) {
    currentState = NULL;
};

void StateMachine::initialize(State *i) {
    currentState = i;
    DEBUG_OUT("StateMachine::initialize()")
}

State *StateMachine::getCurState(void) {
    return currentState;
}

void StateMachine::activate(void) {
    DEBUG_OUT("StateMachine::Activate")
    currentState->entry();
}

void StateMachine::update(void) {
    Transition *t = currentState->getActiveArc();
    if (t != NULL) {
        currentState->exit();
        this->currentState = t->target;
        currentState->entry();
    }
    currentState->during();
}
