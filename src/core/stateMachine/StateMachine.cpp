//
// Created by William Campbell on 2019-09-24.
//
#include <iostream>

#include "StateMachine.h"
#include "DebugMacro.h"

//State machine constructors
StateMachine::StateMachine(void) {
    currentState = NULL;
};

void StateMachine::initialize(State *i) {
    currentState = i;
    DEBUG_OUT("StateMachine::initialize()")
    initialised = true;
}

State *StateMachine::getCurState(void) {
    return currentState;
}

void StateMachine::activate(void) {
    DEBUG_OUT("StateMachine::Activate")
    if(initialised) {
        currentState->entry();
    }
}

void StateMachine::update(void) {
//    DEBUG_OUT("StateMachine::update")
    //something is wrong with this statement
//    Transition *t = currentState->getActiveArc();
    Transition *t = NULL;
    if (t != NULL) {
        currentState->exit();
        this->currentState = t->target;
        currentState->entry();
    }
    currentState->during();
}
