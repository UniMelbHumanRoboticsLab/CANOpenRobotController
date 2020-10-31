//
// Created by William Campbell on 2019-09-24.
//
#include <iostream>

#include "StateMachine.h"

//State machine constructors
StateMachine::StateMachine(void) {
    currentState = NULL;
};

void StateMachine::initialize(State *i) {
    currentState = i;
    spdlog::debug("StateMachine::initialize()");
    initialised = true;
}

State *StateMachine::getCurState(void) {
    return currentState;
}

void StateMachine::activate(void) {
    spdlog::debug("StateMachine::Activate()");
    if(initialised) {
        currentState->entry();
    }
}

void StateMachine::update(void) {
    Transition *t = currentState->getActiveArc();
    if (t != NULL) {
        currentState->exit();
        this->currentState = t->target;
        currentState->entry();
    }
    currentState->during();
    if(logHelper.isStarted() && logHelper.isInitialised())
        logHelper.recordLogData();
}

void StateMachine::init() {}
void StateMachine::init(int argc, char **argv) {}