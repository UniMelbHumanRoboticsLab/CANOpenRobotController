//
// Created by William Campbell on 2019-09-24.
//
#include <iostream>

#include "StateMachine.h"



StateMachine::StateMachine(): running(false) {
}

void StateMachine::setInitState(std::string state_name) {
    currentState = state_name;
    if(states.count(currentState)<1) {
        spdlog::error("Requested initial state {} does not exists. Left to default.", currentState);
    }
}

void StateMachine::activate(void) {
    spdlog::debug("StateMachine::Activate()");
    if(states.count(currentState)>0) {
        states[currentState]->entry();
        running = true;
    }
    else {
        spdlog::critical("StateMachine activation state ({}) does not exist. Exiting...", currentState);
        std::raise(SIGTERM); //Clean exit
    }
}

std::shared_ptr<State> & StateMachine::getCurrentState() {
    return states[currentState];
}

std::shared_ptr<State> & StateMachine::getState(std::string state_name) {
    return states[state_name];
}

void StateMachine::update(void) {
    spdlog::trace("StateMachine::update()");
    hwStateUpdate(); //Call specialised state machine hardware update method

    //Manage possible transition
    bool transitioned = false;
    for (auto& tr : transitions[currentState]) {
        //Transition is active?
        if(tr.first(*this)) {
            states[currentState]->exit();
            currentState=tr.second;
            states[currentState]->entry();
            transitioned=true;
            break;
        }
    }

    //Execute (if not just transitioned)
    if(!transitioned) {
        states[currentState]->during();
    }

    //Logging
    if(logHelper.isStarted() && logHelper.isInitialised())
        logHelper.recordLogData();
}
