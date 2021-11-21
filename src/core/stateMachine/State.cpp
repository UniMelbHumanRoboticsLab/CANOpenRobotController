
//
// Created by William Campbell on 2019-09-24.
//

#include "State.h"

const std::shared_ptr<State> &State::getActiveArc(void) {
    //TODO
    return transitions[0].first;
}

const std::string &State::getName(void) {
    return name;
};

void State::printName(void) {
    std::cout << name << std::endl;
};

State::~State() {
}
