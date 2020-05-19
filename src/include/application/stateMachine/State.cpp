
//
// Created by William Campbell on 2019-09-24.
//

#include "State.h"

Transition *State::getActiveArc(void) {
    int i = 0;
    while (i < numarcs) {
        if (arclist[i]->ev->check())
            return arclist[i];
        i++;
    }
    return NULL;
}

bool State::addArc(Transition *t) {
    if (numarcs < MAXARCS) {
        arclist[numarcs++] = t;
        return true;
    } else
        return false;
};

const char *State::getName(void) {
    return name;
};

void State::printName(void) {
    std::cout << name << std::endl;
};

State::~State() {
    std::cout << "State Deleted" << std::endl;
}