#include "State.h"

const std::string &State::getName(void) {
    return name;
};

void State::printName(void) {
    std::cout << name << std::endl;
};

State::~State() {
}
