#include "IdleState.h"

void IdleState::entry(void) {
    std::cout
            << "==================================" << std::endl
            << " WELCOME TO THE TEST STATE MACHINE" << std::endl
            << "==================================" << std::endl
            << std::endl
            << "========================" << std::endl
            << " PRESS S to start program" << std::endl
            << "========================" << std::endl;
}
void IdleState::during(void) {
}
void IdleState::exit(void) {
    std::cout << "Idle State Exited" << std::endl;
}