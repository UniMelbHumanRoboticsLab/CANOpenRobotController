#include "InitState.h"

void InitState::entry(void) {
    std::cout
        << "==================================" << std::endl
        << " WELCOME TO THE TEST STATE MACHINE" << std::endl
        << "==================================" << std::endl
        << std::endl
        << "========================" << std::endl
        << " PRESS S to start program" << std::endl
        << "========================" << std::endl;
}
void InitState::during(void) {
}
void InitState::exit(void) {
    robot->initPositionControl();
    std::cout << "Initialise State Exited" << std::endl;
}
