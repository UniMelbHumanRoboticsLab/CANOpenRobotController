#include "InitState.h"

void InitState::entry(void) {
    std::cout
        << "==================================" << endl
        << " WELCOME TO THE TEST STATE MACHINE" << endl
        << "==================================" << endl
        << endl
        << "========================" << endl
        << " PRESS S to start program" << endl
        << "========================" << endl;
}
void InitState::during(void) {
}
void InitState::exit(void) {
    robot->initPositionControl();
    std::cout << "Initialise State Exited" << endl;
}
