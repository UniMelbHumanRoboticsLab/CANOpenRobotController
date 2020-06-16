#include "M3TestState.h"

M3TestState::M3TestState(StateMachine *m, RobotM3 *M3, const char *name) : State(m, name), robot(M3)
{}


void M3TestState::entry(void) {
    std::cout
        << "==================================" << std::endl
        << " WELCOME TO M3 TEST STATE MACHINE " << std::endl
        << "==================================" << std::endl
        << std::endl;
		
		//robot->initPositionControl();
}

void M3TestState::during(void) {
	std::cout << "Doing nothing..." << std::endl;
}

void M3TestState::exit(void) {
    std::cout << "Initialise State Exited" << std::endl;
}