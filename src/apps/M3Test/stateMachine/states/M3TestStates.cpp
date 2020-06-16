#include "M3TestStates.h"

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

void M3TestState::entry(void) {
    M3State::entry();
}

void M3TestState::during(void) {
    M3State::during();
    std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
}

void M3TestState::exit(void) {
    M3State::exit();
}


//robot->initPositionControl();