#include "M3TestStates.h"

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

void M3TestState::entry(void) {
    M3State::entry();
    
    std::cout
        << "==================================" << std::endl
        << " WELCOME TO M3 TEST STATE MACHINE " << std::endl
        << "==================================" << std::endl
        << std::endl;
        
        //robot->initPositionControl();
}

void M3TestState::during(void) {
    M3State::during();
    if((elapsedTime-int(elapsedTime)<0.1))
        std::cout << "Doing nothing... ("<< dt << "s)" << std::endl;
}

void M3TestState::exit(void) {
    M3State::exit();
    
    std::cout << "Initialise State Exited" << std::endl;
}