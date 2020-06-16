#include "M3TestStates.h"

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

void M3TestState::entry(void) {
    M3State::entry();
    std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "==================================" << std::endl
        << std::endl;
}

void M3TestState::during(void) {
    M3State::during();
    std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
}

void M3TestState::exit(void) {
    M3State::exit();
    std::cout << "Exit "<< getName() << std::endl;
}






void M3CalibState::entry(void) {
    M3State::entry();
    std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "==================================" << std::endl
        << std::endl;
        
        robot->initVelocityControl();
}

void M3CalibState::during(void) {
    M3State::during();
    //Move slowly on each joint until max force detected
    robot->setVelocity(vel);

    
    std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
}

void M3CalibState::exit(void) {
    M3State::exit();
    
    //Set joint init values
    
       
    std::cout << "Exit "<< getName() << std::endl;
}

