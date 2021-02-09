#include "InitState.h"

void InitState::entry(void) {
    spdlog::info("InitState Entered");
    std::cout
        << "==================================" << std::endl
        << " WELCOME TO THE TEST STATE MACHINE" << std::endl
        << "==================================" << std::endl
        << std::endl
        << "========================" << std::endl
        << " PRESS S to start or A to Home" << std::endl
        << "========================" << std::endl;
    robot->resetErrors();
}
void InitState::during(void) {
}
void InitState::exit(void) {
    robot->initPositionControl();
    robot->setPosControlContinuousProfile(true);
    spdlog::info("InitState Exited");
}
