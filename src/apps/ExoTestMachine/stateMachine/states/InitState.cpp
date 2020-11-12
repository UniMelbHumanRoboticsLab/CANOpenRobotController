#include "InitState.h"

void InitState::entry(void) {
    spdlog::info("InitState Entered");
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
    spdlog::info("InitState Exited");
}
