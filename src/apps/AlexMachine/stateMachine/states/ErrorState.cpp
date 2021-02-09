#include "ErrorState.h"

void ErrorState::entry(void) {
    spdlog::info(" ERROR STATE !!!!");
              
    robot->disableJoints();
    robot->setCurrentState(AlexState::Error);
}
void ErrorState::during(void) {
}
void ErrorState::exit(void) {
    spdlog::debug("EXITING ERROR STATE");
}
