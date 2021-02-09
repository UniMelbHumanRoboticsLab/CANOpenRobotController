#include "SteppingRight.h"

void SteppingRight::entry(void) {
    spdlog::info(" Stepping RIGHT");
    trajectoryGenerator->initialiseTrajectory(robot->getCurrentMotion(), Foot::Left, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::StepR);
}
void SteppingRight::during(void) {
    robot->moveThroughTraj();
}
void SteppingRight::exit(void) {
    spdlog::debug("EXITING STEPPING RIGHT");
}
