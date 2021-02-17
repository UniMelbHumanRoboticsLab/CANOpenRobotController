#include "SteppingLeft.h"

void SteppingLeft::entry(void) {
    spdlog::info(" Stepping Left");
    trajectoryGenerator->initialiseTrajectory(robot->getCurrentMotion(), robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::StepL);
}
void SteppingLeft::during(void) {
    robot->moveThroughTraj();
}
void SteppingLeft::exit(void) {
    spdlog::debug("EXITING STEPPING LEFT");
}
