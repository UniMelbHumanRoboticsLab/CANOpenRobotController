#include "SteppingFirstLeft.h"

void SteppingFirstLeft::entry(void) {
    spdlog::info("Stepping 1st Left");
    /*MUST HAVE A CHECK THAT Its the correct motion here as well - or throw an error and don't move!*/
    trajectoryGenerator->initialiseTrajectory(robot->getCurrentMotion(), robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::StepFirstL);
}
void SteppingFirstLeft::during(void) {
    robot->moveThroughTraj();
}
void SteppingFirstLeft::exit(void) {
    spdlog::debug("EXITING STEPPING FIRST LEFT");
}
