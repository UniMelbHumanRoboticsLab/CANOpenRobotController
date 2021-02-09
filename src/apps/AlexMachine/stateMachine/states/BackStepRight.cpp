#include "BackStepRight.h"

void BackStepRight::entry(void) {
    spdlog::info("==================");

    trajectoryGenerator->initialiseTrajectory(RobotMode::BKSTEP, Foot::Left, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::BackStepR);
}
void BackStepRight::during(void) {
    robot->moveThroughTraj();
}
void BackStepRight::exit(void) {
    spdlog::debug("EXITING Back step RIGHT");
}
