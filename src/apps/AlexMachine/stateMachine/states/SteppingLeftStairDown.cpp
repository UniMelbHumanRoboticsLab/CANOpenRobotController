#include "SteppingLeftStairDown.h"

void SteppingLeftStairDown::entry(void) {
    spdlog::info(" Stepping Left Stair Down");
    trajectoryGenerator->initialiseTrajectory(RobotMode::DWNSTAIR, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::BackStepL);
}
void SteppingLeftStairDown::during(void) {
    robot->moveThroughTraj();
}
void SteppingLeftStairDown::exit(void) {
    spdlog::debug("EXITING STEPPING LEFT STAIR DOWN");
}
