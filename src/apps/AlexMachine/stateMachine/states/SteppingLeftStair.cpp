#include "SteppingLeftStair.h"

void SteppingLeftStair::entry(void) {
    spdlog::info(" Stepping Left Stair");
    trajectoryGenerator->initialiseTrajectory(RobotMode::UPSTAIR, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::StepL);
}
void SteppingLeftStair::during(void) {
    robot->moveThroughTraj();
}
void SteppingLeftStair::exit(void) {
    spdlog::debug("EXITING STEPPING LEFT STAIR");
}
