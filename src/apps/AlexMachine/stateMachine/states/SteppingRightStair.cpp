#include "SteppingRightStair.h"

void SteppingRightStair::entry(void) {
    spdlog::info(" Stepping RIGHT STAIR");
    trajectoryGenerator->initialiseTrajectory(RobotMode::UPSTAIR, Foot::Left, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::StepR);
}
void SteppingRightStair::during(void) {
    robot->moveThroughTraj();
}
void SteppingRightStair::exit(void) {
    spdlog::debug("EXITING STEPPING RIGHT STAIR");
}
