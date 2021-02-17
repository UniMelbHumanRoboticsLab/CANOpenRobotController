#include "SteppingLastLeft.h"

void SteppingLastLeft::entry(void) {
    spdlog::info(" Stepping Last Left");
    trajectoryGenerator->initialiseTrajectory(RobotMode::FTTG, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::StepLastL);
}
void SteppingLastLeft::during(void) {
    robot->moveThroughTraj();
}
void SteppingLastLeft::exit(void) {
    spdlog::debug("EXITING STEPPING LAST LEFT");
}
