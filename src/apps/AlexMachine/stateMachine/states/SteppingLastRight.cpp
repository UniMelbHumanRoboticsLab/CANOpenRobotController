
#include "SteppingLastRight.h"

void SteppingLastRight::entry(void) {
    spdlog::info(" Stepping Last RIGHT");
    trajectoryGenerator->initialiseTrajectory(RobotMode::FTTG, Foot::Left, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::StepLastR);
}
void SteppingLastRight::during(void) {
    robot->moveThroughTraj();
}
void SteppingLastRight::exit(void) {
    spdlog::debug("EXITING STEPPING LAST RIGHT");
}
