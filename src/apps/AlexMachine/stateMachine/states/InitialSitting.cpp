#include "InitialSitting.h"

void InitialSitting::entry(void) {
    spdlog::info("PRESS GREEN TO SIT DOWN");
    trajectoryGenerator->initialiseTrajectory(RobotMode::INITIAL, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::InitSitting);
}
void InitialSitting::during(void) {
    // w/o crutch Go button
    //robot->pb.updateGO(true);
    robot->moveThroughTraj();
}
void InitialSitting::exit(void) {
    // w/o crutch Go button
    //robot->pb.updateGO(false);
    spdlog::debug("Initial SITTING DOWN POS:");
    robot->printStatus();
    spdlog::debug("Initial Sitting State Exited ");
}
