////////// STATE ////////////////////
//-------  Sitting Down ------------/////
////////////////////////////////////
#include "SittingDwn.h"
void SittingDwn::entry(void) {
    spdlog::info("Sitting Down State Entered ");
#ifdef VIRTUAL
    spdlog::info("W ->> Complete trajectory");
#endif
    trajectoryGenerator->initialiseTrajectory(RobotMode::SITDWN, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::SittingDown);
}
void SittingDwn::during(void) {
    // w/o crutch Go button
    //robot->pb.updateGO(true);
    robot->moveThroughTraj();
}
void SittingDwn::exit(void) {
    // w/o crutch Go button
    //robot->pb.updateGO(false);
    spdlog::debug("EXIT SITTING DOWN POS:");
    robot->printStatus();
    spdlog::debug("Sitting Down State Exited ");
}