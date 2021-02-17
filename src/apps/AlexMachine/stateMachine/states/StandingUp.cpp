#include "StandingUp.h"

// Negative bending control machine
void StandingUp::entry(void)
{
    spdlog::info(" Standing Up State Entered");
#ifdef VIRTUAL
    spdlog::info(" W ->> Complete trajectory");
#endif
    trajectoryGenerator->initialiseTrajectory(RobotMode::STNDUP, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::StandingUp);
}

void StandingUp::during(void)
{
    // update go button do using keyboard d input.-> same as setting nm
    //robot->pb.updateGO(true);
    robot->moveThroughTraj();
}
void StandingUp::exit(void)
{
    //robot->pb.updateGO(false);
    robot->printStatus();
    spdlog::debug("Standing up motion State Exited");
}