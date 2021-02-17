#include "BackStepLeft.h"

void BackStepLeft::entry(void)
{
    spdlog::info("Back Stepping Left"); 

#ifdef VIRTUAL
    spdlog::info(" W ->> Complete trajectory");
#endif
    trajectoryGenerator->initialiseTrajectory(RobotMode::BKSTEP, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::BackStepL);
}
void BackStepLeft::during(void)
{
    robot->moveThroughTraj();
}
void BackStepLeft::exit(void)
{
    spdlog::debug("EXITING STEPPING LEFT");
}
