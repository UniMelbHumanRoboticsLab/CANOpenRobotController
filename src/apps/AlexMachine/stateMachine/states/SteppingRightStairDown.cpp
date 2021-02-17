#include "SteppingRightStairDown.h"

void SteppingRightStairDown::entry(void)
{
    spdlog::info(" Stepping RIGHT STAIR DOWN");
#ifdef VIRTUAL
    splog::info(" W ->> Complete trajectory");
#endif
    trajectoryGenerator->initialiseTrajectory(RobotMode::DWNSTAIR, Foot::Left, robot->getJointStates());
    robot->startNewTraj();
    robot->setCurrentState(AlexState::BackStepR);
}
void SteppingRightStairDown::during(void)
{
    robot->moveThroughTraj();
}
void SteppingRightStairDown::exit(void)
{
    spdlog::debug("EXITING STEPPING RIGHT STAIR DOWN");
}
