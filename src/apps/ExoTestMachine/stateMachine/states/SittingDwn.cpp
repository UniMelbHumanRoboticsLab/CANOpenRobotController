////////// STATE ////////////////////
//-------  Sitting Down ------------/////
////////////////////////////////////
#include "SittingDwn.h"
void SittingDwn::entry(void) {
    spdlog::info("Sitting Down State Entered ");

    Eigen::VectorXd pos = robot->getPosition();

    trajectoryGenerator->initialiseTrajectory(SIT, 5, pos);
    currTrajProgress = 0;
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
}
void SittingDwn::during(void) {
    timespec currTime;
    clock_gettime(CLOCK_MONOTONIC, &currTime);

    double elapsedSec = currTime.tv_sec - prevTime.tv_sec + (currTime.tv_nsec - prevTime.tv_nsec) / 1e9;
    prevTime = currTime;

    /**
     *  /todo - Check if the GO button on the robot is pressed
     *
     */
    if (true) {
        currTrajProgress += elapsedSec;
        spdlog::debug("Elapsed Time: {}", currTrajProgress);

        robot->setPosition(trajectoryGenerator->getSetPoint(currTrajProgress));
    }
}
void SittingDwn::exit(void) {
    spdlog::info("Sitting Down State Exited ");
}
