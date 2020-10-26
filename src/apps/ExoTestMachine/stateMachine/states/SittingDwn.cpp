////////// STATE ////////////////////
//-------  Sitting Down ------------/////
////////////////////////////////////
#include "SittingDwn.h"
void SittingDwn::entry(void) {
    spdlog::info("Sitting Down State Entered ");
    std::cout << "===================" << std::endl
              << " GREEN -> SIT DOWN " << std::endl
              << "===================" << std::endl;
    trajectoryGenerator->initialiseTrajectory(SIT, 1);
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
