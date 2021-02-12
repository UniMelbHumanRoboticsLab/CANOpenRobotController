////////// STATE ////////////////////
//-------  Sitting Down ------------/////
////////////////////////////////////
#include "SittingDwn.h"
void SittingDwn::entry(void) {
    spdlog::info("Sitting Down State Entered ");
    std::cout << "===================" << std::endl
              << " GREEN -> SIT DOWN " << std::endl
              << "===================" << std::endl;
    trajectoryGenerator->initialiseTrajectory(SIT, 2, robot->getPosition());
    currTrajProgress = 0;
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
}
void SittingDwn::during(void) {
    timespec currTime;
    clock_gettime(CLOCK_MONOTONIC, &currTime);

    double elapsedSec = currTime.tv_sec - prevTime.tv_sec + (currTime.tv_nsec - prevTime.tv_nsec) / 1e9;
    prevTime = currTime;

    //if (robot->keyboard->getA() ) {
        currTrajProgress += elapsedSec;
        robot->setPosition(trajectoryGenerator->getSetPoint(currTrajProgress));
    //}
}
void SittingDwn::exit(void) {
    spdlog::info("Sitting Down State Exited ");
}
