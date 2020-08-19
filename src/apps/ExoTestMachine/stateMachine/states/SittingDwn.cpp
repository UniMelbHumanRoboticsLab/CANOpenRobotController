////////// STATE ////////////////////
//-------  Sitting Down ------------/////
////////////////////////////////////
#include "SittingDwn.h"
void SittingDwn::entry(void) {
    std::cout << "Sitting Down State Entered " << std::endl
              << "===================" << std::endl
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
        DEBUG_OUT("Elapsed Time: " << currTrajProgress)

        robot->setPosition(trajectoryGenerator->getSetPoint(currTrajProgress));
    }
}
void SittingDwn::exit(void) {
    std::cout << "Sitting Down State Exited " << std::endl;
}