////////// STATE ////////////////////
//-------  Sitting Down ------------/////
////////////////////////////////////
#include "SittingDwn.h"
void SittingDwn::entry(void) {
    std::cout << "Sitting Down State Entered " << std::endl
              << "===================" << std::endl
              << " GREEN -> SIT DOWN " << std::endl
              << "===================" << std::endl;

    // Initialise the current time
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
    trajFinished = false;
}

void SittingDwn::during(void) {
    // This will move immediately - there is no way to stop it.
    timespec currTime;
    clock_gettime(CLOCK_MONOTONIC, &currTime);

    double elapsedSec = currTime.tv_sec - prevTime.tv_sec + (currTime.tv_nsec - prevTime.tv_nsec) / 1e9;
    prevTime = currTime;

    double trajTime = 5;
    double progress = elapsedSec / trajTime;

    double x = 0;  // for task space
    double y = 0;  // for task space

    if (progress > 1) {
        progress = 1;
        trajFinished = true;
    }

    // Calc task space point
    // This is a circle, with the x position linear with time
    x = progress - 1;
    y = progress * progress + 1;

    // For IK
    double knee = acos((x * x + y * y - 2) / 2);
    double hip = atan2(y, x) - knee / 2;

    // These are defined in Radians
    Eigen::VectorXd angles(X2_NUM_JOINTS);
    angles << hip, knee, hip, knee;

    robot->setPosition(angles);
}
void SittingDwn::exit(void) {
    std::cout
        << "Sitting Down State Exited " << std::endl;
    trajFinished = false;
}