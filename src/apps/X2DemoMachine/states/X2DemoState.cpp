#include "X2DemoState.h"

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

    robot->initVelocityControl();
    time0 = std::chrono::steady_clock::now();
}
void X2DemoState::during(void) {

    float t_final = 3.0;
    Eigen::VectorXd desiredVelocity(X2_NUM_JOINTS);
    std::cout<<robot->getPosition()[2]*180.0/M_PI<<std::endl;

    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count() < t_final*1000){
        desiredVelocity << 0, 0, 0, M_PI/180.0*(60.0/3.0);
        robot->setVelocity(desiredVelocity);

    }else{
        desiredVelocity << 0, 0, 0, 0;
        robot->setVelocity(desiredVelocity);
    }
}
void X2DemoState::exit(void) {
    std::cout << "Example State Exited" << std::endl;
}
