#include "X2DemoState.h"

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

//    robot->initPositionControl();
//    robot->initVelocityControl();
    robot->initTorqueControl();
    time0 = std::chrono::steady_clock::now();

}
void X2DemoState::during(void) {

    float t_final = 1.0;
//    Eigen::VectorXd desiredPosition(X2_NUM_JOINTS);
//    Eigen::VectorXd desiredVelocity(X2_NUM_JOINTS);
    Eigen::VectorXd desiredTorque(X2_NUM_JOINTS);

    int motionIntend;
    if(robot->getPosition()[1]>M_PI/4.0) motionIntend = -1;
    else motionIntend = 1;

    desiredTorque = robot->getFeedForwardTorque(motionIntend);

    robot->setTorque(desiredTorque);
    std::cout<<desiredTorque[1]<<std::endl;
}
void X2DemoState::exit(void) {
    std::cout << "Example State Exited" << std::endl;
}
