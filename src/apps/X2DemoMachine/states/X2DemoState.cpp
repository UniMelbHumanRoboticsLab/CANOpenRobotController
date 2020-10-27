#include "X2DemoState.h"

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

//    robot->initPositionControl();
    robot->initVelocityControl();
//    robot->initTorqueControl();
    time0 = std::chrono::steady_clock::now();

}
void X2DemoState::during(void) {

    float t_final = 1.0;
//    Eigen::VectorXd desiredPosition(X2_NUM_JOINTS);
    Eigen::VectorXd desiredVelocity(X2_NUM_JOINTS);
//    Eigen::VectorXd desiredTorque(X2_NUM_JOINTS);

    std::cout<<robot->getPosition()[1]*180.0/M_PI<<std::endl;

    double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
    if(time < t_final){

//        desiredPosition << 0, 0, 0, time*M_PI/180.0*(100.0/3.0);
//        robot->setPosition(desiredPosition);

        desiredVelocity << 0, M_PI/180.0*(60.0/3.0), 0, 0;
        robot->setVelocity(desiredVelocity);

//        desiredTorque << 0, 0, 0, time;
//        robot->setTorque(desiredTorque);


    }else{
//        desiredPosition << 0, 0, 0, 100*M_PI/180.0;
//        robot->setPosition(desiredPosition);

        desiredVelocity << 0, 0, 0, 0;
        robot->setVelocity(desiredVelocity);

//        desiredTorque << 0, 0, 0, 0;
//        robot->setTorque(desiredTorque);

    }
}
void X2DemoState::exit(void) {
    std::cout << "Example State Exited" << std::endl;
}
