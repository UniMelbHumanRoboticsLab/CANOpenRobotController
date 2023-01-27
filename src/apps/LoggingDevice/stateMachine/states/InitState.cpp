/**
 * /file InitState.cpp
 * /author Justin Fong
 * /brief Virtual Class to include all required classes for Logging Robot
 * /version 0.1
 * /date 2020-12-1
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "InitState.h"

InitState::InitState(LoggingRobot *robot, const char *name) : State(name), robot(robot) {
    spdlog::info("InitState Created");
};
void InitState::entry(void) {
    spdlog::info("InitState entry");
};

void InitState::during(void){
    //Eigen::VectorXd force = robot->getCrutchReadings();
    //spdlog::info("Fx1: {}, Fy1: {}, Fz1: {}, Fx2: {}, Fy2: {}, Fz2: {}", force[0], force[1], force[2], force[3], force[4], force[5]);
};

void InitState::exit(void) {
    spdlog::info("InitState Exit");
};