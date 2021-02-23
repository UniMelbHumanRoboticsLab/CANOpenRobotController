/**
 * /file InitState.cpp
 * /author Justin Fong
 * /brief Virtual Class to include all required classes for ForcePlate Robot
 * /version 0.1
 * /date 2020-12-1
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "InitState.h"

InitState::InitState(StateMachine *m, ForcePlate *robot, const char *name) : State(m, name), robot(robot) {
    spdlog::debug("InitState Created");
};
void InitState::entry(void) {
    spdlog::debug("InitState entry");
};

void InitState::during(void){
    //Eigen::VectorXd force = robot->getCrutchReadings();
    //spdlog::info("Fx1: {}, Fy1: {}, Fz1: {}, Fx2: {}, Fy2: {}, Fz2: {}", force[0], force[1], force[2], force[3], force[4], force[5]);
};

void InitState::exit(void) {
    spdlog::debug("InitState Exit");
};