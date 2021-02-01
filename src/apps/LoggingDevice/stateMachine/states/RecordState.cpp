/**
 * /file RecordState.cpp
 * /author Justin Fong
 * /brief Just records data
 * /version 0.1
 * /date 2021-1-21
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "RecordState.h"

RecordState::RecordState(StateMachine *m, LoggingRobot *robot, const char *name) : State(m, name), robot(robot) {
    spdlog::info("RecordState Created");
};
void RecordState::entry(void) {
    spdlog::info("RecordState entry");
    spdlog::info("S to Stop");
    robot->startSensors();
};

void RecordState::during(void){
    Eigen::VectorXd force = robot->getCrutchReadings();
    //spdlog::info("Fx1: {}, Fy1: {}, Fz1: {}, Fx2: {}, Fy2: {}, Fz2: {}", force[0], force[1], force[2], force[3], force[4], force[5]);
};

void RecordState::exit(void) {
    robot->stopSensors();
    spdlog::info("RecordState Exit");
};