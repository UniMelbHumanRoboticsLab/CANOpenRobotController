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
    Eigen::VectorXd crutchForce = robot->getCrutchReadings();
    Eigen::VectorXi forcePlateForce = robot->getForcePlateReadings();
    spdlog::info("Fz1: {}, Fz2: {}, Strain1: {}, Strain 2: {}", crutchForce[2], crutchForce[5], forcePlateForce[0], forcePlateForce[1]);
};

void RecordState::exit(void) {
    robot->stopSensors();
    spdlog::info("RecordState Exit");
};