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

RecordState::RecordState(StateMachine *m, ForcePlate *robot, const char *name) : State(m, name), robot(robot) {
    spdlog::debug("RecordState Created");
};
void RecordState::entry(void) {
    spdlog::info("RecordState entry");
    spdlog::info("S to Stop");
};

void RecordState::during(void){
    Eigen::VectorXd strain = robot->getStrainReadings();
    spdlog::info("S1: {:03.2f}, S2: {:03.2f}, S3: {:03.2f}, S4: {:03.2f}", strain[0], strain[1], strain[2], strain[3]);
};

void RecordState::exit(void) {
    spdlog::debug("RecordState Exit");
};