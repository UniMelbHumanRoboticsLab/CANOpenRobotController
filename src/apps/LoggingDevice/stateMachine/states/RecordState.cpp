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

RecordState::RecordState(LoggingRobot *robot, const char *name) : State(name), robot(robot) {
    spdlog::info("RecordState Created");
};
void RecordState::entry(void) {
    spdlog::info("RecordState entry");
    spdlog::info("S to Stop");
    lastCrutchForce = robot->getCrutchReadings();

    robot->startSensors();
};

void RecordState::during(void){
    Eigen::VectorXd crutchForce = robot->getCrutchReadings();

    // Check if some sensors are not responding properly every one second
    if(ticker % 100 == 99){
        bool ok =true;
        if (lastCrutchForce.isApprox(crutchForce)){
            spdlog::error("Crutches Not Updating");
            ok = false;
        }
        if (ok) {
            spdlog::info("Recording...");
        }
        lastCrutchForce = crutchForce;
    }
    ticker++;

};

void RecordState::exit(void) {
    robot->stopSensors();
    spdlog::info("RecordState Exit");
};
