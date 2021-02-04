/**
 * /file CalibrateState.cpp
 * /author Justin Fong
 * /brief Calibration - takes sensors and offsets sensors by NUM_CALIBRATE_READINGS second average
 * /version 0.1
 * /date 2021-1-21
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "CalibrateState.h"

CalibrateState::CalibrateState(StateMachine *m, LoggingRobot *robot, const char *name) : State(m, name), robot(robot) {
    spdlog::info("CalibrateState Created");
};
void CalibrateState::entry(void) {
    spdlog::info("CalibrateState entry");
    spdlog::info("Calibrating....");

    Eigen::VectorXd force = robot->getCrutchReadings();
    robot->startSensors();
    readings = Eigen::ArrayXXd::Zero(NUM_CALIBRATE_READINGS, force.size());
    currReading =0;
};

void CalibrateState::during(void){
    // Collect data and save
    if (currReading< NUM_CALIBRATE_READINGS){
        Eigen::VectorXd force = robot->getCrutchReadings();
        readings.row(currReading) = force;
    }
    currReading = currReading+1;
};

void CalibrateState::exit(void) {
    // Take average of the matrices
    Eigen::VectorXd force = robot->getCrutchReadings();
    Eigen::VectorXd offsets = Eigen::VectorXd::Zero(force.size());

    // Set offsets
    for (int i = 0; i < force.size(); i++){
        offsets[i] = readings.col(i).sum()/NUM_CALIBRATE_READINGS;
        spdlog::info("Offset {}", offsets[i]);
    }
    robot->setCrutchOffsets(offsets);
    robot->stopSensors();
    spdlog::info("CalibrateState Exit");
};

int CalibrateState::getCurrReading(void) {
    return currReading;
};