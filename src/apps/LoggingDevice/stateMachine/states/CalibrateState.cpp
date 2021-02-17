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
    Eigen::VectorXi strain = robot->getRawStrainReadings();

    robot->startSensors();
    readings = Eigen::ArrayXXd::Zero(NUM_CALIBRATE_READINGS, force.size());
    strainReadings = Eigen::ArrayXXi::Zero(NUM_CALIBRATE_READINGS, strain.size());

    currReading =0;
};

void CalibrateState::during(void){
    // Collect data and save
    if (currReading< NUM_CALIBRATE_READINGS){
        //Eigen::VectorXd force = robot->getCrutchReadings();
        readings.row(currReading) = robot->getCrutchReadings();
        strainReadings.row(currReading) = robot->getRawStrainReadings();
    }
    currReading = currReading+1;
};

void CalibrateState::exit(void) {
    // Take average of the matrices
    //Eigen::VectorXd force = robot->getCrutchReadings();
    Eigen::VectorXd offsets = Eigen::VectorXd::Zero(readings.cols());
    Eigen::VectorXi strainOffsets = Eigen::VectorXi::Zero(strainReadings.cols());

    // Set offsets for crutches
    for (int i = 0; i < readings.cols(); i++) {
        offsets[i] = readings.col(i).sum()/NUM_CALIBRATE_READINGS;
        spdlog::info("Crutch Offset {}", offsets[i]);
    }
    robot->setCrutchOffsets(offsets);
    robot->stopSensors();

    // set offsets for strainGauges
    for (int i = 0; i < strainReadings.cols(); i++) {
        strainOffsets[i] = strainReadings.col(i).sum() / NUM_CALIBRATE_READINGS;
        spdlog::info("Strain Offset {}", strainOffsets[i]);
    }
    robot->setStrainOffsets(strainOffsets);

    spdlog::info("CalibrateState Exit");
};

int CalibrateState::getCurrReading(void) {
    return currReading;
};