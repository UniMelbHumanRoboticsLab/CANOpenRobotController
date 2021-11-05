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

CalibrateState::CalibrateState(StateMachine *m, ForcePlate *robot, const char *name) : State(m, name), robot(robot) {
    spdlog::debug("CalibrateState Created");
};
void CalibrateState::entry(void) {
    spdlog::debug("CalibrateState entry");
    spdlog::info("Calibrating....");

    Eigen::VectorXi strain = robot->getRawStrainReadings();

    strainReadings = Eigen::ArrayXXi::Zero(NUM_CALIBRATE_READINGS, strain.size());

    currReading =0;
};

void CalibrateState::during(void){
    // Collect data and save
    if (currReading< NUM_CALIBRATE_READINGS){
        strainReadings.row(currReading) = robot->getRawStrainReadings();
    }
    currReading = currReading+1;
};

void CalibrateState::exit(void) {
    // Take average of the matrices
    Eigen::VectorXi strainOffsets = Eigen::VectorXi::Zero(strainReadings.cols());

    // set offsets for strainGauges
    for (int i = 0; i < strainReadings.cols(); i++) {
        strainOffsets[i] = strainReadings.col(i).sum() / NUM_CALIBRATE_READINGS;
        spdlog::info("Strain Offset {}", strainOffsets[i]);
    }
    robot->setStrainOffsets(strainOffsets);

    spdlog::debug("CalibrateState Exit");
};

int CalibrateState::getCurrReading(void) {
    return currReading;
};