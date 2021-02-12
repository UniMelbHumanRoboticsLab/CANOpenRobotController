#include "M2ForceSensor.h"

M2ForceSensor::M2ForceSensor(int sensorID) : calibrated(false) {

    this->sensorID = sensorID;

}

void M2ForceSensor::updateInput() {

    forceReading = sensorValueToNewton(*(&CO_OD_RAM.actualSensorForces.sensor1 + sensorID));
}

bool M2ForceSensor::calibrate() {
    spdlog::debug("[M2ForceSensor::calibrate]: Force Sensor {} Zeroing", sensorID);

    //Todo: take zero calibration
}

double M2ForceSensor::getForce() {
    if(!calibrated) {
        spdlog::warn("[M2ForceSensor]: Force sensor {} value used but not zeroed.", sensorID);
    }
    return forceReading;

}

double M2ForceSensor::sensorValueToNewton(int sensorValue) {
    return (sensorValue-1500.0)*4.0; // TODO: CALIBRATE

}


