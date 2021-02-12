#include "M2ForceSensor.h"

M2ForceSensor::M2ForceSensor(int sensorID) : calibrated(false) {

    this->sensorID = sensorID;

}


bool M2ForceSensor::configureMasterPDOs() {
    UNSIGNED16 dataSize[2] = {4, 4};
    void *dataEntry[2] = {(void *)&rawData[0],
                           (void *)&rawData[1],};

    rpdo = new RPDO(0x191+sensorID, 0xff, dataEntry, dataSize, 2);

    return true;
}

void M2ForceSensor::updateInput() {
    forceReading = sensorValueToNewton((double) rawData[0]);
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


