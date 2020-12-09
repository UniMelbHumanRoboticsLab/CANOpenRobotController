#include "M2ForceSensor.h"

M2ForceSensor::M2ForceSensor(int sensorID) : calibrated(false) {

    this->sensorID = sensorID;

}

void M2ForceSensor::updateInput() {

    forceReading = sensorValueToNewton(*(&CO_OD_RAM.actualSensorForces.sensor1 + sensorID));
}

bool M2ForceSensor::calibrate() {
    spdlog::debug("[M2ForceSensor::calibrate]: Force Sensor {} Zeroing", sensorID);

    std::stringstream sstream;
    char *returnMessage;

    sstream << "[1] " << 17+sensorID << " read 0x7050 255 i8";
    std::string strCommand = sstream.str();
    char *SDO_Message = (char *)(strCommand.c_str());
    cancomm_socketFree(SDO_Message, &returnMessage);
    std::string retMsg = returnMessage;
    spdlog::debug(retMsg);

    if (retMsg.find("ERROR") != std::string::npos) {
        spdlog::error("[M2ForceSensor::calibrate]: Force Sensor {} error occured during zeroing", sensorID);
        calibrated = false;
        return false;
    }
    else{
        spdlog::debug("[M2ForceSensor::calibrate]: Force Sensor {} succesfully zeroed.", sensorID);
        calibrated = true;
        return true;
    }
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


