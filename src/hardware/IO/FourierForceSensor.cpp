#include "FourierForceSensor.h"

FourierForceSensor::FourierForceSensor(int sensor_can_node_ID, double scale_factor, double calib_time): InputDevice(),
                                                                                                        sensorNodeID(sensor_can_node_ID),
                                                                                                        scaleFactor(scale_factor),
                                                                                                        calibrated(false),
                                                                                                        calibrationTime(calib_time) {
}

bool FourierForceSensor::configureMasterPDOs() {
    UNSIGNED16 dataSize[2] = {4, 4};
    void *dataEntry[2] = {(void *)&rawData[0],
                           (void *)&rawData[1],};

    rpdo = new RPDO(0x190+sensorNodeID, 0xff, dataEntry, dataSize, 2);

    return true;
}

void FourierForceSensor::updateInput() {
    forceReading = sensorValueToNewton((double)rawData[0]);
}

bool FourierForceSensor::calibrate(double calib_time) {
    spdlog::debug("[FourierForceSensor::calibrate]: Force Sensor {} Zeroing", sensorNodeID);

    if(calib_time>0) {
        calibrationTime = calib_time;
    }

    //Check error message from sensor first
    std::stringstream sstream;
    char *returnMessage;
    sstream << "[1] " << 16+sensorNodeID << " read 0x7050 255 i8";
    std::string strCommand = sstream.str();
    char *SDO_Message = (char *)(strCommand.c_str());
    cancomm_socketFree(SDO_Message, &returnMessage);
    std::string retMsg = returnMessage;
    spdlog::debug(retMsg);
    if (retMsg.find("ERROR") != std::string::npos) {
        spdlog::error("[FourierForceSensor::calibrate]: Force Sensor {} error occured during zeroing", sensorNodeID);
        return false;
    }
    //Perform zeroing
    else{
        sleep(1.5); // this is required because after calibration command, sensor values do not get update around 1.2 seconds
        time0 = std::chrono::steady_clock::now();
        // after sending calibrate command, for some reason sensor value doesn't go back exactly to 1500.
        // Therefore readings are recorded for a second. And average is used as a calibration offset
        std::vector<double> readingVector;
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        while(time < calibrationTime){
            time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
            readingVector.push_back((double) rawData[0]);
        }
        calibrationOffset = std::accumulate(readingVector.begin(), readingVector.end(), 0.0)/readingVector.size();
        calibrated = true;
        spdlog::debug("[FourierForceSensor::calibrate]: Force Sensor {} succesfully zeroed with offset {}.", sensorNodeID, calibrationOffset);
        return true;
    }
}

double FourierForceSensor::getForce() {

    return forceReading;
}

double FourierForceSensor::sensorValueToNewton(int sensorValue) {

    return (sensorValue - calibrationOffset) * scaleFactor;
}

