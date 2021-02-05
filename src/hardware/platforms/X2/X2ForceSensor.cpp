#include "X2ForceSensor.h"

X2ForceSensor::X2ForceSensor(int sensorID, double scaleFactor) {

    this->sensorID = sensorID;
    this->scaleFactor_ = scaleFactor;
}

bool X2ForceSensor::configureMasterPDOs() {
    UNSIGNED16 dataSize[2] = {4, 4};
    void *dataEntry[2] = {(void *)&rawData[0],
                           (void *)&rawData[1],};

    rpdo = new RPDO(0x191+sensorID, 0xff, dataEntry, dataSize, 2);

    return true;
}

void X2ForceSensor::updateInput() {
    forceReading_ = (double) rawData[0];
}

bool X2ForceSensor::calibrate() {
    spdlog::debug("[X2ForceSensor::calibrate]: Force Sensor {} Zeroing", sensorID);

    std::stringstream sstream;
    char *returnMessage;

    sstream << "[1] " << 17+sensorID << " read 0x7050 255 i8";
    std::string strCommand = sstream.str();
    char *SDO_Message = (char *)(strCommand.c_str());
    cancomm_socketFree(SDO_Message, &returnMessage);
    std::string retMsg = returnMessage;
    spdlog::debug(retMsg);
    if (retMsg.find("ERROR") != std::string::npos) {
        spdlog::error("[X2ForceSensor::calibrate]: Force Sensor {} error occured during zeroing", sensorID);
        return false;
    }
    else{
        sleep(1.5); // this is required because after calibration command, sensor values do not get update around 1.2 seconds
        time0 = std::chrono::steady_clock::now();
        // after sending calibrate command, for some reason sensor value doesn't go back exactly to 1500.
        // Therefore readings are recorded for a second. And average is used as a calibration offset
        std::vector<double> readingVector;
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        double calibrationTime = 2.0; // amount of time readings are recorded for calibration [sec]
        while(time < calibrationTime){
            time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
            readingVector.push_back((double) rawData[0]);
        }
        calibrationOffset_ = std::accumulate(readingVector.begin(), readingVector.end(), 0.0)/readingVector.size();

        spdlog::debug("[X2ForceSensor::calibrate]: Force Sensor {} succesfully zeroed with offset {}.", sensorID, calibrationOffset_);
        return true;
    }
}

double X2ForceSensor::getForce() {

    return sensorValueToNewton(forceReading_ - calibrationOffset_, scaleFactor_);
}

double X2ForceSensor::sensorValueToNewton(int sensorValue, double scaleFactor) {
    
    return sensorValue*scaleFactor;
}

