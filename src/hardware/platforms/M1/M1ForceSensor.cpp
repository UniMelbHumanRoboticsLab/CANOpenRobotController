#include "M1ForceSensor.h"

M1ForceSensor::M1ForceSensor(int sensorID) {

    this->sensorID = sensorID;

}

void M1ForceSensor::updateInput() {
    forceReading = sensorValueToNewton(*(&CO_OD_RAM.actualSensorForces.sensor1 + sensorID));
//    DEBUG_OUT("[M1ForceSensor::updateInput]: Force Sensor " << sensorID << " - " <<  forceReading << std::endl);
}

bool M1ForceSensor::calibrate() {
    spdlog::debug("[M1ForceSensor::calibrate]: Force Sensor {} zeroing.", sensorID);

    std::stringstream sstream;
    char *returnMessage;

    sstream << "[1] " << 17 + sensorID << " read 0x7050 255 i8";
    std::cout << sstream.str() << "\n";
//    sstream << "[1] " << sensorID << " read 0x7050 255 i8";
    std::string strCommand = sstream.str();
    char *SDO_Message = (char *)(strCommand.c_str());
    cancomm_socketFree(SDO_Message, &returnMessage);
    std::string retMsg = returnMessage;

    usleep(2000);
    if (retMsg.find("ERROR") != std::string::npos) {
//        DEBUG_OUT("[M1ForceSensor::calibrate]: Force Sensor " << sensorID << " error occured during zeroing")
        spdlog::debug("[M1ForceSensor::calibrate]: Force Sensor {} error occured during zeroing.", sensorID);
        return false;
    }
    else{
        spdlog::debug("[M1ForceSensor::calibrate]: Force Sensor {} succesfully zeroed.", sensorID);
        return true;
    }
}

double M1ForceSensor::getForce() {

    return forceReading;

}

double M1ForceSensor::sensorValueToNewton(int sensorValue) {
//    std::cout << "Sensor readings : " << sensorValue << "\n";
    if(sensorValue > 1000)
        return (sensorValue-1500.0)*0.1;
    else
        return sensorValue*0.1; // /4.0 todo: change after sensor experimentation
//    return sensorValue*4.0; // /4.0 todo: change after sensor experimentation

}


