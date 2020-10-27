#include "M1ForceSensor.h"

M1ForceSensor::M1ForceSensor(int sensorID) {

    this->sensorID = sensorID;

}

void M1ForceSensor::updateInput() {
    forceReading = sensorValueToNewton(*(&CO_OD_RAM.actualSensorForces.sensor1 + sensorID));
//    DEBUG_OUT("[M1ForceSensor::updateInput]: Force Sensor " << sensorID << " - " <<  forceReading << std::endl);
}

bool M1ForceSensor::calibrate() {
    DEBUG_OUT("[M1ForceSensor::calibrate]: Force Sensor " << sensorID << " Zeroing");

    std::stringstream sstream;
    char *returnMessage;

    sstream << "[1] " << 17 + sensorID << " read 0x7050 255 i8";
    std::cout << sstream.str() << "\n";
//    sstream << "[1] " << sensorID << " read 0x7050 255 i8";
    std::string strCommand = sstream.str();
    char *SDO_Message = (char *)(strCommand.c_str());
    cancomm_socketFree(SDO_Message, &returnMessage);
    std::string retMsg = returnMessage;
    DEBUG_OUT(retMsg)

    usleep(2000);
    if (retMsg.find("ERROR") != std::string::npos) {
        DEBUG_OUT("[M1ForceSensor::calibrate]: Force Sensor " << sensorID << " error occured during zeroing")
        return false;
    }
    else{
        DEBUG_OUT("[M1ForceSensor::calibrate]: Force Sensor " << sensorID << " succesfully zeroed.")
        return true;
    }
}

double M1ForceSensor::getForce() {

    return forceReading;

}

double M1ForceSensor::sensorValueToNewton(int sensorValue) {

    return (sensorValue-1500.0); // /4.0 todo: change after sensor experimentation
//    return sensorValue*4.0; // /4.0 todo: change after sensor experimentation

}


