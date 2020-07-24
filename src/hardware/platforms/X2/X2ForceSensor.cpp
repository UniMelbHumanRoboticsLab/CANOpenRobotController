#include "X2ForceSensor.h"

X2ForceSensor::X2ForceSensor(int sensorID) {

    this->sensorID = sensorID;

}

void X2ForceSensor::updateInput() {

    forceReading = sensorValueToNewton(*(&CO_OD_RAM.actualSensorForces.sensor1 + sensorID));
}

bool X2ForceSensor::calibrate() {
    DEBUG_OUT("[X2ForceSensor::calibrate]: Force Sensor " << sensorID << " Zeroing");

    std::stringstream sstream;
    char *returnMessage;

    sstream << "[1] " << 17+sensorID << " read 0x7050 255 i8";
    std::string strCommand = sstream.str();
    char *SDO_Message = (char *)(strCommand.c_str());
    cancomm_socketFree(SDO_Message, &returnMessage);
    std::string retMsg = returnMessage;
    DEBUG_OUT(retMsg)

    if (retMsg.find("ERROR") != std::string::npos) {
        DEBUG_OUT("[X2ForceSensor::calibrate]: Force Sensor " << sensorID << " error occured during zeroing")
        return false;
    }
    else{
        DEBUG_OUT("[X2ForceSensor::calibrate]: Force Sensor " << sensorID << " succesfully zeroed.")
        return true;
    }
}

double X2ForceSensor::getForce() {

    return forceReading;

}

double X2ForceSensor::sensorValueToNewton(int sensorValue) {

    return -(sensorValue-1500.0)*0.1606; //

}


