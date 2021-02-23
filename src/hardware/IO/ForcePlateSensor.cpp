#include "ForcePlateSensor.h"

ForcePlateSensor::ForcePlateSensor(int commandID_, int responseID1_, int responseID2_) {
    spdlog::info("Force Plate Sensor Created");

    // Change the parameters
    commandID = commandID_; 
    responseID1 = responseID1_;
    responseID2 = responseID2_;

    // Initialise variables as zeros
    forces = Eigen::VectorXi::Zero(4);
}

int ForcePlateSensor::getCommandID() {
    return commandID;
}
bool ForcePlateSensor::configureMasterPDOs() {
    //spdlog::debug("ForcePlateSensor {} - TPDO {} Set", commandID, CO_setTPDO(&TPDOcommPara, &TPDOMapParam, TPDOCommEntry, dataStoreRecordCmd, TPDOMapParamEntry));

    UNSIGNED16 dataCmdSize[1] = {4};
    void *dataCmd[1] = {(void *) &command};

    tpdo1 = new TPDO(commandID, 0xff, dataCmd, dataCmdSize, 1);

    //spdlog::debug("ForcePlateSensor {} - RPDO {} Set", commandID, CO_setRPDO(&RPDOcommParaH, &RPDOMapParamH, RPDOCommEntryH, dataStoreRecordH, RPDOMapParamEntryH));
    UNSIGNED16 dataSize[2] = {4,4};
    void *dataEntry1[8] = {(void *)&forces(0), (void *)&forces(1)};
    void *dataEntry2[8] = {(void *)&forces(2), (void *)&forces(3)};
    rpdo1 = new RPDO(responseID1, 0xff, dataEntry1, dataSize, 2);
    rpdo2 = new RPDO(responseID2, 0xff, dataEntry2, dataSize, 2);

    return true;
}

void ForcePlateSensor::updateInput() {
    // Shouldn't need to do anything here - everything should be updated through PDOs 
}

Eigen::VectorXi& ForcePlateSensor::getForces() {
    return forces;
}

bool ForcePlateSensor::startStream(){
    spdlog::info("ForcePlateSensor 0x{0:x} Starting", commandID);

    if (!streaming){
        command = RECORD;
        streaming = true;
        return true; 
    }
    return false;
}
bool ForcePlateSensor::stopStream() {
    spdlog::info("ForcePlateSensor 0x{0:x} Stopping", commandID);

    if (streaming){
        command = STOP;
        streaming = false;
        forces = Eigen::VectorXi::Zero(4);
        return true; 
    }
    return false;
}

bool ForcePlateSensor::getStreaming() {
    return streaming;
}

bool ForcePlateSensor::zero() {
    command = CALIBRATE;
    return true;
}