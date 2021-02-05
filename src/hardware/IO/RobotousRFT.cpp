#include "RobotousRFT.h"

RobotousRFT::RobotousRFT(int commandID_, int responseID1_, int responseID2_) {
    spdlog::info("Robotous Sensor Created");

    // Change the parameters
    commandID = commandID_; 
    responseID1 = responseID1_;
    responseID2 = responseID2_;

    // Initialise variables as zeros
    forces = Eigen::VectorXd::Zero(3);
    torques = Eigen::VectorXd::Zero(3);

    forceOffsets = Eigen::VectorXd::Zero(3);
    torqueOffsets = Eigen::VectorXd::Zero(3);
}

int RobotousRFT::getCommandID() {
    return commandID;
}
bool RobotousRFT::configureMasterPDOs() {
    //spdlog::info("RobotousRFT {} - TPDO {} Set", commandID, CO_setTPDO(&TPDOcommPara, &TPDOMapParam, TPDOCommEntry, dataStoreRecordCmd, TPDOMapParamEntry));

    UNSIGNED16 dataCmdSize[2] = {1,7};
    void *dataCmd[2] = {(void *) &cmdData, (void *) &cmdDataPad};

    tpdo1 = new TPDO(commandID, 0xff, dataCmd, dataCmdSize, lengthCmd);

    //spdlog::info("RobotousRFT {} - RPDO {} Set", commandID, CO_setRPDO(&RPDOcommParaH, &RPDOMapParamH, RPDOCommEntryH, dataStoreRecordH, RPDOMapParamEntryH));
    UNSIGNED16 dataSize[8] = {1,1,1,1,1,1,1,1};
    void *dataEntryH[8] = {(void *)&rawData[0],
                            (void *)&rawData[1],
                            (void *)&rawData[2],
                            (void *)&rawData[3],
                            (void *)&rawData[4],
                            (void *)&rawData[5],
                            (void *)&rawData[6],
                            (void *)&rawData[7]};
    void *dataEntryL[8] = {(void *)&rawData[8],
                          (void *)&rawData[9],
                          (void *)&rawData[10],
                          (void *)&rawData[11],
                          (void *)&rawData[12],
                          (void *)&rawData[13],
                          (void *)&rawData[14],
                          (void *)&rawData[15]};
    rpdo1 = new RPDO(responseID1, 0xff, dataEntryH, dataSize, lengthData);
    rpdo2 = new RPDO(responseID2, 0xff, dataEntryL, dataSize, lengthData);

    return true;
}

void RobotousRFT::updateInput() {
    // If the last command was a streamed command, update the local copy of forces
    if (rawData[0] == 0x0B){
        // This is not that efficient if data is stopped. This could be modified to do a check first
        UNSIGNED16 Fx = rawData[1] * 256 + rawData[2];
        UNSIGNED16 Fy = rawData[3] * 256 + rawData[4];
        UNSIGNED16 Fz = rawData[5] * 256 + rawData[6];
        UNSIGNED16 Tx = rawData[7] * 256 + rawData[8];
        UNSIGNED16 Ty = rawData[9] * 256 + rawData[10];
        UNSIGNED16 Tz = rawData[11] * 256 + rawData[12];

        forces[0] = static_cast<INTEGER16> (Fx)/50.0 - forceOffsets[0];
        forces[1] = static_cast<INTEGER16>(Fy) / 50.0 - forceOffsets[1];
        forces[2] = static_cast<INTEGER16>(Fz) / 50.0 - forceOffsets[2];

        torques[0] = static_cast<INTEGER16> (Tx)/2000.0 - torqueOffsets[0];
        torques[1] = static_cast<INTEGER16>(Ty) / 2000.0 - torqueOffsets[1];
        torques[2] = static_cast<INTEGER16>(Tz) / 2000.0 - torqueOffsets[2];
    } 
    // Else, don't do anything
}

Eigen::VectorXd& RobotousRFT::getForces() {
    return forces;
}

Eigen::VectorXd& RobotousRFT::getTorques() {
    return torques;
}

void RobotousRFT::setOffsets(Eigen::VectorXd forceOffset, Eigen::VectorXd torqueOffset) {
        forceOffsets = forceOffset; 
        torqueOffsets = torqueOffset;
}

bool RobotousRFT::startStream(){
    spdlog::info("RobotousRFT 0x{0:x} Starting", commandID);
    if (!streaming){
        cmdData =0x0B;
        streaming = true;
        return true; 
    }
    return false;
}
bool RobotousRFT::stopStream() {
    spdlog::info("RobotousRFT 0x{0:x} Stopping", commandID);
    if (streaming){
        cmdData = 0x0C;
        streaming = false;
        forces = Eigen::VectorXd::Zero(3);
        torques = Eigen::VectorXd::Zero(3);
        return true; 
    }
    return false;
}

bool RobotousRFT::getStreaming() {
    return streaming;
}