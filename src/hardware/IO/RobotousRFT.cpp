#include "RobotousRFT.h"

RobotousRFT::RobotousRFT(int commandID_, int responseID1_, int responseID2_) {
    spdlog::info("Robotous Sensor Created");

    // Change the parameters
    commandID = commandID_; 
    responseID1 = responseID1_;
    responseID2 = responseID2_;

    // Change on the mapping parameters
    TPDOcommPara.COB_IDUsedByTPDO = commandID;
    RPDOcommParaH.COB_IDUsedByRPDO = responseID1;
    RPDOcommParaL.COB_IDUsedByRPDO = responseID2;

    setupPDO();

    // Initialise variables as zeros
    forces = Eigen::VectorXd::Zero(3);
    torques = Eigen::VectorXd::Zero(3);
}

void RobotousRFT::setupPDO(){
    spdlog::info("RobotousRFT {} - TPDO {} Set", commandID, CO_setTPDO(&TPDOcommPara, &TPDOMapParam, TPDOCommEntry, dataStoreRecordCmd, TPDOMapParamEntry));
    spdlog::info("RobotousRFT {} - RPDO {} Set", commandID, CO_setRPDO(&RPDOcommParaH, &RPDOMapParamH, RPDOCommEntryH, dataStoreRecordH, RPDOMapParamEntryH));
    spdlog::info("RobotousRFT {} - RPDO {} Set", commandID, CO_setRPDO(&RPDOcommParaL, &RPDOMapParamL, RPDOCommEntryL, dataStoreRecordL, RPDOMapParamEntryL));
}

void RobotousRFT::updateInput() {

    // If the last command was a streamed command, update the local copy of forces
    if (rawData[0] == 0x0B){
        spdlog::info("New data");
        UNSIGNED16 Fx = rawData[1] * 256 + rawData[2];
        UNSIGNED16 Fy = rawData[3] * 256 + rawData[4];
        UNSIGNED16 Fz = rawData[5] * 256 + rawData[6];
        UNSIGNED16 Tx = rawData[7] * 256 + rawData[8];
        UNSIGNED16 Ty = rawData[9] * 256 + rawData[10];
        UNSIGNED16 Tz = rawData[11] * 256 + rawData[12];


        forces[0] = static_cast<INTEGER16> (Fx)/50.0;
        forces[1] = static_cast<INTEGER16> (Fy)/50.0;
        forces[2] = static_cast<INTEGER16>(Fz)/50.0;

        torques[0] = static_cast<INTEGER16> (Tx)/2000.0;
        torques[1] = static_cast<INTEGER16> (Ty)/2000.0;
        torques[2] = static_cast<INTEGER16> (Tz)/2000.0;
    } 
    // Else, don't do anything
}

/**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd X,Y,Z forces
         */
Eigen::VectorXd& RobotousRFT::getForces() {
    spdlog::info("forces {}", forces[0]);
    return forces;
}

/**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd 
         */
Eigen::VectorXd& RobotousRFT::getTorques() {
    return torques;
}

bool RobotousRFT::startStream(){
    spdlog::info("RobotousRFT {} Starting", commandID);
    if (!streaming){
        cmdData =0x0B;
        streaming = true;
        return true; 
    }
    return false;
}
bool RobotousRFT::stopStream() {
    spdlog::info("RobotousRFT {} Stopping", commandID);
    if (streaming){
        cmdData = 0x0C;
        streaming = false;
        return true; 
    }
    return false;
}
bool RobotousRFT::getStreaming() {
    return streaming;
}