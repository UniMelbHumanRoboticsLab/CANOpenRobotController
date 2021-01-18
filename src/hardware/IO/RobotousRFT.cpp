#include "RobotousRFT.h"

RobotousRFT::RobotousRFT(int commandID_, int responseID1_, int responseID2_) {
    // Change the parameters
    commandID = commandID_; 
    RPDOcommParaH.COB_IDUsedByRPDO = responseID1_;
    RPDOcommParaL.COB_IDUsedByRPDO = responseID2_;

    setupPDO();

    forces = Eigen::VectorXd::Zero(3);
    torques = Eigen::VectorXd::Zero(3);
}

void RobotousRFT::setupPDO(){
    spdlog::info("RobotousRFT {} - RPDO {} Set", commandID, CO_setRPDO(&RPDOcommParaH, &RPDOMapParamH, RPDOCommEntryH, dataStoreRecordH, RPDOMapParamEntryH));
    spdlog::info("RobotousRFT {} - RPDO {} Set", commandID, CO_setRPDO(&RPDOcommParaL, &RPDOMapParamL, RPDOCommEntryL, dataStoreRecordL, RPDOMapParamEntryL));
}

void RobotousRFT::update() {
    //
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

/**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd X,Y,Z forces
         */
Eigen::VectorXd RobotousRFT::getForces() {
    return forces;
}

/**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd 
         */
Eigen::VectorXd RobotousRFT::getTorques() {
    return torques;
}