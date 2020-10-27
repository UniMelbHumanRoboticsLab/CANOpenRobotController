#include "KincoDrive.h"

#include <iostream>

KincoDrive::KincoDrive(int NodeID) : Drive::Drive(NodeID) {
    //Remap torque reading and writting registers
    OD_Addresses[ACTUAL_TOR] = 0x6078;
    OD_Addresses[TARGET_TOR] = 0x60F6;
}
KincoDrive::~KincoDrive() {
    spdlog::debug("KincoDrive Deleted");
}

bool KincoDrive::init() {
    preop();//Set preop first to disable PDO during initialisation
    if(initPDOs()) {
        return true;
    }
    return false;
}
bool KincoDrive::init(motorProfile profile) {
    preop();//Set preop first to disable PDO during initialisation
    if(setMotorProfile(profile)) {
        if(initPDOs()) {
            return true;
        }
    }
    return false;
}


bool KincoDrive::initPosControl(motorProfile posControlMotorProfile) {
    spdlog::debug("NodeID {} Initialising Position Control", NodeID);

    sendSDOMessages(generatePosControlConfigSDO(posControlMotorProfile));
    /**
     * \todo Move jointMinMap and jointMaxMap to set additional parameters (bit 5 in 0x6041 makes updates happen immediately)
     *
     */
    return true;
}
bool KincoDrive::initPosControl() {
    spdlog::debug("NodeID {} Initialising Position Control", NodeID);

    sendSDOMessages(generatePosControlConfigSDO());
    return true;
}
bool KincoDrive::initVelControl(motorProfile velControlMotorProfile) {
    spdlog::debug("NodeID {} Initialising Velocity Control", NodeID);
    /**
     * \todo create velControlMOTORPROFILE and test on exo
     * \todo Tune velocity loop gain index 0x2381 to optimize V control
     *
    */
    sendSDOMessages(generateVelControlConfigSDO(velControlMotorProfile));
    return true;
}
bool KincoDrive::initVelControl() {
    spdlog::debug("NodeID {} Initialising Velocity Control", NodeID);

    sendSDOMessages(generateVelControlConfigSDO());
    return true;
}
bool KincoDrive::initTorqueControl() {
    spdlog::debug("NodeID {} Initialising Torque Control", NodeID);
    sendSDOMessages(generateTorqueControlConfigSDO());

    return true;
}


bool KincoDrive::initPDOs() {
    spdlog::debug("KincoDrive::initPDOs");

    spdlog::debug("Set up STATUS_WORD TPDO");
    if(sendSDOMessages(generateTPDOConfigSDO({STATUS_WORD}, 1, 0xFF))<0) {
        spdlog::error("Set up STATUS_WORD TPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up ACTUAL_POS and ACTUAL_VEL TPDO");
    if(sendSDOMessages(generateTPDOConfigSDO({ACTUAL_POS, ACTUAL_VEL}, 2, 0x01))<0) {
        spdlog::error("Set up ACTUAL_POS and ACTUAL_VEL TPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up ACTUAL_TOR TPDO");
    if(sendSDOMessages(generateTPDOConfigSDO({ACTUAL_TOR}, 3, 0x01))<0) {
        spdlog::error("Set up ACTUAL_TOR TPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up CONTROL_WORD RPDO");
    if(sendSDOMessages(generateRPDOConfigSDO({CONTROL_WORD}, 1, 0xff))<0) {
        spdlog::error("Set up CONTROL_WORD RPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up TARGET_POS RPDO");
    if(sendSDOMessages(generateRPDOConfigSDO({TARGET_POS}, 2, 0xff))<0) {
        spdlog::error("Set up TARGET_POS RPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up TARGET_VEL RPDO");
    if(sendSDOMessages(generateRPDOConfigSDO({TARGET_VEL}, 3, 0xff))<0) {
        spdlog::error("Set up ARGET_VEL RPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up TARGET_TOR RPDO");
    if(sendSDOMessages(generateRPDOConfigSDO({TARGET_TOR}, 4, 0xff, 0x08))<0) { //Kinco has a specific word for this with a dedicated subindex
        spdlog::error("Set up TARGET_TOR RPDO FAILED on node {}", NodeID);
        return false;
    }

    return true;
}
