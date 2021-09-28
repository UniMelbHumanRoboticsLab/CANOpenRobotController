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
    spdlog::debug("NodeID {} KincoDrive::init()", NodeID);
    preop();//Set preop first to disable PDO during initialisation
    resetError();
    if(initPDOs()) {
        resetError();
        return true;
    }
    return false;
}
bool KincoDrive::init(motorProfile profile) {
    spdlog::debug("NodeID {} KincoDrive::init(motorProfile profile)", NodeID);
    preop();//Set preop first to disable PDO during initialisation
    resetError();
    if(setMotorProfile(profile)) {
        if(initPDOs()) {
            return true;
        }
    }
    return false;
}

bool KincoDrive::posControlConfirmSP() {
    // for kinco driver, there is no need to set postion control confirm
//    DEBUG_OUT("NodeID " << NodeID << " Kinco::posControlConfirmSP")
//    Drive::posControlConfirmSP();
    return true;
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

    sendSDOMessages(Drive::generatePosControlConfigSDO());
    return true;
}
bool KincoDrive::initVelControl(motorProfile velControlMotorProfile) {
    spdlog::debug("NodeID {} Initialising Velocity Control", NodeID);
    resetError();
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

bool KincoDrive::resetError(){
    spdlog::debug("NodeID {} reset error", NodeID);
    sendSDOMessages(generateResetErrorSDO());
    return true;
}

bool KincoDrive::initPDOs() {
    spdlog::debug("KincoDrive::initPDOs");
    spdlog::debug("Set up STATUS_WORD TPDO on Node {}", NodeID);
    int TPDO_Num = 1;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0xFF)) < 0) {
        spdlog::error("Set up STATUS_WORD TPDO FAILED on node {}", NodeID);
        return false;
    } 

    spdlog::debug("Set up ACTUAL_POS and ACTUAL_VEL TPDO on Node {}", NodeID);
    TPDO_Num = 2;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0x01)) < 0) {
        spdlog::error("Set up ACTUAL_POS and ACTUAL_VEL TPDO FAILED on node {}", NodeID);
        return false;
    } 


    spdlog::debug("Set up ACTUAL_TOR TPDO on Node {}", NodeID);
    TPDO_Num = 3;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0x01)) < 0) {
        spdlog::error("Set up ACTUAL_TOR TPDO FAILED on node {}", NodeID);
        return false;
    } 

    // Calculate COB_ID. If RPDO:
    //int COB_ID = 0x100 * (PDO_Num+1) + NodeID;
    spdlog::debug("Set up CONTROL_WORD RPDO on Node {}", NodeID);
    int RPDO_Num = 1;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up CONTROL_WORD RPDO FAILED on node {}", NodeID);
        return false;
    } 
    spdlog::debug("Set up TARGET_POS RPDO on Node {}", NodeID);
    RPDO_Num = 2;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up TARGET_POS RPDO FAILED on node {}", NodeID);
        return false;
    } 
    spdlog::debug("Set up TARGET_VEL RPDO on Node {}", NodeID);
    RPDO_Num = 3;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up ARGET_VEL RPDO FAILED on node {}", NodeID);
        return false;
    } 
    spdlog::debug("Set up TARGET_TOR RPDO on Node {}", NodeID);
    RPDO_Num = 4;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff, 0x08)) < 0) {
        spdlog::error("Set up TARGET_TOR RPDO FAILED on node {}", NodeID);
        return false;
    } 
    return true;
}
