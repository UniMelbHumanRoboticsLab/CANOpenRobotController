/**
 * @brief An implementation of the Drive Object, specifically for the Kinco Drive
 *
 */
#include "KincoDrive.h"

#include <iostream>

#include "DebugMacro.h"

KincoDrive::KincoDrive(int NodeID) : Drive::Drive(NodeID) {
    //Remap torque reading value
    OD_Addresses[ACTUAL_TOR] = 0x6078;
    //OD_Addresses[TARGET_TOR] = 0x60F6;//VINCENT
}

KincoDrive::~KincoDrive() {
    DEBUG_OUT(" KincoDrive Deleted ")
}

bool KincoDrive::Init() {
    preop();//Set preop first to disable PDO during initialisation
    if(initPDOs()) {
        start();
        return true;
    }
    return false;
}


bool KincoDrive::initPosControl(motorProfile posControlMotorProfile) {
    DEBUG_OUT("NodeID " << NodeID << " Initialising Position Control")

    sendSDOMessages(generatePosControlConfigSDO(posControlMotorProfile));
    /**
     * \todo Move jointMinMap and jointMaxMap to set additional parameters (bit 5 in 0x6041 makes updates happen immediately)
     *
     */
    return true;
}

bool KincoDrive::initVelControl(motorProfile velControlMotorProfile) {
    DEBUG_OUT("NodeID " << NodeID << " Initialising Velocity Control")
    /**
     * \todo create velControlMOTORPROFILE and test on exo
     * \todo Tune velocity loop gain index 0x2381 to optimize V control
     *
    */
    sendSDOMessages(generateVelControlConfigSDO(velControlMotorProfile));
    return true;
}

bool KincoDrive::initTorqueControl() {
    DEBUG_OUT("NodeID " << NodeID << " Initialising Torque Control")
    sendSDOMessages(generateTorqueControlConfigSDO());

    return true;
}
