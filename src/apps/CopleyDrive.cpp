/**
 * @brief An implementation of the Drive Object, specifically for the Copley Drive
 * 
 */
#include "CopleyDrive.h"

#include <iostream>

#include "DebugMacro.h"

CopleyDrive::CopleyDrive(int NodeID) : Drive::Drive(NodeID) {
    this->NodeID = NodeID;
}
CopleyDrive::~CopleyDrive() {
    DEBUG_OUT(" CopleyDrive Deleted ")
}

bool CopleyDrive::Init() {
    return false;
}

bool CopleyDrive::initPosControl(motorProfile posControlMotorProfile) {
    DEBUG_OUT("NodeID " << NodeID << " Initialising Position Control")

    sendSDOMessages(generatePosControlConfigSDO(posControlMotorProfile));

    // \todo set additional parameters (bit 5 in 0x6041 makes updates happen immediately)
    return true;
}

bool CopleyDrive::initVelControl(motorProfile velControlMotorProfile) {
    DEBUG_OUT("NodeID " << NodeID << " Initialising Velocity Control")
    /*\todo create velControlMOTORPROFILE and test on exo*/
    /*\todo Tune velocity loop gain index 0x2381 to optimize V control */
    sendSDOMessages(generateVelControlConfigSDO(velControlMotorProfile));
    return true;
}

bool CopleyDrive::initTorqControl() {
    /**
     * \todo Implement the torque control initialisation
     * 
     */
    return false;
}
std::vector<std::string> CopleyDrive::generatePosControlConfigSDO(motorProfile positionProfile) {
    return Drive::generatePosControlConfigSDO(positionProfile); /*<!execute base class function*/
};

std::vector<std::string> CopleyDrive::generateVelControlConfigSDO(motorProfile velocityProfile) {
    return Drive::generateVelControlConfigSDO(velocityProfile); /*<!execute base class function*/
};