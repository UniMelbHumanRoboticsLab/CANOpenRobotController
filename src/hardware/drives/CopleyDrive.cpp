/**
 * @brief An implementation of the Drive Object, specifically for the Copley Drive
 *
 */
#include "CopleyDrive.h"

#include <iostream>

CopleyDrive::CopleyDrive(int NodeID) : Drive::Drive(NodeID) {
    this->NodeID = NodeID;
}
CopleyDrive::~CopleyDrive() {
    spdlog::debug("CopleyDrive Deleted");
}

bool CopleyDrive::init() {
    return false;
}

bool CopleyDrive::initPosControl(motorProfile posControlMotorProfile) {
    spdlog::debug("Node     ID {} Initialising Position Control", NodeID);

    sendSDOMessages(generatePosControlConfigSDO(posControlMotorProfile));
    /**
     * \todo Move jointMinMap and jointMaxMap to set additional parameters (bit 5 in 0x6041 makes updates happen immediately)
     *
     */
    return true;
}

bool CopleyDrive::initVelControl(motorProfile velControlMotorProfile) {
    spdlog::debug("NodeID {} Initialising Velocity Control", NodeID);
    /**
     * \todo create velControlMOTORPROFILE and test on exo
     * \todo Tune velocity loop gain index 0x2381 to optimize V control
     *
    */
    sendSDOMessages(generateVelControlConfigSDO(velControlMotorProfile));
    return true;
}

bool CopleyDrive::initTorqueControl() {
    spdlog::debug("NodeID {} Initialising Torque Control", NodeID);
    sendSDOMessages(generateTorqueControlConfigSDO());

    return true;
}

std::vector<std::string> CopleyDrive::generatePosControlConfigSDO(motorProfile positionProfile) {
    return Drive::generatePosControlConfigSDO(positionProfile); /*<!execute base class function*/
};

std::vector<std::string> CopleyDrive::generateVelControlConfigSDO(motorProfile velocityProfile) {
    return Drive::generateVelControlConfigSDO(velocityProfile); /*<!execute base class function*/
};

std::vector<std::string> CopleyDrive::generateTorqueControlConfigSDO() {
    return Drive::generateTorqueControlConfigSDO(); /*<!execute base class function*/
}

std::vector<std::string> CopleyDrive::generatePositionOffsetSDO(int offset) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;

    // set mode of operation
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 6";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set the home offset
    sstream << "[1] " << NodeID << " write 0x607C 0 i32 "<< std::dec << offset;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set homing method to 0
    sstream << "[1] " << NodeID << " write 0X6098 0 i8 0";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set control word to start homing
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x1f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

bool CopleyDrive::setPositionOffset(int offset) {
    spdlog::debug("NodeID {} Setting Position Offset", NodeID);

    sendSDOMessages(generatePositionOffsetSDO(offset));

    return true;


}

