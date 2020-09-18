/**
 * @brief An implementation of the Drive Object, specifically for the Kinco Drive
 *
 */
#include "KincoDrive.h"

#include <iostream>

#include "DebugMacro.h"

KincoDrive::KincoDrive(int NodeID) : Drive::Drive(NodeID) {
    //Remap torque reading and writting registers
    OD_Addresses[ACTUAL_TOR] = 0x6078;
    OD_Addresses[TARGET_TOR] = 0x60F6;
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

bool KincoDrive::posControlConfirmSP() {
    // for kinco driver, there is no need to set postion control confirm
//    DEBUG_OUT("NodeID " << NodeID << " Kinco::posControlConfirmSP")
//    Drive::posControlConfirmSP();
    return true;
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

bool KincoDrive::resetError(){
    DEBUG_OUT("NodeID " << NodeID << " reset error")
    sendSDOMessages(generateResetErrorSDO());
    return true;
}

bool KincoDrive::initPDOs() {
    DEBUG_OUT("KincoDrive::initPDOs")

    //DEBUG_OUT("Set up STATUS_WORD TPDO")
    if(sendSDOMessages(generateTPDOConfigSDO({STATUS_WORD}, 1, 0xFF))<0) {
        std::cout /*cerr is banned*/ << "Set up STATUS_WORD TPDO FAILED on node" << NodeID  <<std::endl;
        return false;
    }

    //DEBUG_OUT("Set up ACTUAL_POS and ACTUAL_VEL TPDO")
    if(sendSDOMessages(generateTPDOConfigSDO({ACTUAL_POS, ACTUAL_VEL}, 2, 0x01))<0) {
        std::cout /*cerr is banned*/ << "Set up ACTUAL_POS and ACTUAL_VEL TPDO FAILED on node" << NodeID <<std::endl;
        return false;
    }

    //DEBUG_OUT("Set up ACTUAL_TOR TPDO")
    if(sendSDOMessages(generateTPDOConfigSDO({ACTUAL_TOR}, 3, 0x01))<0) {
        std::cout /*cerr is banned*/ << "Set up ACTUAL_TOR TPDO FAILED on node" << NodeID <<std::endl;
        return false;
    }

    //DEBUG_OUT("Set up CONTROL_WORD RPDO")
    if(sendSDOMessages(generateRPDOConfigSDO({CONTROL_WORD}, 1, 0xff))<0) {
        std::cout /*cerr is banned*/ << "Set up CONTROL_WORD RPDO FAILED on node" << NodeID <<std::endl;
        return false;
    }

    //DEBUG_OUT("Set up TARGET_POS RPDO")
    if(sendSDOMessages(generateRPDOConfigSDO({TARGET_POS}, 2, 0xff))<0) {
        std::cout /*cerr is banned*/ << "Set up TARGET_POS RPDO FAILED on node" << NodeID <<std::endl;
        return false;
    }

    //DEBUG_OUT("Set up TARGET_VEL RPDO")
    if(sendSDOMessages(generateRPDOConfigSDO({TARGET_VEL}, 3, 0xff))<0) {
        std::cout /*cerr is banned*/ << "Set up ARGET_VEL RPDO FAILED on node" << NodeID <<std::endl;
        return false;
    }

    //DEBUG_OUT("Set up TARGET_TOR RPDO")
    if(sendSDOMessages(generateRPDOConfigSDO({TARGET_TOR}, 4, 0xff, 0x08))<0) { //Kinco has a specific word for this with a dedicated subindex
        std::cout /*cerr is banned*/ << "Set up TARGET_TOR RPDO FAILED on node" << NodeID <<std::endl;
        return false;
    }

    return true;
}


std::vector<std::string> KincoDrive::generatePosControlConfigSDO(motorProfile positionProfile) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set control word to power up (enable)
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x0f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //enable profile position mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 1";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set velocity profile
    sstream << "[1] " << NodeID << " write 0x6081 0 i32 " << std::dec << positionProfile.profileVelocity;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set acceleration profile
    sstream << "[1] " << NodeID << " write 0x6083 0 i32 " << std::dec << positionProfile.profileAcceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set deceleration profile
    sstream << "[1] " << NodeID << " write 0x6084 0 i32 " << std::dec << positionProfile.profileDeceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set instant position mode; important for kinco
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x103f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}
std::vector<std::string> KincoDrive::generateVelControlConfigSDO(motorProfile velocityProfile) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set control word to power up (enable)
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x0f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //enable profile Velocity mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 3";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set velocity loop gain
    sstream << "[1] " << NodeID << " write 0x60F9 1 u16 " << std::dec << 100;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set acceleration profile
    sstream << "[1] " << NodeID << " write 0x6083 0 i32 " << std::dec << velocityProfile.profileAcceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set deceleration profile
    sstream << "[1] " << NodeID << " write 0x6084 0 i32 " << std::dec << velocityProfile.profileDeceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

std::vector<std::string> KincoDrive::generateTorqueControlConfigSDO() {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    //enable Torque Control mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 4";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

std::vector<std::string> KincoDrive::generateResetErrorSDO() {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;

    // shutdown
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x06";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // reset fault
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x80";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // reset fault
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x06";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

std::vector<std::string> KincoDrive::readSDOMessage(int address, int datetype) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // read message from drive
    sstream.str(std::string());

    switch(datetype){
        case 1:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 u8";
            break;
        case 2:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 u16";
            break;
        case 3:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 i8";
            break;
        case 4:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 i32";
            break;
        default:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 u8";
            break;
    }

    std::cout << sstream.str() << "\n";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    sendSDOMessages(CANCommands);
    return CANCommands;
}

std::vector<std::string> KincoDrive::writeSDOMessage(int address, int value) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // read message from drive
    sstream.str(std::string());
    sstream << "[1] " << NodeID << " write 0x" << std::hex << address << " 0 i32 0x" << std::hex << value;
    std::cout << sstream.str() << "\n";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}