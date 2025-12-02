#include "EPOSDrive.h"


EPOSDrive::EPOSDrive(int NodeID) : Drive::Drive(NodeID) {

}
EPOSDrive::~EPOSDrive() {
    spdlog::debug("EPOSDrive Deleted");
}

bool EPOSDrive::init() {
    spdlog::debug("NodeID {} EPOSDrive::init()", NodeID);
    preop();//Set preop first to disable PDO during initialisation
    if(initPDOs()) {
        return true;
    }
    return false;
}

bool EPOSDrive::init(motorProfile profile) {
    spdlog::debug("NodeID {} EPOSDrive::init(motorProfile profile)", NodeID);
    preop();//Set preop first to disable PDO during initialisation
    if(setMotorProfile(profile)) {
        if(initPDOs()) {
            return true;
        }
    }
    return false;
}

bool EPOSDrive::initPosControl(motorProfile posControlMotorProfile) {
    spdlog::debug("NodeID {} Initialising Position Control", NodeID);
    spdlog::error("EPOSDrive::generatePosControlConfigSDO not implemented with profile. Use alternative one.");
    return false;

    //sendSDOMessages(generatePosControlConfigSDO(posControlMotorProfile));
    //return true;
}

bool EPOSDrive::initVelControl(motorProfile velControlMotorProfile) {
    spdlog::debug("NodeID {} Initialising Velocity Control", NodeID);
    spdlog::error("EPOSDrive::generateVelControlConfigSDO not implemented with profile. Use alternative one.");
    return false;

    //sendSDOMessages(generateVelControlConfigSDO(velControlMotorProfile));
    //return true;
}

bool EPOSDrive::initVelControl() {
    spdlog::debug("NodeID {} Initialising Velocity Control", NodeID);

    resetError();
    sendSDOMessages(EPOSDrive::generateVelControlConfigSDO());
    return true;
}

bool EPOSDrive::initTorqueControl() {
    spdlog::debug("NodeID {} Initialising Torque Control", NodeID);
    sendSDOMessages(generateTorqueControlConfigSDO());

    return true;
}


//! Use direct velocity control without profile
std::vector<std::string> EPOSDrive::generateVelControlConfigSDO() {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //enable profile Velocity mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 9";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}


//! EPOS drive has a CST torque mode only at value 10
std::vector<std::string> EPOSDrive::generateTorqueControlConfigSDO() {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //enable Torque Control mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 10";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}



bool EPOSDrive::cmdTimeoutConfig(int timeout_in_ms) {
    spdlog::debug("NodeID {} Config Timeout to {} ms", NodeID, timeout_in_ms);
    sendSDOMessages(generateCmdTimeoutConfigSDO(timeout_in_ms));

    return true;
}

//! SDO to define the timeout in cyclic velocity and position modes
std::vector<std::string> EPOSDrive::generateCmdTimeoutConfigSDO(int timeout_in_ms) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive

    //write to 0x60C2 sub index 01 (subindex 2 is -3 by default to set ms as unit)
    sstream << "[1] " << NodeID << " write 0x60C2 1 i8 " << timeout_in_ms;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}


bool EPOSDrive::resetError(){
    spdlog::debug("NodeID {} reset error", NodeID);
    sendSDOMessages(generateResetErrorSDO());
    return true;
}

std::vector<std::string> EPOSDrive::generateResetErrorSDO() {
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

    // re-Enable
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x0F";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}


bool EPOSDrive::setFaultMask(UNSIGNED32 mask) {
    spdlog::debug("NodeID {} Fault mask set to {0:x}", NodeID, mask);

    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    sstream << "[1] " << NodeID << " write 0x2182 0 i32 " << std::dec << mask;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    sendSDOMessages(CANCommands);

    return true;
}


int EPOSDrive::SDOReadErrorWord() {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    sstream << "[1] " << NodeID << " read 0x1001 0 u16 " ; //Read a 1 byte value from address 0x1001.

    int errWord=-1;

    #ifndef NOROBOT
        std::string strCommand, retMsg;
        strCommand = sstream.str();
        // explicitly cast c++ string to from const char* to char* for use by cancomm function
        char *SDO_Message = (char *)(strCommand.c_str());
        char returnMessage[STRING_BUFFER_SIZE];
        cancomm_socketFree(SDO_Message, returnMessage);
        retMsg = returnMessage;

        // Because returnMessage includes sequence it is possible value is "[1] OK".
        // Therefore it is checked if return message includes the string "OK".
        if(retMsg.find("0x")!=std::string::npos) {
            std::string errormsg = "sendSDOMessage: ERROR: " + strCommand;
            size_t err_code_l = 10;
            std::string error_code = retMsg.substr(retMsg.find("0x"), err_code_l);
            errormsg += " => " +  SDO_Standard_Error[error_code] + " (" + error_code + ")";
            spdlog::error("SDO read Error word failed on node {}", NodeID);
            spdlog::error(errormsg);
            errWord=-1;
        }
        else {
            //Discard first few characters ("[1]") which is not part of response
            spdlog::trace("SDO read on node {} return: {}", NodeID, retMsg);
            retMsg=retMsg.substr(3, retMsg.length());
            std::string hex[1];
            hex[0]=retMsg.substr(1, 2);
            errWord = std::stoul(hex[0], 0, 16);
        }
    #else
        spdlog::trace("VCAN OK no reply.");
        errWord=0;
    #endif
    return errWord;
}
