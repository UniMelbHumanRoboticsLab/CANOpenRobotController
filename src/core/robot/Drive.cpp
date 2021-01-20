#include "Drive.h"

Drive::Drive() {
    statusWord = 0;
    error = 0;
    this->NodeID = -1;
}

Drive::Drive(int node_id) {
    statusWord = 0;
    error = 0;
    NodeID = node_id;
}

Drive::~Drive() {
}


int Drive::getNodeID() {
    return NodeID;
}

int Drive::preop() {
    // start drive (Node)
    std::stringstream sstream;
    std::vector<std::string> CANCommands;
    sstream << "[1] " << NodeID << " preop";
    CANCommands.push_back(sstream.str());

    return sendSDOMessages(CANCommands);
}

int Drive::start() {
    // start drive (Node)
    std::stringstream sstream;
    std::vector<std::string> CANCommands;
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());

    return sendSDOMessages(CANCommands);
}

int Drive::stop() {
    // stop drive (Node)
    std::stringstream sstream;
    std::vector<std::string> CANCommands;
    sstream << "[1] " << NodeID << " stop";
    CANCommands.push_back(sstream.str());

    return sendSDOMessages(CANCommands);
}


bool Drive::setPos(int position) {
    spdlog::trace("Drive {} Writing {} to 0x607A", NodeID, position);
    *(&CO_OD_RAM.targetMotorPositions.motor1 + (NodeID - 1)) = position;
    return true;
}

bool Drive::setVel(int velocity) {
    spdlog::trace("Drive {} Writing {} to 0x60FF", NodeID, velocity);
    *(&CO_OD_RAM.targetMotorVelocities.motor1 + (NodeID - 1)) = velocity;
    return true;
}

bool Drive::setTorque(int torque) {
    /**
    * \todo add setTorque to object dictionary for all drives
    *
    */
    spdlog::trace("Drive {} Writing {} to 0x{}", NodeID, (short int)torque, OD_Addresses[TARGET_TOR]);
    *(&CO_OD_RAM.targetMotorTorques.motor1 + ((this->NodeID - 1))) = torque;
    return true;
}


int Drive::getPos() {
    int q = *(&CO_OD_RAM.actualMotorPositions.motor1 + ((this->NodeID - 1)));
    return q;
}

int Drive::getVel() {
    return (*(&CO_OD_RAM.actualMotorVelocities.motor1 + ((this->NodeID - 1))));
}

int Drive::getTorque() {
    /**
    *  \todo Remove assumption that only drives 1-4 have access to the motor torques
    *
    */
    if (this->NodeID < 5) {
        return (*(&CO_OD_RAM.actualMotorTorques.motor1 + ((this->NodeID - 1))));
    } else {
        return 0;
    }
}


DriveState Drive::readyToSwitchOn() {
    *(&CO_OD_RAM.controlWords.motor1 + ((this->NodeID - 1))) = 0x06;
    driveState = READY_TO_SWITCH_ON;
    return driveState;
}

DriveState Drive::enable() {
    *(&CO_OD_RAM.controlWords.motor1 + ((this->NodeID - 1))) = 0x0F;
    driveState = ENABLED;
    return driveState;
}

DriveState Drive::disable() {
    *(&CO_OD_RAM.controlWords.motor1 + ((this->NodeID - 1))) = 0x00;
    driveState = DISABLED;
    return driveState;
}

DriveState Drive::getState() {
    return driveState;
}

int Drive::getStatus() {
    statusWord = *(&CO_OD_RAM.statusWords.motor1 + ((this->NodeID - 1)));
    return statusWord;
}

bool Drive::posControlConfirmSP() {
    int controlWord = *(&CO_OD_RAM.controlWords.motor1 + ((this->NodeID - 1)));
    *(&CO_OD_RAM.controlWords.motor1 + ((this->NodeID - 1))) = controlWord ^ 0x10;
    if ((controlWord & 0x10) > 0) {
        return false;
    } else {
        return true;
    }
}

bool Drive::initPDOs() {
    spdlog::debug("Drive::initPDOs");

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
    if(sendSDOMessages(generateRPDOConfigSDO({TARGET_TOR}, 4, 0xff, 0x00))<0) {
        spdlog::error("Set up TARGET_TOR RPDO FAILED on node {}", NodeID);
        return false;
    }

    return true;
}

bool Drive::setMotorProfile(motorProfile profile) {
    spdlog::debug("Drive::initMotorProfile");

    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;

     //Set velocity profile
    sstream << "[1] " << NodeID << " write 0x6081 0 i32 " << std::dec << profile.profileVelocity;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set acceleration profile
    sstream << "[1] " << NodeID << " write 0x6083 0 i32 " << std::dec << profile.profileAcceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set deceleration profile
    sstream << "[1] " << NodeID << " write 0x6084 0 i32 " << std::dec << profile.profileDeceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    if(sendSDOMessages(CANCommands)<0) {
        spdlog::error("Set up Velocity/Acceleration profile failed on node {}", NodeID);
        return false;
    }

    return true;
}

std::vector<std::string> Drive::generateTPDOConfigSDO(std::vector<OD_Entry_t> items, int PDO_Num, int SyncRate, int sub_idx) {
    // TODO: Do a check to make sure that the OD_Entry_t items can be transmitted.

    // Calculate COB_ID. If TPDO:
    int COB_ID = 0x100 * PDO_Num + 0x80 + NodeID;

    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;

    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;

    // Disable PDO
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1800 + PDO_Num - 1 << " 1 u32 0x" << std::hex << 0x80000000 + COB_ID;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // Set so that there no PDO items, enable mapping change
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1A00 + PDO_Num - 1 << " 0 u8 0";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // Set the PDO so that it triggers every SYNC Message
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1800 + PDO_Num - 1 << " 2 u8 0x" << SyncRate;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    for (unsigned int i = 1; i <= items.size(); i++) {
        // Set transmit parameters
        sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1A00 + PDO_Num - 1 << " " << i << " u32 0x" << std::hex << OD_Addresses[items[i - 1]] * 0x10000 + sub_idx * 0x100 + OD_Data_Size[items[i - 1]];
        CANCommands.push_back(sstream.str());
        sstream.str(std::string());
    }

    // Sets Number of PDO items to reenable
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1A00 + PDO_Num - 1 << " 0 u8 " << items.size();
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // Enable  PDO
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1800 + PDO_Num - 1 << " 1 u32 0x" << std::hex << COB_ID;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}
std::vector<std::string> Drive::generateRPDOConfigSDO(std::vector<OD_Entry_t> items, int PDO_Num, int UpdateTiming, int sub_idx) {
    /**
     *  \todo Do a check to make sure that the OD_Entry_t items can be Received
     *
     */

    // Calculate COB_ID. If RPDO:
    int COB_ID = 0x100 * (PDO_Num+1) + NodeID;

    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;

    // Define stringstream for ease of constructing hex strings

    std::stringstream sstream;
    // Disable PDO
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1400 + PDO_Num - 1 << " 1 u32 0x" << std::hex << 0x80000000 + COB_ID;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // Set so that there no PDO items, enable mapping change
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1600 + PDO_Num - 1 << " 0 u8 0";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // Set the PDO so that it triggers every SYNC Message
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1400 + PDO_Num - 1 << " 2 u8 0x" << UpdateTiming;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    for (unsigned int i = 1; i <= items.size(); i++) {
        // Set transmit parameters
        sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1600 + PDO_Num - 1 << " " << i << " u32 0x" << std::hex << OD_Addresses[items[i - 1]] * 0x10000 + sub_idx * 0x100 + OD_Data_Size[items[i - 1]];
        CANCommands.push_back(sstream.str());
        sstream.str(std::string());
    }

    // Sets Number of PDO items to reenable
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1600 + PDO_Num - 1 << " 0 u8 " << items.size();
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // Enable  PDO
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1400 + PDO_Num - 1 << " 1 u32 0x" << std::hex << COB_ID;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

std::vector<std::string> Drive::generatePosControlConfigSDO(motorProfile positionProfile) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
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

    return CANCommands;
}
std::vector<std::string> Drive::generatePosControlConfigSDO() {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    //enable profile position mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 1";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

std::vector<std::string> Drive::generateVelControlConfigSDO(motorProfile velocityProfile) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    //enable profile Velocity mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 3";
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
std::vector<std::string> Drive::generateVelControlConfigSDO() {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    //enable profile Velocity mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 0xFD";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

std::vector<std::string> Drive::generateTorqueControlConfigSDO() {
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

int Drive::sendSDOMessages(std::vector<std::string> messages) {
    int successfulMessages = 0;
    for (auto strCommand : messages) {
        spdlog::trace(strCommand);

#ifndef NOROBOT
        // explicitly cast c++ string to from const char* to char* for use by cancomm function
        char *SDO_Message = (char *)(strCommand.c_str());
        char *returnMessage;
        cancomm_socketFree(SDO_Message, &returnMessage);
        std::string retMsg = returnMessage;

        // Because returnMessage includes sequence it is possible value is "[1] OK".
        // Therefore it is checked if return message includes the string "OK".
        // Another option would be erasing the sequence value before returning in cancomm_socketFree
        if (retMsg.find("OK") != std::string::npos) {
            successfulMessages++;
        }
        else {
            std::string errormsg = "sendSDOMessage: ERROR: " + strCommand;
            if(retMsg.find("0x")!=retMsg.npos) {
                std::string error_code = retMsg.substr(retMsg.find("0x"), retMsg.npos);
                errormsg += " => " +  SDO_Standard_Error[error_code] + " (" + error_code + ")";
            }
            else {
                errormsg += " => " + retMsg;
            }
            spdlog::error(errormsg);
        }
        spdlog::trace(retMsg);
#else
        spdlog::trace("VCAN OK no reply.");
        successfulMessages++;
#endif
    }

    return successfulMessages-messages.size();
}


