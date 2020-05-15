#include "Drive.h"

#include "DebugMacro.h"

Drive::Drive() {
    statusWord = 0;
    error = 0;
    this->NodeID = -1;
}

Drive::Drive(int NodeID) {
    statusWord = 0;
    error = 0;
    this->NodeID = NodeID;
}

int Drive::getNodeID() {
    return NodeID;
}

bool Drive::setPos(int position) {
    DEBUG_OUT("Drive " << this->NodeID << " Writing " << position << " to 0x607A");
    *(&CO_OD_RAM.targetMotorPositions.motor1 + ((this->NodeID - 1))) = position;
    return true;
}

bool Drive::setVel(int velocity) {
    DEBUG_OUT("Drive " << NodeID << " Writing " << velocity << " to 0x60FF");
    *(&CO_OD_RAM.targetMotorVelocities.motor1 + ((this->NodeID - 1))) = velocity;
    return true;
}

bool Drive::setTorque(int torque) {
    /*\todo add setTorque to object dictionary*/
    DEBUG_OUT("Drive " << NodeID << " Writing " << torque << " to 0x6071");
    *(&CO_OD_RAM.targetMotorTorques.motor1 + ((this->NodeID - 1))) = torque;
    return true;
}

int Drive::getPos() {
    /*\todo change to actual motor Position when running on real robot*/
    int q = *(&CO_OD_RAM.targetMotorPositions.motor1 + ((this->NodeID - 1)));
    return q;
}

int Drive::getVel() {
    return (*(&CO_OD_RAM.actualMotorVelocities.motor1 + ((this->NodeID - 1))));
}

int Drive::getTorque() {
    /* \todo Remove assumption that only drives 1-4 have access to the motor torques  
    */
    if (this->NodeID < 5) {
        return (*(&CO_OD_RAM.actualMotorTorques.motor1 + ((this->NodeID - 1))));
    } else {
        return 0;
    }
}

bool Drive::readyToSwitchOn() {
    *(&CO_OD_RAM.controlWords.motor1 + ((this->NodeID - 1))) = 0x06;
    driveState = READY_TO_SWITCH_ON;
}

bool Drive::enable() {
    *(&CO_OD_RAM.controlWords.motor1 + ((this->NodeID - 1))) = 0x0F;
    driveState = ENABLED;
}

bool Drive::disable() {
    *(&CO_OD_RAM.controlWords.motor1 + ((this->NodeID - 1))) = 0x00;
    driveState = DISABLED;
}

DriveState Drive::getDriveState() {
    return driveState;
}

int Drive::updateDriveStatus() {
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
    DEBUG_OUT("Drive::initPDOs")
    //DEBUG_OUT("Set up STATUS_WORD TPDO")
    sendSDOMessages(generateTPDOConfigSDO({STATUS_WORD}, 1, 0xFF));

    //DEBUG_OUT("Set up ACTUAL_POS and ACTUAL_VEL TPDO")
    sendSDOMessages(generateTPDOConfigSDO({ACTUAL_POS, ACTUAL_VEL}, 2, 1));

    //DEBUG_OUT("Set up ACTUAL_TOR TPDO")
    sendSDOMessages(generateTPDOConfigSDO({ACTUAL_TOR}, 3, 1));

    //DEBUG_OUT("Set up TARGET_POS RPDO")
    sendSDOMessages(generateRPDOConfigSDO({TARGET_POS}, 3, 0xff));

    //DEBUG_OUT("Set up TARGET_VEL RPDO")
    sendSDOMessages(generateRPDOConfigSDO({TARGET_VEL}, 4, 0xff));

    //DEBUG_OUT("Set up TARGET_TOR RPDO")
    sendSDOMessages(generateRPDOConfigSDO({TARGET_TOR}, 5, 0xff));
    return true;
}

std::vector<std::string> Drive::generateTPDOConfigSDO(std::vector<OD_Entry_t> items, int PDO_Num, int SyncRate) {
    // TODO: Do a check to make sure that the OD_Entry_t items can be transmitted.

    // Calculate COB_ID. If TPDO:
    int COB_ID = 0x100 * PDO_Num + 0x80 + NodeID;

    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;

    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;

    // Disable PDO
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1800 + PDO_Num - 1 << " 1 u32 0x" << std::hex << 0x800000000 + COB_ID;
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

    for (int i = 1; i <= items.size(); i++) {
        // Set transmit parameters
        sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1A00 + PDO_Num - 1 << " " << i << " u32 0x" << std::hex << OD_Addresses[items[i - 1]] * 0x10000 + OD_Data_Size[items[i - 1]];
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

std::vector<std::string> Drive::generateRPDOConfigSDO(std::vector<OD_Entry_t> items, int PDO_Num, int UpdateTiming) {
    /* \todo Do a check to make sure that the OD_Entry_t items can be Received.*/

    // Calculate COB_ID. If TPDO:
    int COB_ID = 0x100 * PDO_Num + NodeID;

    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;

    // Define stringstream for ease of constructing hex strings

    std::stringstream sstream;
    // Disable PDO
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1400 + PDO_Num - 1 << " 1 u32 0x" << std::hex << 0x800000000 + COB_ID;
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

    for (int i = 1; i <= items.size(); i++) {
        // Set transmit parameters
        sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1600 + PDO_Num - 1 << " " << i << " u32 0x" << std::hex << OD_Addresses[items[i - 1]] * 0x10000 + OD_Data_Size[items[i - 1]];
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
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x6060 << " 0 i8 1";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set velocity profile
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x6081 << " 0 i32 " << positionProfile.profileVelocity;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set acceleration profile
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x6083 << " 0 i32 " << positionProfile.profileAccelration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set deceleration profile
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x6084 << " 0 i32 " << positionProfile.profileDeceleration;
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
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x6060 << " 0 i8 3";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set velocity profile
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x6081 << " 0 i32 " << velocityProfile.profileVelocity;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set acceleration profile
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x6083 << " 0 i32 " << velocityProfile.profileAccelration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set deceleration profile
    sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x6084 << " 0 i32 " << velocityProfile.profileDeceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

int Drive::sendSDOMessages(std::vector<std::string> messages) {
    char *returnMessage;
    DEBUG_OUT("sendSDOMessages");
    // change to = 0 when testing with real network or something which responds. and uncomment
    // return message check
    int successfulMessages = 1;
    for (auto strCommand : messages) {
        // explicitly cast c++ string to from const char* to char* for use by cancomm function
        char *SDO_Message = (char *)(strCommand.c_str());
        // DEBUG_OUT(SDO_Message);

#ifndef NOROBOT
        cancomm_socketFree(SDO_Message, returnMessage);
#endif

        // TODO: in cancomm_socketFree -> return message correctly.
        // std::string retMsg = returnMessage;
        // DEBUG_OUT(retMsg);
        /*if (strcmp(returnMessage, "OK") == 0)
        {
            successfulMessages++;
        }*/
    }
    return successfulMessages;
}
