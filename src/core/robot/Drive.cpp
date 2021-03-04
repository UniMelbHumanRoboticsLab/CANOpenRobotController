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
    // Needs to undo PDOS
    for (auto p : rpdos) {
        spdlog::debug("Deleting RPDO (COB-ID: 0x{0:x})", p->getCOBID());
        delete p;
    }
    for (auto p : tpdos) {
        spdlog::debug("Deleting TPDO (COB-ID: 0x{0:x})", p->getCOBID());
        delete p;
    }
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
    targetPos = position;
    return true;
}

bool Drive::setVel(int velocity) {
    spdlog::trace("Drive {} Writing {} to 0x60FF", NodeID, velocity);
    targetVel = velocity;
    return true;
}

bool Drive::setTorque(int torque) {
    /**
    * \todo add setTorque to object dictionary for all drives
    *
    */
    spdlog::trace("Drive {} Writing {} to 0x{0:x}", NodeID, (short int)torque, OD_Addresses[TARGET_TOR]);
    targetTor = torque;
    return true;
}


int Drive::getPos() {
    return actualPos;
}

int Drive::getVel() {
    return actualVel;
}

int Drive::getTorque() {
    /**
    *  \todo Remove assumption that only drives 1-4 have access to the motor torques
    *
    */
    if (this->NodeID < 5) {
        return actualTor;
    } else {
        return 0;
    }
}

DriveState Drive::resetErrors() {
    controlWord = 0x80;
    driveState = DISABLED;
    return driveState;
}


DriveState Drive::readyToSwitchOn() {
    controlWord = 0x06;
    driveState = READY_TO_SWITCH_ON;
    return driveState;
}

DriveState Drive::enable() {
    controlWord = 0x0F;
    driveState = ENABLED;
    return driveState;
}

DriveState Drive::disable() {
    controlWord = 0x00;
    driveState = DISABLED;
    return driveState;
}

DriveState Drive::getState() {
    return driveState;
}

int Drive::getStatus() {
    return statusWord;
}

bool Drive::posControlConfirmSP() {
    controlWord = controlWord ^ 0x10;
    if (((controlWord ^ 0x10 )& 0x10) > 0) {
        return false;
    } else {
        return true;
    }
}

bool Drive::posControlSetContinuousProfile(bool continuous) {
    if (driveState == ENABLED){
        if (continuous){
            controlWord = controlWord | 0x20;
        } else{
            controlWord = controlWord & ~0x20;
        }
        return true;
    } else {
        return false;
    }
}

bool Drive::configureMasterPDOs(){
    // Set up the PDOs in the OD here
    for (int TPDO_Num = 1; TPDO_Num <= 3; TPDO_Num++){
        generateEquivalentMasterRPDO(TPDO_MappedObjects[TPDO_Num], TPDO_COBID[TPDO_Num] + NodeID, 0xff);
        }
    for (int RPDO_Num = 1; RPDO_Num <= 4; RPDO_Num++){
        generateEquivalentMasterTPDO(RPDO_MappedObjects[RPDO_Num], RPDO_COBID[RPDO_Num] + NodeID, 0xff);
    }

    return true; 
}

bool Drive::initPDOs() {
    spdlog::debug("Drive::initPDOs");

    // Calculate COB_ID. If TPDO:
    //int COB_ID = 0x100 * PDO_Num + 0x80 + NodeID;
    int TPDO_Num = 1;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0xFF)) < 0) {
        spdlog::error("Set up STATUS_WORD TPDO FAILED on node {}", NodeID);
        return false;
    } 

    spdlog::info("Set up ACTUAL_POS and ACTUAL_VEL TPDO on Node {}", NodeID);
    TPDO_Num = 2;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0x01)) < 0) {
        spdlog::error("Set up ACTUAL_POS and ACTUAL_VEL TPDO FAILED on node {}", NodeID);
        return false;
    } 


    spdlog::info("Set up ACTUAL_TOR TPDO on Node {}", NodeID);
    TPDO_Num = 3;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0x01)) < 0) {
        spdlog::error("Set up ACTUAL_TOR TPDO FAILED on node {}", NodeID);
        return false;
    } 

    // Calculate COB_ID. If RPDO:
    //int COB_ID = 0x100 * (PDO_Num+1) + NodeID;
    spdlog::info("Set up CONTROL_WORD RPDO on Node {}", NodeID);
    int RPDO_Num = 1;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up CONTROL_WORD RPDO FAILED on node {}", NodeID);
        return false;
    } 
    spdlog::info("Set up TARGET_POS RPDO on Node {}", NodeID);
    RPDO_Num = 2;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up TARGET_POS RPDO FAILED on node {}", NodeID);
        return false;
    } 
    spdlog::info("Set up TARGET_VEL RPDO on Node {}", NodeID);
    RPDO_Num = 3;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up ARGET_VEL RPDO FAILED on node {}", NodeID);
        return false;
    } 
    spdlog::info("Set up TARGET_TOR RPDO on Node {}", NodeID);
    RPDO_Num = 4;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
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

std::vector<std::string> Drive::generateTPDOConfigSDO(std::vector<OD_Entry_t> items, int PDO_Num, int COB_ID, int SyncRate, int sub_idx) {
    // TODO: Do a check to make sure that the OD_Entry_t items can be transmitted.

    // Calculate COB_ID. If TPDO:
    //int COB_ID = 0x100 * PDO_Num + 0x80 + NodeID;

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
        sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1A00 + PDO_Num - 1 << " " << i << " u32 0x" << std::hex << OD_Addresses[items[i - 1]] * 0x10000 + sub_idx * 0x100 + OD_DataSize[items[i - 1]]*8;
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

void Drive::generateEquivalentMasterRPDO(std::vector<OD_Entry_t> items, int COB_ID, int RPDOSyncRate) {
    void *variables[items.size()];
    UNSIGNED16 variableSize[items.size()];
    for (uint i = 0; i < items.size(); i++) {
        variables[i] = OD_MappedObjectAddresses[items[i]];
        variableSize[i] = OD_DataSize[items[i]];
    }
    // Add to the local (CORC-side) Object Dictionary
    rpdos.push_back(new RPDO(COB_ID, RPDOSyncRate, variables, variableSize, items.size()));

    spdlog::debug("Master RPDO (COB-ID 0x{0:x}) Setup for Node {}", COB_ID, NodeID);
}

std::vector<std::string> Drive::generateRPDOConfigSDO(std::vector<OD_Entry_t> items, int PDO_Num, int COB_ID, int UpdateTiming, int sub_idx) {
    /**
     *  \todo Do a check to make sure that the OD_Entry_t items can be Received
     *
     */


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
        sstream << "[1] " << NodeID << " write 0x" << std::hex << 0x1600 + PDO_Num - 1 << " " << i << " u32 0x" << std::hex << OD_Addresses[items[i - 1]] * 0x10000 + sub_idx * 0x100 + OD_DataSize[items[i - 1]]*8;
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

void Drive::generateEquivalentMasterTPDO(std::vector<OD_Entry_t> items, int COB_ID, int TPDOSyncRate) {
    void *variables[items.size()];
    UNSIGNED16 variableSize[items.size()];
    for (uint i = 0; i < items.size(); i++) {
        variables[i] = OD_MappedObjectAddresses[items[i]];
        variableSize[i] = OD_DataSize[items[i]];
    }
    // Add to the local (CORC-side) Object Dictionary
    tpdos.push_back(new TPDO(COB_ID, TPDOSyncRate, variables, variableSize, items.size()));
    spdlog::debug("Master TPDO (COB-ID 0x{0:x}) Setup for Node {}", COB_ID, NodeID);
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
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 3";
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


