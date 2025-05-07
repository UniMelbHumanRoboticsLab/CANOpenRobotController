#include "FITAbsEncoder.h"


FITAbsEncoder::FITAbsEncoder(int sensor_can_node_ID, double pulse_to_rad):
                            pulseToRadFactor(pulse_to_rad)
                            {
                              NodeID=sensor_can_node_ID;
                            }


double FITAbsEncoder::readValue() {

  std::string ret;
  int val=0;
  if(SDORead(ret)) {
    //return value is absolute position over 4 bytes. LSB first
    spdlog::debug("-{}-", ret);
    val = std::stoul(ret, 0, 16);
    spdlog::debug("{} {}", ret, val);
  }

  return pulseToRadFactor*val;

  //return pulseToRadFactor*val;
}


//TODO: generalise and move to CANDevice
bool FITAbsEncoder::SDORead(std::string &ret) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    sstream << "[1] " << NodeID << " read 0x9210 0 i16 " ; //Read a 4 bytes value (i16) from address 0x9210.
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    ret = "";

    if(sendSDOMessages(CANCommands, ret)<0) {
        spdlog::error("SDO read failed on node {}", NodeID);
        return false;
    }
    return true;
}


int FITAbsEncoder::sendSDOMessages(std::vector<std::string> messages, std::string &retMsg) {
    int successfulMessages = -1;
    for (auto strCommand : messages) {
        spdlog::trace(strCommand);

#ifndef NOROBOT
        // explicitly cast c++ string to from const char* to char* for use by cancomm function
        char *SDO_Message = (char *)(strCommand.c_str());
        char returnMessage[STRING_BUFFER_SIZE];
        cancomm_socketFree(SDO_Message, returnMessage);
        retMsg = returnMessage;

        // Because returnMessage includes sequence it is possible value is "[1] OK".
        // Therefore it is checked if return message includes the string "OK".
        // Another option would be erasing the sequence value before returning in cancomm_socketFree
        if(retMsg.find("0x")!=std::string::npos) {
            std::string errormsg = "sendSDOMessage: ERROR: " + strCommand;
            size_t err_code_l = 10;
            std::string error_code = retMsg.substr(retMsg.find("0x"), err_code_l);
            errormsg += " => " +  SDO_Standard_Error[error_code] + " (" + error_code + ")";
            spdlog::error(errormsg);
        }
        else {
            //Discard first few characters ("[1]") which is not part of response
            retMsg=retMsg.substr(3, retMsg.length());
            //Count
            successfulMessages++;
        }
        spdlog::trace(retMsg);
#else
        spdlog::trace("VCAN OK no reply.");
        successfulMessages++;
        retMsg = "00000000";
#endif
    }

    return successfulMessages;
}
