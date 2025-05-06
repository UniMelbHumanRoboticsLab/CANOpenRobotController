/**
 * \file CANDevice.h
 * \author Justin Fong
 * \brief  Generic CANDevice Class
 *
 * \version 0.1
 * \date 2021-01-12
 * \version 0.1
 *
 * \copyright Copyright (c) 2021
 *
 */



// TODO - THIS IS NOT FINISHED YET


#ifndef CANDEVICE_H
#define CANDEVICE_H

#include <CANopen.h>
#include <CO_command.h>
#include <string>
#include <vector>
#include <sstream>
#include <map>

#include "logging.h"
#include "RPDO.h"
#include "TPDO.h"
/**
 * Map of standard SDOs return error codes
 */
static std::map<std::string, std::string> SDO_Standard_Error = {
    {"0x05030000", "Toggle bit not changed"},
    {"0x05040000", "SDO protocol timed out"},
    {"0x05040001", "Client/server command specifier not valid or unknown"},
    {"0x05040002", "Invalid block size (block mode only)"},
    {"0x05040003", "Invalid sequence number (block mode only)"},
    {"0x05040004", "CRC error (block mode only)"},
    {"0x05040005", "Out of memory"},
    {"0x06010000", "Access to this object is not supported"},
    {"0x06010002", "Attempt to write to a Read_Only parameter"},
    {"0x06020000", "The object is not found in the object directory"},
    {"0x06040041", "The object can not be mapped into the PDO"},
    {"0x06040042", "The number and/or length of mapped objects would exceed the PDO length"},
    {"0x06040043", "General parameter incompatibility"},
    {"0x06040047", "General internal error in device"},
    {"0x06060000", "Access interrupted due to hardware error"},
    {"0x06070010", "Data type or parameter length do not agree or are unknown"},
    {"0x06070012", "Data type does not agree, parameter length too great"},
    {"0x06070013", "Data type does not agree, parameter length too short"},
    {"0x06090011", "Sub-index not present"},
    {"0x06090030", "General value range error"},
    {"0x06090031", "Value range error: parameter value too great"},
    {"0x06090032", "Value range error: parameter value too small"},
    {"0x060A0023", "Resource not available"},
    {"0x08000021", "Access not possible due to local application"},
    {"0x08000022", "Access not possible due to current device status"}
};

/**
 * @ingroup Robot
 * \brief Abstract class describing a CANOpen Device
 *
 */
class CANDevice {
   protected:
    /**
        * \brief The CAN Node ID used to address this particular device on the CAN bus
        *
        */
    int NodeID;

    /**
     * \brief Lists of PDOs
     *
     */
    std::vector<RPDO *> rpdos;
    std::vector<TPDO *> tpdos;
};

#endif
