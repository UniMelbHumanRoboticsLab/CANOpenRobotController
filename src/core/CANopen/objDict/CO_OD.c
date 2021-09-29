/*******************************************************************************

   File - CO_OD.c/CO_OD.h
   CANopen Object Dictionary.

   Copyright (C) 2004-2008 Janez Paternoster

   License: GNU Lesser General Public License (LGPL).

   <http://canopennode.sourceforge.net>

   (For more information see <CO_SDO.h>.)

   This file is part of CANopenNode, an opensource CANopen Stack.
   Project home page is <https://github.com/CANopenNode/CANopenNode>.
   For more information on CANopen see <http://www.can-cia.org/>.

   CANopenNode is free and open source software: you can redistribute
   it and/or modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation, either version 2 of the
   License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.

   Following clarification and special exception to the GNU General Public
   License is included to the distribution terms of CANopenNode:

   Linking this library statically or dynamically with other modules is
   making a combined work based on this library. Thus, the terms and
   conditions of the GNU General Public License cover the whole combination.

   As a special exception, the copyright holders of this library give
   you permission to link this library with independent modules to
   produce an executable, regardless of the license terms of these
   independent modules, and to copy and distribute the resulting
   executable under terms of your choice, provided that you also meet,
   for each linked independent module, the terms and conditions of the
   license of that module. An independent module is a module which is
   not derived from or based on this library. If you modify this
   library, you may extend this exception to your version of the
   library, but you are not obliged to do so. If you do not wish
   to do so, delete this exception statement from your version.

   This file was automatically generated with libedssharp Object
   Dictionary Editor v0.6-xdd-alpha-81-gb562769
   DON'T EDIT THIS FILE MANUALLY !!!!
*******************************************************************************/

#include "CO_OD.h"

#include "CO_SDO.h"
#include "CO_driver.h"

/*******************************************************************************
   DEFINITION AND INITIALIZATION OF OBJECT DICTIONARY VARIABLES
*******************************************************************************/

/***** Definition for ROM variables ********************************************/
struct sCO_OD_ROM CO_OD_ROM = {
    CO_OD_FIRST_LAST_WORD,

    /*100c*/ 0x00,
    /*1012*/ 0x0000L,

    CO_OD_FIRST_LAST_WORD,
};

/***** Definition for RAM variables ********************************************/
struct sCO_OD_RAM CO_OD_RAM = {
    CO_OD_FIRST_LAST_WORD,
    /*1000*/ 0x0000L,
    /*1001*/ 0x0L,
    /*1002*/ 0x0000L,
    /*1003*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1005*/ 0x40000080,
    /*1006*/ 0xF4240,
    /*1007*/ 0x0000L,
    /*1008*/ {'C', 'A', 'N', 'o', 'p', 'e', 'n', 'N', 'o', 'd', 'e'},
    /*1009*/ {'3', '.', '0', '0'},
    /*100a*/ {'3', '.', '0', '0'},
    /*100d*/ 0x0L,
    /*1010*/ {0x0003L},
    /*1011*/ {0x0001L},
    /*1013*/ 0x0000L,
    /*1014*/ 0x0080L,
    /*1015*/ 0x64,
    /*1016*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1017*/ 0x3E8,
    /*1018*/ {0x4L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1019*/ 0x0L,
    /*1029*/ {0x0L, 0x0L, 0x1L, 0x0L, 0x0L, 0x0L},
    /*1200*/ {{0x2L, 0x0600L, 0x0580L}},
    /*1280*/ {{0x3L, 0x0000L, 0x0000L, 0x0L}},
    /*1f80*/ 0x0000L,
    /*1f81*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1f82*/ {0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L},
    /*1f89*/ 0x0000L,
    /*2100*/ {0x0L},
    /*2101*/ 0x20L,
    /*2102*/ 0xfa,
    /*2103*/ 0x00,
    /*2104*/ 0x00,
    /*2106*/ 0x0000L,
    /*2107*/ {0x3e8, 0x00, 0x00, 0x00, 0x00},
    /*2108*/ {0x00},
    /*2109*/ {0x00},
    /*2130*/ {0x3L, {'-'}, 0x00000000L, 0x0000L},

    CO_OD_FIRST_LAST_WORD,
};

/***** Definition for EEPROM variables ********************************************/
struct sCO_OD_EEPROM CO_OD_EEPROM = {
    CO_OD_FIRST_LAST_WORD,

    CO_OD_FIRST_LAST_WORD,
};

/*******************************************************************************
   STRUCTURES FOR RECORD TYPE OBJECTS
*******************************************************************************/

/*0x1018*/ const CO_OD_entryRecord_t OD_record1018[5] = {
    {(void *)&CO_OD_RAM.identity.maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.identity.vendorID, 0x86, 0x4},
    {(void *)&CO_OD_RAM.identity.productCode, 0x86, 0x4},
    {(void *)&CO_OD_RAM.identity.revisionNumber, 0x86, 0x4},
    {(void *)&CO_OD_RAM.identity.serialNumber, 0x86, 0x4},
};

/*0x1200*/ const CO_OD_entryRecord_t OD_record1200[3] = {
    {(void *)&CO_OD_RAM.SDOServerParameter[0].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.SDOServerParameter[0].COB_IDClientToServer, 0x86, 0x4},
    {(void *)&CO_OD_RAM.SDOServerParameter[0].COB_IDServerToClient, 0x86, 0x4},
};

/*0x1280*/ const CO_OD_entryRecord_t OD_record1280[4] = {
    {(void *)&CO_OD_RAM.SDOClientParameter[0].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.SDOClientParameter[0].COB_IDClientToServer, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.SDOClientParameter[0].COB_IDServerToClient, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.SDOClientParameter[0].nodeIDOfTheSDOServer, 0x0e, 0x1},
};

// Parameters setting the RPDO to off
OD_RPDOCommunicationParameter_t RPDOCommParamOff = {0x2L, 0x80000000L, 0xffL};
OD_RPDOMappingParameter_t RPDOMapParamOff = {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L};

// Array of pointers to all RPDOs (initialised to off to start with)
OD_RPDOCommunicationParameter_t *OD_RPDOCommunicationParameter[CO_NO_RPDO] = {&RPDOCommParamOff};
OD_RPDOMappingParameter_t *OD_RPDOMappingParameter[CO_NO_RPDO] = {&RPDOMapParamOff};

// OD_record for Entries that are off
/*0x1400*/ CO_OD_entryRecord_t OD_recordRPDOCommOff[3] = {
    {(void *)&(RPDOCommParamOff.maxSubIndex), 0x06, 0x1},
    {(void *)&(RPDOCommParamOff.COB_IDUsedByRPDO), 0x8e, 0x4},
    {(void *)&(RPDOCommParamOff.transmissionType), 0x0e, 0x1},
};

/*0x1600*/ CO_OD_entryRecord_t OD_recordRPDOMapOff[9] = {
    {(void *)&RPDOMapParamOff.numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&RPDOMapParamOff.mappedObject1, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject2, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject3, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject4, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject5, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject6, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject7, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject8, 0x8e, 0x4},
};

// Parameters setting the TPDO to off
OD_TPDOCommunicationParameter_t TPDOCommParamOff = {0x6L, 0x80000000L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L};
OD_TPDOMappingParameter_t TPDOMapParamOff = {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L};

// Array of pointers to all TPDOs (initialised to off to start with)
OD_TPDOCommunicationParameter_t *OD_TPDOCommunicationParameter[CO_NO_TPDO] = {&TPDOCommParamOff};
OD_TPDOMappingParameter_t *OD_TPDOMappingParameter[CO_NO_TPDO] = {&TPDOMapParamOff};

/*0x1800*/ const CO_OD_entryRecord_t OD_recordTPDOCommOff[7] = {
    {(void *)&TPDOCommParamOff.maxSubIndex, 0x06, 0x1},
    {(void *)&TPDOCommParamOff.COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&TPDOCommParamOff.transmissionType, 0x0e, 0x1},
    {(void *)&TPDOCommParamOff.inhibitTime, 0x8e, 0x2},
    {(void *)&TPDOCommParamOff.compatibilityEntry, 0x0e, 0x1},
    {(void *)&TPDOCommParamOff.eventTimer, 0x8e, 0x2},
    {(void *)&TPDOCommParamOff.SYNCStartValue, 0x0e, 0x1},
};

/*0x1a00*/ const CO_OD_entryRecord_t OD_recordTPDOMapOff[9] = {
    {(void *)&TPDOMapParamOff.numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&TPDOMapParamOff.mappedObject1, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject2, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject3, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject4, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject5, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject6, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject7, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject8, 0x8e, 0x4},
};


/*0x2130*/ const CO_OD_entryRecord_t OD_record2130[4] = {
    {(void *)&CO_OD_RAM.time.maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.time.string, 0x06, 0x1},
    {(void *)&CO_OD_RAM.time.epochTimeBaseMs, 0x86, 0x8},
    {(void *)&CO_OD_RAM.time.epochTimeOffsetMs, 0x9e, 0x4},
};

INTEGER16 junkData =8;

const CO_OD_entryRecord_t OD_DummyDataStoreLocation[9] = {
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
};

/*******************************************************************************
   OBJECT DICTIONARY
*******************************************************************************/
CO_OD_entry_t CO_OD[CO_OD_NoOfElements];

bool_t CO_OD_set_entry(uint16_t element_, uint16_t index_, uint8_t maxSubIndex_, uint16_t attribute_, uint16_t length_, void *pData_) {
    CO_OD[element_].index = index_;
    CO_OD[element_].maxSubIndex = maxSubIndex_;
    CO_OD[element_].attribute = attribute_;
    CO_OD[element_].length = length_;
    CO_OD[element_].pData = pData_;
    return true;
}

bool_t CO_configure(void) {
    int i;
    CO_OD_set_entry(0, 0x1000, 0x00, 0x86, 4, (void *)&CO_OD_RAM.deviceType);
    CO_OD_set_entry(1, 0x1001, 0x00, 0x26, 1, (void *)&CO_OD_RAM.errorRegister);
    CO_OD_set_entry(2, 0x1002, 0x00, 0xa6, 4, (void *)&CO_OD_RAM.manufacturerStatusRegister);
    CO_OD_set_entry(3, 0x1003, 0x08, 0x8e, 4, (void *)&CO_OD_RAM.preDefinedErrorField[0]);
    CO_OD_set_entry(4, 0x1005, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.COB_ID_SYNCMessage);
    CO_OD_set_entry(5, 0x1006, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.communicationCyclePeriod);
    CO_OD_set_entry(6, 0x1007, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.synchronousWindowLength);
    CO_OD_set_entry(7, 0x1008, 0x00, 0x86, 11, (void *)&CO_OD_RAM.manufacturerDeviceName);
    CO_OD_set_entry(8, 0x1009, 0x00, 0x86, 4, (void *)&CO_OD_RAM.manufacturerHardwareVersion);
    CO_OD_set_entry(9, 0x100a, 0x00, 0x86, 4, (void *)&CO_OD_RAM.manufacturerSoftwareVersion);
    CO_OD_set_entry(10, 0x100c, 0x00, 0x85, 2, (void *)&CO_OD_ROM.guardTime);
    CO_OD_set_entry(11, 0x100d, 0x00, 0x06, 1, (void *)&CO_OD_RAM.lifeTimeFactor);
    CO_OD_set_entry(12, 0x1010, 0x01, 0x8e, 4, (void *)&CO_OD_RAM.storeParameters[0]);
    CO_OD_set_entry(13, 0x1011, 0x01, 0x8e, 4, (void *)&CO_OD_RAM.restoreDefaultParameters[0]);
    CO_OD_set_entry(14, 0x1012, 0x00, 0x85, 4, (void *)&CO_OD_ROM.COB_ID_TIME);
    CO_OD_set_entry(15, 0x1013, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.highResolutionTimeStamp);
    CO_OD_set_entry(16, 0x1014, 0x00, 0x86, 4, (void *)&CO_OD_RAM.COB_ID_EMCY);
    CO_OD_set_entry(17, 0x1015, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.inhibitTimeEMCY);
    CO_OD_set_entry(18, 0x1016, 0x04, 0x8e, 4, (void *)&CO_OD_RAM.consumerHeartbeatTime[0]);
    CO_OD_set_entry(19, 0x1017, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.producerHeartbeatTime);
    CO_OD_set_entry(20, 0x1018, 0x04, 0x00, 0, (void *)&OD_record1018);
    CO_OD_set_entry(21, 0x1019, 0x00, 0x0e, 1, (void *)&CO_OD_RAM.synchronousCounterOverflowValue);
    CO_OD_set_entry(22, 0x1029, 0x06, 0x0e, 1, (void *)&CO_OD_RAM.errorBehavior[0]);
    CO_OD_set_entry(23, 0x1200, 0x02, 0x00, 0, (void *)&OD_record1200);
    CO_OD_set_entry(24, 0x1280, 0x03, 0x00, 0, (void *)&OD_record1280);
    // Initialise all PDOs to off
    for (i = 0; i < CO_NO_RPDO; i = i + 1) {
        CO_OD_set_entry(25 + i, 0x1400 + i, 0x02, 0x00, 0, (void *)&OD_recordRPDOCommOff);
    }
    for (i = 0; i < CO_NO_RPDO; i = i + 1) {
        CO_OD_set_entry(25 + CO_NO_RPDO + i, 0x1600 + i, 0x08, 0x00, 0, (void *)&OD_recordRPDOMapOff);

    }
    for (i = 0; i < CO_NO_TPDO; i = i + 1) {
        CO_OD_set_entry(25 + CO_NO_RPDO * 2 + i, 0x1800 + i, 0x06, 0x00, 0, (void *)&OD_recordTPDOCommOff);
    }
    for (i = 0; i < CO_NO_TPDO; i = i + 1) {
        CO_OD_set_entry(25 + CO_NO_RPDO * 2 + CO_NO_TPDO + i, 0x1a00 + i, 0x08, 0x00, 0, (void *)&OD_recordTPDOMapOff);
    }

    // Also modify the pointers to the mapping parameters
    for (int i = 0; i < CO_NO_RPDO; i++) {
        OD_RPDOCommunicationParameter[i] = &RPDOCommParamOff;
        OD_RPDOMappingParameter[i] = &RPDOMapParamOff;
    }
    for (int i = 0; i < CO_NO_TPDO; i++) {
        OD_TPDOCommunicationParameter[i] = &TPDOCommParamOff;
        OD_TPDOMappingParameter[i] = &TPDOMapParamOff;
    }

    // PDOs go here
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 1, 0x1f80, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.NMTStartup);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 2, 0x1f81, 0x7f, 0x8e, 4, (void *)&CO_OD_RAM.slaveAssignment[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 3, 0x1f82, 0x7f, 0x0e, 1, (void *)&CO_OD_RAM.requestNMT[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 4, 0x1f89, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.bootTime);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 5, 0x2100, 0x00, 0xa6, 10, (void *)&CO_OD_RAM.errorStatusBits);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 6, 0x2101, 0x00, 0x0e, 1, (void *)&CO_OD_RAM.CANNodeID);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 7, 0x2102, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.CANBitRate);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 8, 0x2103, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.SYNCCounter);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 9, 0x2104, 0x00, 0x86, 2, (void *)&CO_OD_RAM.SYNCTime);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 10, 0x2106, 0x00, 0x86, 4, (void *)&CO_OD_RAM.powerOnCounter);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 11, 0x2107, 0x05, 0x8e, 2, (void *)&CO_OD_RAM.performance[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 12, 0x2108, 0x01, 0x8e, 2, (void *)&CO_OD_RAM.temperature[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 13, 0x2109, 0x01, 0x8e, 2, (void *)&CO_OD_RAM.voltage[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 14, 0x2130, 0x03, 0x00, 0, (void *)&OD_record2130);
    // Extra data stores for RPDO Data
    for (int i = 0; i < CO_NO_RPDO; i++) {
        CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 15+i, 0x6000+i, 0x08, 0x00, 0, (void *)&OD_DummyDataStoreLocation);
    }
    // Extra data stores for TPDO data
    for (int i = 0; i < CO_NO_TPDO; i++) {
        CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 3 * CO_NO_TPDO + 15 + i, 0x6000 + CO_NO_RPDO + i, 0x08, 0x00, 0, (void *)&OD_DummyDataStoreLocation);
    }
    return true;
}

// Should return RPDO number
/**
 * @brief Sets up an RPDO. Should need:
 * - Number of Objects to Map and Links to where the object should be mapped to (or a matrix of pointers to variables...) (testRecord?)
 * 
 * @return int RPDO number
 */
int currRPDO = 0;

int CO_setRPDO(OD_RPDOCommunicationParameter_t *RPDOCommParams, OD_RPDOMappingParameter_t *RPDOMapParams, CO_OD_entryRecord_t *RPDOCommEntry, CO_OD_entryRecord_t *dataStoreRecord, CO_OD_entryRecord_t *RPDOMapParamsEntry) {
    // Should check that the COB-ID is not being used at the moment
    // Could also add a flag which says whether it should be checked or not

    if (currRPDO < CO_NO_RPDO){
        // Iterate through the Mapped Objects to set the parameters
        //This is super hacky and crap.. seriously... why did they set it up in this way?
        uint32_t *pMap = &RPDOMapParams->mappedObject1;
        for (int i = 0; i < RPDOMapParams->numberOfMappedObjects; i++) {
            uint32_t map = *pMap;

            // Change it to 0x6000
            *pMap = (0x60000000 + currRPDO*0x10000)| (0x0000FFFF & map);
            pMap++;
        }

        // Change the OD entry
        CO_OD[25 + currRPDO].pData = (void *)RPDOCommEntry;
        CO_OD[25 + CO_NO_RPDO+ currRPDO].pData = (void *)RPDOMapParamsEntry;

        // Change the Mapping Parameter Entry
        OD_RPDOCommunicationParameter[currRPDO] = RPDOCommParams;
        OD_RPDOMappingParameter[currRPDO] = RPDOMapParams;

        // Change the relevant OD location
        CO_OD[24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 15 + currRPDO].pData = (void *)dataStoreRecord;

        // increment counter, but return the original value
        currRPDO = currRPDO +1;
        return currRPDO;
    }
    return -1; // Error  - too many PDOs defined
}

// Should return RPDO number
/**
 * @brief Sets up an TPDO. Should need:
 * - Number of Objects to Map and Links to where the object should be mapped to (or a matrix of pointers to variables...) (testRecord?)
 * 
 * @return int TPDO number
 */
int currTPDO = 0;
int CO_setTPDO(OD_TPDOCommunicationParameter_t *TPDOCommParams, OD_TPDOMappingParameter_t *TPDOMapParams, CO_OD_entryRecord_t *TPDOCommEntry, CO_OD_entryRecord_t *dataStoreRecord, CO_OD_entryRecord_t *TPDOMapParamsEntry) {
    // Should check that the COB-ID is not being used at the moment
    // Could also add a flag which says whether it should be checked or not

    if (currTPDO < CO_NO_TPDO) {
        // Iterate through the Mapped Objects to set the parameters
        //This is super hacky and crap.. seriously... why did they set it up in this way?
        uint32_t *pMap = &TPDOMapParams->mappedObject1;
        for (int i = 0; i < TPDOMapParams->numberOfMappedObjects; i++) {
            uint32_t map = *pMap;

            // Change it to 0x6000
            *pMap = (0x60000000 + (CO_NO_RPDO+currTPDO) * 0x10000) | (0x0000FFFF & map);
            pMap++;
        }

        // Change the OD entry
        CO_OD[25 + 2*CO_NO_RPDO +currTPDO].pData = (void *)TPDOCommEntry;
        CO_OD[25 + 2 * CO_NO_RPDO + CO_NO_TPDO + currTPDO].pData = (void *)TPDOMapParamsEntry;

        // Change the Mapping Parameter Entry
        OD_TPDOCommunicationParameter[currTPDO] = TPDOCommParams;
        OD_TPDOMappingParameter[currTPDO] = TPDOMapParams;

        // Change the relevant OD location
        CO_OD[24 + 3 * CO_NO_RPDO + 2 * CO_NO_TPDO + 15 + currTPDO].pData = (void *)dataStoreRecord;
        // increment counter, but return the original value
        currTPDO = currTPDO + 1;
        return currTPDO;
    }
    return -1;  // Error  - too many PDOs defined
}
