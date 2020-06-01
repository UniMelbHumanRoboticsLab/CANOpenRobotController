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

#include "CO_driver.h"

#pragma once

/*******************************************************************************
   CANopen DATA DYPES

   OTE: USER EDITED DOMAIN to CO_DOMAIN to prevent conflict in math.h namespace when using the library.
*******************************************************************************/
typedef bool_t BOOLEAN;
typedef uint8_t UNSIGNED8;
typedef uint16_t UNSIGNED16;
typedef uint32_t UNSIGNED32;
typedef uint64_t UNSIGNED64;
typedef int8_t INTEGER8;
typedef int16_t INTEGER16;
typedef int32_t INTEGER32;
typedef int64_t INTEGER64;
typedef float32_t REAL32;
typedef float64_t REAL64;
typedef char_t VISIBLE_STRING;
typedef oChar_t OCTET_STRING;
typedef domain_t CO_DOMAIN;

/*******************************************************************************
   FILE INFO:
      FileName:     BBBMasterFinal.eds
      FileVersion:  0
      CreationTime: 12:00AM
      CreationDate: 01-01-2001
      CreatedBy:    RG
******************************************************************************/

/*******************************************************************************
   DEVICE INFO:
      VendorName:     CANopenNode
      VendorNumber    0
      ProductName:    CANopenNode
      ProductNumber:  0
******************************************************************************/

/*******************************************************************************
   FEATURES
*******************************************************************************/
#define CO_NO_SYNC 1        //Associated objects: 1005-1007
#define CO_NO_EMERGENCY 1   //Associated objects: 1014, 1015
#define CO_NO_SDO_SERVER 1  //Associated objects: 1200-127F
#define CO_NO_SDO_CLIENT 1  //Associated objects: 1280-12FF
#define CO_NO_LSS_SERVER 0  //LSS Slave
#define CO_NO_LSS_CLIENT 0  //LSS Master
#define CO_NO_RPDO 32       //Associated objects: 14xx, 16xx
#define CO_NO_TPDO 32       //Associated objects: 18xx, 1Axx
#define CO_NO_NMT_MASTER 1

/*******************************************************************************
   OBJECT DICTIONARY
*******************************************************************************/
#define CO_OD_NoOfElements 254

/*******************************************************************************
   TYPE DEFINITIONS FOR RECORDS
*******************************************************************************/
/*1018    */ typedef struct
{
    UNSIGNED8 maxSubIndex;
    UNSIGNED32 vendorID;
    UNSIGNED32 productCode;
    UNSIGNED32 revisionNumber;
    UNSIGNED32 serialNumber;
} OD_identity_t;
/*1200    */ typedef struct
{
    UNSIGNED8 maxSubIndex;
    UNSIGNED32 COB_IDClientToServer;
    UNSIGNED32 COB_IDServerToClient;
} OD_SDOServerParameter_t;
/*1280    */ typedef struct
{
    UNSIGNED8 maxSubIndex;
    UNSIGNED32 COB_IDClientToServer;
    UNSIGNED32 COB_IDServerToClient;
    UNSIGNED8 nodeIDOfTheSDOServer;
} OD_SDOClientParameter_t;
/*1400    */ typedef struct
{
    UNSIGNED8 maxSubIndex;
    UNSIGNED32 COB_IDUsedByRPDO;
    UNSIGNED8 transmissionType;
} OD_RPDOCommunicationParameter_t;
/*1600    */ typedef struct
{
    UNSIGNED8 numberOfMappedObjects;
    UNSIGNED32 mappedObject1;
    UNSIGNED32 mappedObject2;
    UNSIGNED32 mappedObject3;
    UNSIGNED32 mappedObject4;
    UNSIGNED32 mappedObject5;
    UNSIGNED32 mappedObject6;
    UNSIGNED32 mappedObject7;
    UNSIGNED32 mappedObject8;
} OD_RPDOMappingParameter_t;
/*1800    */ typedef struct
{
    UNSIGNED8 maxSubIndex;
    UNSIGNED32 COB_IDUsedByTPDO;
    UNSIGNED8 transmissionType;
    UNSIGNED16 inhibitTime;
    UNSIGNED8 compatibilityEntry;
    UNSIGNED16 eventTimer;
    UNSIGNED8 SYNCStartValue;
} OD_TPDOCommunicationParameter_t;
/*1a00    */ typedef struct
{
    UNSIGNED8 numberOfMappedObjects;
    UNSIGNED32 mappedObject1;
    UNSIGNED32 mappedObject2;
    UNSIGNED32 mappedObject3;
    UNSIGNED32 mappedObject4;
    UNSIGNED32 mappedObject5;
    UNSIGNED32 mappedObject6;
    UNSIGNED32 mappedObject7;
    UNSIGNED32 mappedObject8;
} OD_TPDOMappingParameter_t;
/*2120    */ typedef struct
{
    UNSIGNED8 maxSubIndex;
    INTEGER64 I64;
    UNSIGNED64 U64;
    REAL32 R32;
    REAL64 R64;
    CO_DOMAIN domain;
} OD_testVar_t;
/*2130    */ typedef struct
{
    UNSIGNED8 maxSubIndex;
    VISIBLE_STRING string[1];
    UNSIGNED64 epochTimeBaseMs;
    UNSIGNED32 epochTimeOffsetMs;
} OD_time_t;
/*2209    */ typedef struct
{
    UNSIGNED8 numberOfMotors;
    UNSIGNED16 motor1;
    UNSIGNED16 motor2;
    UNSIGNED16 motor3;
    UNSIGNED16 motor4;
    UNSIGNED16 motor5;
    UNSIGNED16 motor6;
} OD_motorTempSensorVoltages_t;
/*2301    */ typedef struct
{
    UNSIGNED8 maxSubIndex;
    UNSIGNED32 size;
    UNSIGNED8 axisNo;
    VISIBLE_STRING name[6];
    VISIBLE_STRING color[5];
    UNSIGNED32 map;
    UNSIGNED8 format;
    UNSIGNED8 trigger;
    INTEGER32 threshold;
} OD_traceConfig_t;
/*2401    */ typedef struct
{
    UNSIGNED8 maxSubIndex;
    UNSIGNED32 size;
    INTEGER32 value;
    INTEGER32 min;
    INTEGER32 max;
    CO_DOMAIN plot;
    UNSIGNED32 triggerTime;
} OD_trace_t;
/*6040    */ typedef struct
{
    UNSIGNED8 numberOfMotors;
    UNSIGNED16 motor1;
    UNSIGNED16 motor2;
    UNSIGNED16 motor3;
    UNSIGNED16 motor4;
    UNSIGNED16 motor5;
    UNSIGNED16 motor6;
} OD_controlWords_t;
/*6041    */ typedef struct
{
    UNSIGNED8 numberOfMotors;
    UNSIGNED16 motor1;
    UNSIGNED16 motor2;
    UNSIGNED16 motor3;
    UNSIGNED16 motor4;
    UNSIGNED16 motor5;
    UNSIGNED16 motor6;
} OD_statusWords_t;
/*6064    */ typedef struct
{
    UNSIGNED8 numberOfMotors;
    INTEGER32 motor1;
    INTEGER32 motor2;
    INTEGER32 motor3;
    INTEGER32 motor4;
    INTEGER32 motor5;
    INTEGER32 motor6;
} OD_actualMotorPositions_t;
/*606c    */ typedef struct
{
    UNSIGNED8 numberOfMotors;
    INTEGER32 motor1;
    INTEGER32 motor2;
    INTEGER32 motor3;
    INTEGER32 motor4;
    INTEGER32 motor5;
    INTEGER32 motor6;
} OD_actualMotorVelocities_t;
/*6077    */ typedef struct
{
    UNSIGNED8 numberOfMotors;
    INTEGER32 motor1;
    INTEGER32 motor2;
    INTEGER32 motor3;
    INTEGER32 motor4;
} OD_actualMotorTorques_t;
/*607a    */ typedef struct
{
    UNSIGNED8 numberOfMotors;
    INTEGER32 motor1;
    INTEGER32 motor2;
    INTEGER32 motor3;
    INTEGER32 motor4;
    INTEGER32 motor5;
    INTEGER32 motor6;
} OD_targetMotorPositions_t;
/*60ff    */ typedef struct
{
    UNSIGNED8 numberOfMotors;
    INTEGER32 motor1;
    INTEGER32 motor2;
    INTEGER32 motor3;
    INTEGER32 motor4;
    INTEGER32 motor5;
    INTEGER32 motor6;
} OD_targetMotorVelocities_t;
/*6071    */ typedef struct
{
    UNSIGNED8 numberOfMotors;
    INTEGER32 motor1;
    INTEGER32 motor2;
    INTEGER32 motor3;
    INTEGER32 motor4;
} OD_targetMotorTorques_t;

/*******************************************************************************
   TYPE DEFINITIONS FOR OBJECT DICTIONARY INDEXES

   some of those are redundant with CO_SDO.h CO_ObjDicId_t <Common CiA301 object 
   dictionary entries>
*******************************************************************************/
/*1000 */
#define OD_1000_deviceType 0x1000

/*1001 */
#define OD_1001_errorRegister 0x1001

/*1002 */
#define OD_1002_manufacturerStatusRegister 0x1002

/*1003 */
#define OD_1003_preDefinedErrorField 0x1003

#define OD_1003_0_preDefinedErrorField_maxSubIndex 0
#define OD_1003_1_preDefinedErrorField_standardErrorField 1
#define OD_1003_2_preDefinedErrorField_standardErrorField 2
#define OD_1003_3_preDefinedErrorField_standardErrorField 3
#define OD_1003_4_preDefinedErrorField_standardErrorField 4
#define OD_1003_5_preDefinedErrorField_standardErrorField 5
#define OD_1003_6_preDefinedErrorField_standardErrorField 6
#define OD_1003_7_preDefinedErrorField_standardErrorField 7
#define OD_1003_8_preDefinedErrorField_standardErrorField 8

/*1005 */
#define OD_1005_COB_ID_SYNCMessage 0x1005

/*1006 */
#define OD_1006_communicationCyclePeriod 0x1006

/*1007 */
#define OD_1007_synchronousWindowLength 0x1007

/*1008 */
#define OD_1008_manufacturerDeviceName 0x1008

/*1009 */
#define OD_1009_manufacturerHardwareVersion 0x1009

/*100a */
#define OD_100a_manufacturerSoftwareVersion 0x100a

/*100c */
#define OD_100c_guardTime 0x100c

/*100d */
#define OD_100d_lifeTimeFactor 0x100d

/*1010 */
#define OD_1010_storeParameters 0x1010

#define OD_1010_0_storeParameters_maxSubIndex 0
#define OD_1010_1_storeParameters_saveAllParameters 1

/*1011 */
#define OD_1011_restoreDefaultParameters 0x1011

#define OD_1011_0_restoreDefaultParameters_maxSubIndex 0
#define OD_1011_1_restoreDefaultParameters_restoreAllDefaultParameters 1

/*1012 */
#define OD_1012_COB_ID_TIME 0x1012

/*1013 */
#define OD_1013_highResolutionTimeStamp 0x1013

/*1014 */
#define OD_1014_COB_ID_EMCY 0x1014

/*1015 */
#define OD_1015_inhibitTimeEMCY 0x1015

/*1016 */
#define OD_1016_consumerHeartbeatTime 0x1016

#define OD_1016_0_consumerHeartbeatTime_maxSubIndex 0
#define OD_1016_1_consumerHeartbeatTime_consumerHeartbeatTime 1
#define OD_1016_2_consumerHeartbeatTime_consumerHeartbeatTime 2
#define OD_1016_3_consumerHeartbeatTime_consumerHeartbeatTime 3
#define OD_1016_4_consumerHeartbeatTime_consumerHeartbeatTime 4

/*1017 */
#define OD_1017_producerHeartbeatTime 0x1017

/*1018 */
#define OD_1018_identity 0x1018

#define OD_1018_0_identity_maxSubIndex 0
#define OD_1018_1_identity_vendorID 1
#define OD_1018_2_identity_productCode 2
#define OD_1018_3_identity_revisionNumber 3
#define OD_1018_4_identity_serialNumber 4

/*1019 */
#define OD_1019_synchronousCounterOverflowValue 0x1019

/*1029 */
#define OD_1029_errorBehavior 0x1029

#define OD_1029_0_errorBehavior_maxSubIndex 0
#define OD_1029_1_errorBehavior_communication 1
#define OD_1029_2_errorBehavior_communicationOther 2
#define OD_1029_3_errorBehavior_communicationPassive 3
#define OD_1029_4_errorBehavior_generic 4
#define OD_1029_5_errorBehavior_deviceProfile 5
#define OD_1029_6_errorBehavior_manufacturerSpecific 6

/*1200 */
#define OD_1200_SDOServerParameter 0x1200

#define OD_1200_0_SDOServerParameter_maxSubIndex 0
#define OD_1200_1_SDOServerParameter_COB_IDClientToServer 1
#define OD_1200_2_SDOServerParameter_COB_IDServerToClient 2

/*1280 */
#define OD_1280_SDOClientParameter 0x1280

#define OD_1280_0_SDOClientParameter_maxSubIndex 0
#define OD_1280_1_SDOClientParameter_COB_IDClientToServer 1
#define OD_1280_2_SDOClientParameter_COB_IDServerToClient 2
#define OD_1280_3_SDOClientParameter_nodeIDOfTheSDOServer 3

/*1400 */
#define OD_1400_RPDOCommunicationParameter 0x1400

#define OD_1400_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1400_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1400_2_RPDOCommunicationParameter_transmissionType 2

/*1401 */
#define OD_1401_RPDOCommunicationParameter 0x1401

#define OD_1401_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1401_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1401_2_RPDOCommunicationParameter_transmissionType 2

/*1402 */
#define OD_1402_RPDOCommunicationParameter 0x1402

#define OD_1402_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1402_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1402_2_RPDOCommunicationParameter_transmissionType 2

/*1403 */
#define OD_1403_RPDOCommunicationParameter 0x1403

#define OD_1403_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1403_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1403_2_RPDOCommunicationParameter_transmissionType 2

/*1404 */
#define OD_1404_RPDOCommunicationParameter 0x1404

#define OD_1404_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1404_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1404_2_RPDOCommunicationParameter_transmissionType 2

/*1405 */
#define OD_1405_RPDOCommunicationParameter 0x1405

#define OD_1405_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1405_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1405_2_RPDOCommunicationParameter_transmissionType 2

/*1406 */
#define OD_1406_RPDOCommunicationParameter 0x1406

#define OD_1406_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1406_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1406_2_RPDOCommunicationParameter_transmissionType 2

/*1407 */
#define OD_1407_RPDOCommunicationParameter 0x1407

#define OD_1407_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1407_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1407_2_RPDOCommunicationParameter_transmissionType 2

/*1408 */
#define OD_1408_RPDOCommunicationParameter 0x1408

#define OD_1408_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1408_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1408_2_RPDOCommunicationParameter_transmissionType 2

/*1409 */
#define OD_1409_RPDOCommunicationParameter 0x1409

#define OD_1409_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1409_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1409_2_RPDOCommunicationParameter_transmissionType 2

/*140a */
#define OD_140a_RPDOCommunicationParameter 0x140a

#define OD_140a_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_140a_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_140a_2_RPDOCommunicationParameter_transmissionType 2

/*140b */
#define OD_140b_RPDOCommunicationParameter 0x140b

#define OD_140b_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_140b_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_140b_2_RPDOCommunicationParameter_transmissionType 2

/*140c */
#define OD_140c_RPDOCommunicationParameter 0x140c

#define OD_140c_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_140c_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_140c_2_RPDOCommunicationParameter_transmissionType 2

/*140d */
#define OD_140d_RPDOCommunicationParameter 0x140d

#define OD_140d_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_140d_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_140d_2_RPDOCommunicationParameter_transmissionType 2

/*140e */
#define OD_140e_RPDOCommunicationParameter 0x140e

#define OD_140e_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_140e_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_140e_2_RPDOCommunicationParameter_transmissionType 2

/*140f */
#define OD_140f_RPDOCommunicationParameter 0x140f

#define OD_140f_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_140f_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_140f_2_RPDOCommunicationParameter_transmissionType 2

/*1410 */
#define OD_1410_RPDOCommunicationParameter 0x1410

#define OD_1410_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1410_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1410_2_RPDOCommunicationParameter_transmissionType 2

/*1411 */
#define OD_1411_RPDOCommunicationParameter 0x1411

#define OD_1411_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1411_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1411_2_RPDOCommunicationParameter_transmissionType 2

/*1412 */
#define OD_1412_RPDOCommunicationParameter 0x1412

#define OD_1412_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1412_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1412_2_RPDOCommunicationParameter_transmissionType 2

/*1413 */
#define OD_1413_RPDOCommunicationParameter 0x1413

#define OD_1413_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1413_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1413_2_RPDOCommunicationParameter_transmissionType 2

/*1414 */
#define OD_1414_RPDOCommunicationParameter 0x1414

#define OD_1414_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1414_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1414_2_RPDOCommunicationParameter_transmissionType 2

/*1415 */
#define OD_1415_RPDOCommunicationParameter 0x1415

#define OD_1415_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1415_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1415_2_RPDOCommunicationParameter_transmissionType 2

/*1416 */
#define OD_1416_RPDOCommunicationParameter 0x1416

#define OD_1416_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1416_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1416_2_RPDOCommunicationParameter_transmissionType 2

/*1417 */
#define OD_1417_RPDOCommunicationParameter 0x1417

#define OD_1417_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1417_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1417_2_RPDOCommunicationParameter_transmissionType 2

/*1418 */
#define OD_1418_RPDOCommunicationParameter 0x1418

#define OD_1418_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1418_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1418_2_RPDOCommunicationParameter_transmissionType 2

/*1419 */
#define OD_1419_RPDOCommunicationParameter 0x1419

#define OD_1419_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_1419_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_1419_2_RPDOCommunicationParameter_transmissionType 2

/*141a */
#define OD_141a_RPDOCommunicationParameter 0x141a

#define OD_141a_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_141a_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_141a_2_RPDOCommunicationParameter_transmissionType 2

/*141b */
#define OD_141b_RPDOCommunicationParameter 0x141b

#define OD_141b_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_141b_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_141b_2_RPDOCommunicationParameter_transmissionType 2

/*141c */
#define OD_141c_RPDOCommunicationParameter 0x141c

#define OD_141c_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_141c_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_141c_2_RPDOCommunicationParameter_transmissionType 2

/*141d */
#define OD_141d_RPDOCommunicationParameter 0x141d

#define OD_141d_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_141d_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_141d_2_RPDOCommunicationParameter_transmissionType 2

/*141e */
#define OD_141e_RPDOCommunicationParameter 0x141e

#define OD_141e_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_141e_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_141e_2_RPDOCommunicationParameter_transmissionType 2

/*141f */
#define OD_141f_RPDOCommunicationParameter 0x141f

#define OD_141f_0_RPDOCommunicationParameter_maxSubIndex 0
#define OD_141f_1_RPDOCommunicationParameter_COB_IDUsedByRPDO 1
#define OD_141f_2_RPDOCommunicationParameter_transmissionType 2

/*1600 */
#define OD_1600_RPDOMappingParameter 0x1600

#define OD_1600_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1600_1_RPDOMappingParameter_mappedObject1 1
#define OD_1600_2_RPDOMappingParameter_mappedObject2 2
#define OD_1600_3_RPDOMappingParameter_mappedObject3 3
#define OD_1600_4_RPDOMappingParameter_mappedObject4 4
#define OD_1600_5_RPDOMappingParameter_mappedObject5 5
#define OD_1600_6_RPDOMappingParameter_mappedObject6 6
#define OD_1600_7_RPDOMappingParameter_mappedObject7 7
#define OD_1600_8_RPDOMappingParameter_mappedObject8 8

/*1601 */
#define OD_1601_RPDOMappingParameter 0x1601

#define OD_1601_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1601_1_RPDOMappingParameter_mappedObject1 1
#define OD_1601_2_RPDOMappingParameter_mappedObject2 2
#define OD_1601_3_RPDOMappingParameter_mappedObject3 3
#define OD_1601_4_RPDOMappingParameter_mappedObject4 4
#define OD_1601_5_RPDOMappingParameter_mappedObject5 5
#define OD_1601_6_RPDOMappingParameter_mappedObject6 6
#define OD_1601_7_RPDOMappingParameter_mappedObject7 7
#define OD_1601_8_RPDOMappingParameter_mappedObject8 8

/*1602 */
#define OD_1602_RPDOMappingParameter 0x1602

#define OD_1602_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1602_1_RPDOMappingParameter_mappedObject1 1
#define OD_1602_2_RPDOMappingParameter_mappedObject2 2
#define OD_1602_3_RPDOMappingParameter_mappedObject3 3
#define OD_1602_4_RPDOMappingParameter_mappedObject4 4
#define OD_1602_5_RPDOMappingParameter_mappedObject5 5
#define OD_1602_6_RPDOMappingParameter_mappedObject6 6
#define OD_1602_7_RPDOMappingParameter_mappedObject7 7
#define OD_1602_8_RPDOMappingParameter_mappedObject8 8

/*1603 */
#define OD_1603_RPDOMappingParameter 0x1603

#define OD_1603_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1603_1_RPDOMappingParameter_mappedObject1 1
#define OD_1603_2_RPDOMappingParameter_mappedObject2 2
#define OD_1603_3_RPDOMappingParameter_mappedObject3 3
#define OD_1603_4_RPDOMappingParameter_mappedObject4 4
#define OD_1603_5_RPDOMappingParameter_mappedObject5 5
#define OD_1603_6_RPDOMappingParameter_mappedObject6 6
#define OD_1603_7_RPDOMappingParameter_mappedObject7 7
#define OD_1603_8_RPDOMappingParameter_mappedObject8 8

/*1604 */
#define OD_1604_RPDOMappingParameter 0x1604

#define OD_1604_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1604_1_RPDOMappingParameter_mappedObject1 1
#define OD_1604_2_RPDOMappingParameter_mappedObject2 2
#define OD_1604_3_RPDOMappingParameter_mappedObject3 3
#define OD_1604_4_RPDOMappingParameter_mappedObject4 4
#define OD_1604_5_RPDOMappingParameter_mappedObject5 5
#define OD_1604_6_RPDOMappingParameter_mappedObject6 6
#define OD_1604_7_RPDOMappingParameter_mappedObject7 7
#define OD_1604_8_RPDOMappingParameter_mappedObject8 8

/*1605 */
#define OD_1605_RPDOMappingParameter 0x1605

#define OD_1605_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1605_1_RPDOMappingParameter_mappedObject1 1
#define OD_1605_2_RPDOMappingParameter_mappedObject2 2
#define OD_1605_3_RPDOMappingParameter_mappedObject3 3
#define OD_1605_4_RPDOMappingParameter_mappedObject4 4
#define OD_1605_5_RPDOMappingParameter_mappedObject5 5
#define OD_1605_6_RPDOMappingParameter_mappedObject6 6
#define OD_1605_7_RPDOMappingParameter_mappedObject7 7
#define OD_1605_8_RPDOMappingParameter_mappedObject8 8

/*1606 */
#define OD_1606_RPDOMappingParameter 0x1606

#define OD_1606_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1606_1_RPDOMappingParameter_mappedObject1 1
#define OD_1606_2_RPDOMappingParameter_mappedObject2 2
#define OD_1606_3_RPDOMappingParameter_mappedObject3 3
#define OD_1606_4_RPDOMappingParameter_mappedObject4 4
#define OD_1606_5_RPDOMappingParameter_mappedObject5 5
#define OD_1606_6_RPDOMappingParameter_mappedObject6 6
#define OD_1606_7_RPDOMappingParameter_mappedObject7 7
#define OD_1606_8_RPDOMappingParameter_mappedObject8 8

/*1607 */
#define OD_1607_RPDOMappingParameter 0x1607

#define OD_1607_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1607_1_RPDOMappingParameter_mappedObject1 1
#define OD_1607_2_RPDOMappingParameter_mappedObject2 2
#define OD_1607_3_RPDOMappingParameter_mappedObject3 3
#define OD_1607_4_RPDOMappingParameter_mappedObject4 4
#define OD_1607_5_RPDOMappingParameter_mappedObject5 5
#define OD_1607_6_RPDOMappingParameter_mappedObject6 6
#define OD_1607_7_RPDOMappingParameter_mappedObject7 7
#define OD_1607_8_RPDOMappingParameter_mappedObject8 8

/*1608 */
#define OD_1608_RPDOMappingParameter 0x1608

#define OD_1608_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1608_1_RPDOMappingParameter_mappedObject1 1
#define OD_1608_2_RPDOMappingParameter_mappedObject2 2
#define OD_1608_3_RPDOMappingParameter_mappedObject3 3
#define OD_1608_4_RPDOMappingParameter_mappedObject4 4
#define OD_1608_5_RPDOMappingParameter_mappedObject5 5
#define OD_1608_6_RPDOMappingParameter_mappedObject6 6
#define OD_1608_7_RPDOMappingParameter_mappedObject7 7
#define OD_1608_8_RPDOMappingParameter_mappedObject8 8

/*1609 */
#define OD_1609_RPDOMappingParameter 0x1609

#define OD_1609_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1609_1_RPDOMappingParameter_mappedObject1 1
#define OD_1609_2_RPDOMappingParameter_mappedObject2 2
#define OD_1609_3_RPDOMappingParameter_mappedObject3 3
#define OD_1609_4_RPDOMappingParameter_mappedObject4 4
#define OD_1609_5_RPDOMappingParameter_mappedObject5 5
#define OD_1609_6_RPDOMappingParameter_mappedObject6 6
#define OD_1609_7_RPDOMappingParameter_mappedObject7 7
#define OD_1609_8_RPDOMappingParameter_mappedObject8 8

/*160a */
#define OD_160a_RPDOMappingParameter 0x160a

#define OD_160a_0_RPDOMappingParameter_maxSubIndex 0
#define OD_160a_1_RPDOMappingParameter_mappedObject1 1
#define OD_160a_2_RPDOMappingParameter_mappedObject2 2
#define OD_160a_3_RPDOMappingParameter_mappedObject3 3
#define OD_160a_4_RPDOMappingParameter_mappedObject4 4
#define OD_160a_5_RPDOMappingParameter_mappedObject5 5
#define OD_160a_6_RPDOMappingParameter_mappedObject6 6
#define OD_160a_7_RPDOMappingParameter_mappedObject7 7
#define OD_160a_8_RPDOMappingParameter_mappedObject8 8

/*160b */
#define OD_160b_RPDOMappingParameter 0x160b

#define OD_160b_0_RPDOMappingParameter_maxSubIndex 0
#define OD_160b_1_RPDOMappingParameter_mappedObject1 1
#define OD_160b_2_RPDOMappingParameter_mappedObject2 2
#define OD_160b_3_RPDOMappingParameter_mappedObject3 3
#define OD_160b_4_RPDOMappingParameter_mappedObject4 4
#define OD_160b_5_RPDOMappingParameter_mappedObject5 5
#define OD_160b_6_RPDOMappingParameter_mappedObject6 6
#define OD_160b_7_RPDOMappingParameter_mappedObject7 7
#define OD_160b_8_RPDOMappingParameter_mappedObject8 8

/*160c */
#define OD_160c_RPDOMappingParameter 0x160c

#define OD_160c_0_RPDOMappingParameter_maxSubIndex 0
#define OD_160c_1_RPDOMappingParameter_mappedObject1 1
#define OD_160c_2_RPDOMappingParameter_mappedObject2 2
#define OD_160c_3_RPDOMappingParameter_mappedObject3 3
#define OD_160c_4_RPDOMappingParameter_mappedObject4 4
#define OD_160c_5_RPDOMappingParameter_mappedObject5 5
#define OD_160c_6_RPDOMappingParameter_mappedObject6 6
#define OD_160c_7_RPDOMappingParameter_mappedObject7 7
#define OD_160c_8_RPDOMappingParameter_mappedObject8 8

/*160d */
#define OD_160d_RPDOMappingParameter 0x160d

#define OD_160d_0_RPDOMappingParameter_maxSubIndex 0
#define OD_160d_1_RPDOMappingParameter_mappedObject1 1
#define OD_160d_2_RPDOMappingParameter_mappedObject2 2
#define OD_160d_3_RPDOMappingParameter_mappedObject3 3
#define OD_160d_4_RPDOMappingParameter_mappedObject4 4
#define OD_160d_5_RPDOMappingParameter_mappedObject5 5
#define OD_160d_6_RPDOMappingParameter_mappedObject6 6
#define OD_160d_7_RPDOMappingParameter_mappedObject7 7
#define OD_160d_8_RPDOMappingParameter_mappedObject8 8

/*160e */
#define OD_160e_RPDOMappingParameter 0x160e

#define OD_160e_0_RPDOMappingParameter_maxSubIndex 0
#define OD_160e_1_RPDOMappingParameter_mappedObject1 1
#define OD_160e_2_RPDOMappingParameter_mappedObject2 2
#define OD_160e_3_RPDOMappingParameter_mappedObject3 3
#define OD_160e_4_RPDOMappingParameter_mappedObject4 4
#define OD_160e_5_RPDOMappingParameter_mappedObject5 5
#define OD_160e_6_RPDOMappingParameter_mappedObject6 6
#define OD_160e_7_RPDOMappingParameter_mappedObject7 7
#define OD_160e_8_RPDOMappingParameter_mappedObject8 8

/*160f */
#define OD_160f_RPDOMappingParameter 0x160f

#define OD_160f_0_RPDOMappingParameter_maxSubIndex 0
#define OD_160f_1_RPDOMappingParameter_mappedObject1 1
#define OD_160f_2_RPDOMappingParameter_mappedObject2 2
#define OD_160f_3_RPDOMappingParameter_mappedObject3 3
#define OD_160f_4_RPDOMappingParameter_mappedObject4 4
#define OD_160f_5_RPDOMappingParameter_mappedObject5 5
#define OD_160f_6_RPDOMappingParameter_mappedObject6 6
#define OD_160f_7_RPDOMappingParameter_mappedObject7 7
#define OD_160f_8_RPDOMappingParameter_mappedObject8 8

/*1610 */
#define OD_1610_RPDOMappingParameter 0x1610

#define OD_1610_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1610_1_RPDOMappingParameter_mappedObject1 1
#define OD_1610_2_RPDOMappingParameter_mappedObject2 2
#define OD_1610_3_RPDOMappingParameter_mappedObject3 3
#define OD_1610_4_RPDOMappingParameter_mappedObject4 4
#define OD_1610_5_RPDOMappingParameter_mappedObject5 5
#define OD_1610_6_RPDOMappingParameter_mappedObject6 6
#define OD_1610_7_RPDOMappingParameter_mappedObject7 7
#define OD_1610_8_RPDOMappingParameter_mappedObject8 8

/*1611 */
#define OD_1611_RPDOMappingParameter 0x1611

#define OD_1611_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1611_1_RPDOMappingParameter_mappedObject1 1
#define OD_1611_2_RPDOMappingParameter_mappedObject2 2
#define OD_1611_3_RPDOMappingParameter_mappedObject3 3
#define OD_1611_4_RPDOMappingParameter_mappedObject4 4
#define OD_1611_5_RPDOMappingParameter_mappedObject5 5
#define OD_1611_6_RPDOMappingParameter_mappedObject6 6
#define OD_1611_7_RPDOMappingParameter_mappedObject7 7
#define OD_1611_8_RPDOMappingParameter_mappedObject8 8

/*1612 */
#define OD_1612_RPDOMappingParameter 0x1612

#define OD_1612_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1612_1_RPDOMappingParameter_mappedObject1 1
#define OD_1612_2_RPDOMappingParameter_mappedObject2 2
#define OD_1612_3_RPDOMappingParameter_mappedObject3 3
#define OD_1612_4_RPDOMappingParameter_mappedObject4 4
#define OD_1612_5_RPDOMappingParameter_mappedObject5 5
#define OD_1612_6_RPDOMappingParameter_mappedObject6 6
#define OD_1612_7_RPDOMappingParameter_mappedObject7 7
#define OD_1612_8_RPDOMappingParameter_mappedObject8 8

/*1613 */
#define OD_1613_RPDOMappingParameter 0x1613

#define OD_1613_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1613_1_RPDOMappingParameter_mappedObject1 1
#define OD_1613_2_RPDOMappingParameter_mappedObject2 2
#define OD_1613_3_RPDOMappingParameter_mappedObject3 3
#define OD_1613_4_RPDOMappingParameter_mappedObject4 4
#define OD_1613_5_RPDOMappingParameter_mappedObject5 5
#define OD_1613_6_RPDOMappingParameter_mappedObject6 6
#define OD_1613_7_RPDOMappingParameter_mappedObject7 7
#define OD_1613_8_RPDOMappingParameter_mappedObject8 8

/*1614 */
#define OD_1614_RPDOMappingParameter 0x1614

#define OD_1614_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1614_1_RPDOMappingParameter_mappedObject1 1
#define OD_1614_2_RPDOMappingParameter_mappedObject2 2
#define OD_1614_3_RPDOMappingParameter_mappedObject3 3
#define OD_1614_4_RPDOMappingParameter_mappedObject4 4
#define OD_1614_5_RPDOMappingParameter_mappedObject5 5
#define OD_1614_6_RPDOMappingParameter_mappedObject6 6
#define OD_1614_7_RPDOMappingParameter_mappedObject7 7
#define OD_1614_8_RPDOMappingParameter_mappedObject8 8

/*1615 */
#define OD_1615_RPDOMappingParameter 0x1615

#define OD_1615_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1615_1_RPDOMappingParameter_mappedObject1 1
#define OD_1615_2_RPDOMappingParameter_mappedObject2 2
#define OD_1615_3_RPDOMappingParameter_mappedObject3 3
#define OD_1615_4_RPDOMappingParameter_mappedObject4 4
#define OD_1615_5_RPDOMappingParameter_mappedObject5 5
#define OD_1615_6_RPDOMappingParameter_mappedObject6 6
#define OD_1615_7_RPDOMappingParameter_mappedObject7 7
#define OD_1615_8_RPDOMappingParameter_mappedObject8 8

/*1616 */
#define OD_1616_RPDOMappingParameter 0x1616

#define OD_1616_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1616_1_RPDOMappingParameter_mappedObject1 1
#define OD_1616_2_RPDOMappingParameter_mappedObject2 2
#define OD_1616_3_RPDOMappingParameter_mappedObject3 3
#define OD_1616_4_RPDOMappingParameter_mappedObject4 4
#define OD_1616_5_RPDOMappingParameter_mappedObject5 5
#define OD_1616_6_RPDOMappingParameter_mappedObject6 6
#define OD_1616_7_RPDOMappingParameter_mappedObject7 7
#define OD_1616_8_RPDOMappingParameter_mappedObject8 8

/*1617 */
#define OD_1617_RPDOMappingParameter 0x1617

#define OD_1617_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1617_1_RPDOMappingParameter_mappedObject1 1
#define OD_1617_2_RPDOMappingParameter_mappedObject2 2
#define OD_1617_3_RPDOMappingParameter_mappedObject3 3
#define OD_1617_4_RPDOMappingParameter_mappedObject4 4
#define OD_1617_5_RPDOMappingParameter_mappedObject5 5
#define OD_1617_6_RPDOMappingParameter_mappedObject6 6
#define OD_1617_7_RPDOMappingParameter_mappedObject7 7
#define OD_1617_8_RPDOMappingParameter_mappedObject8 8

/*1618 */
#define OD_1618_RPDOMappingParameter 0x1618

#define OD_1618_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1618_1_RPDOMappingParameter_mappedObject1 1
#define OD_1618_2_RPDOMappingParameter_mappedObject2 2
#define OD_1618_3_RPDOMappingParameter_mappedObject3 3
#define OD_1618_4_RPDOMappingParameter_mappedObject4 4
#define OD_1618_5_RPDOMappingParameter_mappedObject5 5
#define OD_1618_6_RPDOMappingParameter_mappedObject6 6
#define OD_1618_7_RPDOMappingParameter_mappedObject7 7
#define OD_1618_8_RPDOMappingParameter_mappedObject8 8

/*1619 */
#define OD_1619_RPDOMappingParameter 0x1619

#define OD_1619_0_RPDOMappingParameter_maxSubIndex 0
#define OD_1619_1_RPDOMappingParameter_mappedObject1 1
#define OD_1619_2_RPDOMappingParameter_mappedObject2 2
#define OD_1619_3_RPDOMappingParameter_mappedObject3 3
#define OD_1619_4_RPDOMappingParameter_mappedObject4 4
#define OD_1619_5_RPDOMappingParameter_mappedObject5 5
#define OD_1619_6_RPDOMappingParameter_mappedObject6 6
#define OD_1619_7_RPDOMappingParameter_mappedObject7 7
#define OD_1619_8_RPDOMappingParameter_mappedObject8 8

/*161a */
#define OD_161a_RPDOMappingParameter 0x161a

#define OD_161a_0_RPDOMappingParameter_maxSubIndex 0
#define OD_161a_1_RPDOMappingParameter_mappedObject1 1
#define OD_161a_2_RPDOMappingParameter_mappedObject2 2
#define OD_161a_3_RPDOMappingParameter_mappedObject3 3
#define OD_161a_4_RPDOMappingParameter_mappedObject4 4
#define OD_161a_5_RPDOMappingParameter_mappedObject5 5
#define OD_161a_6_RPDOMappingParameter_mappedObject6 6
#define OD_161a_7_RPDOMappingParameter_mappedObject7 7
#define OD_161a_8_RPDOMappingParameter_mappedObject8 8

/*161b */
#define OD_161b_RPDOMappingParameter 0x161b

#define OD_161b_0_RPDOMappingParameter_maxSubIndex 0
#define OD_161b_1_RPDOMappingParameter_mappedObject1 1
#define OD_161b_2_RPDOMappingParameter_mappedObject2 2
#define OD_161b_3_RPDOMappingParameter_mappedObject3 3
#define OD_161b_4_RPDOMappingParameter_mappedObject4 4
#define OD_161b_5_RPDOMappingParameter_mappedObject5 5
#define OD_161b_6_RPDOMappingParameter_mappedObject6 6
#define OD_161b_7_RPDOMappingParameter_mappedObject7 7
#define OD_161b_8_RPDOMappingParameter_mappedObject8 8

/*161c */
#define OD_161c_RPDOMappingParameter 0x161c

#define OD_161c_0_RPDOMappingParameter_maxSubIndex 0
#define OD_161c_1_RPDOMappingParameter_mappedObject1 1
#define OD_161c_2_RPDOMappingParameter_mappedObject2 2
#define OD_161c_3_RPDOMappingParameter_mappedObject3 3
#define OD_161c_4_RPDOMappingParameter_mappedObject4 4
#define OD_161c_5_RPDOMappingParameter_mappedObject5 5
#define OD_161c_6_RPDOMappingParameter_mappedObject6 6
#define OD_161c_7_RPDOMappingParameter_mappedObject7 7
#define OD_161c_8_RPDOMappingParameter_mappedObject8 8

/*161d */
#define OD_161d_RPDOMappingParameter 0x161d

#define OD_161d_0_RPDOMappingParameter_maxSubIndex 0
#define OD_161d_1_RPDOMappingParameter_mappedObject1 1
#define OD_161d_2_RPDOMappingParameter_mappedObject2 2
#define OD_161d_3_RPDOMappingParameter_mappedObject3 3
#define OD_161d_4_RPDOMappingParameter_mappedObject4 4
#define OD_161d_5_RPDOMappingParameter_mappedObject5 5
#define OD_161d_6_RPDOMappingParameter_mappedObject6 6
#define OD_161d_7_RPDOMappingParameter_mappedObject7 7
#define OD_161d_8_RPDOMappingParameter_mappedObject8 8

/*161e */
#define OD_161e_RPDOMappingParameter 0x161e

#define OD_161e_0_RPDOMappingParameter_maxSubIndex 0
#define OD_161e_1_RPDOMappingParameter_mappedObject1 1
#define OD_161e_2_RPDOMappingParameter_mappedObject2 2
#define OD_161e_3_RPDOMappingParameter_mappedObject3 3
#define OD_161e_4_RPDOMappingParameter_mappedObject4 4
#define OD_161e_5_RPDOMappingParameter_mappedObject5 5
#define OD_161e_6_RPDOMappingParameter_mappedObject6 6
#define OD_161e_7_RPDOMappingParameter_mappedObject7 7
#define OD_161e_8_RPDOMappingParameter_mappedObject8 8

/*161f */
#define OD_161f_RPDOMappingParameter 0x161f

#define OD_161f_0_RPDOMappingParameter_maxSubIndex 0
#define OD_161f_1_RPDOMappingParameter_mappedObject1 1
#define OD_161f_2_RPDOMappingParameter_mappedObject2 2
#define OD_161f_3_RPDOMappingParameter_mappedObject3 3
#define OD_161f_4_RPDOMappingParameter_mappedObject4 4
#define OD_161f_5_RPDOMappingParameter_mappedObject5 5
#define OD_161f_6_RPDOMappingParameter_mappedObject6 6
#define OD_161f_7_RPDOMappingParameter_mappedObject7 7
#define OD_161f_8_RPDOMappingParameter_mappedObject8 8

/*1800 */
#define OD_1800_TPDOCommunicationParameter 0x1800

#define OD_1800_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1800_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1800_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1800_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1800_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1800_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1800_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1801 */
#define OD_1801_TPDOCommunicationParameter 0x1801

#define OD_1801_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1801_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1801_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1801_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1801_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1801_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1801_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1802 */
#define OD_1802_TPDOCommunicationParameter 0x1802

#define OD_1802_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1802_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1802_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1802_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1802_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1802_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1802_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1803 */
#define OD_1803_TPDOCommunicationParameter 0x1803

#define OD_1803_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1803_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1803_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1803_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1803_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1803_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1803_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1804 */
#define OD_1804_TPDOCommunicationParameter 0x1804

#define OD_1804_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1804_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1804_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1804_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1804_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1804_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1804_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1805 */
#define OD_1805_TPDOCommunicationParameter 0x1805

#define OD_1805_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1805_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1805_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1805_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1805_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1805_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1805_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1806 */
#define OD_1806_TPDOCommunicationParameter 0x1806

#define OD_1806_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1806_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1806_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1806_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1806_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1806_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1806_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1807 */
#define OD_1807_TPDOCommunicationParameter 0x1807

#define OD_1807_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1807_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1807_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1807_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1807_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1807_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1807_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1808 */
#define OD_1808_TPDOCommunicationParameter 0x1808

#define OD_1808_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1808_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1808_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1808_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1808_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1808_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1808_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1809 */
#define OD_1809_TPDOCommunicationParameter 0x1809

#define OD_1809_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1809_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1809_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1809_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1809_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1809_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1809_6_TPDOCommunicationParameter_SYNCStartValue 6

/*180a */
#define OD_180a_TPDOCommunicationParameter 0x180a

#define OD_180a_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_180a_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_180a_2_TPDOCommunicationParameter_transmissionType 2
#define OD_180a_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_180a_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_180a_5_TPDOCommunicationParameter_eventTimer 5
#define OD_180a_6_TPDOCommunicationParameter_SYNCStartValue 6

/*180b */
#define OD_180b_TPDOCommunicationParameter 0x180b

#define OD_180b_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_180b_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_180b_2_TPDOCommunicationParameter_transmissionType 2
#define OD_180b_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_180b_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_180b_5_TPDOCommunicationParameter_eventTimer 5
#define OD_180b_6_TPDOCommunicationParameter_SYNCStartValue 6

/*180c */
#define OD_180c_TPDOCommunicationParameter 0x180c

#define OD_180c_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_180c_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_180c_2_TPDOCommunicationParameter_transmissionType 2
#define OD_180c_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_180c_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_180c_5_TPDOCommunicationParameter_eventTimer 5
#define OD_180c_6_TPDOCommunicationParameter_SYNCStartValue 6

/*180d */
#define OD_180d_TPDOCommunicationParameter 0x180d

#define OD_180d_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_180d_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_180d_2_TPDOCommunicationParameter_transmissionType 2
#define OD_180d_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_180d_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_180d_5_TPDOCommunicationParameter_eventTimer 5
#define OD_180d_6_TPDOCommunicationParameter_SYNCStartValue 6

/*180e */
#define OD_180e_TPDOCommunicationParameter 0x180e

#define OD_180e_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_180e_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_180e_2_TPDOCommunicationParameter_transmissionType 2
#define OD_180e_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_180e_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_180e_5_TPDOCommunicationParameter_eventTimer 5
#define OD_180e_6_TPDOCommunicationParameter_SYNCStartValue 6

/*180f */
#define OD_180f_TPDOCommunicationParameter 0x180f

#define OD_180f_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_180f_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_180f_2_TPDOCommunicationParameter_transmissionType 2
#define OD_180f_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_180f_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_180f_5_TPDOCommunicationParameter_eventTimer 5
#define OD_180f_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1810 */
#define OD_1810_TPDOCommunicationParameter 0x1810

#define OD_1810_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1810_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1810_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1810_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1810_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1810_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1810_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1811 */
#define OD_1811_TPDOCommunicationParameter 0x1811

#define OD_1811_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1811_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1811_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1811_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1811_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1811_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1811_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1812 */
#define OD_1812_TPDOCommunicationParameter 0x1812

#define OD_1812_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1812_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1812_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1812_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1812_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1812_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1812_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1813 */
#define OD_1813_TPDOCommunicationParameter 0x1813

#define OD_1813_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1813_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1813_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1813_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1813_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1813_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1813_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1814 */
#define OD_1814_TPDOCommunicationParameter 0x1814

#define OD_1814_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1814_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1814_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1814_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1814_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1814_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1814_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1815 */
#define OD_1815_TPDOCommunicationParameter 0x1815

#define OD_1815_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1815_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1815_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1815_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1815_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1815_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1815_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1816 */
#define OD_1816_TPDOCommunicationParameter 0x1816

#define OD_1816_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1816_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1816_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1816_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1816_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1816_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1816_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1817 */
#define OD_1817_TPDOCommunicationParameter 0x1817

#define OD_1817_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1817_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1817_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1817_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1817_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1817_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1817_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1818 */
#define OD_1818_TPDOCommunicationParameter 0x1818

#define OD_1818_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1818_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1818_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1818_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1818_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1818_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1818_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1819 */
#define OD_1819_TPDOCommunicationParameter 0x1819

#define OD_1819_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_1819_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_1819_2_TPDOCommunicationParameter_transmissionType 2
#define OD_1819_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_1819_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_1819_5_TPDOCommunicationParameter_eventTimer 5
#define OD_1819_6_TPDOCommunicationParameter_SYNCStartValue 6

/*181a */
#define OD_181a_TPDOCommunicationParameter 0x181a

#define OD_181a_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_181a_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_181a_2_TPDOCommunicationParameter_transmissionType 2
#define OD_181a_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_181a_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_181a_5_TPDOCommunicationParameter_eventTimer 5
#define OD_181a_6_TPDOCommunicationParameter_SYNCStartValue 6

/*181b */
#define OD_181b_TPDOCommunicationParameter 0x181b

#define OD_181b_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_181b_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_181b_2_TPDOCommunicationParameter_transmissionType 2
#define OD_181b_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_181b_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_181b_5_TPDOCommunicationParameter_eventTimer 5
#define OD_181b_6_TPDOCommunicationParameter_SYNCStartValue 6

/*181c */
#define OD_181c_TPDOCommunicationParameter 0x181c

#define OD_181c_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_181c_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_181c_2_TPDOCommunicationParameter_transmissionType 2
#define OD_181c_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_181c_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_181c_5_TPDOCommunicationParameter_eventTimer 5
#define OD_181c_6_TPDOCommunicationParameter_SYNCStartValue 6

/*181d */
#define OD_181d_TPDOCommunicationParameter 0x181d

#define OD_181d_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_181d_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_181d_2_TPDOCommunicationParameter_transmissionType 2
#define OD_181d_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_181d_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_181d_5_TPDOCommunicationParameter_eventTimer 5
#define OD_181d_6_TPDOCommunicationParameter_SYNCStartValue 6

/*181e */
#define OD_181e_TPDOCommunicationParameter 0x181e

#define OD_181e_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_181e_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_181e_2_TPDOCommunicationParameter_transmissionType 2
#define OD_181e_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_181e_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_181e_5_TPDOCommunicationParameter_eventTimer 5
#define OD_181e_6_TPDOCommunicationParameter_SYNCStartValue 6

/*181f */
#define OD_181f_TPDOCommunicationParameter 0x181f

#define OD_181f_0_TPDOCommunicationParameter_maxSubIndex 0
#define OD_181f_1_TPDOCommunicationParameter_COB_IDUsedByTPDO 1
#define OD_181f_2_TPDOCommunicationParameter_transmissionType 2
#define OD_181f_3_TPDOCommunicationParameter_inhibitTime 3
#define OD_181f_4_TPDOCommunicationParameter_compatibilityEntry 4
#define OD_181f_5_TPDOCommunicationParameter_eventTimer 5
#define OD_181f_6_TPDOCommunicationParameter_SYNCStartValue 6

/*1a00 */
#define OD_1a00_TPDOMappingParameter 0x1a00

#define OD_1a00_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a00_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a00_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a00_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a00_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a00_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a00_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a00_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a00_8_TPDOMappingParameter_mappedObject8 8

/*1a01 */
#define OD_1a01_TPDOMappingParameter 0x1a01

#define OD_1a01_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a01_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a01_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a01_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a01_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a01_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a01_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a01_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a01_8_TPDOMappingParameter_mappedObject8 8

/*1a02 */
#define OD_1a02_TPDOMappingParameter 0x1a02

#define OD_1a02_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a02_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a02_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a02_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a02_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a02_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a02_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a02_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a02_8_TPDOMappingParameter_mappedObject8 8

/*1a03 */
#define OD_1a03_TPDOMappingParameter 0x1a03

#define OD_1a03_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a03_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a03_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a03_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a03_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a03_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a03_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a03_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a03_8_TPDOMappingParameter_mappedObject8 8

/*1a04 */
#define OD_1a04_TPDOMappingParameter 0x1a04

#define OD_1a04_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a04_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a04_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a04_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a04_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a04_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a04_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a04_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a04_8_TPDOMappingParameter_mappedObject8 8

/*1a05 */
#define OD_1a05_TPDOMappingParameter 0x1a05

#define OD_1a05_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a05_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a05_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a05_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a05_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a05_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a05_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a05_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a05_8_TPDOMappingParameter_mappedObject8 8

/*1a06 */
#define OD_1a06_TPDOMappingParameter 0x1a06

#define OD_1a06_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a06_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a06_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a06_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a06_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a06_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a06_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a06_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a06_8_TPDOMappingParameter_mappedObject8 8

/*1a07 */
#define OD_1a07_TPDOMappingParameter 0x1a07

#define OD_1a07_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a07_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a07_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a07_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a07_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a07_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a07_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a07_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a07_8_TPDOMappingParameter_mappedObject8 8

/*1a08 */
#define OD_1a08_TPDOMappingParameter 0x1a08

#define OD_1a08_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a08_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a08_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a08_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a08_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a08_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a08_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a08_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a08_8_TPDOMappingParameter_mappedObject8 8

/*1a09 */
#define OD_1a09_TPDOMappingParameter 0x1a09

#define OD_1a09_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a09_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a09_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a09_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a09_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a09_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a09_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a09_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a09_8_TPDOMappingParameter_mappedObject8 8

/*1a0a */
#define OD_1a0a_TPDOMappingParameter 0x1a0a

#define OD_1a0a_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a0a_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a0a_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a0a_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a0a_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a0a_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a0a_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a0a_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a0a_8_TPDOMappingParameter_mappedObject8 8

/*1a0b */
#define OD_1a0b_TPDOMappingParameter 0x1a0b

#define OD_1a0b_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a0b_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a0b_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a0b_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a0b_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a0b_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a0b_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a0b_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a0b_8_TPDOMappingParameter_mappedObject8 8

/*1a0c */
#define OD_1a0c_TPDOMappingParameter 0x1a0c

#define OD_1a0c_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a0c_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a0c_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a0c_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a0c_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a0c_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a0c_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a0c_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a0c_8_TPDOMappingParameter_mappedObject8 8

/*1a0d */
#define OD_1a0d_TPDOMappingParameter 0x1a0d

#define OD_1a0d_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a0d_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a0d_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a0d_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a0d_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a0d_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a0d_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a0d_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a0d_8_TPDOMappingParameter_mappedObject8 8

/*1a0e */
#define OD_1a0e_TPDOMappingParameter 0x1a0e

#define OD_1a0e_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a0e_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a0e_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a0e_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a0e_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a0e_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a0e_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a0e_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a0e_8_TPDOMappingParameter_mappedObject8 8

/*1a0f */
#define OD_1a0f_TPDOMappingParameter 0x1a0f

#define OD_1a0f_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a0f_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a0f_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a0f_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a0f_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a0f_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a0f_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a0f_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a0f_8_TPDOMappingParameter_mappedObject8 8

/*1a10 */
#define OD_1a10_TPDOMappingParameter 0x1a10

#define OD_1a10_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a10_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a10_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a10_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a10_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a10_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a10_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a10_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a10_8_TPDOMappingParameter_mappedObject8 8

/*1a11 */
#define OD_1a11_TPDOMappingParameter 0x1a11

#define OD_1a11_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a11_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a11_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a11_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a11_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a11_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a11_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a11_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a11_8_TPDOMappingParameter_mappedObject8 8

/*1a12 */
#define OD_1a12_TPDOMappingParameter 0x1a12

#define OD_1a12_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a12_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a12_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a12_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a12_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a12_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a12_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a12_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a12_8_TPDOMappingParameter_mappedObject8 8

/*1a13 */
#define OD_1a13_TPDOMappingParameter 0x1a13

#define OD_1a13_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a13_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a13_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a13_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a13_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a13_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a13_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a13_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a13_8_TPDOMappingParameter_mappedObject8 8

/*1a14 */
#define OD_1a14_TPDOMappingParameter 0x1a14

#define OD_1a14_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a14_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a14_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a14_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a14_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a14_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a14_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a14_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a14_8_TPDOMappingParameter_mappedObject8 8

/*1a15 */
#define OD_1a15_TPDOMappingParameter 0x1a15

#define OD_1a15_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a15_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a15_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a15_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a15_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a15_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a15_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a15_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a15_8_TPDOMappingParameter_mappedObject8 8

/*1a16 */
#define OD_1a16_TPDOMappingParameter 0x1a16

#define OD_1a16_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a16_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a16_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a16_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a16_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a16_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a16_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a16_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a16_8_TPDOMappingParameter_mappedObject8 8

/*1a17 */
#define OD_1a17_TPDOMappingParameter 0x1a17

#define OD_1a17_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a17_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a17_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a17_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a17_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a17_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a17_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a17_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a17_8_TPDOMappingParameter_mappedObject8 8

/*1a18 */
#define OD_1a18_TPDOMappingParameter 0x1a18

#define OD_1a18_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a18_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a18_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a18_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a18_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a18_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a18_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a18_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a18_8_TPDOMappingParameter_mappedObject8 8

/*1a19 */
#define OD_1a19_TPDOMappingParameter 0x1a19

#define OD_1a19_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a19_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a19_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a19_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a19_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a19_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a19_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a19_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a19_8_TPDOMappingParameter_mappedObject8 8

/*1a1a */
#define OD_1a1a_TPDOMappingParameter 0x1a1a

#define OD_1a1a_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a1a_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a1a_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a1a_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a1a_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a1a_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a1a_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a1a_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a1a_8_TPDOMappingParameter_mappedObject8 8

/*1a1b */
#define OD_1a1b_TPDOMappingParameter 0x1a1b

#define OD_1a1b_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a1b_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a1b_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a1b_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a1b_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a1b_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a1b_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a1b_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a1b_8_TPDOMappingParameter_mappedObject8 8

/*1a1c */
#define OD_1a1c_TPDOMappingParameter 0x1a1c

#define OD_1a1c_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a1c_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a1c_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a1c_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a1c_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a1c_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a1c_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a1c_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a1c_8_TPDOMappingParameter_mappedObject8 8

/*1a1d */
#define OD_1a1d_TPDOMappingParameter 0x1a1d

#define OD_1a1d_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a1d_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a1d_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a1d_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a1d_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a1d_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a1d_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a1d_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a1d_8_TPDOMappingParameter_mappedObject8 8

/*1a1e */
#define OD_1a1e_TPDOMappingParameter 0x1a1e

#define OD_1a1e_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a1e_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a1e_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a1e_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a1e_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a1e_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a1e_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a1e_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a1e_8_TPDOMappingParameter_mappedObject8 8

/*1a1f */
#define OD_1a1f_TPDOMappingParameter 0x1a1f

#define OD_1a1f_0_TPDOMappingParameter_maxSubIndex 0
#define OD_1a1f_1_TPDOMappingParameter_mappedObject1 1
#define OD_1a1f_2_TPDOMappingParameter_mappedObject2 2
#define OD_1a1f_3_TPDOMappingParameter_mappedObject3 3
#define OD_1a1f_4_TPDOMappingParameter_mappedObject4 4
#define OD_1a1f_5_TPDOMappingParameter_mappedObject5 5
#define OD_1a1f_6_TPDOMappingParameter_mappedObject6 6
#define OD_1a1f_7_TPDOMappingParameter_mappedObject7 7
#define OD_1a1f_8_TPDOMappingParameter_mappedObject8 8

/*1f80 */
#define OD_1f80_NMTStartup 0x1f80

/*1f81 */
#define OD_1f81_slaveAssignment 0x1f81

#define OD_1f81_0_slaveAssignment_maxSubIndex 0
#define OD_1f81_1_slaveAssignment_ 1
#define OD_1f81_2_slaveAssignment_ 2
#define OD_1f81_3_slaveAssignment_ 3
#define OD_1f81_4_slaveAssignment_ 4
#define OD_1f81_5_slaveAssignment_ 5
#define OD_1f81_6_slaveAssignment_ 6
#define OD_1f81_7_slaveAssignment_ 7
#define OD_1f81_8_slaveAssignment_ 8
#define OD_1f81_9_slaveAssignment_ 9
#define OD_1f81_10_slaveAssignment_ 10
#define OD_1f81_11_slaveAssignment_ 11
#define OD_1f81_12_slaveAssignment_ 12
#define OD_1f81_13_slaveAssignment_ 13
#define OD_1f81_14_slaveAssignment_ 14
#define OD_1f81_15_slaveAssignment_ 15
#define OD_1f81_16_slaveAssignment_ 16
#define OD_1f81_17_slaveAssignment_ 17
#define OD_1f81_18_slaveAssignment_ 18
#define OD_1f81_19_slaveAssignment_ 19
#define OD_1f81_20_slaveAssignment_ 20
#define OD_1f81_21_slaveAssignment_ 21
#define OD_1f81_22_slaveAssignment_ 22
#define OD_1f81_23_slaveAssignment_ 23
#define OD_1f81_24_slaveAssignment_ 24
#define OD_1f81_25_slaveAssignment_ 25
#define OD_1f81_26_slaveAssignment_ 26
#define OD_1f81_27_slaveAssignment_ 27
#define OD_1f81_28_slaveAssignment_ 28
#define OD_1f81_29_slaveAssignment_ 29
#define OD_1f81_30_slaveAssignment_ 30
#define OD_1f81_31_slaveAssignment_ 31
#define OD_1f81_32_slaveAssignment_ 32
#define OD_1f81_33_slaveAssignment_ 33
#define OD_1f81_34_slaveAssignment_ 34
#define OD_1f81_35_slaveAssignment_ 35
#define OD_1f81_36_slaveAssignment_ 36
#define OD_1f81_37_slaveAssignment_ 37
#define OD_1f81_38_slaveAssignment_ 38
#define OD_1f81_39_slaveAssignment_ 39
#define OD_1f81_40_slaveAssignment_ 40
#define OD_1f81_41_slaveAssignment_ 41
#define OD_1f81_42_slaveAssignment_ 42
#define OD_1f81_43_slaveAssignment_ 43
#define OD_1f81_44_slaveAssignment_ 44
#define OD_1f81_45_slaveAssignment_ 45
#define OD_1f81_46_slaveAssignment_ 46
#define OD_1f81_47_slaveAssignment_ 47
#define OD_1f81_48_slaveAssignment_ 48
#define OD_1f81_49_slaveAssignment_ 49
#define OD_1f81_50_slaveAssignment_ 50
#define OD_1f81_51_slaveAssignment_ 51
#define OD_1f81_52_slaveAssignment_ 52
#define OD_1f81_53_slaveAssignment_ 53
#define OD_1f81_54_slaveAssignment_ 54
#define OD_1f81_55_slaveAssignment_ 55
#define OD_1f81_56_slaveAssignment_ 56
#define OD_1f81_57_slaveAssignment_ 57
#define OD_1f81_58_slaveAssignment_ 58
#define OD_1f81_59_slaveAssignment_ 59
#define OD_1f81_60_slaveAssignment_ 60
#define OD_1f81_61_slaveAssignment_ 61
#define OD_1f81_62_slaveAssignment_ 62
#define OD_1f81_63_slaveAssignment_ 63
#define OD_1f81_64_slaveAssignment_ 64
#define OD_1f81_65_slaveAssignment_ 65
#define OD_1f81_66_slaveAssignment_ 66
#define OD_1f81_67_slaveAssignment_ 67
#define OD_1f81_68_slaveAssignment_ 68
#define OD_1f81_69_slaveAssignment_ 69
#define OD_1f81_70_slaveAssignment_ 70
#define OD_1f81_71_slaveAssignment_ 71
#define OD_1f81_72_slaveAssignment_ 72
#define OD_1f81_73_slaveAssignment_ 73
#define OD_1f81_74_slaveAssignment_ 74
#define OD_1f81_75_slaveAssignment_ 75
#define OD_1f81_76_slaveAssignment_ 76
#define OD_1f81_77_slaveAssignment_ 77
#define OD_1f81_78_slaveAssignment_ 78
#define OD_1f81_79_slaveAssignment_ 79
#define OD_1f81_80_slaveAssignment_ 80
#define OD_1f81_81_slaveAssignment_ 81
#define OD_1f81_82_slaveAssignment_ 82
#define OD_1f81_83_slaveAssignment_ 83
#define OD_1f81_84_slaveAssignment_ 84
#define OD_1f81_85_slaveAssignment_ 85
#define OD_1f81_86_slaveAssignment_ 86
#define OD_1f81_87_slaveAssignment_ 87
#define OD_1f81_88_slaveAssignment_ 88
#define OD_1f81_89_slaveAssignment_ 89
#define OD_1f81_90_slaveAssignment_ 90
#define OD_1f81_91_slaveAssignment_ 91
#define OD_1f81_92_slaveAssignment_ 92
#define OD_1f81_93_slaveAssignment_ 93
#define OD_1f81_94_slaveAssignment_ 94
#define OD_1f81_95_slaveAssignment_ 95
#define OD_1f81_96_slaveAssignment_ 96
#define OD_1f81_97_slaveAssignment_ 97
#define OD_1f81_98_slaveAssignment_ 98
#define OD_1f81_99_slaveAssignment_ 99
#define OD_1f81_100_slaveAssignment_ 100
#define OD_1f81_101_slaveAssignment_ 101
#define OD_1f81_102_slaveAssignment_ 102
#define OD_1f81_103_slaveAssignment_ 103
#define OD_1f81_104_slaveAssignment_ 104
#define OD_1f81_105_slaveAssignment_ 105
#define OD_1f81_106_slaveAssignment_ 106
#define OD_1f81_107_slaveAssignment_ 107
#define OD_1f81_108_slaveAssignment_ 108
#define OD_1f81_109_slaveAssignment_ 109
#define OD_1f81_110_slaveAssignment_ 110
#define OD_1f81_111_slaveAssignment_ 111
#define OD_1f81_112_slaveAssignment_ 112
#define OD_1f81_113_slaveAssignment_ 113
#define OD_1f81_114_slaveAssignment_ 114
#define OD_1f81_115_slaveAssignment_ 115
#define OD_1f81_116_slaveAssignment_ 116
#define OD_1f81_117_slaveAssignment_ 117
#define OD_1f81_118_slaveAssignment_ 118
#define OD_1f81_119_slaveAssignment_ 119
#define OD_1f81_120_slaveAssignment_ 120
#define OD_1f81_121_slaveAssignment_ 121
#define OD_1f81_122_slaveAssignment_ 122
#define OD_1f81_123_slaveAssignment_ 123
#define OD_1f81_124_slaveAssignment_ 124
#define OD_1f81_125_slaveAssignment_ 125
#define OD_1f81_126_slaveAssignment_ 126
#define OD_1f81_127_slaveAssignment_ 127

/*1f82 */
#define OD_1f82_requestNMT 0x1f82

#define OD_1f82_0_requestNMT_maxSubIndex 0
#define OD_1f82_1_requestNMT_ 1
#define OD_1f82_2_requestNMT_ 2
#define OD_1f82_3_requestNMT_ 3
#define OD_1f82_4_requestNMT_ 4
#define OD_1f82_5_requestNMT_ 5
#define OD_1f82_6_requestNMT_ 6
#define OD_1f82_7_requestNMT_ 7
#define OD_1f82_8_requestNMT_ 8
#define OD_1f82_9_requestNMT_ 9
#define OD_1f82_10_requestNMT_ 10
#define OD_1f82_11_requestNMT_ 11
#define OD_1f82_12_requestNMT_ 12
#define OD_1f82_13_requestNMT_ 13
#define OD_1f82_14_requestNMT_ 14
#define OD_1f82_15_requestNMT_ 15
#define OD_1f82_16_requestNMT_ 16
#define OD_1f82_17_requestNMT_ 17
#define OD_1f82_18_requestNMT_ 18
#define OD_1f82_19_requestNMT_ 19
#define OD_1f82_20_requestNMT_ 20
#define OD_1f82_21_requestNMT_ 21
#define OD_1f82_22_requestNMT_ 22
#define OD_1f82_23_requestNMT_ 23
#define OD_1f82_24_requestNMT_ 24
#define OD_1f82_25_requestNMT_ 25
#define OD_1f82_26_requestNMT_ 26
#define OD_1f82_27_requestNMT_ 27
#define OD_1f82_28_requestNMT_ 28
#define OD_1f82_29_requestNMT_ 29
#define OD_1f82_30_requestNMT_ 30
#define OD_1f82_31_requestNMT_ 31
#define OD_1f82_32_requestNMT_ 32
#define OD_1f82_33_requestNMT_ 33
#define OD_1f82_34_requestNMT_ 34
#define OD_1f82_35_requestNMT_ 35
#define OD_1f82_36_requestNMT_ 36
#define OD_1f82_37_requestNMT_ 37
#define OD_1f82_38_requestNMT_ 38
#define OD_1f82_39_requestNMT_ 39
#define OD_1f82_40_requestNMT_ 40
#define OD_1f82_41_requestNMT_ 41
#define OD_1f82_42_requestNMT_ 42
#define OD_1f82_43_requestNMT_ 43
#define OD_1f82_44_requestNMT_ 44
#define OD_1f82_45_requestNMT_ 45
#define OD_1f82_46_requestNMT_ 46
#define OD_1f82_47_requestNMT_ 47
#define OD_1f82_48_requestNMT_ 48
#define OD_1f82_49_requestNMT_ 49
#define OD_1f82_50_requestNMT_ 50
#define OD_1f82_51_requestNMT_ 51
#define OD_1f82_52_requestNMT_ 52
#define OD_1f82_53_requestNMT_ 53
#define OD_1f82_54_requestNMT_ 54
#define OD_1f82_55_requestNMT_ 55
#define OD_1f82_56_requestNMT_ 56
#define OD_1f82_57_requestNMT_ 57
#define OD_1f82_58_requestNMT_ 58
#define OD_1f82_59_requestNMT_ 59
#define OD_1f82_60_requestNMT_ 60
#define OD_1f82_61_requestNMT_ 61
#define OD_1f82_62_requestNMT_ 62
#define OD_1f82_63_requestNMT_ 63
#define OD_1f82_64_requestNMT_ 64
#define OD_1f82_65_requestNMT_ 65
#define OD_1f82_66_requestNMT_ 66
#define OD_1f82_67_requestNMT_ 67
#define OD_1f82_68_requestNMT_ 68
#define OD_1f82_69_requestNMT_ 69
#define OD_1f82_70_requestNMT_ 70
#define OD_1f82_71_requestNMT_ 71
#define OD_1f82_72_requestNMT_ 72
#define OD_1f82_73_requestNMT_ 73
#define OD_1f82_74_requestNMT_ 74
#define OD_1f82_75_requestNMT_ 75
#define OD_1f82_76_requestNMT_ 76
#define OD_1f82_77_requestNMT_ 77
#define OD_1f82_78_requestNMT_ 78
#define OD_1f82_79_requestNMT_ 79
#define OD_1f82_80_requestNMT_ 80
#define OD_1f82_81_requestNMT_ 81
#define OD_1f82_82_requestNMT_ 82
#define OD_1f82_83_requestNMT_ 83
#define OD_1f82_84_requestNMT_ 84
#define OD_1f82_85_requestNMT_ 85
#define OD_1f82_86_requestNMT_ 86
#define OD_1f82_87_requestNMT_ 87
#define OD_1f82_88_requestNMT_ 88
#define OD_1f82_89_requestNMT_ 89
#define OD_1f82_90_requestNMT_ 90
#define OD_1f82_91_requestNMT_ 91
#define OD_1f82_92_requestNMT_ 92
#define OD_1f82_93_requestNMT_ 93
#define OD_1f82_94_requestNMT_ 94
#define OD_1f82_95_requestNMT_ 95
#define OD_1f82_96_requestNMT_ 96
#define OD_1f82_97_requestNMT_ 97
#define OD_1f82_98_requestNMT_ 98
#define OD_1f82_99_requestNMT_ 99
#define OD_1f82_100_requestNMT_ 100
#define OD_1f82_101_requestNMT_ 101
#define OD_1f82_102_requestNMT_ 102
#define OD_1f82_103_requestNMT_ 103
#define OD_1f82_104_requestNMT_ 104
#define OD_1f82_105_requestNMT_ 105
#define OD_1f82_106_requestNMT_ 106
#define OD_1f82_107_requestNMT_ 107
#define OD_1f82_108_requestNMT_ 108
#define OD_1f82_109_requestNMT_ 109
#define OD_1f82_110_requestNMT_ 110
#define OD_1f82_111_requestNMT_ 111
#define OD_1f82_112_requestNMT_ 112
#define OD_1f82_113_requestNMT_ 113
#define OD_1f82_114_requestNMT_ 114
#define OD_1f82_115_requestNMT_ 115
#define OD_1f82_116_requestNMT_ 116
#define OD_1f82_117_requestNMT_ 117
#define OD_1f82_118_requestNMT_ 118
#define OD_1f82_119_requestNMT_ 119
#define OD_1f82_120_requestNMT_ 120
#define OD_1f82_121_requestNMT_ 121
#define OD_1f82_122_requestNMT_ 122
#define OD_1f82_123_requestNMT_ 123
#define OD_1f82_124_requestNMT_ 124
#define OD_1f82_125_requestNMT_ 125
#define OD_1f82_126_requestNMT_ 126
#define OD_1f82_127_requestNMT_ 127

/*1f89 */
#define OD_1f89_bootTime 0x1f89

/*2100 */
#define OD_2100_errorStatusBits 0x2100

/*2101 */
#define OD_2101_CANNodeID 0x2101

/*2102 */
#define OD_2102_CANBitRate 0x2102

/*2103 */
#define OD_2103_SYNCCounter 0x2103

/*2104 */
#define OD_2104_SYNCTime 0x2104

/*2106 */
#define OD_2106_powerOnCounter 0x2106

/*2107 */
#define OD_2107_performance 0x2107

#define OD_2107_0_performance_maxSubIndex 0
#define OD_2107_1_performance_cyclesPerSecond 1
#define OD_2107_2_performance_timerCycleTime 2
#define OD_2107_3_performance_timerCycleMaxTime 3
#define OD_2107_4_performance_mainCycleTime 4
#define OD_2107_5_performance_mainCycleMaxTime 5

/*2108 */
#define OD_2108_temperature 0x2108

#define OD_2108_0_temperature_maxSubIndex 0
#define OD_2108_1_temperature_mainPCB 1

/*2109 */
#define OD_2109_voltage 0x2109

#define OD_2109_0_voltage_maxSubIndex 0
#define OD_2109_1_voltage_mainPCBSupply 1

/*2110 */
#define OD_2110_variableInt32 0x2110

#define OD_2110_0_variableInt32_maxSubIndex 0
#define OD_2110_1_variableInt32_int32 1
#define OD_2110_2_variableInt32_int32 2
#define OD_2110_3_variableInt32_int32 3
#define OD_2110_4_variableInt32_int32 4
#define OD_2110_5_variableInt32_int32 5
#define OD_2110_6_variableInt32_int32 6
#define OD_2110_7_variableInt32_int32 7
#define OD_2110_8_variableInt32_int32 8
#define OD_2110_9_variableInt32_int32 9
#define OD_2110_10_variableInt32_int32 10
#define OD_2110_11_variableInt32_int32 11
#define OD_2110_12_variableInt32_int32 12
#define OD_2110_13_variableInt32_int32 13
#define OD_2110_14_variableInt32_int32 14
#define OD_2110_15_variableInt32_int32 15
#define OD_2110_16_variableInt32_int32 16
#define OD_2110_17_variableInt32_int32 17
#define OD_2110_18_variableInt32_int32 18
#define OD_2110_19_variableInt32_int32 19
#define OD_2110_20_variableInt32_int32 20
#define OD_2110_21_variableInt32_int32 21
#define OD_2110_22_variableInt32_int32 22
#define OD_2110_23_variableInt32_int32 23
#define OD_2110_24_variableInt32_int32 24
#define OD_2110_25_variableInt32_int32 25
#define OD_2110_26_variableInt32_int32 26
#define OD_2110_27_variableInt32_int32 27
#define OD_2110_28_variableInt32_int32 28
#define OD_2110_29_variableInt32_int32 29
#define OD_2110_30_variableInt32_int32 30
#define OD_2110_31_variableInt32_int32 31
#define OD_2110_32_variableInt32_int32 32

/*2111 */
#define OD_2111_variableROM_Int32 0x2111

#define OD_2111_0_variableROM_Int32_maxSubIndex 0
#define OD_2111_1_variableROM_Int32_int32 1
#define OD_2111_2_variableROM_Int32_int32 2
#define OD_2111_3_variableROM_Int32_int32 3
#define OD_2111_4_variableROM_Int32_int32 4
#define OD_2111_5_variableROM_Int32_int32 5
#define OD_2111_6_variableROM_Int32_int32 6
#define OD_2111_7_variableROM_Int32_int32 7
#define OD_2111_8_variableROM_Int32_int32 8
#define OD_2111_9_variableROM_Int32_int32 9
#define OD_2111_10_variableROM_Int32_int32 10
#define OD_2111_11_variableROM_Int32_int32 11
#define OD_2111_12_variableROM_Int32_int32 12
#define OD_2111_13_variableROM_Int32_int32 13
#define OD_2111_14_variableROM_Int32_int32 14
#define OD_2111_15_variableROM_Int32_int32 15
#define OD_2111_16_variableROM_Int32_int32 16

/*2112 */
#define OD_2112_variableNV_Int32 0x2112

#define OD_2112_0_variableNV_Int32_maxSubIndex 0
#define OD_2112_1_variableNV_Int32_int32 1
#define OD_2112_2_variableNV_Int32_int32 2
#define OD_2112_3_variableNV_Int32_int32 3
#define OD_2112_4_variableNV_Int32_int32 4
#define OD_2112_5_variableNV_Int32_int32 5
#define OD_2112_6_variableNV_Int32_int32 6
#define OD_2112_7_variableNV_Int32_int32 7
#define OD_2112_8_variableNV_Int32_int32 8
#define OD_2112_9_variableNV_Int32_int32 9
#define OD_2112_10_variableNV_Int32_int32 10
#define OD_2112_11_variableNV_Int32_int32 11
#define OD_2112_12_variableNV_Int32_int32 12
#define OD_2112_13_variableNV_Int32_int32 13
#define OD_2112_14_variableNV_Int32_int32 14
#define OD_2112_15_variableNV_Int32_int32 15
#define OD_2112_16_variableNV_Int32_int32 16

/*2120 */
#define OD_2120_testVar 0x2120

#define OD_2120_0_testVar_maxSubIndex 0
#define OD_2120_1_testVar_I64 1
#define OD_2120_2_testVar_U64 2
#define OD_2120_3_testVar_R32 3
#define OD_2120_4_testVar_R64 4
#define OD_2120_5_testVar_domain 5

/*2130 */
#define OD_2130_time 0x2130

#define OD_2130_0_time_maxSubIndex 0
#define OD_2130_1_time_string 1
#define OD_2130_2_time_epochTimeBaseMs 2
#define OD_2130_3_time_epochTimeOffsetMs 3

/*2209 */
#define OD_2209_motorTempSensorVoltages 0x2209

#define OD_2209_0_motorTempSensorVoltages_maxSubIndex 0
#define OD_2209_1_motorTempSensorVoltages_motor1 1
#define OD_2209_2_motorTempSensorVoltages_motor2 2
#define OD_2209_3_motorTempSensorVoltages_motor3 3
#define OD_2209_4_motorTempSensorVoltages_motor4 4
#define OD_2209_5_motorTempSensorVoltages_motor5 5
#define OD_2209_6_motorTempSensorVoltages_motor6 6

/*2301 */
#define OD_2301_traceConfig 0x2301

#define OD_2301_0_traceConfig_maxSubIndex 0
#define OD_2301_1_traceConfig_size 1
#define OD_2301_2_traceConfig_axisNo 2
#define OD_2301_3_traceConfig_name 3
#define OD_2301_4_traceConfig_color 4
#define OD_2301_5_traceConfig_map 5
#define OD_2301_6_traceConfig_format 6
#define OD_2301_7_traceConfig_trigger 7
#define OD_2301_8_traceConfig_threshold 8

/*2302 */
#define OD_2302_traceConfig 0x2302

#define OD_2302_0_traceConfig_maxSubIndex 0
#define OD_2302_1_traceConfig_size 1
#define OD_2302_2_traceConfig_axisNo 2
#define OD_2302_3_traceConfig_name 3
#define OD_2302_4_traceConfig_color 4
#define OD_2302_5_traceConfig_map 5
#define OD_2302_6_traceConfig_format 6
#define OD_2302_7_traceConfig_trigger 7
#define OD_2302_8_traceConfig_threshold 8

/*2303 */
#define OD_2303_traceConfig 0x2303

#define OD_2303_0_traceConfig_maxSubIndex 0
#define OD_2303_1_traceConfig_size 1
#define OD_2303_2_traceConfig_axisNo 2
#define OD_2303_3_traceConfig_name 3
#define OD_2303_4_traceConfig_color 4
#define OD_2303_5_traceConfig_map 5
#define OD_2303_6_traceConfig_format 6
#define OD_2303_7_traceConfig_trigger 7
#define OD_2303_8_traceConfig_threshold 8

/*2304 */
#define OD_2304_traceConfig 0x2304

#define OD_2304_0_traceConfig_maxSubIndex 0
#define OD_2304_1_traceConfig_size 1
#define OD_2304_2_traceConfig_axisNo 2
#define OD_2304_3_traceConfig_name 3
#define OD_2304_4_traceConfig_color 4
#define OD_2304_5_traceConfig_map 5
#define OD_2304_6_traceConfig_format 6
#define OD_2304_7_traceConfig_trigger 7
#define OD_2304_8_traceConfig_threshold 8

/*2305 */
#define OD_2305_traceConfig 0x2305

#define OD_2305_0_traceConfig_maxSubIndex 0
#define OD_2305_1_traceConfig_size 1
#define OD_2305_2_traceConfig_axisNo 2
#define OD_2305_3_traceConfig_name 3
#define OD_2305_4_traceConfig_color 4
#define OD_2305_5_traceConfig_map 5
#define OD_2305_6_traceConfig_format 6
#define OD_2305_7_traceConfig_trigger 7
#define OD_2305_8_traceConfig_threshold 8

/*2306 */
#define OD_2306_traceConfig 0x2306

#define OD_2306_0_traceConfig_maxSubIndex 0
#define OD_2306_1_traceConfig_size 1
#define OD_2306_2_traceConfig_axisNo 2
#define OD_2306_3_traceConfig_name 3
#define OD_2306_4_traceConfig_color 4
#define OD_2306_5_traceConfig_map 5
#define OD_2306_6_traceConfig_format 6
#define OD_2306_7_traceConfig_trigger 7
#define OD_2306_8_traceConfig_threshold 8

/*2307 */
#define OD_2307_traceConfig 0x2307

#define OD_2307_0_traceConfig_maxSubIndex 0
#define OD_2307_1_traceConfig_size 1
#define OD_2307_2_traceConfig_axisNo 2
#define OD_2307_3_traceConfig_name 3
#define OD_2307_4_traceConfig_color 4
#define OD_2307_5_traceConfig_map 5
#define OD_2307_6_traceConfig_format 6
#define OD_2307_7_traceConfig_trigger 7
#define OD_2307_8_traceConfig_threshold 8

/*2308 */
#define OD_2308_traceConfig 0x2308

#define OD_2308_0_traceConfig_maxSubIndex 0
#define OD_2308_1_traceConfig_size 1
#define OD_2308_2_traceConfig_axisNo 2
#define OD_2308_3_traceConfig_name 3
#define OD_2308_4_traceConfig_color 4
#define OD_2308_5_traceConfig_map 5
#define OD_2308_6_traceConfig_format 6
#define OD_2308_7_traceConfig_trigger 7
#define OD_2308_8_traceConfig_threshold 8

/*2309 */
#define OD_2309_traceConfig 0x2309

#define OD_2309_0_traceConfig_maxSubIndex 0
#define OD_2309_1_traceConfig_size 1
#define OD_2309_2_traceConfig_axisNo 2
#define OD_2309_3_traceConfig_name 3
#define OD_2309_4_traceConfig_color 4
#define OD_2309_5_traceConfig_map 5
#define OD_2309_6_traceConfig_format 6
#define OD_2309_7_traceConfig_trigger 7
#define OD_2309_8_traceConfig_threshold 8

/*230a */
#define OD_230a_traceConfig 0x230a

#define OD_230a_0_traceConfig_maxSubIndex 0
#define OD_230a_1_traceConfig_size 1
#define OD_230a_2_traceConfig_axisNo 2
#define OD_230a_3_traceConfig_name 3
#define OD_230a_4_traceConfig_color 4
#define OD_230a_5_traceConfig_map 5
#define OD_230a_6_traceConfig_format 6
#define OD_230a_7_traceConfig_trigger 7
#define OD_230a_8_traceConfig_threshold 8

/*230b */
#define OD_230b_traceConfig 0x230b

#define OD_230b_0_traceConfig_maxSubIndex 0
#define OD_230b_1_traceConfig_size 1
#define OD_230b_2_traceConfig_axisNo 2
#define OD_230b_3_traceConfig_name 3
#define OD_230b_4_traceConfig_color 4
#define OD_230b_5_traceConfig_map 5
#define OD_230b_6_traceConfig_format 6
#define OD_230b_7_traceConfig_trigger 7
#define OD_230b_8_traceConfig_threshold 8

/*230c */
#define OD_230c_traceConfig 0x230c

#define OD_230c_0_traceConfig_maxSubIndex 0
#define OD_230c_1_traceConfig_size 1
#define OD_230c_2_traceConfig_axisNo 2
#define OD_230c_3_traceConfig_name 3
#define OD_230c_4_traceConfig_color 4
#define OD_230c_5_traceConfig_map 5
#define OD_230c_6_traceConfig_format 6
#define OD_230c_7_traceConfig_trigger 7
#define OD_230c_8_traceConfig_threshold 8

/*230d */
#define OD_230d_traceConfig 0x230d

#define OD_230d_0_traceConfig_maxSubIndex 0
#define OD_230d_1_traceConfig_size 1
#define OD_230d_2_traceConfig_axisNo 2
#define OD_230d_3_traceConfig_name 3
#define OD_230d_4_traceConfig_color 4
#define OD_230d_5_traceConfig_map 5
#define OD_230d_6_traceConfig_format 6
#define OD_230d_7_traceConfig_trigger 7
#define OD_230d_8_traceConfig_threshold 8

/*230e */
#define OD_230e_traceConfig 0x230e

#define OD_230e_0_traceConfig_maxSubIndex 0
#define OD_230e_1_traceConfig_size 1
#define OD_230e_2_traceConfig_axisNo 2
#define OD_230e_3_traceConfig_name 3
#define OD_230e_4_traceConfig_color 4
#define OD_230e_5_traceConfig_map 5
#define OD_230e_6_traceConfig_format 6
#define OD_230e_7_traceConfig_trigger 7
#define OD_230e_8_traceConfig_threshold 8

/*230f */
#define OD_230f_traceConfig 0x230f

#define OD_230f_0_traceConfig_maxSubIndex 0
#define OD_230f_1_traceConfig_size 1
#define OD_230f_2_traceConfig_axisNo 2
#define OD_230f_3_traceConfig_name 3
#define OD_230f_4_traceConfig_color 4
#define OD_230f_5_traceConfig_map 5
#define OD_230f_6_traceConfig_format 6
#define OD_230f_7_traceConfig_trigger 7
#define OD_230f_8_traceConfig_threshold 8

/*2310 */
#define OD_2310_traceConfig 0x2310

#define OD_2310_0_traceConfig_maxSubIndex 0
#define OD_2310_1_traceConfig_size 1
#define OD_2310_2_traceConfig_axisNo 2
#define OD_2310_3_traceConfig_name 3
#define OD_2310_4_traceConfig_color 4
#define OD_2310_5_traceConfig_map 5
#define OD_2310_6_traceConfig_format 6
#define OD_2310_7_traceConfig_trigger 7
#define OD_2310_8_traceConfig_threshold 8

/*2311 */
#define OD_2311_traceConfig 0x2311

#define OD_2311_0_traceConfig_maxSubIndex 0
#define OD_2311_1_traceConfig_size 1
#define OD_2311_2_traceConfig_axisNo 2
#define OD_2311_3_traceConfig_name 3
#define OD_2311_4_traceConfig_color 4
#define OD_2311_5_traceConfig_map 5
#define OD_2311_6_traceConfig_format 6
#define OD_2311_7_traceConfig_trigger 7
#define OD_2311_8_traceConfig_threshold 8

/*2312 */
#define OD_2312_traceConfig 0x2312

#define OD_2312_0_traceConfig_maxSubIndex 0
#define OD_2312_1_traceConfig_size 1
#define OD_2312_2_traceConfig_axisNo 2
#define OD_2312_3_traceConfig_name 3
#define OD_2312_4_traceConfig_color 4
#define OD_2312_5_traceConfig_map 5
#define OD_2312_6_traceConfig_format 6
#define OD_2312_7_traceConfig_trigger 7
#define OD_2312_8_traceConfig_threshold 8

/*2313 */
#define OD_2313_traceConfig 0x2313

#define OD_2313_0_traceConfig_maxSubIndex 0
#define OD_2313_1_traceConfig_size 1
#define OD_2313_2_traceConfig_axisNo 2
#define OD_2313_3_traceConfig_name 3
#define OD_2313_4_traceConfig_color 4
#define OD_2313_5_traceConfig_map 5
#define OD_2313_6_traceConfig_format 6
#define OD_2313_7_traceConfig_trigger 7
#define OD_2313_8_traceConfig_threshold 8

/*2314 */
#define OD_2314_traceConfig 0x2314

#define OD_2314_0_traceConfig_maxSubIndex 0
#define OD_2314_1_traceConfig_size 1
#define OD_2314_2_traceConfig_axisNo 2
#define OD_2314_3_traceConfig_name 3
#define OD_2314_4_traceConfig_color 4
#define OD_2314_5_traceConfig_map 5
#define OD_2314_6_traceConfig_format 6
#define OD_2314_7_traceConfig_trigger 7
#define OD_2314_8_traceConfig_threshold 8

/*2315 */
#define OD_2315_traceConfig 0x2315

#define OD_2315_0_traceConfig_maxSubIndex 0
#define OD_2315_1_traceConfig_size 1
#define OD_2315_2_traceConfig_axisNo 2
#define OD_2315_3_traceConfig_name 3
#define OD_2315_4_traceConfig_color 4
#define OD_2315_5_traceConfig_map 5
#define OD_2315_6_traceConfig_format 6
#define OD_2315_7_traceConfig_trigger 7
#define OD_2315_8_traceConfig_threshold 8

/*2316 */
#define OD_2316_traceConfig 0x2316

#define OD_2316_0_traceConfig_maxSubIndex 0
#define OD_2316_1_traceConfig_size 1
#define OD_2316_2_traceConfig_axisNo 2
#define OD_2316_3_traceConfig_name 3
#define OD_2316_4_traceConfig_color 4
#define OD_2316_5_traceConfig_map 5
#define OD_2316_6_traceConfig_format 6
#define OD_2316_7_traceConfig_trigger 7
#define OD_2316_8_traceConfig_threshold 8

/*2317 */
#define OD_2317_traceConfig 0x2317

#define OD_2317_0_traceConfig_maxSubIndex 0
#define OD_2317_1_traceConfig_size 1
#define OD_2317_2_traceConfig_axisNo 2
#define OD_2317_3_traceConfig_name 3
#define OD_2317_4_traceConfig_color 4
#define OD_2317_5_traceConfig_map 5
#define OD_2317_6_traceConfig_format 6
#define OD_2317_7_traceConfig_trigger 7
#define OD_2317_8_traceConfig_threshold 8

/*2318 */
#define OD_2318_traceConfig 0x2318

#define OD_2318_0_traceConfig_maxSubIndex 0
#define OD_2318_1_traceConfig_size 1
#define OD_2318_2_traceConfig_axisNo 2
#define OD_2318_3_traceConfig_name 3
#define OD_2318_4_traceConfig_color 4
#define OD_2318_5_traceConfig_map 5
#define OD_2318_6_traceConfig_format 6
#define OD_2318_7_traceConfig_trigger 7
#define OD_2318_8_traceConfig_threshold 8

/*2319 */
#define OD_2319_traceConfig 0x2319

#define OD_2319_0_traceConfig_maxSubIndex 0
#define OD_2319_1_traceConfig_size 1
#define OD_2319_2_traceConfig_axisNo 2
#define OD_2319_3_traceConfig_name 3
#define OD_2319_4_traceConfig_color 4
#define OD_2319_5_traceConfig_map 5
#define OD_2319_6_traceConfig_format 6
#define OD_2319_7_traceConfig_trigger 7
#define OD_2319_8_traceConfig_threshold 8

/*231a */
#define OD_231a_traceConfig 0x231a

#define OD_231a_0_traceConfig_maxSubIndex 0
#define OD_231a_1_traceConfig_size 1
#define OD_231a_2_traceConfig_axisNo 2
#define OD_231a_3_traceConfig_name 3
#define OD_231a_4_traceConfig_color 4
#define OD_231a_5_traceConfig_map 5
#define OD_231a_6_traceConfig_format 6
#define OD_231a_7_traceConfig_trigger 7
#define OD_231a_8_traceConfig_threshold 8

/*231b */
#define OD_231b_traceConfig 0x231b

#define OD_231b_0_traceConfig_maxSubIndex 0
#define OD_231b_1_traceConfig_size 1
#define OD_231b_2_traceConfig_axisNo 2
#define OD_231b_3_traceConfig_name 3
#define OD_231b_4_traceConfig_color 4
#define OD_231b_5_traceConfig_map 5
#define OD_231b_6_traceConfig_format 6
#define OD_231b_7_traceConfig_trigger 7
#define OD_231b_8_traceConfig_threshold 8

/*231c */
#define OD_231c_traceConfig 0x231c

#define OD_231c_0_traceConfig_maxSubIndex 0
#define OD_231c_1_traceConfig_size 1
#define OD_231c_2_traceConfig_axisNo 2
#define OD_231c_3_traceConfig_name 3
#define OD_231c_4_traceConfig_color 4
#define OD_231c_5_traceConfig_map 5
#define OD_231c_6_traceConfig_format 6
#define OD_231c_7_traceConfig_trigger 7
#define OD_231c_8_traceConfig_threshold 8

/*231d */
#define OD_231d_traceConfig 0x231d

#define OD_231d_0_traceConfig_maxSubIndex 0
#define OD_231d_1_traceConfig_size 1
#define OD_231d_2_traceConfig_axisNo 2
#define OD_231d_3_traceConfig_name 3
#define OD_231d_4_traceConfig_color 4
#define OD_231d_5_traceConfig_map 5
#define OD_231d_6_traceConfig_format 6
#define OD_231d_7_traceConfig_trigger 7
#define OD_231d_8_traceConfig_threshold 8

/*231e */
#define OD_231e_traceConfig 0x231e

#define OD_231e_0_traceConfig_maxSubIndex 0
#define OD_231e_1_traceConfig_size 1
#define OD_231e_2_traceConfig_axisNo 2
#define OD_231e_3_traceConfig_name 3
#define OD_231e_4_traceConfig_color 4
#define OD_231e_5_traceConfig_map 5
#define OD_231e_6_traceConfig_format 6
#define OD_231e_7_traceConfig_trigger 7
#define OD_231e_8_traceConfig_threshold 8

/*231f */
#define OD_231f_traceConfig 0x231f

#define OD_231f_0_traceConfig_maxSubIndex 0
#define OD_231f_1_traceConfig_size 1
#define OD_231f_2_traceConfig_axisNo 2
#define OD_231f_3_traceConfig_name 3
#define OD_231f_4_traceConfig_color 4
#define OD_231f_5_traceConfig_map 5
#define OD_231f_6_traceConfig_format 6
#define OD_231f_7_traceConfig_trigger 7
#define OD_231f_8_traceConfig_threshold 8

/*2320 */
#define OD_2320_traceConfig 0x2320

#define OD_2320_0_traceConfig_maxSubIndex 0
#define OD_2320_1_traceConfig_size 1
#define OD_2320_2_traceConfig_axisNo 2
#define OD_2320_3_traceConfig_name 3
#define OD_2320_4_traceConfig_color 4
#define OD_2320_5_traceConfig_map 5
#define OD_2320_6_traceConfig_format 6
#define OD_2320_7_traceConfig_trigger 7
#define OD_2320_8_traceConfig_threshold 8

/*2400 */
#define OD_2400_traceEnable 0x2400

/*2401 */
#define OD_2401_trace 0x2401

#define OD_2401_0_trace_maxSubIndex 0
#define OD_2401_1_trace_size 1
#define OD_2401_2_trace_value 2
#define OD_2401_3_trace_min 3
#define OD_2401_4_trace_max 4
#define OD_2401_5_trace_plot 5
#define OD_2401_6_trace_triggerTime 6

/*2402 */
#define OD_2402_trace 0x2402

#define OD_2402_0_trace_maxSubIndex 0
#define OD_2402_1_trace_size 1
#define OD_2402_2_trace_value 2
#define OD_2402_3_trace_min 3
#define OD_2402_4_trace_max 4
#define OD_2402_5_trace_plot 5
#define OD_2402_6_trace_triggerTime 6

/*2403 */
#define OD_2403_trace 0x2403

#define OD_2403_0_trace_maxSubIndex 0
#define OD_2403_1_trace_size 1
#define OD_2403_2_trace_value 2
#define OD_2403_3_trace_min 3
#define OD_2403_4_trace_max 4
#define OD_2403_5_trace_plot 5
#define OD_2403_6_trace_triggerTime 6

/*2404 */
#define OD_2404_trace 0x2404

#define OD_2404_0_trace_maxSubIndex 0
#define OD_2404_1_trace_size 1
#define OD_2404_2_trace_value 2
#define OD_2404_3_trace_min 3
#define OD_2404_4_trace_max 4
#define OD_2404_5_trace_plot 5
#define OD_2404_6_trace_triggerTime 6

/*2405 */
#define OD_2405_trace 0x2405

#define OD_2405_0_trace_maxSubIndex 0
#define OD_2405_1_trace_size 1
#define OD_2405_2_trace_value 2
#define OD_2405_3_trace_min 3
#define OD_2405_4_trace_max 4
#define OD_2405_5_trace_plot 5
#define OD_2405_6_trace_triggerTime 6

/*2406 */
#define OD_2406_trace 0x2406

#define OD_2406_0_trace_maxSubIndex 0
#define OD_2406_1_trace_size 1
#define OD_2406_2_trace_value 2
#define OD_2406_3_trace_min 3
#define OD_2406_4_trace_max 4
#define OD_2406_5_trace_plot 5
#define OD_2406_6_trace_triggerTime 6

/*2407 */
#define OD_2407_trace 0x2407

#define OD_2407_0_trace_maxSubIndex 0
#define OD_2407_1_trace_size 1
#define OD_2407_2_trace_value 2
#define OD_2407_3_trace_min 3
#define OD_2407_4_trace_max 4
#define OD_2407_5_trace_plot 5
#define OD_2407_6_trace_triggerTime 6

/*2408 */
#define OD_2408_trace 0x2408

#define OD_2408_0_trace_maxSubIndex 0
#define OD_2408_1_trace_size 1
#define OD_2408_2_trace_value 2
#define OD_2408_3_trace_min 3
#define OD_2408_4_trace_max 4
#define OD_2408_5_trace_plot 5
#define OD_2408_6_trace_triggerTime 6

/*2409 */
#define OD_2409_trace 0x2409

#define OD_2409_0_trace_maxSubIndex 0
#define OD_2409_1_trace_size 1
#define OD_2409_2_trace_value 2
#define OD_2409_3_trace_min 3
#define OD_2409_4_trace_max 4
#define OD_2409_5_trace_plot 5
#define OD_2409_6_trace_triggerTime 6

/*240a */
#define OD_240a_trace 0x240a

#define OD_240a_0_trace_maxSubIndex 0
#define OD_240a_1_trace_size 1
#define OD_240a_2_trace_value 2
#define OD_240a_3_trace_min 3
#define OD_240a_4_trace_max 4
#define OD_240a_5_trace_plot 5
#define OD_240a_6_trace_triggerTime 6

/*240b */
#define OD_240b_trace 0x240b

#define OD_240b_0_trace_maxSubIndex 0
#define OD_240b_1_trace_size 1
#define OD_240b_2_trace_value 2
#define OD_240b_3_trace_min 3
#define OD_240b_4_trace_max 4
#define OD_240b_5_trace_plot 5
#define OD_240b_6_trace_triggerTime 6

/*240c */
#define OD_240c_trace 0x240c

#define OD_240c_0_trace_maxSubIndex 0
#define OD_240c_1_trace_size 1
#define OD_240c_2_trace_value 2
#define OD_240c_3_trace_min 3
#define OD_240c_4_trace_max 4
#define OD_240c_5_trace_plot 5
#define OD_240c_6_trace_triggerTime 6

/*240d */
#define OD_240d_trace 0x240d

#define OD_240d_0_trace_maxSubIndex 0
#define OD_240d_1_trace_size 1
#define OD_240d_2_trace_value 2
#define OD_240d_3_trace_min 3
#define OD_240d_4_trace_max 4
#define OD_240d_5_trace_plot 5
#define OD_240d_6_trace_triggerTime 6

/*240e */
#define OD_240e_trace 0x240e

#define OD_240e_0_trace_maxSubIndex 0
#define OD_240e_1_trace_size 1
#define OD_240e_2_trace_value 2
#define OD_240e_3_trace_min 3
#define OD_240e_4_trace_max 4
#define OD_240e_5_trace_plot 5
#define OD_240e_6_trace_triggerTime 6

/*240f */
#define OD_240f_trace 0x240f

#define OD_240f_0_trace_maxSubIndex 0
#define OD_240f_1_trace_size 1
#define OD_240f_2_trace_value 2
#define OD_240f_3_trace_min 3
#define OD_240f_4_trace_max 4
#define OD_240f_5_trace_plot 5
#define OD_240f_6_trace_triggerTime 6

/*2410 */
#define OD_2410_trace 0x2410

#define OD_2410_0_trace_maxSubIndex 0
#define OD_2410_1_trace_size 1
#define OD_2410_2_trace_value 2
#define OD_2410_3_trace_min 3
#define OD_2410_4_trace_max 4
#define OD_2410_5_trace_plot 5
#define OD_2410_6_trace_triggerTime 6

/*2411 */
#define OD_2411_trace 0x2411

#define OD_2411_0_trace_maxSubIndex 0
#define OD_2411_1_trace_size 1
#define OD_2411_2_trace_value 2
#define OD_2411_3_trace_min 3
#define OD_2411_4_trace_max 4
#define OD_2411_5_trace_plot 5
#define OD_2411_6_trace_triggerTime 6

/*2412 */
#define OD_2412_trace 0x2412

#define OD_2412_0_trace_maxSubIndex 0
#define OD_2412_1_trace_size 1
#define OD_2412_2_trace_value 2
#define OD_2412_3_trace_min 3
#define OD_2412_4_trace_max 4
#define OD_2412_5_trace_plot 5
#define OD_2412_6_trace_triggerTime 6

/*2413 */
#define OD_2413_trace 0x2413

#define OD_2413_0_trace_maxSubIndex 0
#define OD_2413_1_trace_size 1
#define OD_2413_2_trace_value 2
#define OD_2413_3_trace_min 3
#define OD_2413_4_trace_max 4
#define OD_2413_5_trace_plot 5
#define OD_2413_6_trace_triggerTime 6

/*2414 */
#define OD_2414_trace 0x2414

#define OD_2414_0_trace_maxSubIndex 0
#define OD_2414_1_trace_size 1
#define OD_2414_2_trace_value 2
#define OD_2414_3_trace_min 3
#define OD_2414_4_trace_max 4
#define OD_2414_5_trace_plot 5
#define OD_2414_6_trace_triggerTime 6

/*2415 */
#define OD_2415_trace 0x2415

#define OD_2415_0_trace_maxSubIndex 0
#define OD_2415_1_trace_size 1
#define OD_2415_2_trace_value 2
#define OD_2415_3_trace_min 3
#define OD_2415_4_trace_max 4
#define OD_2415_5_trace_plot 5
#define OD_2415_6_trace_triggerTime 6

/*2416 */
#define OD_2416_trace 0x2416

#define OD_2416_0_trace_maxSubIndex 0
#define OD_2416_1_trace_size 1
#define OD_2416_2_trace_value 2
#define OD_2416_3_trace_min 3
#define OD_2416_4_trace_max 4
#define OD_2416_5_trace_plot 5
#define OD_2416_6_trace_triggerTime 6

/*2417 */
#define OD_2417_trace 0x2417

#define OD_2417_0_trace_maxSubIndex 0
#define OD_2417_1_trace_size 1
#define OD_2417_2_trace_value 2
#define OD_2417_3_trace_min 3
#define OD_2417_4_trace_max 4
#define OD_2417_5_trace_plot 5
#define OD_2417_6_trace_triggerTime 6

/*2418 */
#define OD_2418_trace 0x2418

#define OD_2418_0_trace_maxSubIndex 0
#define OD_2418_1_trace_size 1
#define OD_2418_2_trace_value 2
#define OD_2418_3_trace_min 3
#define OD_2418_4_trace_max 4
#define OD_2418_5_trace_plot 5
#define OD_2418_6_trace_triggerTime 6

/*2419 */
#define OD_2419_trace 0x2419

#define OD_2419_0_trace_maxSubIndex 0
#define OD_2419_1_trace_size 1
#define OD_2419_2_trace_value 2
#define OD_2419_3_trace_min 3
#define OD_2419_4_trace_max 4
#define OD_2419_5_trace_plot 5
#define OD_2419_6_trace_triggerTime 6

/*241a */
#define OD_241a_trace 0x241a

#define OD_241a_0_trace_maxSubIndex 0
#define OD_241a_1_trace_size 1
#define OD_241a_2_trace_value 2
#define OD_241a_3_trace_min 3
#define OD_241a_4_trace_max 4
#define OD_241a_5_trace_plot 5
#define OD_241a_6_trace_triggerTime 6

/*241b */
#define OD_241b_trace 0x241b

#define OD_241b_0_trace_maxSubIndex 0
#define OD_241b_1_trace_size 1
#define OD_241b_2_trace_value 2
#define OD_241b_3_trace_min 3
#define OD_241b_4_trace_max 4
#define OD_241b_5_trace_plot 5
#define OD_241b_6_trace_triggerTime 6

/*241c */
#define OD_241c_trace 0x241c

#define OD_241c_0_trace_maxSubIndex 0
#define OD_241c_1_trace_size 1
#define OD_241c_2_trace_value 2
#define OD_241c_3_trace_min 3
#define OD_241c_4_trace_max 4
#define OD_241c_5_trace_plot 5
#define OD_241c_6_trace_triggerTime 6

/*241d */
#define OD_241d_trace 0x241d

#define OD_241d_0_trace_maxSubIndex 0
#define OD_241d_1_trace_size 1
#define OD_241d_2_trace_value 2
#define OD_241d_3_trace_min 3
#define OD_241d_4_trace_max 4
#define OD_241d_5_trace_plot 5
#define OD_241d_6_trace_triggerTime 6

/*241e */
#define OD_241e_trace 0x241e

#define OD_241e_0_trace_maxSubIndex 0
#define OD_241e_1_trace_size 1
#define OD_241e_2_trace_value 2
#define OD_241e_3_trace_min 3
#define OD_241e_4_trace_max 4
#define OD_241e_5_trace_plot 5
#define OD_241e_6_trace_triggerTime 6

/*241f */
#define OD_241f_trace 0x241f

#define OD_241f_0_trace_maxSubIndex 0
#define OD_241f_1_trace_size 1
#define OD_241f_2_trace_value 2
#define OD_241f_3_trace_min 3
#define OD_241f_4_trace_max 4
#define OD_241f_5_trace_plot 5
#define OD_241f_6_trace_triggerTime 6

/*2420 */
#define OD_2420_trace 0x2420

#define OD_2420_0_trace_maxSubIndex 0
#define OD_2420_1_trace_size 1
#define OD_2420_2_trace_value 2
#define OD_2420_3_trace_min 3
#define OD_2420_4_trace_max 4
#define OD_2420_5_trace_plot 5
#define OD_2420_6_trace_triggerTime 6

/*6000 */
#define OD_6000_readInput8Bit 0x6000

#define OD_6000_0_readInput8Bit_maxSubIndex 0
#define OD_6000_1_readInput8Bit_input 1
#define OD_6000_2_readInput8Bit_input 2
#define OD_6000_3_readInput8Bit_input 3
#define OD_6000_4_readInput8Bit_input 4
#define OD_6000_5_readInput8Bit_input 5
#define OD_6000_6_readInput8Bit_input 6
#define OD_6000_7_readInput8Bit_input 7
#define OD_6000_8_readInput8Bit_input 8

/*6001 */
#define OD_6001_currentState 0x6001

/*6002 */
#define OD_6002_currentMovement 0x6002

/*6003 */
#define OD_6003_nextMovement 0x6003

/*6004 */
#define OD_6004_goButton 0x6004

/*6005 */
#define OD_6005_HB 0x6005

/*6040 */
#define OD_6040_controlWords 0x6040

#define OD_6040_0_controlWords_maxSubIndex 0
#define OD_6040_1_controlWords_motor1 1
#define OD_6040_2_controlWords_motor2 2
#define OD_6040_3_controlWords_motor3 3
#define OD_6040_4_controlWords_motor4 4
#define OD_6040_5_controlWords_motor5 5
#define OD_6040_6_controlWords_motor6 6

/*6041 */
#define OD_6041_statusWords 0x6041

#define OD_6041_0_statusWords_maxSubIndex 0
#define OD_6041_1_statusWords_motor1 1
#define OD_6041_2_statusWords_motor2 2
#define OD_6041_3_statusWords_motor3 3
#define OD_6041_4_statusWords_motor4 4
#define OD_6041_5_statusWords_motor5 5
#define OD_6041_6_statusWords_motor6 6

/*6064 */
#define OD_6064_actualMotorPositions 0x6064

#define OD_6064_0_actualMotorPositions_maxSubIndex 0
#define OD_6064_1_actualMotorPositions_motor1 1
#define OD_6064_2_actualMotorPositions_motor2 2
#define OD_6064_3_actualMotorPositions_motor3 3
#define OD_6064_4_actualMotorPositions_motor4 4
#define OD_6064_5_actualMotorPositions_motor5 5
#define OD_6064_6_actualMotorPositions_motor6 6

/*606c */
#define OD_606c_actualMotorVelocities 0x606c

#define OD_606c_0_actualMotorVelocities_maxSubIndex 0
#define OD_606c_1_actualMotorVelocities_motor1 1
#define OD_606c_2_actualMotorVelocities_motor2 2
#define OD_606c_3_actualMotorVelocities_motor3 3
#define OD_606c_4_actualMotorVelocities_motor4 4
#define OD_606c_5_actualMotorVelocities_motor5 5
#define OD_606c_6_actualMotorVelocities_motor6 6

/*6077 */
#define OD_6077_actualMotorTorques 0x6077

#define OD_6077_0_actualMotorTorques_maxSubIndex 0
#define OD_6077_1_actualMotorTorques_motor1 1
#define OD_6077_2_actualMotorTorques_motor2 2
#define OD_6077_3_actualMotorTorques_motor3 3
#define OD_6077_4_actualMotorTorques_motor4 4

/*607a */
#define OD_607a_targetMotorPositions 0x607a

#define OD_607a_0_targetMotorPositions_maxSubIndex 0
#define OD_607a_1_targetMotorPositions_motor1 1
#define OD_607a_2_targetMotorPositions_motor2 2
#define OD_607a_3_targetMotorPositions_motor3 3
#define OD_607a_4_targetMotorPositions_motor4 4
#define OD_607a_5_targetMotorPositions_motor5 5
#define OD_607a_6_targetMotorPositions_motor6 6

/*60ff */
#define OD_60ff_targetMotorVelocities 0x60ff

#define OD_60ff_0_targetMotorVelocities_maxSubIndex 0
#define OD_60ff_1_targetMotorVelocities_motor1 1
#define OD_60ff_2_targetMotorVelocities_motor2 2
#define OD_60ff_3_targetMotorVelocities_motor3 3
#define OD_60ff_4_targetMotorVelocities_motor4 4
#define OD_60ff_5_targetMotorVelocities_motor5 5
#define OD_60ff_6_targetMotorVelocities_motor6 6

/*6071 */
#define OD_6071_targetMotorTorques 0x6071

#define OD_6071_0_targetMotorTorques_maxSubIndex 0
#define OD_6071_1_targetMotorTorques_motor1 1
#define OD_6071_2_targetMotorTorques_motor2 2
#define OD_6071_3_targetMotorTorques_motor3 3
#define OD_6071_4_targetMotorTorques_motor4 4

/*6200 */
#define OD_6200_writeOutput8Bit 0x6200

#define OD_6200_0_writeOutput8Bit_maxSubIndex 0
#define OD_6200_1_writeOutput8Bit_output 1
#define OD_6200_2_writeOutput8Bit_output 2
#define OD_6200_3_writeOutput8Bit_output 3
#define OD_6200_4_writeOutput8Bit_output 4
#define OD_6200_5_writeOutput8Bit_output 5
#define OD_6200_6_writeOutput8Bit_output 6
#define OD_6200_7_writeOutput8Bit_output 7
#define OD_6200_8_writeOutput8Bit_output 8

/*6401 */
#define OD_6401_readAnalogueInput16Bit 0x6401

#define OD_6401_0_readAnalogueInput16Bit_maxSubIndex 0
#define OD_6401_1_readAnalogueInput16Bit_input 1
#define OD_6401_2_readAnalogueInput16Bit_input 2
#define OD_6401_3_readAnalogueInput16Bit_input 3
#define OD_6401_4_readAnalogueInput16Bit_input 4
#define OD_6401_5_readAnalogueInput16Bit_input 5
#define OD_6401_6_readAnalogueInput16Bit_input 6
#define OD_6401_7_readAnalogueInput16Bit_input 7
#define OD_6401_8_readAnalogueInput16Bit_input 8
#define OD_6401_9_readAnalogueInput16Bit_input 9
#define OD_6401_10_readAnalogueInput16Bit_input 10
#define OD_6401_11_readAnalogueInput16Bit_input 11
#define OD_6401_12_readAnalogueInput16Bit_input 12

/*6411 */
#define OD_6411_writeAnalogueOutput16Bit 0x6411

#define OD_6411_0_writeAnalogueOutput16Bit_maxSubIndex 0
#define OD_6411_1_writeAnalogueOutput16Bit_output 1
#define OD_6411_2_writeAnalogueOutput16Bit_output 2
#define OD_6411_3_writeAnalogueOutput16Bit_output 3
#define OD_6411_4_writeAnalogueOutput16Bit_output 4
#define OD_6411_5_writeAnalogueOutput16Bit_output 5
#define OD_6411_6_writeAnalogueOutput16Bit_output 6
#define OD_6411_7_writeAnalogueOutput16Bit_output 7
#define OD_6411_8_writeAnalogueOutput16Bit_output 8

/*******************************************************************************
   STRUCTURES FOR VARIABLES IN DIFFERENT MEMORY LOCATIONS
*******************************************************************************/
#define CO_OD_FIRST_LAST_WORD 0x55  //Any value from 0x01 to 0xFE. If changed, EEPROM will be reinitialized.

/***** Structure for ROM variables ********************************************/
struct sCO_OD_ROM {
    UNSIGNED32 FirstWord;

    /*100c      */ UNSIGNED16 guardTime;
    /*1012      */ UNSIGNED32 COB_ID_TIME;

    UNSIGNED32 LastWord;
};

/***** Structure for RAM variables ********************************************/
struct sCO_OD_RAM {
    UNSIGNED32 FirstWord;

    /*1000      */ UNSIGNED32 deviceType;
    /*1001      */ UNSIGNED8 errorRegister;
    /*1002      */ UNSIGNED32 manufacturerStatusRegister;
    /*1003      */ UNSIGNED32 preDefinedErrorField[8];
    /*1005      */ UNSIGNED32 COB_ID_SYNCMessage;
    /*1006      */ UNSIGNED32 communicationCyclePeriod;
    /*1007      */ UNSIGNED32 synchronousWindowLength;
    /*1008      */ VISIBLE_STRING manufacturerDeviceName[11];
    /*1009      */ VISIBLE_STRING manufacturerHardwareVersion[4];
    /*100a      */ VISIBLE_STRING manufacturerSoftwareVersion[4];
    /*100d      */ UNSIGNED8 lifeTimeFactor;
    /*1010      */ UNSIGNED32 storeParameters[1];
    /*1011      */ UNSIGNED32 restoreDefaultParameters[1];
    /*1013      */ UNSIGNED32 highResolutionTimeStamp;
    /*1014      */ UNSIGNED32 COB_ID_EMCY;
    /*1015      */ UNSIGNED16 inhibitTimeEMCY;
    /*1016      */ UNSIGNED32 consumerHeartbeatTime[4];
    /*1017      */ UNSIGNED16 producerHeartbeatTime;
    /*1018      */ OD_identity_t identity;
    /*1019      */ UNSIGNED8 synchronousCounterOverflowValue;
    /*1029      */ UNSIGNED8 errorBehavior[6];
    /*1200      */ OD_SDOServerParameter_t SDOServerParameter[1];
    /*1280      */ OD_SDOClientParameter_t SDOClientParameter[1];
    /*1400      */ OD_RPDOCommunicationParameter_t RPDOCommunicationParameter[32];
    /*1600      */ OD_RPDOMappingParameter_t RPDOMappingParameter[32];
    /*1800      */ OD_TPDOCommunicationParameter_t TPDOCommunicationParameter[32];
    /*1a00      */ OD_TPDOMappingParameter_t TPDOMappingParameter[32];
    /*1f80      */ UNSIGNED32 NMTStartup;
    /*1f81      */ UNSIGNED32 slaveAssignment[127];
    /*1f82      */ UNSIGNED8 requestNMT[127];
    /*1f89      */ UNSIGNED32 bootTime;
    /*2100      */ OCTET_STRING errorStatusBits[10];
    /*2101      */ UNSIGNED8 CANNodeID;
    /*2102      */ UNSIGNED16 CANBitRate;
    /*2103      */ UNSIGNED16 SYNCCounter;
    /*2104      */ UNSIGNED16 SYNCTime;
    /*2106      */ UNSIGNED32 powerOnCounter;
    /*2107      */ UNSIGNED16 performance[5];
    /*2108      */ INTEGER16 temperature[1];
    /*2109      */ INTEGER16 voltage[1];
    /*2110      */ INTEGER32 variableInt32[32];
    /*2111      */ INTEGER32 variableROM_Int32[16];
    /*2112      */ INTEGER32 variableNV_Int32[16];
    /*2120      */ OD_testVar_t testVar;
    /*2130      */ OD_time_t time;
    /*2209      */ OD_motorTempSensorVoltages_t motorTempSensorVoltages;
    /*2301      */ OD_traceConfig_t traceConfig[32];
    /*2400      */ UNSIGNED8 traceEnable;
    /*2401      */ OD_trace_t trace[32];
    /*6000      */ UNSIGNED8 readInput8Bit[8];
    /*6001      */ UNSIGNED16 currentState;
    /*6002      */ UNSIGNED16 currentMovement;
    /*6003      */ UNSIGNED16 nextMovement;
    /*6004      */ UNSIGNED16 goButton;
    /*6005      */ UNSIGNED16 HB;
    /*6040      */ OD_controlWords_t controlWords;
    /*6041      */ OD_statusWords_t statusWords;
    /*6064      */ OD_actualMotorPositions_t actualMotorPositions;
    /*606c      */ OD_actualMotorVelocities_t actualMotorVelocities;
    /*6077      */ OD_actualMotorTorques_t actualMotorTorques;
    /*607a      */ OD_targetMotorPositions_t targetMotorPositions;
    /*60ff      */ OD_targetMotorVelocities_t targetMotorVelocities;
    /*6071      */ OD_targetMotorTorques_t targetMotorTorques;
    /*6200      */ UNSIGNED8 writeOutput8Bit[8];
    /*6401      */ INTEGER16 readAnalogueInput16Bit[12];
    /*6411      */ INTEGER16 writeAnalogueOutput16Bit[8];

    UNSIGNED32 LastWord;
};

/***** Structure for EEPROM variables ********************************************/
struct sCO_OD_EEPROM {
    UNSIGNED32 FirstWord;

    UNSIGNED32 LastWord;
};

/***** Declaration of Object Dictionary variables *****************************/
extern struct sCO_OD_ROM CO_OD_ROM;

extern struct sCO_OD_RAM CO_OD_RAM;

extern struct sCO_OD_EEPROM CO_OD_EEPROM;

/*******************************************************************************
   ALIASES FOR OBJECT DICTIONARY VARIABLES
*******************************************************************************/
/*1000, Data Type: UNSIGNED32 */
#define OD_deviceType CO_OD_RAM.deviceType

/*1001, Data Type: UNSIGNED8 */
#define OD_errorRegister CO_OD_RAM.errorRegister

/*1002, Data Type: UNSIGNED32 */
#define OD_manufacturerStatusRegister CO_OD_RAM.manufacturerStatusRegister

/*1003, Data Type: UNSIGNED32, Array[8] */
#define OD_preDefinedErrorField CO_OD_RAM.preDefinedErrorField
#define ODL_preDefinedErrorField_arrayLength 8
#define ODA_preDefinedErrorField_standardErrorField 0

/*1005, Data Type: UNSIGNED32 */
#define OD_COB_ID_SYNCMessage CO_OD_RAM.COB_ID_SYNCMessage

/*1006, Data Type: UNSIGNED32 */
#define OD_communicationCyclePeriod CO_OD_RAM.communicationCyclePeriod

/*1007, Data Type: UNSIGNED32 */
#define OD_synchronousWindowLength CO_OD_RAM.synchronousWindowLength

/*1008, Data Type: VISIBLE_STRING */
#define OD_manufacturerDeviceName CO_OD_RAM.manufacturerDeviceName
#define ODL_manufacturerDeviceName_stringLength 11

/*1009, Data Type: VISIBLE_STRING */
#define OD_manufacturerHardwareVersion CO_OD_RAM.manufacturerHardwareVersion
#define ODL_manufacturerHardwareVersion_stringLength 4

/*100a, Data Type: VISIBLE_STRING */
#define OD_manufacturerSoftwareVersion CO_OD_RAM.manufacturerSoftwareVersion
#define ODL_manufacturerSoftwareVersion_stringLength 4

/*100c, Data Type: UNSIGNED16 */
#define OD_guardTime CO_OD_ROM.guardTime

/*100d, Data Type: UNSIGNED8 */
#define OD_lifeTimeFactor CO_OD_RAM.lifeTimeFactor

/*1010, Data Type: UNSIGNED32, Array[1] */
#define OD_storeParameters CO_OD_RAM.storeParameters
#define ODL_storeParameters_arrayLength 1
#define ODA_storeParameters_saveAllParameters 0

/*1011, Data Type: UNSIGNED32, Array[1] */
#define OD_restoreDefaultParameters CO_OD_RAM.restoreDefaultParameters
#define ODL_restoreDefaultParameters_arrayLength 1
#define ODA_restoreDefaultParameters_restoreAllDefaultParameters 0

/*1012, Data Type: UNSIGNED32 */
#define OD_COB_ID_TIME CO_OD_ROM.COB_ID_TIME

/*1013, Data Type: UNSIGNED32 */
#define OD_highResolutionTimeStamp CO_OD_RAM.highResolutionTimeStamp

/*1014, Data Type: UNSIGNED32 */
#define OD_COB_ID_EMCY CO_OD_RAM.COB_ID_EMCY

/*1015, Data Type: UNSIGNED16 */
#define OD_inhibitTimeEMCY CO_OD_RAM.inhibitTimeEMCY

/*1016, Data Type: UNSIGNED32, Array[4] */
#define OD_consumerHeartbeatTime CO_OD_RAM.consumerHeartbeatTime
#define ODL_consumerHeartbeatTime_arrayLength 4
#define ODA_consumerHeartbeatTime_consumerHeartbeatTime 0

/*1017, Data Type: UNSIGNED16 */
#define OD_producerHeartbeatTime CO_OD_RAM.producerHeartbeatTime

/*1018, Data Type: identity_t */
#define OD_identity CO_OD_RAM.identity

/*1019, Data Type: UNSIGNED8 */
#define OD_synchronousCounterOverflowValue CO_OD_RAM.synchronousCounterOverflowValue

/*1029, Data Type: UNSIGNED8, Array[6] */
#define OD_errorBehavior CO_OD_RAM.errorBehavior
#define ODL_errorBehavior_arrayLength 6
#define ODA_errorBehavior_communication 0
#define ODA_errorBehavior_communicationOther 1
#define ODA_errorBehavior_communicationPassive 2
#define ODA_errorBehavior_generic 3
#define ODA_errorBehavior_deviceProfile 4
#define ODA_errorBehavior_manufacturerSpecific 5

/*1200, Data Type: SDOServerParameter_t */
#define OD_SDOServerParameter CO_OD_RAM.SDOServerParameter

/*1280, Data Type: SDOClientParameter_t */
#define OD_SDOClientParameter CO_OD_RAM.SDOClientParameter

/*1400, Data Type: RPDOCommunicationParameter_t */
#define OD_RPDOCommunicationParameter CO_OD_RAM.RPDOCommunicationParameter

/*1600, Data Type: RPDOMappingParameter_t */
#define OD_RPDOMappingParameter CO_OD_RAM.RPDOMappingParameter

/*1800, Data Type: TPDOCommunicationParameter_t */
#define OD_TPDOCommunicationParameter CO_OD_RAM.TPDOCommunicationParameter

/*1a00, Data Type: TPDOMappingParameter_t */
#define OD_TPDOMappingParameter CO_OD_RAM.TPDOMappingParameter

/*1f80, Data Type: UNSIGNED32 */
#define OD_NMTStartup CO_OD_RAM.NMTStartup

/*1f81, Data Type: UNSIGNED32, Array[127] */
#define OD_slaveAssignment CO_OD_RAM.slaveAssignment
#define ODL_slaveAssignment_arrayLength 127
#define ODA_slaveAssignment_ 0

/*1f82, Data Type: UNSIGNED8, Array[127] */
#define OD_requestNMT CO_OD_RAM.requestNMT
#define ODL_requestNMT_arrayLength 127
#define ODA_requestNMT_ 0

/*1f89, Data Type: UNSIGNED32 */
#define OD_bootTime CO_OD_RAM.bootTime

/*2100, Data Type: OCTET_STRING */
#define OD_errorStatusBits CO_OD_RAM.errorStatusBits
#define ODL_errorStatusBits_stringLength 10

/*2101, Data Type: UNSIGNED8 */
#define OD_CANNodeID CO_OD_RAM.CANNodeID

/*2102, Data Type: UNSIGNED16 */
#define OD_CANBitRate CO_OD_RAM.CANBitRate

/*2103, Data Type: UNSIGNED16 */
#define OD_SYNCCounter CO_OD_RAM.SYNCCounter

/*2104, Data Type: UNSIGNED16 */
#define OD_SYNCTime CO_OD_RAM.SYNCTime

/*2106, Data Type: UNSIGNED32 */
#define OD_powerOnCounter CO_OD_RAM.powerOnCounter

/*2107, Data Type: UNSIGNED16, Array[5] */
#define OD_performance CO_OD_RAM.performance
#define ODL_performance_arrayLength 5
#define ODA_performance_cyclesPerSecond 0
#define ODA_performance_timerCycleTime 1
#define ODA_performance_timerCycleMaxTime 2
#define ODA_performance_mainCycleTime 3
#define ODA_performance_mainCycleMaxTime 4

/*2108, Data Type: INTEGER16, Array[1] */
#define OD_temperature CO_OD_RAM.temperature
#define ODL_temperature_arrayLength 1
#define ODA_temperature_mainPCB 0

/*2109, Data Type: INTEGER16, Array[1] */
#define OD_voltage CO_OD_RAM.voltage
#define ODL_voltage_arrayLength 1
#define ODA_voltage_mainPCBSupply 0

/*2110, Data Type: INTEGER32, Array[32] */
#define OD_variableInt32 CO_OD_RAM.variableInt32
#define ODL_variableInt32_arrayLength 32
#define ODA_variableInt32_int32 0

/*2111, Data Type: INTEGER32, Array[16] */
#define OD_variableROM_Int32 CO_OD_RAM.variableROM_Int32
#define ODL_variableROM_Int32_arrayLength 16
#define ODA_variableROM_Int32_int32 0

/*2112, Data Type: INTEGER32, Array[16] */
#define OD_variableNV_Int32 CO_OD_RAM.variableNV_Int32
#define ODL_variableNV_Int32_arrayLength 16
#define ODA_variableNV_Int32_int32 0

/*2120, Data Type: testVar_t */
#define OD_testVar CO_OD_RAM.testVar

/*2130, Data Type: time_t */
#define OD_time CO_OD_RAM.time

/*2209, Data Type: motorTempSensorVoltages_t */
#define OD_motorTempSensorVoltages CO_OD_RAM.motorTempSensorVoltages

/*2301, Data Type: traceConfig_t */
#define OD_traceConfig CO_OD_RAM.traceConfig

/*2400, Data Type: UNSIGNED8 */
#define OD_traceEnable CO_OD_RAM.traceEnable

/*2401, Data Type: trace_t */
#define OD_trace CO_OD_RAM.trace

/*6000, Data Type: UNSIGNED8, Array[8] */
#define OD_readInput8Bit CO_OD_RAM.readInput8Bit
#define ODL_readInput8Bit_arrayLength 8
#define ODA_readInput8Bit_input 0

/*6001, Data Type: UNSIGNED16 */
#define OD_currentState CO_OD_RAM.currentState

/*6002, Data Type: UNSIGNED16 */
#define OD_currentMovement CO_OD_RAM.currentMovement

/*6003, Data Type: UNSIGNED16 */
#define OD_nextMovement CO_OD_RAM.nextMovement

/*6004, Data Type: UNSIGNED16 */
#define OD_goButton CO_OD_RAM.goButton

/*6005, Data Type: UNSIGNED16 */
#define OD_HB CO_OD_RAM.HB

/*6040, Data Type: controlWords_t */
#define OD_controlWords CO_OD_RAM.controlWords

/*6041, Data Type: statusWords_t */
#define OD_statusWords CO_OD_RAM.statusWords

/*6064, Data Type: actualMotorPositions_t */
#define OD_actualMotorPositions CO_OD_RAM.actualMotorPositions

/*606c, Data Type: actualMotorVelocities_t */
#define OD_actualMotorVelocities CO_OD_RAM.actualMotorVelocities

/*6077, Data Type: actualMotorTorques_t */
#define OD_actualMotorTorques CO_OD_RAM.actualMotorTorques

/*607a, Data Type: targetMotorPositions_t */
#define OD_targetMotorPositions CO_OD_RAM.targetMotorPositions

/*60ff, Data Type: targetMotorVelocities_t */
#define OD_targetMotorVelocities CO_OD_RAM.targetMotorVelocities

/*6071, Data Type: targetMotorTorques_t */
#define OD_targetMotorTorques CO_OD_RAM.targetMotorTorques

/*6200, Data Type: UNSIGNED8, Array[8] */
#define OD_writeOutput8Bit CO_OD_RAM.writeOutput8Bit
#define ODL_writeOutput8Bit_arrayLength 8
#define ODA_writeOutput8Bit_output 0

/*6401, Data Type: INTEGER16, Array[12] */
#define OD_readAnalogueInput16Bit CO_OD_RAM.readAnalogueInput16Bit
#define ODL_readAnalogueInput16Bit_arrayLength 12
#define ODA_readAnalogueInput16Bit_input 0

/*6411, Data Type: INTEGER16, Array[8] */
#define OD_writeAnalogueOutput16Bit CO_OD_RAM.writeAnalogueOutput16Bit
#define ODL_writeAnalogueOutput16Bit_arrayLength 8
#define ODA_writeAnalogueOutput16Bit_output 0
