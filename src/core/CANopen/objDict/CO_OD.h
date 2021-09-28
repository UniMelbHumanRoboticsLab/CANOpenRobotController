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
#ifndef CO_OD_H
#define CO_OD_H

#include "CO_driver.h"
#include "CO_SDO.h"

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
#define CO_OD_NoOfElements (39 + CO_NO_RPDO * 3 + CO_NO_TPDO * 3)

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

/*2130 */
#define OD_2130_time 0x2130

#define OD_2130_0_time_maxSubIndex 0
#define OD_2130_1_time_string 1
#define OD_2130_2_time_epochTimeBaseMs 2
#define OD_2130_3_time_epochTimeOffsetMs 3


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
    /*2130      */ OD_time_t time;

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
extern OD_RPDOCommunicationParameter_t *OD_RPDOCommunicationParameter[CO_NO_RPDO];

// Change to store the parameters in a dedicated array
// #define OD_RPDOCommunicationParameter CO_OD_RAM.RPDOCommunicationParameter
extern OD_RPDOCommunicationParameter_t *OD_RPDOCommunicationParameter[CO_NO_RPDO];
/*1600, Data Type: RPDOMappingParameter_t */
extern OD_RPDOMappingParameter_t *OD_RPDOMappingParameter[CO_NO_RPDO];

/*1800, Data Type: TPDOCommunicationParameter_t */
extern OD_TPDOCommunicationParameter_t *OD_TPDOCommunicationParameter[CO_NO_TPDO];

/*1a00, Data Type: TPDOMappingParameter_t */
extern OD_TPDOMappingParameter_t *OD_TPDOMappingParameter[CO_NO_TPDO];

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

/**
 * Configures the Object Dictionary with blank PDOs.
 *
 * @return Success or failure of the configuration
 */
bool_t CO_configure(void);

/**
 * Configures a single element in the object dictionary
 *
 * @return Success or failure of the configuration
 */
bool_t CO_OD_set_entry(uint16_t element_, uint16_t index_, uint8_t maxSubIndex_, uint16_t attribute_, uint16_t length_, void *pData_);

/**
 * \brief Sets up a RPDO on the CANOpen Stack, including setting up the PDO backend and adding to the OD 
 * 
 * \param RPDOCommParams Communication Paramters of the RPDO
 * \param RPDOMapParams Mapping Parameters of the RPDO
 * \param RPDOCommEntry A OD Entry pointing to the Communication Parameters
 * \param dataStoreRecord An OD Entry pointing to the location of the stored data 
 * \param RPDOMapParamsEntry An OD Entry pointing to the mapping parameters
 * \return int The Number of the RPDO which was created 
 */
int CO_setRPDO(OD_RPDOCommunicationParameter_t *RPDOCommParams, OD_RPDOMappingParameter_t *RPDOMapParams, CO_OD_entryRecord_t *RPDOCommEntry, CO_OD_entryRecord_t *dataStoreRecord, CO_OD_entryRecord_t *RPDOMapParamsEntry);

/**
 * \brief Sets up a TPDO on the CANOpen Stack, including setting up the PDO backend and adding to the OD 
 * 
 * \param TPDOCommParams Communication Paramters of the TPDO
 * \param TPDOMapParams Mapping Parameters of the TPDO
 * \param TPDOCommEntry A OD Entry pointing to the Communication Parameters
 * \param dataStoreRecord An OD Entry pointing to the location of the stored data 
 * \param TPDOMapParamsEntry An OD Entry pointing to the mapping parameters
 * \return int The Number of the RPDO which was created 
 */
int CO_setTPDO(OD_TPDOCommunicationParameter_t *TPDOCommParams, OD_TPDOMappingParameter_t *TPDOMapParams, CO_OD_entryRecord_t *TPDOCommEntry, CO_OD_entryRecord_t *dataStoreRecord, CO_OD_entryRecord_t *TPDOmapparamEntry);

/**
 * \brief Sets up a RPDO on the CANOpen Stack, including setting up the PDO backend and adding to the OD 
 * 
 * \param RPDOCommParams Communication Paramters of the RPDO
 * \param RPDOMapParams Mapping Parameters of the RPDO
 * \param RPDOCommEntry A OD Entry pointing to the Communication Parameters
 * \param dataStoreRecord An OD Entry pointing to the location of the stored data 
 * \param RPDOMapParamsEntry An OD Entry pointing to the mapping parameters
 * \return int The Number of the RPDO which was created 
 */
int CO_setRPDO(OD_RPDOCommunicationParameter_t *RPDOCommParams, OD_RPDOMappingParameter_t *RPDOMapParams, CO_OD_entryRecord_t *RPDOCommEntry, CO_OD_entryRecord_t *dataStoreRecord, CO_OD_entryRecord_t *RPDOMapParamsEntry);

/**
 * \brief Sets up a TPDO on the CANOpen Stack, including setting up the PDO backend and adding to the OD 
 * 
 * \param TPDOCommParams Communication Paramters of the TPDO
 * \param TPDOMapParams Mapping Parameters of the TPDO
 * \param TPDOCommEntry A OD Entry pointing to the Communication Parameters
 * \param dataStoreRecord An OD Entry pointing to the location of the stored data 
 * \param TPDOMapParamsEntry An OD Entry pointing to the mapping parameters
 * \return int The Number of the RPDO which was created 
 */
int CO_setTPDO(OD_TPDOCommunicationParameter_t *TPDOCommParams, OD_TPDOMappingParameter_t *TPDOMapParams, CO_OD_entryRecord_t *TPDOCommEntry, CO_OD_entryRecord_t *dataStoreRecord, CO_OD_entryRecord_t *TPDOmapparamEntry);

#endif
