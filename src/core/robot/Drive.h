/**
 * \file Drive.h
 * \author Justin Fong
 * \brief  The <code>Drive</code> class is used to interface with a CANOpen motor drive. According to the CiA402 standard
 *
 * This class enables low level functions to the system. It does limited error
 * checking.
 * \version 0.1
 * \date 2020-04-07
 * \version 0.1
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef DRIVE_H_INCLUDED
#define DRIVE_H_INCLUDED

#include <CANopen.h>
#include <CO_command.h>
#include <string.h>

#include <map>
#include <sstream>
#include <vector>

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
    {"0x08000022", "Access not possible due to current device status"}};

/**
 * An enum type.
 * Constants representing the control mode of the drive
 */
enum ControlMode {
    CM_UNCONFIGURED = 0,
    CM_POSITION_CONTROL = 1,
    CM_VELOCITY_CONTROL = 2,
    CM_TORQUE_CONTROL = 3,
    CM_ERROR = -1,
    CM_UNACTUATED_JOINT = -2
};
/**
 * An enum type.
 * Constants representing the Drives State
 */
enum DriveState {
    DISABLED = 0,
    READY_TO_SWITCH_ON = 1,
    ENABLED = 2,
};

/**
 * An enum type
 * Commonly-used entries defined in the Object Dictionary for CiA402 Drives
 *
 */
enum OD_Entry_t {
    STATUS_WORD = 0,
    ACTUAL_POS = 1,
    ACTUAL_VEL = 2,
    ACTUAL_TOR = 3,
    ERROR_WORD = 4,
    CONTROL_WORD = 10,
    TARGET_POS = 11,
    TARGET_VEL = 12,
    TARGET_TOR = 13,
    DIGITAL_IN = 14,
    DIGITAL_OUT = 15,
    ANALOG_IN1 = 16,
    ANALOG_IN2 = 17
};

/**
 * \brief struct to hold desired velocity, acceleration and deceleration values for a
 *     drives motor controller profile.
 */
struct motorProfile {
    int profileVelocity;
    int profileAcceleration;
    int profileDeceleration;
};

/**
 * @ingroup Robot
 * \brief Abstract class describing a Drive used to communicate with a CANbus device. Note that many functions are implemented according to the CiA 402 Standard (but can be overridden)
 *
 */
class Drive {
   protected:
    /**
        * \brief The CAN Node ID used to address this particular drive on the CAN bus
        *
        */
    int NodeID;

    /**
     * \brief Lists of PDOs
     *
     */
    std::vector<RPDO *> rpdos;
    std::vector<TPDO *> tpdos;

    /**
        * \brief Generates the list of commands required to configure TPDOs on the drives
        *
        * \param items A list of OD_Entry_t items which are to be configured with this TPDO
        * \param PDO_Num The number/index of this PDO
        * \param COB_ID the COB-ID of the PDO
        * \param SyncRate The rate at which this PDO transmits (e.g. number of Sync Messages. 0xFF represents internal trigger event)
        * \param sub_idx The register sub index
        * \return std::string
        */
    std::vector<std::string> generateTPDOConfigSDO(std::vector<OD_Entry_t> items, int PDO_Num, int COB_ID, int SyncRate);

    /**
        * \brief Creates a RPDO in the Local Object Dictionary for the equivalent TPDO on the drive
        *
        * \param items A list of OD_Entry_t items which are to be configured with this RPDO
        * \param COB_ID the COB-ID of the PDO
        * \param RPDOSyncRate The rate at which the RPDO processes NOTE: This is not the same as for the equivalent TPDO
        */
    void generateEquivalentMasterRPDO(std::vector<OD_Entry_t> items,  int COB_ID, int RPDOSyncRate);


    /**
        * \brief Generates the list of commands required to configure RPDOs on the drives
        *
        * \param items A list of OD_Entry_t items which are to be configured with this RPDO
        * \param PDO_Num The number/index of this PDO
        * \param COB_ID the COB-ID of the PDO
        * \param UpdateTiming 0-240 represents hold until next sync message, 0xFF represents immediate update
        * \param sub_idx The register sub index
        * \return std::string
        */
    std::vector<std::string> generateRPDOConfigSDO(std::vector<OD_Entry_t> items, int PDO_Num, int COB_ID,int UpdateTiming);

    /**
        * \brief Creates a TPDO in the Local Object Dictionary for the equivalent RPDO on the drive
        *
        * \param items A list of OD_Entry_t items which are to be configured with this TPDO
        * \param COB_ID the COB-ID of the PDO
        * \param RPDOSyncRate The rate at which the TPDO sends messages NOTE: This is not the same as for the equivalent RPDO
        */
    void generateEquivalentMasterTPDO(std::vector<OD_Entry_t> items, int COB_ID, int TPDOSyncRate);

    /**
       *
       * \brief  Generates the list of commands required to configure Position control in CANopen motor drive
       *
       *
       * \param Profile Velocity, value used by position mode motor trajectory generator.
       * \param Profile Acceleration, value position mode motor trajectory generator will attempt to achieve.
       * \param Profile Deceleration, value position mode motor trajectory generator will use at end of trapezoidal profile.
       *
       * NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
       *           https://www.can-cia.org/can-knowledge/canopen/cia402/
       */
    std::vector<std::string> generatePosControlConfigSDO(motorProfile positionProfile);

    /**
       *
       * \brief  Start the drive node and set the Position control in CANopen motor drive
       *
       *
       * NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
       *           https://www.can-cia.org/can-knowledge/canopen/cia402/
       */
    std::vector<std::string> generatePosControlConfigSDO();

    /**
       *
       * \brief  Generates the list of commands required to configure Velocity control in CANopen motor drive
       *
       *
       * \param Profile Velocity, value used by Velocity mode motor trajectory generator.
       *
       * \param Profile Acceleration, value Velocity mode motor trajectory generator will attempt to achieve.
       *
       * \param Profile Deceleration, value Velocity mode motor trajectory generator will use at end of trapezoidal profile.
       *
       * NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
       *           https://www.can-cia.org/can-knowledge/canopen/cia402/
       */
    std::vector<std::string> generateVelControlConfigSDO(motorProfile velocityProfile);

    /**
       *
       * \brief  Start the drive node and set the Velcoity control in CANopen motor drive
       *
       *
       * NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
       *           https://www.can-cia.org/can-knowledge/canopen/cia402/
       */
    std::vector<std::string> generateVelControlConfigSDO();

    /**
       *
       * \brief  Generates the list of commands required to configure Torque control in CANopen motor drive
       *
       * NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
       *           https://www.can-cia.org/can-knowledge/canopen/cia402/
       */
    std::vector<std::string> generateTorqueControlConfigSDO();

    /**
     * \brief Send a list (vector) of properly formatted SDO Messages
     *
     * \return int -number_of_unsuccesfull messages (0 means OK for all). vcan will always return 0 (no reply check).
     */
    int sendSDOMessages(std::vector<std::string> messages);


    /**
     * \brief Map between the TPDO Number and their Mapped Objects
     *
     */
    std::map<UNSIGNED8, std::vector<OD_Entry_t>> TPDO_MappedObjects = {
        {1, {STATUS_WORD}},
        {2, {ACTUAL_POS, ACTUAL_VEL}},
        {3, {ACTUAL_TOR}},
        {4, {DIGITAL_IN}}};

    /**
     * \brief Map between the RPDO Number and their Mapped Objects
     *
     */
    std::map<UNSIGNED8, std::vector<OD_Entry_t>> RPDO_MappedObjects = {
        {1, {CONTROL_WORD, DIGITAL_OUT}},
        {2, {TARGET_POS}},
        {3, {TARGET_VEL}},
        {4, {TARGET_TOR}}};

    /**
     * \brief Map between the RPDO Number and their base COB-ID (actualy is base COB-ID + Node_ID)
     *
     * NOTE: This is an arbitrary default mapping unique to CORC. This can be changed in derived classes
     */
    std::map<UNSIGNED8, UNSIGNED32> TPDO_COBID = {
        {1, 0x180},
        {2, 0x280},
        {3, 0x380},
        {4, 0x480}};

    /**
     * \brief Map between the RPDO Number and their base COB-ID (actualy is base COB-ID + Node_ID)
     *
     * NOTE: This is an arbitrary default mapping unique to CORC. This can be changed in derived classes
     */
    std::map<UNSIGNED8, UNSIGNED32> RPDO_COBID = {
        {1, 0x200},
        {2, 0x300},
        {3, 0x400},
        {4, 0x500}};

    /**
     * \brief Map between the Commonly-used OD entries and their data lengths - used to generate PDO Configurations
     *        NOTE: These are in bytes. The PDO configurations requires this to be in bits, so a conversion is required.
     *
     */
    std::map<OD_Entry_t, int> OD_DataSize = {
        {STATUS_WORD, 2},
        {ERROR_WORD, 2},
        {ACTUAL_POS, 4},
        {ACTUAL_VEL, 4},
        {ACTUAL_TOR, 2},
        {CONTROL_WORD, 2},
        {TARGET_POS, 4},
        {TARGET_VEL, 4},
        {TARGET_TOR, 2},
        {DIGITAL_IN, 4},
        {DIGITAL_OUT, 4},
        {ANALOG_IN1, 2},
        {ANALOG_IN2, 2}};

    /**
     * \brief Map between the Commonly-used OD entries and their addresses and sub-index - used to generate PDO Configurations
     *        NOTE: These are written in hexadecimal. They can be altered in derived classes if required.
     *
     */
    std::map<OD_Entry_t, std::array<int, 2>> OD_Addresses = {
        {STATUS_WORD, {0x6041, 0x00}},
        {ERROR_WORD, {0x603F, 0x00}},
        {ACTUAL_POS, {0x6064, 0x00}},
        {ACTUAL_VEL, {0x606C, 0x00}},
        {ACTUAL_TOR, {0x6077, 0x00}},
        {CONTROL_WORD, {0x6040, 0x00}},
        {TARGET_POS, {0x607A, 0x00}},
        {TARGET_VEL, {0x60FF, 0x00}},
        {TARGET_TOR, {0x6071, 0x00}},
        {DIGITAL_IN, {0x60FD, 0x00}},
        {DIGITAL_OUT, {0x60FE, 0x01}},
        {ANALOG_IN1, {0x3160, 0x01}},
        {ANALOG_IN2, {0x3160, 0x02}}};

    std::map<OD_Entry_t, void *> OD_MappedObjectAddresses = {
        {STATUS_WORD, (void *)&statusWord},
        {ERROR_WORD, (void *)&errorWord},
        {ACTUAL_POS, (void *)&actualPos},
        {ACTUAL_VEL, (void *)&actualVel},
        {ACTUAL_TOR, (void *)&actualTor},
        {CONTROL_WORD, (void *)&controlWord},
        {TARGET_POS, (void *)&targetPos},
        {TARGET_VEL, (void *)&targetVel},
        {TARGET_TOR, (void *)&targetTor},
        {DIGITAL_IN, (void *)&digitalIn},
        {DIGITAL_OUT, (void *)&digitalOut},
        {ANALOG_IN1, (void *)&analogIn[0]},
        {ANALOG_IN2, (void *)&analogIn[1]}};

   private:
    /**
        * \brief Current status word of the drive
        *
        */
    UNSIGNED16 statusWord =0;
    UNSIGNED16 errorWord=0;
    INTEGER32 actualPos=0;
    INTEGER32 actualVel=0;
    INTEGER16 actualTor=0;
    UNSIGNED16 controlWord=0;
    INTEGER32 targetPos=0;
    INTEGER32 targetVel=0;
    INTEGER16 targetTor=0;
    UNSIGNED32 digitalIn=0;
    UNSIGNED32 digitalOut=0;
    INTEGER16 analogIn[2]={0,0};
    /**
     * \brief Current error state of the drive
     *
     */
    int error;

    /**
        * \brief State of the drive
        *
        */
    DriveState driveState = DISABLED;

    /**
        * \brief The mode in which the drive is currently configured
        *
        */
    ControlMode controlMode = CM_UNCONFIGURED;

   public:
    /**
        * \brief Construct a new Drive object
        *
        */
    Drive();

    /**
       * \brief Construct a new Drive object
       *
       * \param node_id the CANopen Node ID of this drive
       */
    Drive(int node_id);

    /**
       * \brief Destroy the Drive object
       *
       */
    virtual ~Drive();

    /**
       * \brief Initialises the drive (SDO start message)
       *
       * \return True if successful, False if not
       */
    virtual bool init() = 0;

    /**
       * \brief Send NMT preop command to the drive node
       *
       * \return 0 if unsuccesfull
       */
    int preop();

    /**
       * \brief Send NMT start command to the drive node
       *
       * \return 0 if unsuccesfull
       */
    int start();

    /**
       * \brief Send NMT stop command to the drive node
       *
       * \return 0 if unsuccesfull
       */
    int stop();

    /**
           * Writes the desired digital out value to the drive
           *
           * \return true if successful
           * \return false if not
           */
    virtual bool setDigitalOut(int digital_out);

    /**
           * \brief Initialises a standard set of PDOs for the use of the drive. These are:
           *
           *   TPDO1: COB-ID 180+{NODE-ID}: Status Word (0x6041), Send on Internal Event Trigger
           *   TPDO2: COB-ID 280+{NODE-ID}: Actual Position (0x6064), Actual Velocity (0x606C), Sent every SYNC Message
           *   TPDO3: COB-ID 380+{NODE-ID}: Actual Torque (0x607C), Sent every SYNC MEssage
           *
           *   RPDO3: COB-ID 300+{NODE-ID}: Target Position (0x607A), Applied immediately when received
           *   RPDO4: COB-ID 400+{NODE-ID}: Target Velocity (0x60FF), Applied immediately when received
           *   RPDO5: COB-ID 500+{NODE-ID}: Target Torque (0x6071), Applied immediately when received
           *
           * \return true
           * \return false
           */
    virtual bool initPDOs();

    bool configureMasterPDOs();

    /**
       * \brief Initialises velocity and acceleration profiles (used by position and velocity controls) through SDOs write
       *
       * \return true if sucessfull
       * \return false otherwise
       */
    virtual bool setMotorProfile(motorProfile profile);

    /**
       * Sets the drive to Position control with set parameters (through SDO messages)
       *
       * Note: Should be overloaded to allow parameters to be set
       *
       * \param motorProfile The position control motor profile to be used
       *
       * \return true if successful
       * \return false if not
       */
    virtual bool initPosControl(motorProfile posControlMotorProfile) { return false; };
    virtual bool initPosControl() { return false; };

    /**
       * Sets the drive to Velocity control with default parameters (through SDO messages)
       *
       * Note: Should be overloaded to allow parameters to be set
       *
       * \return true if successful
       * \return false if not
       */
    virtual bool initVelControl(motorProfile velControlMotorProfile) { return false; };
    virtual bool initVelControl() { return false; };

    /**
       * Sets the drive to Torque control with default parameters (through SDO messages)
       *
       * Note: Should be overloaded to allow parameters to be set
       *
       * \return true if successful
       * \return false if not
       */
    virtual bool initTorqueControl() { return false; };

    /**
       * Updates the internal representation of the state of the drive
       *
       * \return The current value of the status word (0x6041)
       */
    virtual int getStatus();

    /**
       * Writes the desired position to the Target Position of the motor drive (0x607A)
       *
       * \return true if successful
       * \return false if not
       */
    virtual bool setPos(int position);

    /**
       * Writes the desired velocity to the Target Velocity of the motor drive (0x60FF)
       *
       * \return true if successful
       * \return false if not
       */
    virtual bool setVel(int velocity);

    /**
       * Writes the desired torque to the Target Torque of the motor drive (0x6071)
       *
       * \return true if successful
       * \return false if not
       */
    virtual bool setTorque(int torque);

    /**
       * Returns the current position from the motor drive (0x6064)
       *
       * \return Position from the motor drive
       */
    virtual int getPos();

    /**
       * Returns the current velocity from the motor drive (0x606C)
       * Returns 0 if NODEID is 5 or 6: ankles. They have no OD entry.
       * \return Velocity from the motor drive
       */
    virtual int getVel();

    /**
       * Returns the current torque from the motor drive (0x6077)
       * Returns 0 if NODEID is 5 or 6: ankles. They have no OD entry.
       * \return Torque from the motor drive
       */
    virtual int getTorque();

    /**
       * Returns the value of digital IN
       * \return Digital in state from the motor drive
       */
    virtual int getDigitalIn();

    /**
       * Returns the value of an Analog In
       *
       * \param index number of the AI
       *
       * \return Latest AI value read from the drive if on PDO
       */
    virtual double getAnalogIn(unsigned int index);


    // Drive State Modifiers
    /**
       * \brief Clears errors (and changes the state of the drive to "disabled".
       *
       * This is equivalent to setting bits 7 Control Word (0x6064) to 1.
       * See also the CANopen Programmer's Manual (from Copley Controls)
       *
       * \return true if operation successful
       * \return false if operation unsuccessful
       */
    virtual DriveState resetErrors();

    /**
       * \brief Changes the state of the drive to "ready to switch on".
       *
       * This is equivalent to setting bits 2 and 3 of Control Word (0x6064) to 1.
       * See also the CANopen Programmer's Manual (from Copley Controls)
       *
       * \return true if operation successful
       * \return false if operation unsuccessful
       */
    virtual DriveState readyToSwitchOn();

    /**
       * \brief Sets the state of the drive to "enabled"
       *
       * This is equivalent to setting bits 0, 1, 2, 3 of the control word (0x06064) to 1
       * See also the CANopen Programmer's Manual (from Copley Controls)
       *
       * \return true if operation successful
       * \return false if operation unsuccessful
       */
    virtual DriveState enable();

    /**
       * \brief sets the state of the drive to "disabled"
       *
       * This is equivalent to setting the control word (0x06064) to 0
       * See also the CANopen Programmer's Manual (from Copley Controls)
       *
       * \return true if operation successful
       * \return false if operation unsuccessful
       */
    virtual DriveState disable();

    /**
        * \brief Flips Bit 4 of Control Word (0x6041) - A new set point is only confirmed if the transition is from 0 to 1
        *
        * \return true The control word was previously 0 (i.e. successful set point confirm)
        * \return false The control word was previously 1 (i.e. unsuccessful set point confirm)
        */
    virtual bool posControlConfirmSP();

    /**
        * \brief Sets the continous/not continous profile bit
        *
        * \param continuous if this is true, continous movement is enabled, otherwise it is disabled
        * \return true If change is successful
        * \return false If drive was not in position control mode
        */
    virtual bool posControlSetContinuousProfile(bool continuous);

    /**
        * \brief Get the current state of the drive
        *
        * \return DriveState
        */
    virtual DriveState getState();

    /**
        * \brief Get the current control mode of the drive
        *
        * \return controlMode
        */
    virtual ControlMode getControlMode() { return controlMode; };

    // CANOpen
    /**
       * \brief Get returns the CanNode ID
       *
       * \return int the Node ID
       */
    int getNodeID();
};

#endif
