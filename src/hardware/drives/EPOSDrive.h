
/**
 * \file EPOSDrive.h
 * \author Vincent Crocher
 * \brief  An implementation of the Drive class, specifically for the EPOS Drive
 * Inspired mostly from CopleyDrive class.
 *
 * \version 0.1
 * \date 2025-03-13
 * \copyright Copyright (c) 2025
 *
 */
#ifndef EPOSDRIVE_H
#define EPOSDRIVE_H
#include "Drive.h"

/**
 * \brief An implementation of the Drive Object, specifically for EPOS devices
 *
 */
class EPOSDrive : public Drive {
   public:
    /**
     * \brief Construct a new EPOS Drive object
     *
     * \param NodeID CANopen Node ID
     */
    EPOSDrive(int NodeID);

    /**
     * \brief Destroy the EPOS Drive object
     *
     */
    ~EPOSDrive();
    /**
     * Initialises the drive (SDO start message)
     *
     * \return True if successful, False if not
     */
    bool init();
    /**
     * Setup PDOs and motor profile (acceleration and velocity)
     *
     * \return True if successful, False if not
     */
    bool init(motorProfile profile);


    /**
     * Sets the drive to Position control with default parameters (through SDO messages)
     *
     *
     * \return true if successful
     * \return false if not
     */
    bool initPosControl(motorProfile posControlMotorProfile);
    /**
     * Sets the drive to Velocity control with default parameters (through SDO messages)
     *
     *
     * \return true if successful
     * \return false if not
     */
    bool initVelControl(motorProfile velControlMotorProfile);
    bool initVelControl();

    /**
     * Sets the drive to Torque control with default parameters (through SDO messages)
     *
     *
     * \return true if successful
     * \return false if not
     */
    bool initTorqueControl();
    
    /**
      * \brief Overloaded method from Drive
      *     Generates the list of commands required to configure Position control in CANopen motor drive
      *
      * \param Profile Velocity, value used by position mode motor trajectory generator.
      *            Units: 0.1 counts/sec
      *            Range:0 - 500,000,000
      * \param Profile Acceleration, value position mode motor trajectory generator will attempt to achieve.
      *            Units: 10 counts/sec^2
      *            Range:0 - 200,000,000
      * \param Profile Deceleration, value position mode motor trajectory generator will use at end of trapezoidal profile.
      *             see programmers manual for other profile types use.
      *            Units: 10 counts/sec^2
      *            Range:0 - 200,000,000
      *
      *    NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
      *           https://www.can-cia.org/can-knowledge/canopen/cia402/
      *
      */
    std::vector<std::string> generatePosControlConfigSDO(motorProfile positionProfile);
    
    /**
      * \brief Overloaded method from Drive
      *     Generates the list of commands required to configure Velocity control in CANopen motor drive
      *
      * \param Profile Acceleration, value Velocity mode motor trajectory generator will attempt to achieve.
      *            Units: 10 counts/sec^2
      *            Range:0 - 200,000,000
      * \param Profile Deceleration, value Velocity mode motor trajectory generator will use at end of trapezoidal profile.
      *             see programmers manual for other profile types use.
      *            Units: 10 counts/sec^2
      *            Range:0 - 200,000,000
      *
      *    NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
      *           https://www.can-cia.org/can-knowledge/canopen/cia402/
      *
      */
    std::vector<std::string> generateVelControlConfigSDO(motorProfile velocityProfile);
    std::vector<std::string> generateVelControlConfigSDO();

    /**
      * \brief Overloaded method from Drive
      *     Generates the list of commands required to configure Torque control in CANopen motor drive
      *
      *    NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
      *           https://www.can-cia.org/can-knowledge/canopen/cia402/
      *
      */
    std::vector<std::string> generateTorqueControlConfigSDO();


    bool cmdTimeoutConfig(int timeout_in_ms);

    //! SDO to define the timeout in cyclic velocity and position modes
    std::vector<std::string> generateCmdTimeoutConfigSDO(int timeout_in_ms);


    /**
     * \brief Sends SDO value to set the fault mask
     *
     * \param mask value of the mask
     *
     * \return true if successful
     * \return false if not
     */
    bool setFaultMask(UNSIGNED32 mask);

    bool resetError();
    std::vector<std::string> generateResetErrorSDO();

    /**
     * \brief Read error word via SDO read.
     *
     * \return Error word value if succesful
     * \return -1 if failed (read error)
     */
    int SDOReadErrorWord();
};
#endif
