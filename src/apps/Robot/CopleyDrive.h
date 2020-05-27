
/**
 * \file CopleyDrive.h
 * \author Justin Fong
 * \brief  An implementation of the Drive Object, specifically for the Copley Drive
 * 
 * This class enables low level functions to the system. It does limited error 
 * checking. 
 * \version 0.1
 * \date 2020-04-07
 * \version 0.1
 * \copyright Copyright (c) 2020
 * 
 */
#ifndef COPLEYDRIVE_H_INCLUDED
#define COPLEYDRIVE_H_INCLUDED
#include "Drive.h"

/**
 * \brief An implementation of the Drive Object, specifically for Copley-branded devices (currently used on the X2 Exoskeleton)
 * 
 */
class CopleyDrive : public Drive {
   public:
    /**
         * \brief Construct a new Copley Drive object
         * 
         * \param NodeID CANopen Node ID
         */
    CopleyDrive(int NodeID);

    /**
         * \brief Destroy the Copley Drive object
         * 
         */
    ~CopleyDrive();
    /**
         * Initialises the drive (SDO start message)
         * 
         * \return True if successful, False if not
         */
    bool Init();
    /**
     * \todo Move jointMinMap and jointMaxMap to set additional parameters (bit 5 in 0x6041 makes updates happen immediately)
     * 
     */
    /**
         * Sets the drive to Position control with default parameters (through SDO messages)
         * 
         * Note: Should be overloaded to allow parameters to be set
         * 
         * \return true if successful
         * \return false if not
         */
    bool initPosControl(motorProfile posControlMotorProfile);
    /**
         * Sets the drive to Velocity control with default parameters (through SDO messages)
         * 
         * Note: Should be overloaded to allow parameters to be set
         * 
         * \return true if successful
         * \return false if not
         */
    bool initVelControl(motorProfile velControlMotorProfile);

    /**
         * Sets the drive to Torque control with default parameters (through SDO messages)
         * 
         * Note: Should be overloaded to allow parameters to be set
         * 
         * \return true if successful
         * \return false if not
         */
    bool initTorqueControl();
    /**
          * \brief Overloaded method from Drive, specifically for Copley Drive implementation.
          *     Generates the list of commands required to configure Position control in CANopen motor drive
          * 
          * /param Profile Velocity, value used by position mode motor trajectory generator.
          *            Units: 0.1 counts/sec
          *            Range:0 - 500,000,000
          * /param Profile Acceleration, value position mode motor trajectory generator will attempt to achieve.
          *            Units: 10 counts/sec^2
          *            Range:0 - 200,000,000
          * /param Profile Deceleration, value position mode motor trajectory generator will use at end of trapezoidal profile.
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
          * \brief Overloaded method from Drive, specifically for Copley Drive implementation.
          *     Generates the list of commands required to configure Velocity control in CANopen motor drive
          * 
          * /param Profile Acceleration, value Velocity mode motor trajectory generator will attempt to achieve.
          *            Units: 10 counts/sec^2
          *            Range:0 - 200,000,000
          * /param Profile Deceleration, value Velocity mode motor trajectory generator will use at end of trapezoidal profile.
          *             see programmers manual for other profile types use.
          *            Units: 10 counts/sec^2
          *            Range:0 - 200,000,000
          * 
          *    NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
          *           https://www.can-cia.org/can-knowledge/canopen/cia402/
          * 
          */
    std::vector<std::string> generateVelControlConfigSDO(motorProfile velocityProfile);
    /**
          * \brief Overloaded method from Drive, specifically for Copley Drive implementation.
          *     Generates the list of commands required to configure Torque control in CANopen motor drive
          *
          *    NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
          *           https://www.can-cia.org/can-knowledge/canopen/cia402/
          *
          */
    std::vector<std::string> generateTorqueControlConfigSDO();
};

#endif