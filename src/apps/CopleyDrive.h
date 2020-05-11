/**
/**
 * @brief An implementation of the Drive Object, specifically for the Copley Drive
 * 
 * @version 0.1
 * @date 2020-04-09
 */
#ifndef COPLEYDRIVE_H_INCLUDED
#define COPLEYDRIVE_H_INCLUDED
#include "Drive.h"

/**
 * @brief An implementation of the Drive Object, specifically for Copley-branded devices (currently used on the X2 Exoskeleton)
 * 
 */
class CopleyDrive : public Drive {
   public:
    /**
         * @brief Construct a new Copley Drive object
         * 
         * @param NodeID CANopen Node ID
         */
    CopleyDrive(int NodeID);

    /**
         * @brief Destroy the Copley Drive object
         * 
         */
    ~CopleyDrive();
    /**
         * Initialises the drive (SDO start message)
         * 
         * @return True if successful, False if not
         */
    bool Init();

    /**
         * Sets the drive to Position control with default parameters (through SDO messages)
         * 
         * Note: Should be overloaded to allow parameters to be set
         * 
         * @return true if successful
         * @return false if not
         */
    bool initPosControl(motorProfile posControlMotorProfile);

    /**
         * Sets the drive to Velocity control with default parameters (through SDO messages)
         * 
         * Note: Should be overloaded to allow parameters to be set
         * 
         * @return true if successful
         * @return false if not
         */
    bool initVelControl();

    /**
         * Sets the drive to Torque control with default parameters (through SDO messages)
         * 
         * Note: Should be overloaded to allow parameters to be set
         * 
         * @return true if successful
         * @return false if not
         */
    bool initTorqControl();
};

#endif