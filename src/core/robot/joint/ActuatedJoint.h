/**
 * \file ActuatedJoint.h
 * \author Justin Fong
 * \brief The <code>ActuatedJoint</code> class is a abstract class which represents a joint in a
 * <code>Robot</code> objec. This class implements the Joint class, and specifically 
 * represents a joint which is actuated. This therefore requires a Drive object
 * which will be used to interact with the physical hardware.  
 * \version 0.1
 * \date 2020-04-09
 * \version 0.1
 * \copyright Copyright (c) 2020
 * 
 */

/**
 * The <code>ActuatedJoint</code> class is a abstract class which represents a joint in a
 * <code>Robot</code> objec. This class implements the Joint class, and specifically 
 * represents a joint which is actuated. This therefore requires a Drive object
 * which will be used to interact with the physical hardware.  
 * 
 *
 * Version 0.1
 * Date: 07/04/2020
 */

#ifndef ACTUATEDJOINT_H_INCLUDED
#define ACTUATEDJOINT_H_INCLUDED
#include "Drive.h"
#include "Joint.h"

/**
 * The <code>setMovementReturnCode_t<code> is used to determine whether the movement was a
 * success, or whether an error occurred in its application. 
 */
enum setMovementReturnCode_t {
    SUCCESS = 1,
    OUTSIDE_LIMITS = -1,
    INCORRECT_MODE = -2,
    UNKNOWN_ERROR = -100
};

/**
 * @ingroup Joint
 * \brief Abstract class representing an actuated joint in a Robot Class (extending joint). Requires a Drive object through which commands are sent.
 * 
 */
class ActuatedJoint : public Joint {
   protected:
    /**
         * \brief Contains a Drive object, which is a CANOpen device which is used to control the
         * physical hardware. 
         * 
         */
    Drive *drive;

    /**
         * \brief The current mode of the drive
         * 
         */
    ControlMode driveMode = UNCONFIGURED;

    /**
         * \brief Converts from the joint value to the equivalent value for the drive
         * 
         * Notes:
         * - The drive value is always an integer (due to the CANOpen specification)
         *      and the joint value is always a double (data type of q)
         * - This may be a linear relationship (e.g. degrees to encoder counts) or a more
         *      complicated one (e.g. linear actuator position to degrees) depending on the 
         *      structure of the device and system.
         * 
         * 
         * \param jointValue The joint value to be converted
         * \return int The equivalent drive value for the given joint value
         */
    virtual int toDriveUnits(double jointValue) = 0;

    /**
         * \brief Converts from the drive value to the equivalent value for the joint
         * 
         * Notes:
         * - The drive value is always an integer (due to the CANOpen specification)
         *      and the joint value is always a double (data type of q)
         * - This may be a linear relationship (e.g. degrees to encoder counts) or a more
         *      complicated one (e.g. linear actuator position to degrees) depending on the 
         *      structure of the device and system.
         * 
         * \param driveValue The drive value to be converted
         * \return The equivalent joint value for the given drive value
         */
    virtual double fromDriveUnits(int driveValue) = 0;

   public:
    /**
         * \brief Construct a new Actuated Joint object
         * 
         * \param jointID Unique ID representing the joint (not checked in this class)
         * \param jointMin Minimum allowable value for the joint
         * \param jointMax Maximum allowable value for the joint
         */
    ActuatedJoint(int jointID, double jointMin, double jointMax, Drive *drive);

    /**
         * \brief Set the mode of the device (nominally, position, velocity or torque control)
         * 
         * \param driveMode The mode to be used if possible
         * \param motorProfile variables for desired mode, e.g. postion: v,a and deceleration.
         * \return ControlMode Configured Drive Mode, -1 if unsuccessful
         */
    virtual ControlMode setMode(ControlMode driveMode_, motorProfile = motorProfile{0,0,0});

    /**
         * \brief Set the Position object
         * 
         * \param desQ The desired set position
         * \return setMovementReturnCode_t The result of the setting
         */
    virtual setMovementReturnCode_t setPosition(double desQ);

    /**
         * \brief Sets a velocity set point (in joint units)
         * 
         * \param velocity The desired set position
         * \return setMovementReturnCode_t The result of the setting
         */
    virtual setMovementReturnCode_t setVelocity(double velocity);

    /**
         * \brief Set the torque set point
         * 
         * \param torque The desired set position
         * \return setMovementReturnCode_t The result of the setting
         */
    virtual setMovementReturnCode_t setTorque(double torque);

    /**
      * \brief Set the joint ready to switch On 
      * 
      */
    virtual void readyToSwitchOn();

    /**
     * \brief Enable the joint
     * 
     * \return true if succesful
     * \return false if drive is currently not in the correct state to enable
     */
    bool enable();
};

#endif