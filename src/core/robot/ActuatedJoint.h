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
    NOT_CALIBRATED = -3,
    UNKNOWN_ERROR = -100
};
/**
 * setMovementReturnCode_t explicit description
 */
static std::map<setMovementReturnCode_t, std::string> setMovementReturnCodeString = {
    {SUCCESS, "Ok."},
    {OUTSIDE_LIMITS, "Outside of limits."},
    {INCORRECT_MODE, "Incorrect drive mode"},
    {NOT_CALIBRATED, "Not calibrated."},
    {UNKNOWN_ERROR, "Unknown error."}
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
    bool calibrated;

    /**
         * \brief Converts from the joint position value to the equivalent value for the drive
         *
         * Notes:
         * - The drive value is always an integer (due to the CANOpen specification)
         *      and the joint value is always a double (data type of position)
         * - This may be a linear relationship (e.g. degrees to encoder counts) or a more
         *      complicated one (e.g. linear actuator position to degrees) depending on the
         *      structure of the device and system.
         *
         *
         * \param jointValue The joint value to be converted
         * \return int The equivalent drive value for the given joint value
         */
    virtual int jointPositionToDriveUnit(double jointValue) = 0;

    /**
         * \brief Converts from the drive value to the equivalent value for the joint position
         *
         * Notes:
         * - The drive value is always an integer (due to the CANOpen specification)
         *      and the joint value is always a double (data type of position)
         * - This may be a linear relationship (e.g. degrees to encoder counts) or a more
         *      complicated one (e.g. linear actuator position to degrees) depending on the
         *      structure of the device and system.
         *
         * \param driveValue The drive value to be converted
         * \return The equivalent joint value for the given drive value
         */
    virtual double driveUnitToJointPosition(int driveValue) = 0;

    /**
         * \brief Converts from the joint velocity value to the equivalent value for the drive
         *
         * Notes:
         * - The drive value is always an integer (due to the CANOpen specification)
         *      and the joint value is always a double (data type of position)
         * - This may be a linear relationship (e.g. degrees/s to encoder count/s) or a more
         *      complicated one (e.g. linear actuator velocity to degrees/s) depending on the
         *      structure of the device and system.
         *
         *
         * \param jointValue The joint value to be converted
         * \return int The equivalent drive value for the given joint value
         */
    virtual int jointVelocityToDriveUnit(double jointValue) = 0;

    /**
         * \brief Converts from the drive value to the equivalent value for the joint position
         *
         * Notes:
         * - The drive value is always an integer (due to the CANOpen specification)
         *      and the joint value is always a double (data type of position)
         * - This may be a linear relationship (e.g. degrees/s to encoder count/s) or a more
         *      complicated one (e.g. linear actuator velocity to degrees/s) depending on the
         *      structure of the device and system.
         *
         * \param driveValue The drive value to be converted
         * \return The equivalent joint value for the given drive value
         */
    virtual double driveUnitToJointVelocity(int driveValue) = 0;

    /**
         * \brief Converts from the joint torque value to the equivalent value for the drive
         *
         * Notes:
         * - The drive value is always an integer (due to the CANOpen specification)
         *      and the joint value is always a double (data type of position)
         * - This may be a linear relationship (e.g. Nm to rated torque) or a more
         *      complicated one depending on the
         *      structure of the device and system.
         *
         *
         * \param jointValue The joint value to be converted
         * \return int The equivalent drive value for the given joint value
         */
    virtual int jointTorqueToDriveUnit(double jointValue) = 0;

    /**
         * \brief Converts from the drive value to the equivalent value for the joint position
         *
         * Notes:
         * - The drive value is always an integer (due to the CANOpen specification)
         *      and the joint value is always a double (data type of position)
         * - This may be a linear relationship (e.g. Nm to rated torque) or a more
         *      complicated one depending on the
         *      structure of the device and system.
         *
         * \param driveValue The drive value to be converted
         * \return The equivalent joint value for the given drive value
         */
    virtual double driveUnitToJointTorque(int driveValue) = 0;

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
         * \brief Start the associated drive CAN node (will start produce PDOs)
         *
         * \return Return value of drive->start();
         */
    bool start();

    /**
         * \brief Set the mode of the device (nominally, position, velocity or torque control)
         *
         * \param driveMode The mode to be used if possible
         * \param motorProfile variables for desired mode, e.g. postion: v,a and deceleration. Not used in torque control
         * \return ControlMode Configured Drive Mode, -1 if unsuccessful
         */
    virtual ControlMode setMode(ControlMode driveMode_, motorProfile);

    /**
         * \brief Set the mode of the device (nominally, position, velocity or torque control)
         *
         * \param driveMode The mode to be used if possible
         * \return ControlMode Configured Drive Mode, -1 if unsuccessful
         */
    virtual ControlMode setMode(ControlMode driveMode_);

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
     * \brief Set current position as joint position offset (q0)
     * such that current position is now qcalib
     *
     */
    void setPositionOffset(double qcalib);

    /**
         * \brief get the joint position
         *
         * \return int The current joint position [encoder count]
         */
    virtual double getPosition();

    /**
         * \brief get the joint velocity
         *
         * \return int The current joint velocity [encoder count/0.1sec]
         */
    virtual double getVelocity();

    /**
     * \brief get the joint torque
     *
     * \return int The current joint torque [rated torque in Nmm]
     */
    virtual double getTorque();

    /**
     * \brief get the drive status word value
     *
     * \return int The current status word of the drive
     */
    int getDriveStatus() {return drive->getStatus(); }

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

    /**
     * \brief Disable the drive
     *
     * \return true if succesful
     * \return false if drive is currently not in the correct state to enable
     */
    bool disable();
};

#endif
