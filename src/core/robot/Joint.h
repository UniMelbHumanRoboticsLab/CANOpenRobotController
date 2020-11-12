/**
 * \file Joint.h
 * \brief The <code>Joint</code> class is a abstract class which represents a joint in a
 * <code>Robot</code> objec. This class can be used to represent all types of joints,
 * including actuated, non-actuated, revolute, prismatic, etc.
 * \version 0.1
 * \date 2020-04-10
 * \version 0.1
 * \copyright Copyright (c) 2020
 */
#ifndef JOINT_H_INCLUDED
#define JOINT_H_INCLUDED
#include <iomanip>
#include <iostream>
#include <cstring>

#include "Drive.h"
/** @defgroup Joint Joint Module
 *  @ingroup Robot
 *  A group of abstract joint classes, acting as the software representation of a Joint.
 */

/**
 * The <code>setMovementReturnCode_t<code> is used to determine whether the movement was a
 * success, or whether an error occurred in its application.
 */
enum setMovementReturnCode_t {
    SUCCESS = 1,
    OUTSIDE_LIMITS = -1,
    INCORRECT_MODE = -2,
    NOT_CALIBRATED = -3,
    UNACTUATED_JOINT = -99,
    UNKNOWN_ERROR = -100
};

/**
 * Used to created viewable messages based on setMovementReturnCode_t
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
 * \brief Abstract class representing any joints within a Robot.
 *
 */
class Joint {
   protected:
    /**
     * An identifier for this joint. Note that this identifier is designed to be unique,
     * but this is not managed by the joint class.
     */
    const int id;
    /**
     * Joint name. Used primarily for debug display and ROS state publishing (if applicable).
     */
    const std::string name;
    /**
     * The current state of the joint (i.e. the value), to be returned in SI units.
     */
    double position;
    /**
     * The current state of the joint velocity (i.e. the value), to be returned in SI units.
     */
    double velocity;
    /**
     * The current state of the joint torque(i.e. the value), to be returned in SI units.
     */
    double torque;
    /**
     * The allowable limits of the joint. This should represent the theoretical limits
     * of the joint. Should these be exceeded, an error should be thrown.
     */
    const double qMin, qMax;
    /**
     * The joint offset position in SI units. By default this is zero.s
     */
    double q0 = 0;

    /**
    * @brief Defines whether this joint is actuated or not (i.e. if it has a drive object attached )
    *
    */
    const bool actuated;

    /**
      * \brief Contains a Drive object, which is a CANOpen device which is used to control the
      * physical hardware. If joint is unactuated, set to NULL.
      *
      */
    Drive *drive;

    /**
      * \brief The current mode of the drive (if actuated joint)
      *
      */
    ControlMode driveMode = CM_UNCONFIGURED;

    /**
     * @brief Indicates whether a calibration has been performed on this joint yet.
     *
     */
    bool calibrated = false;

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
    virtual int jointPositionToDriveUnit(double jointValue) { if(actuated){spdlog::error("Joint::error: using default conversion drive to joint units!");} return 0; };

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
    virtual double driveUnitToJointPosition(int driveValue) { if(actuated){spdlog::error("Joint::error: using default conversion drive to joint units!");} return 0; };

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
    virtual int jointVelocityToDriveUnit(double jointValue) { if(actuated){spdlog::error("Joint::error: using default conversion drive to joint units!");} return 0; };

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
    virtual double driveUnitToJointVelocity(int driveValue) { if(actuated){spdlog::error("Joint::error: using default conversion drive to joint units!");} return 0; };

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
    virtual int jointTorqueToDriveUnit(double jointValue) { if(actuated){spdlog::error("Joint::error: using default conversion drive to joint units!");} return 0; };

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
    virtual double driveUnitToJointTorque(int driveValue) { if(actuated){spdlog::error("Joint::error: using default conversion drive to joint units!");} return 0; };

    /**
    * @brief Fetches the joint position from the hardware (e.g. Drive) and converts to joint units
    *
    * Default behaviour is to take value from the Drive object if actuated. Should be overloaded for non-drive joints
    *
    * @return double the current position of the joint
    */
    virtual double updatePosition();

    /**
    * @brief Fetches the joint velocity from the hardware (e.g. Drive) and converts to joint units
    *
    * Default behaviour is to take value from the Drive object if actuated. Should be overloaded for non-drive joints
    * @return double the current velocity of the joint
    */
    virtual double updateVelocity();

    /**
    * @brief Fetches the joint torque from the hardware (e.g. Drive) and converts to joint units
    *
    * Default behaviour is to take value from the Drive object if actuated. Should be overloaded for non-drive joints
    *
    * @return double the current torque of the joint
    */
    virtual double updateTorque();

   public:
    /**
     * @brief Construct a new Joint object
     *
     * @param jointID The joint ID for this object
     * @param jointMin The minimum allowable value for this joint (below this will cause an error)
     * @param jointMax The maximum allowable value for this joint (above this will cause an error)
     */
    Joint(int jointID, double jointMin, double jointMax, const std::string& name="");

    /**
     * @brief Construct a new Joint object
     *
     * @param jointID The joint ID for this object
     * @param jointMin The minimum allowable value for this joint (below this will cause an error)
     * @param jointMax The maximum allowable value for this joint (above this will cause an error)
     * @param q0 Initial value for the position
     */
    Joint(int jointID, double jointMin, double jointMax, double q0, const std::string& name="");

    /**
     * @brief Construct a new Joint object
     *
     * @param jointID The joint ID for this object
     * @param jointMin The minimum allowable value for this joint (below this will cause an error)
     * @param jointMax The maximum allowable value for this joint (above this will cause an error)
     * @param jointDrive A pointer to the drive object (if this joint is actuated)
     */
    Joint(int jointID, double jointMin, double jointMax, Drive *jointDrive, const std::string& name="");

    /**
     * @brief Construct a new Joint object
     *
     * @param jointID The joint ID for this object
     * @param jointMin The minimum allowable value for this joint (below this will cause an error)
     * @param jointMax The maximum allowable value for this joint (above this will cause an error)
     * @param q0 Initial value for the position
     * @param jointDrive A pointer to the drive object (if this joint is actuated)
     */
    Joint(int jointID, double jointMin, double jointMax, double q0, Drive *jointDrive, const std::string& name="");
    /**
     * @brief Destroy the Joint object
     *
     */
    virtual ~Joint();

    /**
     * @brief Get the Id object
     *
     * @return int The ID used for the joint.
     */
    int getId();

    /**
     * @brief Returns the internal value of the joint (e.g. Angle, length, depending on joint type)
     *
     * NOTES:
     * - This returns only a single double value. Implementations of this joint may
     *      choose to include other methods to return other states of the joint.
     * - This does not necessarily reflect the actual value of the joint, it will only return
     *      the last value called by the updateValue() function. This allows for the update and
     *      use of the value to be updated independently (and potentially at different rates) such
     *      that the same value can be called multiple times in parallel.
     *
     * @return double The current internal representation of the value of the joint
     */
    double getPosition();
    /**
     * @brief Returns the internal value of the joint (e.g. del Angle, del length, depending on joint type)
     *
     * NOTES:
     * - This returns only a single double value. Implementations of this joint may
     *      choose to include other methods to return other states of the joint.
     * - This does not necessarily reflect the actual value of the joint, it will only return
     *      the last value called by the updateValue() function. This allows for the update and
     *      use of the value to be updated independently (and potentially at different rates) such
     *      that the same value can be called multiple times in parallel.
     *
     * @return double The current internal representation of the value of the joint
     */
    double getVelocity();
    /**
    * @brief Returns the internal value of the joint torque (e.g. del Angle, del length, depending on joint type)
    *
    * NOTES:
    * - This returns only a single double value. Implementations of this joint may
    *      choose to include other methods to return other states of the joint.
    * - This does not necessarily reflect the actual value of the joint, it will only return
    *      the last value called by the updateValue() function. This allows for the update and
    *      use of the value to be updated independently (and potentially at different rates) such
    *      that the same value can be called multiple times in parallel.
    *
    * @return double The current internal representation of the value of the joint
    */
    double getTorque();
    /**
     * @brief prints out the status of the joints current position in degrees
     *
     */
    void printStatus();
    /**
     * @brief  Updates the value of the joint. This will read the value from hardware, and
     * update the software's current representation of the value
     *
     * @return true if successful
     * @return false if unsuccessful
     */
    virtual bool updateValue();
    /**
     * @brief Pure virtual function for initialising the underlying CANopen Network
     * to send and recieve PDO messages for this joint.
     *
     * @return true if successful
     * @return false if unsuccessful
     */
    virtual bool initNetwork() = 0;

    /// Functions for Actuated joints:

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
     * \brief get the drive status word value
     *
     * \return int The current status word of the drive
     */
    int getDriveStatus() { return drive->getStatus(); }

    /**
         * \brief Start the associated drive CAN node (will start produce PDOs)
         *
         * \return Return value of drive->start();
         */
    bool start();
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
