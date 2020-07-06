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
#include <iostream>
#include <iomanip>
/** @defgroup Joint Joint Module
 *  @ingroup Robot
 *  A group of abstract joint classes, acting as the software representation of a Joint.
 */
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
     * The joint offset position in SI units.
     */
    double q0;

   public:
    /**
     * @brief Construct a new Joint object
     *
     * @param jointID The joint ID for this object
     * @param jointMin The minimum allowable value for this joint (below this will cause an error)
     * @param jointMax The maximum allowable value for this joint (above this will cause an error)
     */
    Joint(int jointID, double jointMin, double jointMax);

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
    void getStatus();
    /**
     * @brief  Updates the value of the joint. This will read the value from hardware, and
     * update the software's current representation of the value
     *
     * @return true if successful
     * @return false if unsuccessful
     */
    virtual bool updateValue() = 0;
    /**
     * @brief Pure virtual function for initialising the underlying CANopen Network
     * to send and recieve PDO messages for this joint.
     *
     * @return true if successful
     * @return false if unsuccessful
     */
    virtual bool initNetwork() = 0;
};

#endif
