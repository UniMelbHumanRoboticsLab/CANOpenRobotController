/**
 * \file DummyActJoint.h
 * \author Justin Fong
 * \brief A dummy class to test whether the actuated joint inheritence stuff works
 * \version 0.1
 * \date 2020-04-09
 * 
 * \copyright Copyright (c) 2020
 * 
 */
#ifndef DUMMYACTJOINT_H_INCLUDED
#define DUMMYACTJOINT_H_INCLUDED

#include "ActuatedJoint.h"

/**
 * \brief Example implementation of the ActuatedJoints class. 
 * 
 * Important to note the simple implementation between the driveValue and jointValue
 * 
 */
class DummyActJoint : public ActuatedJoint {
private:
    double lastQCommand = 0;

    /**
         * \brief Converts from the joint position[rad] to the equivalent value for the drive [encoder count]
         * \param jointPosition joint position[rad]
         * \return int The equivalent drive value for the given joint position
         */
    int jointPositionToDriveUnit(double jointPosition) {return jointPosition * 159715.1684916456; };

    /**
         * \brief Converts from the drive value[encoder count] to the equivalent value for the joint position[rad]
         * \param driveValue The drive value to be converted [encoder count]
         * \return The equivalent joint position for the given drive value [rad]
         */
    double driveUnitToJointPosition(int driveValue) {return driveValue / 159715.1684916456; };
    /**
         * \brief Converts from the joint velocity[rad/s] to the equivalent value for the drive [encoder count/0.1sec]
         * \param jointVelocity joint velocity[rad/s]
         * \return int The equivalent drive value for the given joint position [encoder count/0.1sec]
         */
    int jointVelocityToDriveUnit(double jointVelocity) {return jointVelocity * 1597151.684916456; };

    /**
         * \brief Converts from the drive value[encoder count/0.1sec] to the equivalent value for the joint velocity[rad/s]
         * \param driveValue The drive value to be converted [encoder count/0.1sec]
         * \return The equivalent joint velocity for the given drive value [rad/s]
         */
    double driveUnitToJointVelocity(int driveValue) {return driveValue / 1597151.684916456; };
    /**
         * \brief Converts from the joint torque[Nm] to the equivalent value for the drive [rated torque in Nmm]
         * \param jointTorque joint velocity[Nm]
         * \return int The equivalent drive value for the given joint position [rated torque in Nmm]
         */
    int jointTorqueToDriveUnit(double jointTorque) {return jointTorque / 0.0390775; };

    /**
         * \brief Converts from the drive value[rated torque in Nmm] to the equivalent value for the joint velocity[Nm]
         * \param driveValue The drive value to be converted [rated torque in Nmm]
         * \return The equivalent joint velocity for the given drive value [Nm]
         */
    double driveUnitToJointTorque(int driveValue) {return driveValue * 0.0390775; };


public:
    DummyActJoint(int jointID, double jointMin, double jointMax, Drive *drive);
    bool updateValue();
    setMovementReturnCode_t setPosition(double desQ);
    bool initNetwork();
    double getQ();
};

#endif