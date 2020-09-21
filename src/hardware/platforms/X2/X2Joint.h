/**
 * \file X2Joint.h
 * \author Justin Fong
 * \brief A class presenting the hip or knee joints of the X2 Exoskeleton Robot
 * \version 0.1
 * \date 2020-04-09
 * 
 * \copyright Copyright (c) 2020
 * 
 */
#ifndef X2JOINT_H_INCLUDED
#define X2JOINT_H_INCLUDED

#include "CopleyDrive.h"
#include "Joint.h"

#define MOTOR_RATED_TORQUE 0.319
#define REDUCTION_RATIO 122.5

/**
 * \brief Structure which is used for joint conversions. Defines two Drive Position/Joint Position Pairs, 
 * which are then used to construct a linear relationship between Drive Position and Joint Position.
 * 
 */
typedef struct JointDrivePairs {
    int drivePosA;
    int drivePosB;
    double jointPosA;
    double jointPosB;
} JointDrivePairs;

/**
 * \brief Example implementation of the ActuatedJoints class. 
 * 
 * Important to note the simple implementation between the driveValue and jointValue
 * 
 */
class X2Joint : public Joint {
   private:
    double JDSlope;
    double JDIntercept;

    /**
         * \brief Converts from the joint position[rad] to the equivalent value for the drive [encoder count]
         * \param jointPosition joint position[rad]
         * \return int The equivalent drive value for the given joint position
         */
    int jointPositionToDriveUnit(double jointPosition);

    /**
         * \brief Converts from the drive value[encoder count] to the equivalent value for the joint position[rad]
         * \param driveValue The drive value to be converted [encoder count]
         * \return The equivalent joint position for the given drive value [rad]
         */
    double driveUnitToJointPosition(int driveValue);
    /**
         * \brief Converts from the joint velocity[rad/s] to the equivalent value for the drive [encoder count/0.1sec]
         * \param jointVelocity joint velocity[rad/s]
         * \return int The equivalent drive value for the given joint position [encoder count/0.1sec]
         */
    int jointVelocityToDriveUnit(double jointVelocity);

    /**
         * \brief Converts from the drive value[encoder count/0.1sec] to the equivalent value for the joint velocity[rad/s]
         * \param driveValue The drive value to be converted [encoder count/0.1sec]
         * \return The equivalent joint velocity for the given drive value [rad/s]
         */
    double driveUnitToJointVelocity(int driveValue);
    /**
         * \brief Converts from the joint torque[Nm] to the equivalent value for the drive [1000ths of rated torque]
         * \param jointTorque joint velocity[Nm]
         * \return int The equivalent drive value for the given joint position [rated torque in Nmm]
         */
    int jointTorqueToDriveUnit(double jointTorque);

    /**
         * \brief Converts from the drive value[1000ths of rated torque] to the equivalent value for the joint velocity[Nm]
         * \param driveValue The drive value to be converted [1000ths of rated torque]
         * \return The equivalent joint velocity for the given drive value [Nm]
         */
    double driveUnitToJointTorque(int driveValue);

   public:
    X2Joint(int jointID, double jointMin, double jointMax, JointDrivePairs jdp, Drive *drive);
    ~X2Joint(){};

    bool initNetwork();
    double getPosition();
    double getVelocity();
    double getTorque();

    /**
          * \brief Set the current position as offset
          *
          * /param offset, joint position value to be at the homing position [rad]
          *
         * \return true if successful
         * \return false if not
         */
    void setPositionOffset(double offset);
};

#endif