/**
 * \file AlexJoint.h
 * \author William Campbell 
 * \version 0.1
 * \date 2020-06-10
 * \brief A dummy class to test whether the actuated joint inheritence stuff works
 * \copyright Copyright (c) 2020
 * 
 */
#ifndef AlexJoint_H_INCLUDED
#define AlexJoint_H_INCLUDED

#include "CopleyDrive.h"
#include "Joint.h"
#include "RobotParams.h"

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
 * Paramater definitions: Hip motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
static JointDrivePairs ALEXhipJDP{
    250880,       // drivePosA
    0,            // drivePosB
    deg2rad(90),  //jointPosA
    deg2rad(0)    //jointPosB
};
/**
 * Paramater definitions: Knee motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
static JointDrivePairs ALEXkneeJDP{
    250880,       // drivePosA
    0,            //drivePosB
    deg2rad(90),  //jointPosA
    deg2rad(0)    //jointPosB
};


/**
 * \brief implementation of the ActuatedJoints class for the Alex Exoskeleton. 
 * 
 * 
 * 
 */
class AlexJoint : public Joint {
   private:
    double lastQCommand = 0;
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
    AlexJoint(int jointID, double jointMin, double jointMax, JointDrivePairs jdp, Drive *drive);




    bool updateValue();
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