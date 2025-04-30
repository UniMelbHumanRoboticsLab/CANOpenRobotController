/**
 * \file JointFITHVExo.h
 * \author Vincent Crocher
 * \brief FITHV exoskeleton actuated joint
 * \version 0.1
 * \date 2025-04-15
 *
 * \copyright Copyright (c) 2025
 *
 */
#ifndef JOINTFITHV_H
#define JOINTFITHV_H

#include <iostream>
#include <cmath>

#include "Joint.h"
#include "CopleyDrive.h"

/**
 * \brief FITHV exoskeleton actuated joints, using Copley drives.
 *
 */
class JointFITHVExo : public Joint {
   private:
    const short int sign;
    const double qMin, qMax, dqMin, dqMax, tauMin, tauMax;
    int encoderCounts = 10240;  //Encoder counts per turn (approximated based on pos and torque measure)
    double reductionRatio = 35.0; //Mechanical reduction (approximated based on pos and torque measure)
    double ratedTorque = 0.570; //As read from drive (0x3A02 in 0x6076). 0.570 in litt-endian (it would be 14.850 (unlikely) in big-endian)

    double positionRatio = (2. * M_PI) / (double)encoderCounts / reductionRatio; // Pos in drive is in encoder count
    double driveUnitToJointPosition(int driveValue) { return sign * (double) driveValue * positionRatio; };
    int jointPositionToDriveUnit(double jointValue) { return sign * jointValue / positionRatio; };

    double velocityRatio = (2. * M_PI) / 10. / (double)encoderCounts / reductionRatio;  // Vel in drive in 0.1count/s
    double driveUnitToJointVelocity(int driveValue) { return sign * (double) driveValue * velocityRatio; };
    int jointVelocityToDriveUnit(double jointValue) { return sign * jointValue / velocityRatio; };

    double torqueRatio = ratedTorque / 1000. * reductionRatio;  // Torque in rated torque / 1000 in drive. Rating torque of 0.570 as read from drive (3A 02 00 00 in 0x6076)
    double driveUnitToJointTorque(int driveValue) { return sign * driveValue * torqueRatio; }; //
    int jointTorqueToDriveUnit(double jointValue) { return sign * jointValue / torqueRatio; };

   public:
    JointFITHVExo(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0, CopleyDrive *drive = NULL, const std::string& name="");
    ~JointFITHVExo();
    /**
     * \brief Check if current velocity and torque are within limits.
     *
     * \return OUTSIDE_LIMITS if outside the limits (!), SUCCESS otherwise
     */
    setMovementReturnCode_t safetyCheck();

    setMovementReturnCode_t setPosition(double qd);
    setMovementReturnCode_t setVelocity(double dqd);
    setMovementReturnCode_t setTorque(double taud);

    bool initNetwork();
};

#endif
