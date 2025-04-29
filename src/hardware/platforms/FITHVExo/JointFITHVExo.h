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
    int encoderCounts = 1024;  //Encoder counts per turn
    double reductionRatio = 350.0;
    double torqueRatio = 1.0;         //

    double driveUnitToJointPosition(int driveValue) { return sign * (double) driveValue * (2. * M_PI) / (double)encoderCounts / reductionRatio; }; // Pos in drive in encoder count
    int jointPositionToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * (double)encoderCounts * reductionRatio; };
    double driveUnitToJointVelocity(int driveValue) { return sign * (double) driveValue * 10 * (2. * M_PI) / (double)encoderCounts / reductionRatio; }; // Vel in drive in 0.1count/s
    int jointVelocityToDriveUnit(double jointValue) { return sign * jointValue / 10. / (2. * M_PI) * (double)encoderCounts * reductionRatio; };
    double driveUnitToJointTorque(int driveValue) { return sign * driveValue / torqueRatio * reductionRatio; };
    int jointTorqueToDriveUnit(double jointValue) { return sign * jointValue * torqueRatio / reductionRatio; };

    /**
     * \brief motor drive position control profile paramaters, user defined.
     *
     */
    motorProfile posControlMotorProfile{4000000, 240000, 240000};

   public:
    JointFITHVExo(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0, CopleyDrive *drive = NULL, const std::string& name="");
    ~JointFITHVExo();
    /**
     * \brief Cehck if current velocity and torque are within limits.
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
