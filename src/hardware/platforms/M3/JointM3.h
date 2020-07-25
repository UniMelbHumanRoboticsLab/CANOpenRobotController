/**
 * \file JointM3.h
 * \author Vincent Crocher
 * \brief An M3 actuated joint
 * \version 0.1
 * \date 2020-06-16
 *
 * \copyright Copyright (c) 2020
 *
 */
#ifndef JOINTM3_H_INCLUDED
#define JOINTM3_H_INCLUDED

#include <cmath>

#include "ActuatedJoint.h"
#include "KincoDrive.h"

/**
 * \brief M3 actuated joints, using Kinoc drives.
 *
 */
class JointM3 : public ActuatedJoint {
   private:
    double q, dq, tau;
    double qMin, qMax, dqMin, dqMax, tauMin, tauMax;
    short int sign;
    int encoderCounts = 10000;  //Encoder counts per turn
    double reductionRatio = 22.;

    double Ipeak = 45.0;                 //Kinco FD123 peak current
    double motorTorqueConstant = 0.132;  //SMC60S-0020 motor torque constant

    double driveUnitToJointPosition(int driveValue) { return sign * driveValue * (2. * M_PI) / (double)encoderCounts / reductionRatio; };
    int jointPositionToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * (double)encoderCounts * reductionRatio; };
    double driveUnitToJointVelocity(int driveValue) { return sign * driveValue * (2. * M_PI) / 60. / 512. / (double)encoderCounts * 1875 / reductionRatio; };
    int jointVelocityToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * 60. * 512. * (double)encoderCounts / 1875 * reductionRatio; };
    double driveUnitToJointTorque(int driveValue) { return sign * driveValue / Ipeak / 1.414 * motorTorqueConstant * reductionRatio; };
    int jointTorqueToDriveUnit(double jointValue) { return sign * jointValue * Ipeak * 1.414 / motorTorqueConstant / reductionRatio; };

   public:
    JointM3(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0);
    ~JointM3();

    bool updateValue();

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
