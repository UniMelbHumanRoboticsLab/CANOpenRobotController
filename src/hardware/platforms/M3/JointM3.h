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
    double q, dq, tau, q0;
    double qMin, qMax, dqMin, dqMax, tauMin, tauMax;
    int encoderCounts = 10000;  //Encoder counts per turn
    double reductionRatio = 22.;

    double driveUnitToJointPosition(int driveValue) { return driveValue * 2. * M_PI / (double)encoderCounts / reductionRatio; };
    int jointPositionToDriveUnit(double jointValue) { return jointValue / 2. * M_PI * (double)encoderCounts * reductionRatio; };
    double driveUnitToJointVelocity(int driveValue) { return driveValue * 2. * M_PI / (double)encoderCounts / reductionRatio; };
    int jointVelocityToDriveUnit(double jointValue) { return jointValue / 2. * M_PI * (double)encoderCounts * reductionRatio; };
    double driveUnitToJointTorque(int driveValue) { return driveValue * 1000. / 24.13 / 32. * reductionRatio; };
    int jointTorqueToDriveUnit(double jointValue) { return jointValue / 1000. * 24.13 * 32. / reductionRatio; };

   public:
    JointM3(int jointID, double q_min, double q_max, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0);
    ~JointM3();
    bool updateValue();
    setMovementReturnCode_t setPosition(double qd);
    setMovementReturnCode_t setVelocity(double dqd);
    setMovementReturnCode_t setTorque(double taud);

    /**
     * \brief Set current position as joint position offset (q0)
     * such that current position is now qcalib
     *
     */
    void setCurrentOffset(double qcalib);
    bool initNetwork();
    double getQ();
    double getDq();
    double getTau();
};

#endif
