/**
 * \file JointM1.h
 * \author Tim Haswell borrowing heavily from Vincent Crocher
 * \brief An M1 actuated joint
 * \version 0.1
 * \date 2020-07-08
 *
 * \copyright Copyright (c) 2020
 *
 */
#ifndef JointM1_H_INCLUDED
#define JointM1_H_INCLUDED

#include <cmath>

#include "ActuatedJoint.h"
#include "KincoDrive.h"

/**
 * \brief M1 actuated joint, using Kinco drive.
 *
 */
class JointM1 : public ActuatedJoint {
   private:
    double qMin, qMax, dqMin, dqMax, tauMin, tauMax;
    double d2j_Pos, d2j_Vel, d2j_Trq, j2d_Pos, j2d_Vel, j2d_Trq;
    short int sign;
    int encoderCounts;       //Encoder counts per turn
    double reductionRatio;   // Reduction ratio due to gear head

    double Ipeak;                //Kinco FD123 peak current
    double motorTorqueConstant;  //SMC60S-0020 motor torque constant

    double driveUnitToJointPosition(int driveValue);
    int jointPositionToDriveUnit(double jointValue);
    double driveUnitToJointVelocity(int driveValue);
    int jointVelocityToDriveUnit(double jointValue);
    double driveUnitToJointTorque(int driveValue);
    int jointTorqueToDriveUnit(double jointValue);

   public:
    JointM1(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0);
    ~JointM1();

    bool updateValue();

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

    /**
     * \brief Get error message
     *
     * \return true if succesful
     * \return false if drive is currently not in the correct state to enable
     */
    void errorMessage(setMovementReturnCode_t errorCode);
};

#endif
