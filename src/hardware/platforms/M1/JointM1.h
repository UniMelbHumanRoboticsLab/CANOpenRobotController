/**
 * \file JointM1.h
 * \author Yue Wen, Tim Haswell borrowing heavily from Vincent Crocher
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

#include "Joint.h"
#include "KincoDrive.h"

/**
 * \brief M1 actuated joint, using Kinco drive.
 *
 */
class JointM1 : public Joint {
   private:
    short int sign;
    double qMin, qMax, dqMin, dqMax, tauMin, tauMax;
    double d2j_Pos, d2j_Vel, d2j_Trq, j2d_Pos, j2d_Vel, j2d_Trq;
    double d2r, r2d;
    int encoderCounts;       //Encoder counts per turn
    double reductionRatio;   // Reduction ratio due to gear head

    double Ipeak;                //Kinco FD123 peak current
    double motorTorqueConstant;  //SMC60S-0020 motor torque constant

    /**
     * \brief Conversion between drive unit (encoder count) and joint unit (radian).
     *
     * \return drive unit for low-level control purpose
     * \return joint unit for high-level control purpose
     */
    double driveUnitToJointPosition(int driveValue);
    int jointPositionToDriveUnit(double jointValue);
    double driveUnitToJointVelocity(int driveValue);
    int jointVelocityToDriveUnit(double jointValue);
    double driveUnitToJointTorque(int driveValue);
    int jointTorqueToDriveUnit(double jointValue);
    motorProfile posControlMotorProfile{4000000, 240000, 240000};

   public:
    JointM1(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0, KincoDrive *drive = NULL, const std::string& name="");
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

    /**
           * Writes the desired digital out value to the drive
           *
           * \return true if successful
           * \return false if not
           */
    bool setDigitalOut(int digital_out);

    /**
           * Returns the value of digital IN
           * \return Digital in state from the motor drive
           */
    virtual int getDigitalIn();
};

#endif
