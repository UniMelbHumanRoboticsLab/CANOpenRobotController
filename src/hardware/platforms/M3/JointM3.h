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
    double reductionRatio=23.;
    

    double qFromDriveUnits(int driveValue) { return driveValue / 10000 / reductionRatio; };
    int qToDriveUnits(double jointValue) { return jointValue * 10000 * reductionRatio; };
    double dqFromDriveUnits(int driveValue) { return driveValue / 10000 / reductionRatio; };
    int dqToDriveUnits(double jointValue) { return jointValue * 10000 * reductionRatio; };
    double tauFromDriveUnits(int driveValue) { return driveValue / 10000 * reductionRatio; };
    int tauToDriveUnits(double jointValue) { return jointValue * 10000 / reductionRatio; };
    
    //For compatibility
    virtual int toDriveUnits(double jointValue){return qToDriveUnits(jointValue);}
    virtual double fromDriveUnits(int driveValue){return qFromDriveUnits(driveValue);}

   public:
    JointM3(int jointID, double q_min, double q_max, double dq_min=0, double dq_max=0, double tau_min=0, double tau_max=0);
    ~JointM3();
    bool updateValue();
    setMovementReturnCode_t setPosition(double qd);
    setMovementReturnCode_t setVelocity(double dqd);
    setMovementReturnCode_t setTorque(double taud);
    bool initNetwork();
    double getQ();
    double getdQ();
    double getTau();
};

#endif