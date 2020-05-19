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
     * \brief  These functions are defined here for example
    *   In this implementation they do essentially nothing 
    * - it's a straight 1:1 relation between drive and motor units
    */
    double fromDriveUnits(int driveValue) { return driveValue / 10000; };
    int toDriveUnits(double jointValue) { return jointValue * 10000; };

   public:
    DummyActJoint(int jointID, double jointMin, double jointMax, Drive *drive);
    bool updateValue();
    setMovementReturnCode_t setPosition(double desQ);
    bool initNetwork();
    double getQ();
};

#endif