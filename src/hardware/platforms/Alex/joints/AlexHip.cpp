/**
 * @file AlexHip.cpp
 * @author William Campbell
 * @brief 
 * @version 0.1
 * @date 2020-06-09
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "AlexHip.h"

#include <iostream>

#include "DebugMacro.h"

AlexHip::AlexHip(int jointID, double jointMin, double jointMax, Drive *drive, JointKnownPos jointParams) : AlexJoint(jointID, jointMin, jointMax, drive, jointParams) {
    spdlog::debug("ALEX HIP JOINT: " << this->id)
    // Do nothing else
}
