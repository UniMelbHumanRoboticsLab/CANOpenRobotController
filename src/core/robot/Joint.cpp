/**
 * The <code>Joint</code> class is a abstract class which represents a joint in a
 * <code>Robot</code> objec. This class can be used to represent all types of joints,
 * including actuated, non-actuated, revolute, prismatic, etc.
 *
 *
 * Version 0.1
 * Date: 07/04/2020
 *
 */
#include "Joint.h"

#include <iostream>

#include "DebugMacro.h"

Joint::Joint(int jointID, double jointMin, double jointMax) : id(jointID), qMin(jointMin), qMax(jointMax) {
    position = 0;
    velocity = 0;
    torque = 0;
}

Joint::Joint(int jointID, double jointMin, double jointMax, double q0) : id(jointID), qMin(jointMin), qMax(jointMax) {
    position = q0;
    velocity = 0;
    torque = 0;
}

Joint::~Joint() {
    DEBUG_OUT(" Joint object deleted")
}
int Joint::getId() {
    return id;
}

double Joint::getPosition() {
    return position;
}
double Joint::getVelocity() {
    return velocity;
}
double Joint::getTorque() {
    return torque;
}

void Joint::getStatus() {
    std::cout << "Joint ID " << id << " @ pos " << getPosition() << " deg" << std::endl;
}

