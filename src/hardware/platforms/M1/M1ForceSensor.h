/**
 * \file M1ForceSensor.h
 * \author Emek Baris Kucuktabak
 * \brief A class presenting the force sensors of the M1 Exoskeleton Robot
 * \version 0.1
 * \date 2020-07-13
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef SRC_M1FORCESENSOR_H
#define SRC_M1FORCESENSOR_H

#include "FourierForceSensor.h"


class M1ForceSensor : public FourierForceSensor {
  public:
    M1ForceSensor(int sensor_can_node_ID): FourierForceSensor(sensor_can_node_ID) {}

  private:
    double sensorValueToNewton(int sensorValue);
};


#endif //SRC_M1FORCESENSOR_H
