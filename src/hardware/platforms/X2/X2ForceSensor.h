/**
 * \file X2ForceSensor.h
 * \author Emek Baris Kucuktabak
 * \brief A class presenting the force sensors of the X2 Exoskeleton Robot
 * \version 0.1
 * \date 2020-07-13
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef SRC_X2FORCESENSOR_H
#define SRC_X2FORCESENSOR_H

#include "InputDevice.h"
#include <CANopen.h>
#include <CO_command.h>
#include <sstream>
#include <numeric>

class X2ForceSensor : public InputDevice {
public:

    /**
     * Construct a new X2ForceSensor object
     *
     */
    X2ForceSensor(int sensorID, double scaleFactor);

    /**
    * updates the force readings
    *
    */
    void updateInput();

    /**
    * calibrate the force sensor. When called, sets the sensor value to 0.
    *
    * \return bool success of calibration
    */
    bool calibrate();

    /**
    * returns the force reading of the sensor with sensorID
    *
    */
    double getForce();

    bool configureMasterPDOs();

private : int sensorID;
    INTEGER32 rawData[2] = {0};
    double sensorValueToNewton(int sensorValue, double scaleFactor);
    double forceReading_;
    double calibrationOffset_;
    double scaleFactor_;
    std::chrono::steady_clock::time_point time0;

    RPDO *rpdo; 

};


#endif //SRC_X2FORCESENSOR_H
