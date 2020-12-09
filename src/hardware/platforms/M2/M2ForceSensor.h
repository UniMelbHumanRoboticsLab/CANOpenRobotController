/**
 * \file M2ForceSensor.h
 * \author Emek Baris Kucuktabak
 * \brief A class presenting the force sensors of the M2 Robot
 * \version 0.1
 * \date 2020-07-13
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef SRC_M2FORCESENSOR_H
#define SRC_M2FORCESENSOR_H

#include "InputDevice.h"
#include <CANopen.h>
#include <CO_command.h>
#include <sstream>


class M2ForceSensor : public InputDevice {
public:

    /**
     * Construct a new M2ForceSensor object
     *
     */
    M2ForceSensor(int sensorID);

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

private:
    int sensorID;
    double sensorValueToNewton(int sensorValue);
    double forceReading;
    bool calibrated;

};


#endif //SRC_M2FORCESENSOR_H
