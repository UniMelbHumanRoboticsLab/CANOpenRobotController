/**
 * \file FourierForceSensor.h
 * \author Emek Baris Kucuktabak, Vincent Crocher
 * \brief A class for the Fourier force sensors (ASU) on CAN bus (but not CANOpen compatible or standard)
 * \version 0.1
 * \date 2021-02-13
 *
 * \copyright Copyright (c) 2021
 *
 */

#ifndef FOURIER_FORCE_SENSOR_H
#define FOURIER_FORCE_SENSOR_H

#include "InputDevice.h"
#include <CANopen.h>
#include <CO_command.h>
#include <sstream>
#include <numeric>

class FourierForceSensor : public InputDevice {

  public:
    /**
     * Construct a new FourierForceSensor object
     *
     */
    FourierForceSensor(int sensor_can_node_ID, double scale_factor = 1.0, double calib_time = 2.0);

    /**
     * Configure Master (controller) side PDO for force sensor reading.
     *
     */
    bool configureMasterPDOs();

    /**
    * Updates the force readings from last updated PDO and applying zeroing and scaling.
    *
    */
    void updateInput();

    /**
    * Zero the force sensor. When called, sets the sensor value to 0 by measuring over a short period of time (Blocking).
    *
    * \return bool success of calibration
    */
    bool calibrate(double calib_time = -1);

    /**
    * Return true if calibraated (zeroed).
    */
    bool isCalibrated() { return calibrated; }

    /**
    * Returns the lastest updated sensor reading in N.
    *
    */
    double getForce();

  protected:
    virtual double sensorValueToNewton(int sensorValue);

  private:
    int sensorNodeID;
    double scaleFactor;
    RPDO *rpdo;
    INTEGER32 rawData[2] = {0};
    double forceReading;              //!< Store latest updated sensor reading (in N)
    double calibrationOffset;         //!< Sensor offset from calibration/zeroing (raw value)
    bool calibrated;
    double calibrationTime;           //!< Zeroing time (in s).

    std::chrono::steady_clock::time_point time0;
};


#endif //FOURIER_FORCE_SENSOR_H
