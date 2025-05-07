/**
 * \file FITAbsEncoder.h
 * \author Vincent Crocher
 * \brief Absolute encoder of the FIT-HV exoskeleton (SDO only)
 * \date 2025-05-02
 *
 * \copyright Copyright (c) 2025
 *
 */

#ifndef FITABSENCODER_H
#define FITABSENCODER_H

#include "CANDevice.h"

class FITAbsEncoder: public CANDevice {

  public:
    /**
    * Construct a new FITAbsEncoder object
    *
    */
    FITAbsEncoder(int sensor_can_node_ID, double pulse_to_rad);

    /**
    * Send SDO request and process return value.
    * \return Value of absolute encoder in [rad]
    */
    double readValue();

  private:

    /**
    * \brief Quick debugging conveninece method to read a value via SDO
    *
    * \return false if unsuccesfull
    */
    bool SDORead(std::string & ret);

    /**
    * \brief Send a list (vector) of properly formatted SDO Messages
    *
    * \return string response
    */
    int sendSDOMessages(std::vector<std::string> messages, std::string & ret);

    double pulseToRadFactor;
};


#endif
