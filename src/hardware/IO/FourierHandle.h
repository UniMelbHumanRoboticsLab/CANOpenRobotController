/**
 * \file FourierHandle.h
 * \author Emek Baris Kucuktabak
 * \brief A class for the Fourier button handles
 * \version 0.2
 * \date 2026-03-04
 * \copyright Copyright (c) 2020 - 2026
 *
 */

#ifndef SRC_FOURIERHANDLE_H
#define SRC_FOURIERHANDLE_H

#include <sstream>
#include <numeric>
#include <Eigen/Core>

#include "InputDevice.h"
#include "CANDevice.h"

enum ButtonColor {
    RED = 0,
    BLUE = 1,
    YELLOW = 2,
    GREEN = 3
};

/**
 * \ingroup IO
 *
 */
class FourierHandle : public InputDevice, public CANDevice {

public:
    /**
     * Construct a new FourierHandle object
     *
     */
    FourierHandle(int sensor_can_node_ID);

    /**
     * Configure Master (controller) side PDO for button value reading.
     *
     */
    bool configureMasterPDOs();

    /**
    * Updates the button readings from last updated PDO
    *
    */
    void updateInput();

    /**
    * Returns the lastest updated button reading.
    *
    */
    Eigen::VectorXd& getButtonValues();

private:
    int sensorNodeID_;
    RPDO *rpdo_;
    INTEGER32 rawData_[4] = {0};
    Eigen::VectorXd buttonValues_; // Values of red, blue, yellow, and green, respectively. if pressed 1, else 0

};


#endif //SRC_FOURIERHANDLE_H
