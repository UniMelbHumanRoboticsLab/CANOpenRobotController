/**
 * \file InputDevice.h
 * \author William Campbell
 * \brief The <code>Input</code> class is a abstract class which represents an input device.
 * The Update function is called in a main program to query the devices input and
 * update any memory representation of the device implemented.
 * For example the keyboard implementation checks for key presses and fills key memory
 * with a boolean value when pressed. See <code>Keyboard</code> for further detail.
 * \version 0.1
 * \date 2020-04-09
 * \version 0.1
 * \copyright Copyright (c) 2020
 */

#ifndef InputDevice_H_INCLUDED
#define InputDevice_H_INCLUDED
#include <iostream>

#include "logging.h"

/**
 * @brief Abstract class representing any input device to be used in a Robot object
 *
 */
class InputDevice {
   private:
   public:
    InputDevice();
    virtual ~InputDevice();
    /**
 * @brief pure virtual method to be implemented by specific input devices and used by the virtual robot object to update the
 * current state of the robotic systems input device.
 *
 */
    virtual void updateInput() = 0;
};
#endif
