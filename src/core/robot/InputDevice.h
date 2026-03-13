/**
 * \file InputDevice.h
 * \author William Campbell, Justin Fong, Vincent Crocher
 * \brief The Input class is a abstract class which represents an input device.
 * The Update function is called in a main program to query the devices input and
 * update any memory representation of the device implemented.
 * \version 0.2
 * \date 2026-03-04
 * \copyright Copyright (c) 2020-2026
 */

#ifndef InputDevice_H
#define InputDevice_H
#include "logging.h"

/**
 *  @defgroup IO IO module
 *  A group of all IO devices related class.
 */
/**
 * @ingroup Robot
 * @brief Abstract class representing any Input device (sensor, joystick..) to be used in a Robot object
 * In order to specifically create an CAN based input device, inherit your class from both CANDevice and InputDevice.
 *
 */
class InputDevice {
   public:
        InputDevice() {};
        virtual ~InputDevice() {};
        /**
         * @brief pure virtual method to be implemented by specific input devices and used by the virtual robot object to update the
         * current state of the robotic systems input device.
         *
         */
        virtual void updateInput() = 0;
};
#endif
