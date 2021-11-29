
/**
 *
 * \file Joystick.h
 * \author Vincent Crocher (Mostly stollen from Jason White (https://gist.github.com/jasonwhite/))
 * \version 0.1
 * \date 2020-07-02
 * \copyright Copyright (c) 2020
 *
 * \brief  Joystick as Input device.
 *
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>


#include "InputDevice.h"


/**
 * Current state of an axis.
 */
struct axis_state {
    short x, y;
};

#define MAX_NB_STICKS 5
#define STICK_MAX_VALUE 65535.0
#define MAX_NB_BUTTONS 20

/**
 * \brief Joystick support class. Mostly stollen from Jason White (https://gist.github.com/jasonwhite/)
 */
class Joystick : public InputDevice
{
    public:
        /**
         * \brief Standard joystick on /dev/input/js#id#
         * \param id: the joystick id (#id#), default is 0
         *
         */
        Joystick(int id=0);
        ~Joystick();

        void updateInput();

        bool isButtonPressed(int id) {return button[id];} //!< True if button currently pressed (at last call of updateInput())
        int isButtonTransition(int id) {return button_transition[id];} //!< Return +1 if pressing transition detected, -1 if releasing one, 0 otherwise. Reseted every call of updateInput() ( every loop )
        double getAxis(int axis_id) {
            int stick = axis_id / 2;
            if (axis_id % 2 == 0)
                return axes[stick].x/STICK_MAX_VALUE;
            else
                return axes[stick].y/STICK_MAX_VALUE;
        }

        /**
         * \brief Does nothing as there are none here
         *
         */
        bool configureMasterPDOs(){return true;};

       protected:
       private:
        bool initialised;
        char device[50];
        int js;
        struct js_event event;
        struct axis_state axes[MAX_NB_STICKS] = {0};
        size_t axis;
        bool button[MAX_NB_BUTTONS] = {false};
        int8_t button_transition[MAX_NB_BUTTONS] = {0};
};

#endif // JOYSTICK_H
