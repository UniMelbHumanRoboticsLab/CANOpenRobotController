
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

/**
 * \brief Joystick support class. Mostly stollen from Jason White (https://gist.github.com/jasonwhite/)
 */
class Joystick : public InputDevice
{
    public:
        Joystick();
        ~Joystick();

        void updateInput();

        bool isButtonPressed(int id) {return button[id];}
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
        const char *device;
        int js;
        struct js_event event;
        struct axis_state axes[MAX_NB_STICKS] = {0};
        size_t axis;
        bool button[100];
};

#endif // JOYSTICK_H
