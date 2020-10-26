#include "Joystick.h"

Joystick::Joystick() : initialised(false) {
    device = "/dev/input/js0";
    js = open(device, O_RDONLY | O_NONBLOCK);
    if (js == -1) {
        spdlog::info("Could not open joystick ({})", device);
    }
    else {
        initialised=true;
    }
}

Joystick::~Joystick() {
    if(initialised) {
        close(js);
    }
}


/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event) {
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd) {
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t get_button_count(int fd) {
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}


/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[MAX_NB_STICKS]) {
    size_t axis = event->number / 2;

    if (axis < MAX_NB_STICKS)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}



void Joystick::updateInput() {
    if(initialised) {
        read_event(js, &event);
        switch (event.type)
        {
            case JS_EVENT_BUTTON:
                button[event.number] = event.value; //true for pressed, false for released
                break;
            case JS_EVENT_AXIS:
                axis = get_axis_state(&event, axes);
                break;
            default:
                /* Ignore init events. */
                break;
        }
    }
}
