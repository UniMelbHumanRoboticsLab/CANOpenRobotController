/**
 * /file IdleState.h
 * /author Justin Fong
 * /brief Calibration - takes sensors and offsets sensors by NUM_CALIBRATE_READINGS average
 * /version 0.1
 * /date 2021-1-21
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef LR_CALIBRATESTATE_H
#define LR_CALIBRATESTATE_H

#include "LoggingRobot.h"
#include "State.h"

#define NUM_CALIBRATE_READINGS 1000

class CalibrateState : public State {
    private:
        Eigen::ArrayXXd readings;
        int currReading = 0;

    public :
        LoggingRobot * robot;

        CalibrateState(StateMachine *m, LoggingRobot *robot, const char *name = NULL);

        void entry(void);
        void during(void);
        void exit(void);

        int getCurrReading();
};
#endif