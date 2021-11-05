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

#ifndef FORCEPLATE_CALIBRATESTATE_H
#define FORCEPLATE_CALIBRATESTATE_H

#include "ForcePlate.h"
//#include "ForcePlate4.h"
#include "State.h"

#define NUM_CALIBRATE_READINGS 100

class CalibrateState : public State {
    private:
        Eigen::ArrayXXi strainReadings;

        int currReading = 0;

    public :
     ForcePlate *robot;

     CalibrateState(StateMachine *m, ForcePlate *robot, const char *name = NULL);

     void entry(void);
     void during(void);
     void exit(void);

     int getCurrReading();
};
#endif