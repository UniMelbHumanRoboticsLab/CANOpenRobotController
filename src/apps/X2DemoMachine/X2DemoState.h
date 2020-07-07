/**
 * /file X2DemoState.h
 * /author Emek Baris Kucuktabak
 * /brief Concrete implementation of IdleState
 * /version 0.1
 * /date 2020-07-06
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef SRC_X2DEMOSTATE_H
#define SRC_X2DEMOSTATE_H

#include "State.h"
#include "ExoRobot.h"
#include <ctime>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <math.h>

/**
 * \brief Demo State for the X2DemoMachine
 *
 *
 */
class X2DemoState : public State {
    ExoRobot *robot;

public:
    void entry(void);
    void during(void);
    void exit(void);
    X2DemoState(StateMachine *m, ExoRobot *exo, const char *name = NULL) : State(m, name), robot(exo){};

private:
    std::chrono::steady_clock::time_point time0;
};

#endif