/**
 * /file X2DemoState.h
 * /author Emek Baris Kucuktabak
 * /brief Concrete implementation of DemoState
 * /version 0.1
 * /date 2020-07-06
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef SRC_X2DEMOSTATE_H
#define SRC_X2DEMOSTATE_H

#include "State.h"
#include "X2Robot.h"
#include <ctime>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <fstream>

#include "signal_logger/signal_logger.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"

/**
 * \brief Demo State for the X2DemoMachine
 *
 *
 */
class X2DemoState : public State {
    X2Robot *robot;

public:
    void entry(void);
    void during(void);
    void exit(void);
    X2DemoState(StateMachine *m, X2Robot *exo, const char *name = NULL) : State(m, name), robot(exo){};

private:
    std::chrono::steady_clock::time_point time0;
    std::ofstream logJoint, logTime;
    void log();
    void initializeLogger(int bufferSize);
    void updateLogElements();

    int controller_type = 4;

    bool logSaved = false;

    double inputHistory_[2] = {0,0};
    double outputHistory_[2] = {0,0};
    double t_step = 0.0025;

    double m = 5;
    double b = 2;

    Eigen::VectorXd jointPositions_;
    Eigen::VectorXd jointVelocities_;
    Eigen::VectorXd jointTorques_;
    Eigen::VectorXd desiredJointVelocities_;
    Eigen::VectorXd desiredJointTorques_;
    Eigen::VectorXd interactionForces_;
    double time;

    bool move = false;
    double timeStop;

    double static_fric;
    double dynamic_const_fric;
    double vel_theresh;
    double J;
    double virtMassRatio;
    double feedForwardTorque;
    double feedBackTorque;
    double M;
    double c0;
    double c1;

};

#endif