/**
 * /file X2DemoState.h
 * /author Emek Baris Kucuktabak
 * /brief Concrete implementation of DemoState
 * /version 1.1
 * /date 2022-02-22
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

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <CORC/dynamic_paramsConfig.h>

/**
 * \brief Demo State for the X2DemoMachine
 *
 *
 */
class X2DemoState : public State {

protected:
    X2Robot *robot_;

public:
    void entry(void);
    void during(void);
    void exit(void);
    X2DemoState(X2Robot *exo, const char *name = "X2DemoState");

    Eigen::VectorXd& getDesiredJointTorques();
    Eigen::VectorXd& getDesiredJointVelocities();

    int controller_mode_;

    Eigen::VectorXd enableJoints;


private:
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig> server_;
    void dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level);

    double t_step_ = 0.003; // 0.003 todo: get from main

    std::chrono::steady_clock::time_point time0;
    Eigen::VectorXd desiredJointTorques_;
    Eigen::VectorXd desiredJointVelocities_;

    Eigen::VectorXd kTransperancy_;
    double amplitude_, period_, offset_;

};

#endif