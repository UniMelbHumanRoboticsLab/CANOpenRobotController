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

#include "X2DemoMachineNode.h"

// dynamic reconfigure
//#include <dynamic_reconfigure/server.h>
//#include <CORC/dynamic_paramsConfig.h>

/**
 * \brief Demo State for the X2DemoMachineROS2
 *
 *
 */
class X2DemoState : public State {
    X2Robot *robot_;
    X2DemoMachineROS *x2DemoMachineRos_;

public:
    void entry(void);
    void during(void);
    void exit(void);
    X2DemoState(StateMachine *m, X2Robot *exo, X2DemoMachineROS *x2DemoMachineRos, const char *name = NULL);

    Eigen::VectorXd& getDesiredJointTorques();
    int controller_mode_;
    double virtualMassRatio_;
    double desiredInteractionForce_;
    double desiredJointAcceleration_;
private:
    std::chrono::steady_clock::time_point time0;

    //dynamic_reconfigure::Server<CORC::dynamic_paramsConfig> server_;
    //void dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level);

    Eigen::VectorXd desiredJointVelocities_;
    Eigen::VectorXd desiredJointTorques_;

    double admittanceInputHistory_[2] = {0,0};
    double admittanceOutputHistory_[2] = {0,0};
    double t_step_ = 0.002; // todo: get from main

    double mAdmittance_ = 5;
    double bAdmittance_ = 2;


};

#endif