/**
 * /file MultiControllerState.h
 * \author Emek Baris Kucuktabak
 * \version 0.1
 * \date 2020-11-09
 * \copyright Copyright (c) 2020
 *
 *
 */

#ifndef SRC_MULTICONTROLLERSTATE_H
#define SRC_MULTICONTROLLERSTATE_H

#include "State.h"
#include "RobotM1.h"
#include "MultiM1MachineROS.h"

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <CORC/dynamic_paramsConfig.h>

/**
 * \brief A multi purpose state with different controllers implemented
 *
 *
 */
class MultiControllerState : public State {
    RobotM1 *robot_;
    MultiM1MachineROS *multiM1MachineRos_;

public:
    void entry(void);
    void during(void);
    void exit(void);
    MultiControllerState(StateMachine *m, RobotM1 *exo, MultiM1MachineROS *multiM1MachineRos, const char *name = NULL) :
                        State(m, name), robot_(exo), multiM1MachineRos_(multiM1MachineRos){};

    int controller_mode_;

    // FOR TRANSPERANCY EXPERIMENTS
    double kp_;
    double kd_;
    double ffRatio_;
    double torque_error_last_time_step = 0;
    double error;
    double delta_error;
    Eigen::VectorXd q;     //positive dorsi flexion
    Eigen::VectorXd dq;
    Eigen::VectorXd tau;
    Eigen::VectorXd tau_s;
    Eigen::VectorXd tau_cmd;


private:
    // dynamic reconfigure server and callback
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig> server_;
    void dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level);
};


#endif //SRC_MULTICONTROLLERSTATE_H
