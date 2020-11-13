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
#include "M1DemoMachineROS.h"

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
    M1DemoMachineROS *m1DemoMachineRos_;

public:
    void entry(void);
    void during(void);
    void exit(void);
    MultiControllerState(StateMachine *m, RobotM1 *exo, M1DemoMachineROS *m1DemoMachineRos, const char *name = NULL) :
                        State(m, name), robot_(exo), m1DemoMachineRos_(m1DemoMachineRos){};

    int controller_mode_;
private:
    // dynamic reconfigure server and callback
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig> server_;
    void dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level);
};


#endif //SRC_MULTICONTROLLERSTATE_H
