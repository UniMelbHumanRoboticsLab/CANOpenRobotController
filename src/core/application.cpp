 /**
 * \file application.c
 * \author William Campbell, Justin Fong, Vincent Crocher
 * \version 0.3
 * \date 2021-12-18
 * \copyright Copyright (c) 2020 - 2021
 *
 * \brief  Application interface of CORC. Based on CANopenSocket.
 *
 */
#include "application.h"

//Select state machine to use for this application (can be set in cmake)
#ifndef STATE_MACHINE_TYPE
#error "State Machine Type not defined"
#endif

std::unique_ptr<STATE_MACHINE_TYPE> stateMachine;

/******************** RUNS BEFORE CO_init() ********************/
void app_communicationReset(int argc, char *argv[]) {
#if ROS > 0
    stateMachine = std::make_unique<STATE_MACHINE_TYPE>(argc, argv);
#else
    stateMachine = std::make_unique<STATE_MACHINE_TYPE>();
#endif
    stateMachine->configureMasterPDOs();
}

/******************** Runs at the Start of rt_control_thread********************/
void app_programStart(void) {
    spdlog::info("CORC Start application");

#ifdef NOROBOT
    spdlog::info("Running in NOROBOT (virtual) mode.");
#endif  // NOROBOT
    spdlog::info("Application thread running at {}Hz.", (int)(1000./(float)controlLoopPeriodInms));
    stateMachine->init();
    stateMachine->activate();
}

/******************** Runs in low priority thread ********************/
void app_programAsync(uint16_t timer1msDiffy) {
}

/******************** Runs in rt_control_thread ********************/
void app_programControlLoop(void) {
    std::chrono::steady_clock::time_point _t0 = std::chrono::steady_clock::now();

    //StateMachine execution
    if (stateMachine->running()) {
        stateMachine->update();
    }

    //Warn if time overflow (this is the effective used time, normally lower than the allocated time period)
    double dt = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - _t0).count()) / 1e6;
    if(dt>controlLoopPeriodInms/1000.)
        spdlog::warn("Applicaton thread time overflow: {}ms (>{}ms) !", dt*1000., controlLoopPeriodInms);

}

/******************** Runs at the End of rt_control_thread********************/
void app_programEnd(void) {
    stateMachine->end();
    stateMachine.reset(); //Explicit delete of the state machine to answer deletion on time
    spdlog::info("CORC End application");
}
