/*
 * Application interface for Alex Exoskeleton Software
 *
 * @file        application.c
 * @author      William Campbell
 *

 */
#include "application.h"


#ifdef TIMING_LOG
#include "LoopTiming.h"
LoopTiming loopTimer;
#endif

//Select state machine to use for this application (can be set in cmake)
#ifndef STATE_MACHINE_TYPE
#define STATE_MACHINE_TYPE ExoTestMachine
#endif

STATE_MACHINE_TYPE stateMachine;


/*For master-> node SDO message sending*/
#define CO_COMMAND_SDO_BUFFER_SIZE 100000
#define STRING_BUFFER_SIZE (CO_COMMAND_SDO_BUFFER_SIZE * 4 + 100)
char buf[STRING_BUFFER_SIZE];
char ret[STRING_BUFFER_SIZE];
/******************************************************************************/
void app_programStart(int argc, char *argv[]) {
    spdlog::info("CORC Start application");
#ifdef NOROBOT
    spdlog::info("Running in NOROBOT (virtual) mode.");
#endif // NOROBOT
#ifndef USEROS
    spdlog::info("stateMachine.init();");
    stateMachine.init();
#else
    stateMachine.init(argc, argv);
#endif
    spdlog::info(" stateMachine.activate()");
    stateMachine.activate();
}

/******************************************************************************/
void app_communicationReset(void) {
}
/******************************************************************************/
void app_programEnd(void) {
    stateMachine.end();
    spdlog::info("CORC End application");
    #ifdef TIMING_LOG
    loopTimer.end();
    #endif
}
/******************************************************************************/
void app_programAsync(uint16_t timer1msDiffy) {
}

void app_programControlLoop(void) {
    if (stateMachine.running) {
//        spdlog::info("CORC app update");
        stateMachine.update();
        stateMachine.hwStateUpdate();
    }
#ifdef TIMING_LOG
    loopTimer.tick();
    #endif
}
