/**
 * /file InitState.cpp
 * /author Justin Fong
 * /brief Virtual Class to include all required classes for Logging Robot
 * /version 0.1
 * /date 2020-12-1
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "InitState.h"

InitState::InitState(StateMachine *m, LoggingRobot *robot, const char *name) : State(m, name), robot(robot) {
    spdlog::info("InitState Created");
};
void InitState::entry(void) {
    spdlog::info("InitState entry");
};

void InitState::during(void){
    //spdlog::info("COB-ID {}, Mapped Ob {} Value {} {}", CO_OD_RAM.RPDOCommunicationParameter[31].COB_IDUsedByRPDO, CO_OD_RAM.RPDOMappingParameter[31].mappedObject1, CO_OD_RAM.statusWords.motor2, CO_OD_RAM.statusWords.motor1);
};

void InitState::exit(void) {
    spdlog::info("InitState Exit");
    //CO->RPDO[31]->RPDOCommPar->COB_IDUsedByRPDO = 0x0f1;
    reset_local = CO_RESET_COMM;
};