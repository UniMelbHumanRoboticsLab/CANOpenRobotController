/**
 * /file IdleState.cpp
 * /author Justin Fong
 * /brief Nothing happens in this state, just waiting for action
 * /version 0.1
 * /date 2021-1-21
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "IdleState.h"

IdleState::IdleState(LoggingRobot *robot, const char *name) : State(name), robot(robot) {
    spdlog::info("IdleState Created");
};
void IdleState::entry(void) {
    spdlog::info("IdleState entry");
    spdlog::info("To Zero: A = Crutches");
    spdlog::info("S to start logging");
};

void IdleState::during(void){
    // Do nothing
};

void IdleState::exit(void) {
    spdlog::info("IdleState Exit");
};
