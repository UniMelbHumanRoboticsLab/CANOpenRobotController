//
// Created by ubuntu on 10/28/20.
//

#include "M1TestMachine.h"

#define OWNER ((M1TestMachine *)owner)

M1TestMachine::M1TestMachine() {
    spdlog::debug("M1TestMachine::Constructed!");
//    robot = new RobotM1();
    std::cout << "M1TESTMACHINE" << std::endl;
    running = true;
}

M1TestMachine::~M1TestMachine() {
    spdlog::debug("M1TestMachine::Distructed()");
//    delete demoState;
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */

void M1TestMachine::init() {
    spdlog::debug("M1TestMachine::init()");
//    if(robot->initialise()) {
//        initialised = true;
//    }
//    else {
//        initialised = false;
//        std::cout /*cerr is banned*/ << "Failed robot initialisation. Exiting..." << std::endl;
//        std::raise(SIGTERM); //Clean exit
//    }
//    running = true;
}

void M1TestMachine::end() {
    spdlog::debug("M1TestMachine::end()");
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M1TestMachine::hwStateUpdate(void) {
//    spdlog::debug("M1TestMachine::hwStateUpdate()");
//    robot->updateRobot();
    std::cout << "." << std::endl;
}