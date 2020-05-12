/**
 * @file testSM.cpp
 * @author William Campbell
 * @brief Test for the statemachine class and associated classes: state, event and transition
 * @version 0.1
 * @date 2020-04-09
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "CANopen.h"
#include "ExoRobot.h"
#include "TestMachine.h"

pthread_mutex_t CO_CAN_VALID_mtx = PTHREAD_MUTEX_INITIALIZER;
volatile uint32_t CO_timer1ms = 0U;

/* Helper functions ***********************************************************/
void CO_errExit(char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}

/* send CANopen generic emergency message */
void CO_error(const uint32_t info) {
    CO_errorReport(CO->em, CO_EM_GENERIC_SOFTWARE_ERROR, CO_EMC_SOFTWARE_INTERNAL, info);
    fprintf(stderr, "canopend generic error: 0x%X\n", info);
}

int main(void) {
    // Create Exo object + initialise derived Joints + trajectory Generator
    bool exit = false;
    cout << ">>> Creating Test state machine" << endl;
    ExoRobot exo;
    TestMachine testMachine;
    printf("app_Program Start \n");
    testMachine.initRobot(&exo);
    testMachine.init();
    testMachine.activate();
    while (!exo.keyboard.getQ()) {
        usleep(500000);
        testMachine.hwStateUpdate();
        testMachine.update();
    }

    // exit(0);
}