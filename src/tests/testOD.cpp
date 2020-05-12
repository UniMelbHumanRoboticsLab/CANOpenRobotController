
/**
 * \file testOD.cpp
 * \author William Campbell
 * \brief A script to test the functionality of the Object dictionary access
 * \version 0.1
 * \date 2020-04-21
 * 
 * \copyright Copyright (c) 2020
 * 
 */
#include <iostream>

#include "CANopen.h"
#include "CopleyDrive.h"
#include "Drive.h"

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

using namespace std;
int main() {
    // Create a Drive Object
    std::cout << "1. Construct a CopleyDrive Object (which implements Drive Class), with NODE ID = 1 \n";
    Drive *testDrive = new CopleyDrive(3);

    std::cout << "2. Read form OD and change OD\n";
    cout << "current OD position : " << testDrive->getPos() << std::endl;
    testDrive->setPos(10);
    cout << "current OD position : " << testDrive->getPos() << std::endl;
    cout << "Read Specifically from CO_OD_RAM.targetMotorPositions.motor1: " << CO_OD_RAM.targetMotorPositions.motor1 << std::endl;
    cout << "Read Specifically from CO_OD_RAM.targetMotorPositions.motor2: " << CO_OD_RAM.targetMotorPositions.motor2 << std::endl;
    cout << "Read Specifically from CO_OD_RAM.targetMotorPositions.motor3: " << CO_OD_RAM.targetMotorPositions.motor3 << std::endl;
    cout << "Read Specifically from CO_OD_RAM.targetMotorPositions.motor4: " << CO_OD_RAM.targetMotorPositions.motor4 << std::endl;
    cout << "Read Specifically from CO_OD_RAM.targetMotorPositions.motor5: " << CO_OD_RAM.targetMotorPositions.motor5 << std::endl;
    cout << "Read Specifically from CO_OD_RAM.targetMotorPositions.motor6: " << CO_OD_RAM.targetMotorPositions.motor6 << std::endl;
}
