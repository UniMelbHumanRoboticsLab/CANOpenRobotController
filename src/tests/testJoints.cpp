/**
 * \file testJoints.cpp
 * \author Justin Fong
 * \brief Tests for the Joint, ActuatedJoint, ExoJoint and Drive classes 
 * \version 0.1
 * \date 2020-04-09
 * 
 * \copyright Copyright (c) 2020
 * 
 */

#include <iostream>

#include "ActuatedJoint.h"
#include "CANopen.h"
#include "CopleyDrive.h"
#include "Drive.h"
#include "DummyActJoint.h"
#include "Joint.h"

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

int main() {
    std::cout << "This is a script to test the implementation of the Joints, ActuatedJoints and Drive Classes! \n";

    std::cout << "1. Construct a CopleyDrive Object (which implements Drive Class) \n";
    // Construct different types of joints
    Drive *testDrive = new CopleyDrive(100);

    std::cout << "2. Construct a TestActJoint Object (Cast as a ActuatedJoint), using the testDrive Object \n";
    ActuatedJoint *normalJoint = new DummyActJoint(1, -1, 1, testDrive);

    std::cout << "Read the ID of the Joint (Expected Value 1): " << normalJoint->getId() << "\n";
    std::cout << "Read Node ID of the Drive (Expected Value 100): " << testDrive->getNodeID() << "\n";

    std::cout << "Read Value of the Joint (Expected Value 0): " << normalJoint->getQ() << "\n";
    motorProfile testProfile{4000000, 240000, 240000};
    std::cout << "Set the Joint into Position Control Mode: " << normalJoint->setMode(POSITION_CONTROL, testProfile) << "\n";

    std::cout << "Set the position of the Joint to 1 (expected result: true): " << normalJoint->setPosition(1) << "\n";

    std::cout << "Read Value of the Joint (Expected Value 0): " << normalJoint->getQ() << "\n";

    std::cout << "Call a updateValue() defined in Joint: " << normalJoint->updateValue() << "\n";

    std::cout << "Read Value of the Joint (Expected Value 1): " << normalJoint->getQ() << "\n";
    //test velocity control commands

    std::cout << "Set the Joint into Velocity Control Mode: " << normalJoint->setMode(VELOCITY_CONTROL, testProfile) << "\n";

    std::cout << "Set the position of the Joint to 1 (expected result: true): " << normalJoint->setVelocity(100) << "\n";

    std::cout << "Read Value of the Joint (Expected Value 0): " << normalJoint->getQ() << "\n";

    std::cout << "Call a updateValue() defined in Joint: " << normalJoint->updateValue() << "\n";

    std::cout << "Read Value of the Joint (Expected Value 1): " << normalJoint->getQ() << "\n";
    return 0;
}