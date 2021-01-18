/*
 * Application interface for Alex Exoskeleton Software
 *
 * @file        application.c
 * @author      William Campbell
 *

 */
#include "application.h"

#include "RobotousRFT.h"

#ifdef TIMING_LOG
#include "LoopTiming.h"
LoopTiming loopTimer;
#endif


//Select state machine to use for this application (can be set in cmake)
#ifndef STATE_MACHINE_TYPE
#define STATE_MACHINE_TYPE ExoTestMachine
#endif

extern CO_OD_entry_t CO_OD[CO_OD_NoOfElements];

STATE_MACHINE_TYPE stateMachine;
/*For master-> node SDO message sending*/
#define CO_COMMAND_SDO_BUFFER_SIZE 100000
#define STRING_BUFFER_SIZE (CO_COMMAND_SDO_BUFFER_SIZE * 4 + 100)

char buf[STRING_BUFFER_SIZE];
char ret[STRING_BUFFER_SIZE];
int counter = 0;

RobotousRFT *sensor;
RobotousRFT *sensor2;

/******************************************************************************/
void app_programStart(int argc, char *argv[]) {
    spdlog::info("CORC Start application");

#ifdef NOROBOT
    spdlog::info("Running in NOROBOT (virtual) mode.");
#endif  // NOROBOT

    CO_configure();

    sensor = new RobotousRFT(0xf8, 0xf9, 0xfa);
    sensor2 = new RobotousRFT(0xf0, 0xf1, 0xf2);

    // For each PDO:
    // - Create Comm Parameter Object
    // - Create Mapping Paramter Object
    // - Create Storage location (R/W)
    //spdlog::info("RPDO {} Set", CO_setRPDO(&RPDOcommPara, &RPDOmapparam, RPDOCommEntry, dataStoreRecord, RPDOmapparamEntry));
    //spdlog::info("RPDO {} Set", CO_setRPDO(&RPDOcommPara2, &RPDOmapparam2, RPDOCommEntry2, dataStoreRecord2, RPDOmapparamEntry2));

/*
    // Change the OD entry
    CO_OD[25].pData = (void *)&PRDOCommEntry;
    CO_OD[25 + CO_NO_RPDO].pData = (void *)&RPDOmapparamEntry;

    // Change the Mapping Parameter Entry
    OD_RPDOCommunicationParameter[0] = &RPDOcommPara;
    OD_RPDOMappingParameter[0] = &RPDOmapparam;

    // Change the relevant OD location
    CO_OD[24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 85].pData = (void *)&dataStoreRecord;*/

//CO_OD
#ifndef USEROS
                                                                  stateMachine.init();
#else
                                                                  stateMachine.init(argc, argv);
#endif
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
        stateMachine.update();
        stateMachine.hwStateUpdate();
    }
    counter = counter + 1;
    if (counter % 100 == 0) {
        sensor->update();
        sensor2->update();

        Eigen::VectorXd forces= sensor->getForces();
        Eigen::VectorXd torques = sensor2->getForces();

        /*UNSIGNED16 Fx = ODtestthing[1] * 256 + ODtestthing[2];
        UNSIGNED16 Fy = ODtestthing[3] * 256 + ODtestthing[4];
        UNSIGNED16 Fz = ODtestthing[5] * 256 + ODtestthing[6];
        UNSIGNED16 Tx = ODtestthing[7] * 256 + ODtestthing[8];
        UNSIGNED16 Ty = ODtestthing[9] * 256 + ODtestthing[10];
        UNSIGNED16 Tz = ODtestthing[11] * 256 + ODtestthing[12];*/

        spdlog::info("ODTEST {}, {},{},{},{},{}", forces[0], forces[1], forces[2], torques[0], torques[1], torques[2]);
        
    };
#ifdef TIMING_LOG
    loopTimer.tick();
#endif
}
