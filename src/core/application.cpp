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

extern CO_OD_entry_t CO_OD[CO_OD_NoOfElements];

STATE_MACHINE_TYPE stateMachine;
/*For master-> node SDO message sending*/
#define CO_COMMAND_SDO_BUFFER_SIZE 100000
#define STRING_BUFFER_SIZE (CO_COMMAND_SDO_BUFFER_SIZE * 4 + 100)

char buf[STRING_BUFFER_SIZE];
char ret[STRING_BUFFER_SIZE];
int counter = 0;


// This should be in the object itself 
UNSIGNED8 ODtestthing[8] = {0, 0, 0, 0, 0, 0, 0};

// This should be
CO_OD_entryRecord_t dataStoreRecord[9] = {
    {(void *)&CO_OD_RAM.controlWords.numberOfMotors, 0x06, 0x1},
    {(void *)&ODtestthing[0], 0xfe, 0x2},
    {(void *)&ODtestthing[1], 0xfe, 0x2},
    {(void *)&ODtestthing[2], 0xfe, 0x2},
    {(void *)&ODtestthing[3], 0xfe, 0x2},
    {(void *)&ODtestthing[4], 0xfe, 0x2},
    {(void *)&ODtestthing[5], 0xfe, 0x2},
    {(void *)&ODtestthing[6], 0xfe, 0x2},
    {(void *)&ODtestthing[7], 0xfe, 0x2},
};

OD_RPDOCommunicationParameter_t RPDOcommPara = {0x2L, 0x0f1L, 0xffL};

CO_OD_entryRecord_t PRDOCommEntry[3] = {
    {(void *)&RPDOcommPara.maxSubIndex, 0x06, 0x1},
    {(void *)&RPDOcommPara.COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&RPDOcommPara.transmissionType, 0x0e, 0x1},
};


// This should be owned by the OD
OD_RPDOMappingParameter_t RPDOmapparam = {0x8L, 0x60000108L, 0x60000208L, 0x60000308L, 0x60000408L, 0x60000508L, 0x60000608L, 0x60000708L, 0x60000808L};


// This should be owned by the object?
CO_OD_entryRecord_t RPDOmapparamEntry[9] = {
    {(void *)&RPDOmapparam.numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&RPDOmapparam.mappedObject1, 0x8e, 0x4},
    {(void *)&RPDOmapparam.mappedObject2, 0x8e, 0x4},
    {(void *)&RPDOmapparam.mappedObject3, 0x8e, 0x4},
    {(void *)&RPDOmapparam.mappedObject4, 0x8e, 0x4},
    {(void *)&RPDOmapparam.mappedObject5, 0x8e, 0x4},
    {(void *)&RPDOmapparam.mappedObject6, 0x8e, 0x4},
    {(void *)&RPDOmapparam.mappedObject7, 0x8e, 0x4},
    {(void *)&RPDOmapparam.mappedObject8, 0x8e, 0x4},
};

/******************************************************************************/
void app_programStart(int argc, char *argv[]) {
    spdlog::info("CORC Start application");

#ifdef NOROBOT
    spdlog::info("Running in NOROBOT (virtual) mode.");
#endif  // NOROBOT

    CO_configure();
    // For each PDO:
    // - Create Comm Parameter Object
    // - Create Mapping Paramter Object
    // - Create Storage location (R/W)
    CO_setRPDO(0xf1, PRDOCommEntry, dataStoreRecord, &RPDOcommPara, RPDOmapparamEntry, &RPDOmapparam);

    uint32_t *pMap = &RPDOmapparam.mappedObject1;

    for (int i = 0; i < RPDOmapparam.numberOfMappedObjects; i++) {
        uint32_t map = *pMap;
        printf("%d %d \n", map, pMap);
        pMap++;
    }

    // Change the OD entry
    CO_OD[25].pData = (void *)&PRDOCommEntry;
    CO_OD[25 + CO_NO_RPDO].pData = (void *)&RPDOmapparamEntry;

    // Change the Mapping Parameter Entry
    OD_RPDOCommunicationParameter[0] = &RPDOcommPara;
    OD_RPDOMappingParameter[0] = &RPDOmapparam;



    // Change the relevant OD location
    CO_OD[24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 85].pData = (void *)&dataStoreRecord;

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
        UNSIGNED16 Fx = ODtestthing[1] * 256 + ODtestthing[2];
        UNSIGNED16 Fy = ODtestthing[3] * 256 + ODtestthing[4];
        UNSIGNED16 Fz = ODtestthing[5] * 256 + ODtestthing[6];
        spdlog::info("ODTEST {}, {},{},{}", ODtestthing[0], static_cast<INTEGER16>(Fx), static_cast<INTEGER16>(Fy), static_cast<INTEGER16>(Fz));
    };
#ifdef TIMING_LOG
    loopTimer.tick();
#endif
}
