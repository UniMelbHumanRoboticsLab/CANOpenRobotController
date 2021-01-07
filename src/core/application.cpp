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
<<<<<<< Updated upstream
/******************************************************************************/
=======
char buf[STRING_BUFFER_SIZE];
char ret[STRING_BUFFER_SIZE];
INTEGER16 ODtestthing = 0;

CO_OD_entryRecord_t testRecord[7] = {
    {(void *)&CO_OD_RAM.controlWords.numberOfMotors, 0x06, 0x1},
    {(void *)&ODtestthing, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor2, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor3, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor4, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor5, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor6, 0xfe, 0x2},
};

OD_RPDOCommunicationParameter_t RPDOcommPara = {0x2L, 0x0faL, 0xffL};

CO_OD_entryRecord_t PRDOCommEntry[3] = {
    {(void *)&RPDOcommPara.maxSubIndex, 0x06, 0x1},
    {(void *)&RPDOcommPara.COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&RPDOcommPara.transmissionType, 0x0e, 0x1},
};

OD_RPDOMappingParameter_t RPDOmapparam = {0x1L, 0x60410110L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L};

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
extern OD_RPDOCommunicationParameter_t RPDOCommParamOff;
extern OD_RPDOMappingParameter_t RPDOMapParamOff;
    /******************************************************************************/

>>>>>>> Stashed changes
void app_programStart(int argc, char *argv[]) {
    spdlog::info("CORC Start application");
#ifdef NOROBOT
    spdlog::info("Running in NOROBOT (virtual) mode.");
<<<<<<< Updated upstream
#endif // NOROBOT
=======
#endif  // NOROBOT

    CO_configure();
    spdlog::info("ODTEST {} {} {}", CO_OD[25].index, CO_OD[25].attribute, CO_OD[25].pData);

    // Initialise all RPDOs to off
    for (int i = 0; i < CO_NO_RPDO; i++ ){
        OD_RPDOCommunicationParameter[i] = &RPDOCommParamOff;
        OD_RPDOMappingParameter[i] = &RPDOMapParamOff;
    }

    // Initialise all TPDOs to off


    // For each PDO:
    // - Create Comm Parameter Object
    // - Create Mapping Paramter Object
    // - Create Storage location (R/W)


    CO_OD[25].pData = (void *)&PRDOCommEntry;
    CO_OD[25 + CO_NO_RPDO].pData = (void *)&RPDOmapparamEntry;
    OD_RPDOCommunicationParameter[0] = &RPDOcommPara;
    OD_RPDOMappingParameter[0] = &RPDOmapparam;


    CO_OD[24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 92].pData = (void *)&testRecord;
//CO_OD
>>>>>>> Stashed changes
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
    #ifdef TIMING_LOG
    loopTimer.tick();
    #endif
}
