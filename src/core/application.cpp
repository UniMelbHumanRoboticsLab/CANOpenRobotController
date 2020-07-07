/*
 * Application interface for Alex Exoskeleton Software
 *
 * @file        application.c
 * @author      William Campbell
 *

 */
#include "application.h"

//Select state machine to use for this application (can be set in cmake)
#ifndef STATE_MACHINE_TYPE
#define STATE_MACHINE_TYPE ExoTestMachine
#endif

STATE_MACHINE_TYPE testMachine;

#ifdef USEROS
    #ifndef STATE_MACHINE_TYPE_ROS
    #define STATE_MACHINE_TYPE_ROS* X2DemoMachineROS
    #endif
    STATE_MACHINE_TYPE_ROS* testMachineROS;
#endif

/*For master-> node SDO message sending*/
#define CO_COMMAND_SDO_BUFFER_SIZE 100000
#define STRING_BUFFER_SIZE (CO_COMMAND_SDO_BUFFER_SIZE * 4 + 100)
char buf[STRING_BUFFER_SIZE];
char ret[STRING_BUFFER_SIZE];
/******************************************************************************/
void app_programStart(void) {
    printf("app_Program Start \n");
    testMachine.init();
    testMachine.activate();
}

#ifdef USEROS
void app_ROSStart(int argc, char *argv[]) {
    ros::init(argc, argv, "x2_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle;
    testMachineROS = new STATE_MACHINE_TYPE_ROS(testMachine.robot);
    testMachineROS->initialize(nodeHandle);

}
#endif
/******************************************************************************/
void app_communicationReset(void) {
}
/******************************************************************************/
void app_programEnd(void) {
    testMachine.end();
    printf("app_programEnd \n");
}
/******************************************************************************/
void app_programAsync(uint16_t timer1msDiffy) {
}

void app_programControlLoop(void) {
    if (testMachine.running) {
        testMachine.update();
        testMachine.hwStateUpdate();
    }
}

#ifdef USEROS
void app_ROSLoop(void) {
    testMachineROS->publishJointStates();
    ros::spinOnce();

}
#endif