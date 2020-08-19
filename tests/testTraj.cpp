/**
 * @file testSM.cpp
 * @author William Campbell
 * @brief Test for the trajectory Generator class
 * @version 0.1
 * @date 2020-04-09
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "ExoRobot.h"
//using namespace std;

#include "CANopen.h"

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
    cout << ">>> Create ALEXTrajectory Generator >>>" << endl;

    ALEXTrajectoryGenerator *trajectoryGenerator = new ALEXTrajectoryGenerator();

    PilotParameters exoParams = {
        .lowerleg_length = 0.44,
        .upperleg_length = 0.44,
        .ankle_height = 0.12,
        .foot_length = 0.30,
        .hip_width = 0.43,
        .torso_length = 0.4,
        .buttocks_height = 0.05};

    // Initial Pose -> Standing up
    jointspace_state initialPose;
    for (int i = 0; i < NUM_JOINTS; i++)
        initialPose.q[i] = 0;
    initialPose.time = 0;

    trajectoryGenerator->setPilotParameters(exoParams);
    trajectoryGenerator->initialiseTrajectory(RobotMode::STNDUP, initialPose);
    // Should be standing up

    double positions[NUM_JOINTS];
    for (double i = 0; i <= 2; i = i + 0.1) {
        trajectoryGenerator->calcPosition(i, positions);
        for (int j = 0; j < NUM_JOINTS; j++) {
            std::cout << rad2deg(positions[j]) << " ";
        }
        std::cout << endl;
    }

    // exit(0);
}