/**
 * \file MultiM1Machine.h
 * \author Emek Baris Kucuktabak
 * \version 0.1
 * \date 2020-11-22
 * \copyright Copyright (c) 2020
 *
 * /brief The MultiM1Machine class represents an example implementation of a stateMachine in multi-robot control setting
 *
 */
#ifndef M1_SM_H
#define M1_SM_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <csignal> //For raise()

#include "RobotM1.h"
#include "StateMachine.h"

// State Classes
#include "states/MultiControllerState.h"
#include "spdlog/helper/LogHelper.h"
#include "logging.h"

#include "MultiM1MachineROS.h"

/**
 * @brief Example implementation of a StateMachine for the M1Robot class. States should implemented M1DemoState
 *
 */
class MultiM1Machine : public StateMachine {
   public:
    bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
//    MultiM1Machine();
    MultiM1Machine(int argc, char *argv[]);
    ~MultiM1Machine();
//    void init(int argc, char *argv[]);
    void init();
    void end();

    bool configureMasterPDOs();

    void hwStateUpdate();

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    MultiControllerState *multiControllerState_;

   protected:
    RobotM1 *robot_; /*<!Pointer to the Robot*/
    MultiM1MachineROS *multiM1MachineRos_; /*<!Pointer to the ROS Class*/

   private:
    std::string robotName_; // robot name(obtained from node name)
    std::chrono::steady_clock::time_point time0_; // initial time that machine started
    double time_; // time passed after tim0 in [s]
};

#endif /*M1_SM_H*/
