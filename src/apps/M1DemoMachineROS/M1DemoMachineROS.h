/**
 * \file M1DemoMachineROS.h
 * \author Emek Baris Kucuktabak
 * \version 0.2
 * \date 2022-10-25
 * \copyright Copyright (c) 2020
 *
 * /brief The M1DemoMachineROS class represents an example implementation of a stateMachine in multi-robot control setting
 *
 */
#ifndef M1_SM_H
#define M1_SM_H

#include "RobotM1.h"
#include "StateMachine.h"

// State Classes
#include "states/MultiControllerState.h"

#include "M1MachineROS.h"

/**
 * @brief Example implementation of a StateMachine for the M1Robot class. States should implemented M1DemoState
 *
 */
class M1DemoMachineROS : public StateMachine {
   public:
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M1DemoMachineROS(int argc, char *argv[]);
    ~M1DemoMachineROS();
    void init();

    void hwStateUpdate();

    RobotM1 *robot() { return static_cast<RobotM1*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)
   protected:
    M1MachineROS *M1MachineRos_; /*<!Pointer to the ROS Class*/

   private:
    std::string robotName_; // robot name(obtained from node name)
};

#endif /*M1_SM_H*/
