/**
 * \file StatesTemplate.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2025-04-14
 *
 * \copyright Copyright (c) 2025
 *
 */

#ifndef STATESTEMPLATE_H
#define STATESTEMPLATE_H

#include "State.h"
#include "PlatformName.h"


class StateMachineTemplate;

/**
 * \brief Generic state type including a pointer to PlatformName
 *
 */
class PlatformNameState : public State {
   protected:
    PlatformName * robot;                               //!< Pointer to state machines robot object

    PlatformNameState(PlatformName* _robot, const char *name = NULL): State(name), robot(_robot){spdlog::debug("Created PlatformNameState {}", name);};
};


class StandbyState : public PlatformNameState {

   public:
    StandbyState(PlatformName * _robot, const char *name = "PlatformName Standby"):PlatformNameState(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);
};



/**
 * \brief Position calibration example. Go to stops of robot at constant torque for absolute position calibration.
 *
 */
class CalibState : public PlatformNameState {

   public:
    CalibState(PlatformName * _robot, const char *name = "PlatformName Standby"):PlatformNameState(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isCalibDone() {return calibDone;}

   private:
     double stop_reached_time;
     bool at_stop;
     bool calibDone=false;

};

#endif
