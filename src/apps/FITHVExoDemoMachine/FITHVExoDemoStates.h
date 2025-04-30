/**
 * \file FITHVExoDemo.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2025-04-14
 *
 * \copyright Copyright (c) 2025
 *
 */

#ifndef FITHVEXODEMO_H
#define FITHVEXODEMO_H

#include "State.h"
#include "RobotFITHVExo.h"


class FITHVExoDemoMachine;

/**
 * \brief Generic state type including a pointer to RobotFITHVExo
 *
 */
class RobotFITHVExoState : public State {
   protected:
    RobotFITHVExo * robot;                               //!< Pointer to state machines robot object

    RobotFITHVExoState(RobotFITHVExo* _robot, const char *name = NULL): State(name), robot(_robot){spdlog::debug("Created RobotFITHVExoState {}", name);};
};


class StandbyState : public RobotFITHVExoState {

   public:
    StandbyState(RobotFITHVExo * _robot, const char *name = "RobotFITHVExo Standby"):RobotFITHVExoState(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);

    V2 cmd;
};



/**
 * \brief Position calibration example. Go to stops of robot at constant torque for absolute position calibration.
 *
 */
class CalibState : public RobotFITHVExoState {

   public:
    CalibState(RobotFITHVExo * _robot, const char *name = "RobotFITHVExo Standby"):RobotFITHVExoState(_robot, name){};

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
