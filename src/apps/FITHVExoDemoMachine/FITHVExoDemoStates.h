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
    StandbyState(RobotFITHVExo * _robot, const char *name = "Standby"):RobotFITHVExoState(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);

    V2 tau;     //!< Additional torque command to supply
    double b=0; //!< Viscosity to provide in addition to friction comp. Can be positive for assistance.
};


class TestState : public RobotFITHVExoState {

   public:
    TestState(RobotFITHVExo * _robot, const char *name = "Test"):RobotFITHVExoState(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);

    V2 cmd;
    double a=0.6, b=0.2;
};


class WallAssistState : public RobotFITHVExoState {

   public:
        WallAssistState(RobotFITHVExo * _robot, const char *name = "Wall Assist"):RobotFITHVExoState(_robot, name){};

        void entry(void);
        void during(void);
        void exit(void);

        //! Assign wall parameters. Only when nwot running.
        bool setParameters(double _k, double _q0t) {
            if(!active()) {
                k = _k;
                q0t = _q0t;
                spdlog::info("WallAssist:setParameters parameters k={} q0={}.", _k, _q0t*180./M_PI);
                k = fmin(fmax(k, 0), maxk);
                q0t = fmin(fmax(q0t, 0), M_PI/2.);

                if(q0t == _q0t && k == _k) {
                    spdlog::info("WallAssist:setParameters parameters k={} q0={}.", _k, _q0t*180./M_PI);
                    return true;
                }
                else {
                    spdlog::warn("ForceControl:setParameters parameters saturated: {} {}.", k, q0t*180./M_PI);
                    return false;
                }
            }
        }

        double & getk(){return k;}
        double & getq0(){return q0t;}

    private:
        double k=30., maxk=50.;
        double q0t=45*M_PI/180.;
        V2 q0;
};


/**
 * \brief Position calibration example. Go to stops of robot at constant torque for absolute position calibration.
 *
 */
class CalibState : public RobotFITHVExoState {

   public:
    CalibState(RobotFITHVExo * _robot, const char *name = "Calib"):RobotFITHVExoState(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isCalibDone() {return calibDone;}

   private:
    bool calibDone=false;
};

#endif
