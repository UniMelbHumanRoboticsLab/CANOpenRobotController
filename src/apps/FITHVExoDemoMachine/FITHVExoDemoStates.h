/**
 * \file FITHVExoDemo.h
 * \author Vincent Crocher
 * \version 0.2
 * \date 2025-11-10
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
    RobotFITHVExo * robot;      //!< Pointer to state machines robot object

    RobotFITHVExoState(RobotFITHVExo* _robot, const char *name = NULL): State(name), robot(_robot){spdlog::debug("Created RobotFITHVExoState {}", name);};
};


/**
 * \brief Simple calibration state using internal absolute encoders via RobotFITHVExo dedicated method.
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


/**
 * \brief State providing friction compensation and arbitrary viscous field on both joints
 *
 */
class StandbyState : public RobotFITHVExoState {

   public:
    StandbyState(RobotFITHVExo * _robot, const char *name = "Standby"):RobotFITHVExoState(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);

    V2 tau;     //!< Additional torque command to supply
    double b=0; //!< Viscosity to provide in addition to friction comp. Can be positive for assistance.
};



/**
 * \brief Demo assistive state providing various levels of gravity, virtual wall or viscous assistance
 *
 */
class WallAssistState : public RobotFITHVExoState {

   public:
        WallAssistState(RobotFITHVExo * _robot, const char *name = "Wall Assist"):RobotFITHVExoState(_robot, name){};

        void entry(void);
        void during(void);
        void exit(void);

        //! Assign/update wall parameters
        bool setParameters(double _k, double _q0t, double _b, double _mgl) {
            k = _k;
            b = _b;
            q0t = _q0t;
            mgl = _mgl;
            spdlog::info("WallAssist:settingParameters parameters k={} q0={} b={}, mgl={}...", _k, _q0t*180./M_PI, _b, _mgl);
            k = fmin(fmax(k, 0), maxk);
            q0t = fmin(fmax(q0t, 0), M_PI/2.);
            mgl = fmin(fmax(mgl, 0), maxmgl);
            b = fmin(fmax(b, 0), b);
            if(q0t == _q0t && k == _k && b == _b && mgl ==_mgl) {
                spdlog::info("WallAssist:setParameters parameters {} {} {} {}. Ok.", _k, _q0t*180./M_PI, _b, _mgl);
                return true;
            }
            else {
                spdlog::warn("WallAssist:setParameters parameters saturated: {} {} {} {}.", k, q0t*180./M_PI, b, mgl);
                return false;
            }
        }

        double & getk(){return k;}
        double & getq0(){return q0t;}
        double & getb(){return b;}
        double & getmgl(){return mgl;}

    private:
        double k=100., maxk=140.; //in [Nm/rad]
        double q0t=20.*M_PI/180.; //in [rad]
        double b=.2, maxb=1.; //Viscous assistance outside wall [Nm/rad.s]
        double mgl=0, maxmgl=9.8*0.5*50.; //gravity x upper-body mass x center of mass for gravity compensation
        V2 q0;
};


/**
 * \brief Demo assistance state providing Pviscous assistance/resistance
 *
 */
class AmplificationState: public RobotFITHVExoState {

   public:
        AmplificationState(RobotFITHVExo * _robot, const char *name = "Amplification Assist"):RobotFITHVExoState(_robot, name){};

        void entry(void);
        void during(void);
        void exit(void);

        //! Assign/update parameters
        bool setParameters(double _b) {
            b = _b;
            spdlog::info("AmplificationState:settingParameters parameters b={}...", _b);
            b = fmin(fmax(b, 0), b);
            if(b == _b) {
                spdlog::info("AmplificationState:setParameters parameters {}. Ok.", _b);
                return true;
            }
            else {
                spdlog::warn("AmplificationState:setParameters parameters saturated: {}.", b);
                return false;
            }
        }

        double & getb(){return b;}

    private:
        double b=.2, maxb=10.; //!< Viscous assistance parameters
};

#endif
