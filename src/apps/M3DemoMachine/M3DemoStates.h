/**
 * \file M3DemoState.h
 * \author Vincent Crocher
 * \version 0.3
 * \date 2020-07-27
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef M3DemoSTATE_H_DEF
#define M3DemoSTATE_H_DEF

#include "State.h"
#include "RobotM3.h"


class M3DemoMachine;

/**
 * \brief Generic state type for used with M3DemoMachine, providing running time and iterations number: been superseeded by default state.
 *
 */
class M3TimedState : public State {
   protected:
    RobotM3 * robot;                               //!< Pointer to state machines robot object

    M3TimedState(RobotM3* M3, const char *name = NULL): State(name), robot(M3){spdlog::debug("Created M3TimedState {}", name);};
   private:
    void entry(void) final {
        std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "----------------------------------" << std::endl
        << std::endl;

        //Actual state entry
        entryCode();
    };
    void during(void) final {
        //Actual state during
        duringCode();
    };
    void exit(void) final {
        exitCode();
        std::cout
        << "----------------------------------" << std::endl
        << "EXIT "<< getName() << std::endl
        << "==================================" << std::endl
        << std::endl;
    };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};
};


class M3DemoState : public M3TimedState {

   public:
    M3DemoState(RobotM3 * M3, const char *name = "M3 Test State"):M3TimedState(M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    VM3 qi, Xi;
    double f=0.0;
};



/**
 * \brief Position calibration of M3. Go to the bottom left stops of robot at constant torque for absolute position calibration.
 *
 */
class M3CalibState : public M3TimedState {

   public:
    M3CalibState(RobotM3 * M3, const char *name = "M3 Calib State"):M3TimedState(M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() {return calibDone;}

   private:
     VM3 stop_reached_time;
     bool at_stop[3];
     bool calibDone=false;

};


/**
 * \brief Provide end-effector mass compensation on M3. Mass is controllable through keyboard inputs.
 *
 */
class M3MassCompensation : public M3TimedState {

   public:
    M3MassCompensation(RobotM3 * M3, const char *name = "M3 Mass Compensation"):M3TimedState(M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
     double mass = 0;

};


/**
 * \brief End-effector velocity control through joystick input.
 *
 */
class M3EndEffDemo : public M3TimedState {

   public:
    M3EndEffDemo(RobotM3 * M3, const char *name = "M3 Velocity Control Demo"):M3TimedState(M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);
};

/**
 * \brief Basic impedance control on a static point.
 *
 */
class M3DemoImpedanceState : public M3TimedState {

   public:
    M3DemoImpedanceState(RobotM3 * M3, const char *name = "M3 Demo Impedance State"):M3TimedState(M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    VM3 Xi;
    double k = 2000;     //! Impedance proportional gain (spring)
    double d = 2;       //! Impedance derivative gain (damper)
    bool init=false;

    unsigned int nb_samples=10000;
    double dts[10000];
    double dX[10000];
    int new_value;
};


/**
 * \brief Teleoperation state
 *
 */
class M3TeleopState : public M3TimedState {

   public:
    M3TeleopState(RobotM3 * M3, M3DemoMachine &sm, const char *name = "M3 Teleop State"):M3TimedState(M3, name), SM(sm) {};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    M3DemoMachine &SM;
    VM3 F;
    VM3 Xi;
};


/**
 * \brief Path contraint with viscous assistance.
 *
 */
class M3DemoPathState : public M3TimedState {

   public:
    M3DemoPathState(RobotM3 * M3, const char *name = "M3 Demo Path State"):M3TimedState(M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    short int sign(double val) {return (val>0)?1:((val<0)?-1:0); };

   private:
    double k = 600;                 //! Impedance proportional gain (spring)
    double d = 6;                   //! Impedance derivative gain (damper)
    double viscous_assistance=15;   //! Viscous assistance along path
    VM3 Xi;
    VM3 Xf={-0.4, 0, 0};              //! Path target point
};


/**
 * \brief Point to point position control with min jerk trajectory interpolation
 *
 */
class M3DemoMinJerkPosition: public M3TimedState {

   public:
    M3DemoMinJerkPosition(RobotM3 * M3, const char *name = "M3 Demo Minimum Jerk Position"):M3TimedState(M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    static const unsigned int TrajNbPts=4;
    unsigned int TrajPtIdx=0;
    double startTime;
    VM3 TrajPt[TrajNbPts]={VM3(-0.6, 0, 0), VM3(-0.6, 0., 0.2), VM3(-0.6, 0., -0.2), VM3(-0.6, 0, 0.2)};
    double TrajTime[TrajNbPts]={5, 5, 5, 1};
    VM3 Xi, Xf;
    double T;
    float k_i=1.; //Integral gain
};



/**
 * \brief Sampling frequency estimation state.
 *
 */
class M3SamplingEstimationState : public M3TimedState {

   public:
    M3SamplingEstimationState(RobotM3 * M3, const char *name = "M3 Sampling time estimation State"):M3TimedState(M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    unsigned int nb_samples=10000;
    double dts[10000];
    double dX[10000];
    int new_value;
};


#endif
