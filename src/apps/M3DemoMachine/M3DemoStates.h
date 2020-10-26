/**
 * /file M3DemoState.h
 * \author Vincent Crocher
 * \version 0.3
 * \date 2020-07-27
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef M3DemoSTATE_H_DEF
#define M3DemoSTATE_H_DEF

#include <time.h>
#include <iostream>

#include "RobotM3.h"
#include "State.h"

using namespace std;

/**
 * \brief Conversion from a timespec structure to seconds (double)
 *
 */
double timeval_to_sec(struct timespec *ts);

/**
 * \brief Generic state type for used with M3DemoMachine, providing running time and iterations number.
 *
 */
class M3TimedState : public State {
   protected:
    /**
    *  \todo Might be good to make these Const
    *
    */
    RobotM3 *robot;                               /*<!Pointer to state machines robot object*/

    M3TimedState(StateMachine *m, RobotM3 *M3, const char *name = NULL): State(m, name), robot(M3){};
   private:
    void entry(void) final {
        std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "==================================" << std::endl
        << std::endl;

        //Timing
        clock_gettime(CLOCK_MONOTONIC, &initTime);
        lastTime = timeval_to_sec(&initTime);

        elapsedTime=0;
        iterations=0;

        //Actual state entry
        entryCode();
    };
    void during(void) final {
        //Compute some basic time values
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);

        double now = timeval_to_sec(&ts);
        elapsedTime = (now-timeval_to_sec(&initTime));
        dt = now - lastTime;
        lastTime = now;

        iterations++;

        //Actual state during
        duringCode();
    };
    void exit(void) final {
        exitCode();
        std::cout
        << "==================================" << std::endl
        << "EXIT "<< getName() << std::endl
        << "==================================" << std::endl
        << std::endl;
    };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};


   protected:
    struct timespec initTime;   /*<! Time of state init */
    double lastTime;            /*<! Time of last during() call (in seconds since state init())*/
    double elapsedTime;         /*<! Time since state init() in seconds*/
    double dt;                  /*<! Time between last two during() calls (in seconds)*/
    unsigned long int iterations;
};


class M3DemoState : public M3TimedState {

   public:
    M3DemoState(StateMachine *m, RobotM3 *M3, const char *name = "M3 Test State"):M3TimedState(m, M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    VM3 qi, Xi;
};



/**
 * \brief Position calibration of M3. Go to the bottom left stops of robot at constant torque for absolute position calibration.
 *
 */
class M3CalibState : public M3TimedState {

   public:
    M3CalibState(StateMachine *m, RobotM3 *M3, const char *name = "M3 Calib State"):M3TimedState(m, M3, name){};

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
    M3MassCompensation(StateMachine *m, RobotM3 *M3, const char *name = "M3 Mass Compensation"):M3TimedState(m, M3, name){};

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
    M3EndEffDemo(StateMachine *m, RobotM3 *M3, const char *name = "M3 Velocity Control Demo"):M3TimedState(m, M3, name){};

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
    M3DemoImpedanceState(StateMachine *m, RobotM3 *M3, const char *name = "M3 Demo Impedance State"):M3TimedState(m, M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    VM3 Xi;
    double k = 700;     //! Impedance proportional gain (spring)
    double d = 2;       //! Impedance derivative gain (damper)
    bool init=false;

    unsigned int nb_samples=10000;
    double dts[10000];
    double dX[10000];
    int new_value;
};


/**
 * \brief Path contraint with viscous assistance.
 *
 */
class M3DemoPathState : public M3TimedState {

   public:
    M3DemoPathState(StateMachine *m, RobotM3 *M3, const char *name = "M3 Demo Path State"):M3TimedState(m, M3, name){};

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
 * \brief Point to tpoint position control with min jerk trajectory interpolation
 *
 */
class M3DemoMinJerkPosition: public M3TimedState {

   public:
    M3DemoMinJerkPosition(StateMachine *m, RobotM3 *M3, const char *name = "M3 Demo Minimum Jerk Position"):M3TimedState(m, M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    static const unsigned int TrajNbPts=4;
    unsigned int TrajPtIdx=0;
    double startTime;
    VM3 TrajPt[TrajNbPts]={VM3(-0.55, 0, 0), VM3(-0.7, 0, -0.2), VM3(-0.7, 0.1, -0.1), VM3(-0.7, -0.1, -0.1)};
    double TrajTime[TrajNbPts]={5, 3, 0.5, 0.5};
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
    M3SamplingEstimationState(StateMachine *m, RobotM3 *M3, const char *name = "M3 Sampling time estimation State"):M3TimedState(m, M3, name){};

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
