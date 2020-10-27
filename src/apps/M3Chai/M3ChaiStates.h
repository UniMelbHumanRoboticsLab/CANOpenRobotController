/**
 * /file M3ChaiStates.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-07-04
 * \copyright Copyright (c) 2020
 *
 *
 */

#ifndef M3CHAISTATE_H_DEF
#define M3CHAISTATE_H_DEF

#include <time.h>
#include <iostream>
#include <csignal> //For raise()

#include "RobotM3.h"
#include "State.h"
#include "FLNL.h"

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
    long int iterations;
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
     Eigen::Vector3d stop_reached_time;
     bool at_stop[3];
     bool calibDone=false;

};


class M3ChaiWaitForCommunication : public M3TimedState {

   public:
    M3ChaiWaitForCommunication(StateMachine *m, RobotM3 *M3, server *s, const char *name = "M3 Chai 3D wait for communication"):M3TimedState(m, M3, name), chaiServer(s){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
     server *chaiServer;
     double watchDogTime = 0.010; //Watchdog time in s
     Eigen::Vector3d F;
};

/**
 * \brief Receive and apply force commands from Chai and send robot state update update.
 *
 */
class M3ChaiCommunication : public M3TimedState {

   public:
    M3ChaiCommunication(StateMachine *m, RobotM3 *M3, server *s, const char *name = "M3 Chai 3D communication"):M3TimedState(m, M3, name), chaiServer(s){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
     server *chaiServer;
     double watchDogTime = 0.010; //Watchdog time in s
     double lastReceivedTime;
     Eigen::Vector3d F, X, dX;

};


#endif
