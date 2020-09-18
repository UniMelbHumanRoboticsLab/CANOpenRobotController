/**
 * /file M1DemoState.h
 * \author Yue Wen adapted from Vincent Crocher
 * \version 0.1
 * \date 2020-08-25
 * \copyright Copyright (c) 2020
 *
 *
 */

#ifndef M1DemoSTATE_H_DEF
#define M1DemoSTATE_H_DEF

#include <time.h>

#include <iostream>

#include "DebugMacro.h"
#include "RobotM1.h"
#include "State.h"

#include "server.h"
#include <csignal> //For raise()

#define IP_ADDRESS "192.168.137.196" //Local IP address to use

using namespace std;

/**
 * \brief Conversion from a timespec structure to seconds (double)
 *
 */
double timeval_to_sec(struct timespec *ts);

/**
 * \brief Generic state type for used with M1DemoMachine, providing running time and iterations number.
 *
 */
class M1TimedState : public State {
   protected:
    /**
    *  \todo Might be good to make these Const
    *
    */
    RobotM1 *robot;                               /*<!Pointer to state machines robot object*/

    M1TimedState(StateMachine *m, RobotM1 *M1, const char *name = NULL): State(m, name), robot(M1){};
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
//        if(dt > 0.005){
//            std::cout << std::setprecision(4) << dt << std::endl;
//        }
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
 * \brief Idle State for the X2DemoMachine
 *
 * State holds until event triggers its exit, and runs initPositionControl on exit
 * Control of transition is independent of this class and is defined in X2DemoMachine.
 *
 */
class IdleState : public State {
    RobotM1 *robot;
    server *chaiServer;

public:
    void entry(void);
    void during(void);
    void exit(void);
    IdleState(StateMachine *m, RobotM1 *exo, const char *name = "Idle state") : State(m, name), robot(exo){};
};

/**
 * \brief Monitoring State for the X2DemoMachine
 *
 * State holds until event triggers its exit, and runs initPositionControl on exit
 * Control of transition is independent of this class and is defined in X2DemoMachine.
 *
 */
class Monitoring : public State {
    RobotM1 *robot;

public:
    void entry(void);
    void during(void);
    void exit(void);
    Monitoring(StateMachine *m, RobotM1 *exo, const char *name = NULL) : State(m, name), robot(exo){};
    long int iterations;
};

class M1DemoState : public M1TimedState {

   public:
    M1DemoState(StateMachine *m, RobotM1 *M1, const char *name = "M1 Test State"):M1TimedState(m, M1, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    JointVec qi;
    EndEffVec Xi;
    bool flag = true;
    double freq;
    double counter;
//    Eigen::Matrix<float, 1, 1> qi, Xi;
};

/**
 * \brief Position tracking of M1
 *
 */
class M1PositionTracking: public M1TimedState {

public:
    M1PositionTracking(StateMachine *m, RobotM1 *M1, const char *name = "M1 Test State"):M1TimedState(m, M1, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);
    void positionControl(void);
    void velocityControl(void);
    void torqueControl(void);
    void admittanceControl(void);

    JointVec q;     //positive dorsi flexion
    JointVec dq;
    JointVec tau;

    double Ks;
    double dt;
    double B;
    double Mass;
    double net_tau;
    double gain;
    double acc;

    EndEffVec Xi;
    bool flag = true;
    double freq;
    double counter;
    bool status = true;
    int mode = 1;
    double magnitude = 20;
//    Eigen::Matrix<float, 1, 1> qi, Xi;
};

///**
// * \brief Position calibration of M1. Go to the bottom left stops of robot at constant torque for absolute position calibration.
// *
// */
//class M1CalibState : public M1TimedState {
//
//   public:
//    M1CalibState(StateMachine *m, RobotM1 *M1, const char *name = "M1 Calib State"):M1TimedState(m, M1, name){};
//
//    void entryCode(void);
//    void duringCode(void);
//    void exitCode(void);
//
//    bool isCalibDone() {return calibDone;}
//
//   private:
//     Eigen::Vector3d stop_reached_time;
//     bool at_stop[3];
//     bool calibDone=false;
//
//};
//
//
///**
// * \brief Provide end-effector mass compensation on M1. Mass is controllable through keyboard inputs.
// *
// */
//class M1MassCompensation : public M1TimedState {
//
//   public:
//    M1MassCompensation(StateMachine *m, RobotM1 *M1, const char *name = "M1 Mass Compensation"):M1TimedState(m, M1, name){};
//
//    void entryCode(void);
//    void duringCode(void);
//    void exitCode(void);
//
//   private:
//     double mass = 0;
//
//};
//
//
///**
// * \brief End-effector velocity control through joystick input.
// *
// */
//class M1EndEffDemo : public M1TimedState {
//
//   public:
//    M1EndEffDemo(StateMachine *m, RobotM1 *M1, const char *name = "M1 Velocity Control Demo"):M1TimedState(m, M1, name){};
//
//    void entryCode(void);
//    void duringCode(void);
//    void exitCode(void);
//};
//
///**
// * \brief Basic impedance control on a static point.
// *
// */
//class M1DemoImpedanceState : public M1TimedState {
//
//   public:
//    M1DemoImpedanceState(StateMachine *m, RobotM1 *M1, const char *name = "M1 Demo Impedance State"):M1TimedState(m, M1, name){};
//
//    void entryCode(void);
//    void duringCode(void);
//    void exitCode(void);
//
//   private:
//    Eigen::Vector3d Xi;
//    double k = 800;
//    double d = 2;
//    bool init=false;
//
//    int nb_samples=10000;
//    double dts[10000];
//    double dX[10000];
//    int new_value;
//};
//
//
///**
// * \brief Sampling frequency estimation state.
// *
// */
//class M1SamplingEstimationState : public M1TimedState {
//
//   public:
//    M1SamplingEstimationState(StateMachine *m, RobotM1 *M1, const char *name = "M1 Sampling time estimation State"):M1TimedState(m, M1, name){};
//
//    void entryCode(void);
//    void duringCode(void);
//    void exitCode(void);
//
//   private:
//    int nb_samples=10000;
//    double dts[10000];
//    double dX[10000];
//    int new_value;
//};

#endif
