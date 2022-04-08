/**
 * /file M2DemoState.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-12-09
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef M2DemoSTATE_H_DEF
#define M2DemoSTATE_H_DEF

#include "RobotM2.h"
#include "State.h"

using namespace std;

/**
 * \brief Conversion from a timespec structure to seconds (double)
 *
 */
double timeval_to_sec(struct timespec *ts);

/**
 * \brief Generic state type for used with M2DemoMachine, providing running time and iterations number: been superseeded by default state.
 *
 */
class M2TimedState : public State {
   protected:
    /**
    *  \todo Might be good to make these Const
    *
    */
    RobotM2 *robot;                               /*<!Pointer to state machines robot object*/

    M2TimedState(RobotM2 *M2, const char *name = NULL): State(name), robot(M2){};
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


class M2DemoState : public M2TimedState {

   public:
    M2DemoState(RobotM2 *M2, const char *name = "M2 Test State"):M2TimedState(M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    VM2 qi, Xi, tau;
};



/**
 * \brief Position calibration of M2. Go to the bottom left stops of robot at constant torque for absolute position calibration.
 *
 */
class M2CalibState : public M2TimedState {

   public:
    M2CalibState(RobotM2 *M2, const char *name = "M2 Calib State"):M2TimedState(M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() {return calibDone;}

   private:
     VM2 stop_reached_time;
     bool at_stop[2];
     bool calibDone=false;
};


/**
 * \brief Provide end-effector mass compensation on M2. Mass is controllable through keyboard inputs.
 *
 */
class M2Transparent : public M2TimedState {

   public:
    M2Transparent(RobotM2 *M2, const char *name = "M2 Transparent"):M2TimedState(M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);
};


/**
 * \brief End-effector velocity control through joystick input.
 *
 */
class M2EndEffDemo : public M2TimedState {

   public:
    M2EndEffDemo(RobotM2 *M2, const char *name = "M2 Velocity Control Demo"):M2TimedState(M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);
};

/**
 * \brief End-effector arc circle trajectory (position over velocity)
 *
 */
class M2ArcCircle : public M2TimedState {

   public:
    M2ArcCircle(RobotM2 *M2, const char *name = "M2 Arc Circle"):M2TimedState(M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    bool finished;
    double radius;
    double theta_s;
    double thetaRange;
    double theta;
    int sign;
    double dTheta_t; //Movement target velocity (max of profile) in deg.s-1
    double ddTheta=200; //in deg.s-2
    VM2 centerPt;
    VM2 startingPt;
    double t_init, t_end_accel, t_end_cstt, t_end_decel;
};

/**
 * \brief Path contraint with viscous assistance.
 *
 */
class M2DemoPathState : public M2TimedState {

   public:
    M2DemoPathState(RobotM2 *M2, const char *name = "M2 Demo Path State"):M2TimedState(M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    short int sign(double val) {return (val>0)?1:((val<0)?-1:0); };

   private:
    double k = 600;                 //! Impedance proportional gain (spring)
    double d = 6;                   //! Impedance derivative gain (damper)
    double viscous_assistance=15;   //! Viscous assistance along path
    VM2 Xi;
    VM2 Xf={0.4, 0.3};              //! Path target point
};


/**
 * \brief Point to tpoint position control with min jerk trajectory interpolation
 *
 */
class M2DemoMinJerkPosition: public M2TimedState {

   public:
    M2DemoMinJerkPosition(RobotM2 *M2, const char *name = "M2 Demo Minimum Jerk Position"):M2TimedState(M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    static const unsigned int TrajNbPts=4;
    unsigned int TrajPtIdx=0;
    double startTime;
    VM2 TrajPt[TrajNbPts]={VM2(0.1, 0.1), VM2(0.3, 0.3), VM2(0.4, 0.1), VM2(0.4, 0.4)};
    double TrajTime[TrajNbPts]={5, 3, 2.5, 1.5};
    VM2 Xi, Xf;
    double T;
    float k_i=1.; //Integral gain
};


#endif
