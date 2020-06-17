/**
 * /file M3TestState.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-06-16
 * \copyright Copyright (c) 2020
 * 
 * 
 */

#ifndef M3TESTSTATE_H_DEF
#define M3TESTSTATE_H_DEF

#include <time.h>
#include <time.h>

#include <iostream>

#include "DebugMacro.h"
#include "RobotM3.h"
#include "State.h"

using namespace std;


double timeval_to_sec(struct timespec *ts);

/**
 * \brief Generic state type for used with M3TestMachine
 * 
 */
class M3State : public State {
   protected:
    /**
    *  \todo Might be good to make these Const
    * 
    */
    RobotM3 *robot;                               /*<!Pointer to state machines robot object*/

   public:
    M3State(StateMachine *m, RobotM3 *M3, const char *name = NULL): State(m, name), robot(M3){};
   
    void entry(void){
        clock_gettime(CLOCK_REALTIME, &initTime);
        lastTime = timeval_to_sec(&initTime);
    };
    void during(void) {
        //Compute some basic time values
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        
        double now = timeval_to_sec(&ts);
        elapsedTime = (now-timeval_to_sec(&initTime));
        dt = now - lastTime;
        lastTime = now;
    };
    void exit(void){};
    
    
   protected:
    struct timespec initTime; /*<! Time of state init */
    double lastTime; /*<! Time of last during() call (in seconds since state init())*/
    double elapsedTime; /*<! Time since state init() in seconds*/
    double dt; /*<! Time between last two during() calls (in seconds)*/
};


class M3TestState : public M3State {
   protected:
    /**
    *  \todo Might be good to make these Const
    * 
    */
    RobotM3 *robot;                               /*<!Pointer to state machines robot object*/

   public:
    M3TestState(StateMachine *m, RobotM3 *M3, const char *name = "M3TestState"):M3State(m, M3, name){};
   
    void entry(void);
    void during(void);
    void exit(void);
};


class M3CalibState : public M3State {
   protected:
    /**
    *  \todo Might be good to make these Const
    * 
    */
    RobotM3 *robot;                               /*<!Pointer to state machines robot object*/

   public:
    M3CalibState(StateMachine *m, RobotM3 *M3, const char *name = "M3CalibState"):M3State(m, M3, name){};
   
    void entry(void);
    void during(void);
    void exit(void);

   private:
     vector<double> vel{0.01, 0.01, 0.01};
     double tau_threshold = 1;
};


#endif