/**
 * @file LoopTiming.h
 * @author Justin Fong 
 * @brief Timing functions for logging loops
 *  Initially created for WeRob Abstract - at this stage, may not be long term
 * @version 0.1
 * @date 2020-07-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef LOOPTIMING_H_INCLUDED
#define LOOPTIMING_H_INCLUDED

#include <time.h>

#define MAX_ITERATIONS 10000000

class LoopTiming {
   public:
    void init();
    void tick();
    void end();

   protected:
    struct timespec initTime; /*<! Time of state init */
    double lastTime;          /*<! Time of last during() call (in seconds since state init())*/
    double elapsedTime;       /*<! Time since state init() in seconds*/
    double dt;                /*<! Time between last two during() calls (in seconds)*/
    long int iterations;
    double tickTimes[MAX_ITERATIONS];

   private:
    double timeval_to_sec(struct timespec *ts) {
        return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
    }
};

#endif