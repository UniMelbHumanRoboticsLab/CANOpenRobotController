/**
 * @file LoopTiming.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-07-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "LoopTiming.h"

#include <fstream>
#include <iostream>

#include "stdio.h"

void LoopTiming::init() {
    //Timing
    clock_gettime(CLOCK_MONOTONIC, &initTime);
    lastTime = timeval_to_sec(&initTime);

    iterations = 0;
}

void LoopTiming::tick() {
    //Compute some basic time values
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    double now = timeval_to_sec(&ts);
    elapsedTime = (now - timeval_to_sec(&initTime));
    dt = now - lastTime;
    lastTime = now;
    if (iterations < MAX_ITERATIONS) {
        tickTimes[iterations] = dt;
    } else {
        printf("Max Iterations Exceeded");
    }
    iterations++;
}

void LoopTiming::end() {
    // Save everything to file
    printf("Saving Loop Iterations to File");
    std::ofstream myfile;
    myfile.open("Timing.csv");
    for (int i = 0; i < iterations; i++) {
        myfile << std::dec << tickTimes[i] << std::endl;
    }
    myfile.close();
    printf("LoopTiming::end() \n");
}