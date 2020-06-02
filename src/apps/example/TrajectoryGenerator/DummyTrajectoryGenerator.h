/**
 * \file DummyTrajectoryGenerator.h
 * \author Justin Fong
 * \brief A trajectory generator to be used for testing purposes
 * \version 0.1
 * \date 2020-05-04
 * 
 * \copyright Copyright (c) 2020
 * 
 */

#ifndef DUMMYTRAJECTORYGENERATOR_H_INCLUDED
#define DUMMYTRAJECTORYGENERATOR_H_INCLUDED

#include <time.h>

#include <cmath>
#include <vector>

#include "DebugMacro.h"

#define deg2rad(deg) ((deg)*M_PI / 180.0)
#define rad2deg(rad) ((rad)*180.0 / M_PI)

/**
 * \brief Enum containing possible trajectory types for DummyTrajectoryGenerator
 * 
 */
enum Trajectory {
    SIT = 0,
    STAND = 1,
};

/**
 * \brief Example Implementation of TrajectoryGenerator. Includes only two trajectories (Sit-to-Stand and Stand-to-sit) for an Exoskeleton
 * 
 */
class DummyTrajectoryGenerator {
   private:
    std::vector<double[2]> endPoints;
    Trajectory currTraj = SIT;
    double trajTime = 2;
    int numJoints = 6;
    double lastProgress = 0;

    /** Parameters associated with Trajectory Progression */
    double currTrajProgress = 0;
    timespec prevTime;

   public:
    DummyTrajectoryGenerator(int NumOfJoints);
    /**
     * \brief Implementation of the initialiseTrajectory method in TrajectoryGenerator
     * 
     * \return true 
     * \return false 
     */
    bool initialiseTrajectory();

    /**
     * \brief Initialise the Trajectory Generator object variables with a traj type (enum defined type)
     * and length of time the trajectory will take.
     * 
     */
    bool initialiseTrajectory(Trajectory traj, double time);

    /**
     * \brief Implementation of the getSetPoint method in TrajectoryGenerator
     * \param time The time corresponding to the point. 
     * 
     * \return vector<double> 
     */
    std::vector<double> getSetPoint(double time);

    /**
     * \brief Check if the trajectory has been completed based on last elapsed time
     * 
     * \return true if trajectory has been completed
     * \return false if trajectory has not been completed
     */
    bool isTrajectoryFinished();
};
#endif