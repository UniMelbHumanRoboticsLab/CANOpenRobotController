/**
 * \file TrajectoryGenerator.h
 * \author Justin Fong
 * \brief Base class of Trajectory Generator. Will always be extended with specific implementations, and is relatively
 * scarce, given the differences expected in the different trajectories.
 * \version 0.1
 * \date 2020-04-24
 * 
 * \copyright Copyright (c) 2020
 * 
 */

#ifndef TRAJECTORYGENERATOR_H_INCLUDED
#define TRAJECTORYGENERATOR_H_INCLUDED

#include <vector>

/**
 * @ingroup Robot
 *  \brief Abstract class which is used to generate trajectorys for a Robot to follow.
 * 
 */
class TrajectoryGenerator {
   public:
    TrajectoryGenerator();

    /**
     * \brief Pure Virtual Function which must be configured to configure the trajectory generator with the 
     *  appropriate parameters. Likely to be overloaded.
     * 
     * \return true if configured successfully
     * \return false  if an error occurred in the configuration.
     */
    virtual bool initialiseTrajectory() = 0;

    /**
     * \brief Get the next step point in the trajectory
     * \param time The time corresponding to the point. 
     * 
     * \return vector<double> 
     */
    virtual std::vector<double> getSetPoint(double time) = 0;
};

#endif