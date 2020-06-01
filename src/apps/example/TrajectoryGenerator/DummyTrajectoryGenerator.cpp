/**
 * @file DummyTrajectoryGenerator.cpp
 * @author Justin Fong
 * @brief 
 * @version 0.1
 * @date 2020-05-04
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "DummyTrajectoryGenerator.h"

double sitting[6] = {90, 90, 90, 90, 0, 0};
double standing[6] = {180, 180, 0, 0, 0, 0};

DummyTrajectoryGenerator::DummyTrajectoryGenerator(int NumOfJoints) {
    numJoints = NumOfJoints;
}

bool DummyTrajectoryGenerator::initialiseTrajectory() {
    currTraj = SIT;
    trajTime = 2;
    return true;
}

/**
     * @brief 
     * 
     */
bool DummyTrajectoryGenerator::initialiseTrajectory(Trajectory traj, double time) {
    currTraj = traj;
    trajTime = time;
    return true;
}

/**
     * @brief Implementation of the getSetPoint method in TrajectoryGenerator
     * @param time The time corresponding to the point. 
     * 
     * @return vector<double> 
     */
std::vector<double> DummyTrajectoryGenerator::getSetPoint(double time) {
    double progress = time / trajTime;
    std::vector<double> angles;

    if (currTraj == SIT) {
        for (int i = 0; i < numJoints; i++) {
            if (progress > 1) {
                angles.push_back(sitting[i]);
            } else {
                angles.push_back(standing[i] + progress * (sitting[i] - standing[i]));
            }
        }
    } else {
        for (int i = 0; i < numJoints; i++) {
            if (progress > 1) {
                angles.push_back(standing[i]);
            } else {
                angles.push_back(sitting[i] + progress * (standing[i] - sitting[i]));
            }
        }
    }
    lastProgress = progress;
    return angles;
}

bool DummyTrajectoryGenerator::isTrajectoryFinished() {
    if (lastProgress > 1.0) {
        return true;
    } else {
        return false;
    }
}