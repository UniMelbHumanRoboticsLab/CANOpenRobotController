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

double sitting[6] = {deg2rad(95), deg2rad(95), deg2rad(90), deg2rad(90), 0, 0};
double standing[6] = {0, 0, 0, 0, 0, 0};

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

Eigen::VectorXd DummyTrajectoryGenerator::getSetPoint(double time) {
    double progress = time / trajTime;
    Eigen::VectorXd angles(numJoints);

    if (currTraj == SIT) {
        for (int i = 0; i < numJoints; i++) {
            if (progress > 1) {
                angles(i) = sitting[i];
            } else {
                angles(i) = standing[i] + progress * (sitting[i] - standing[i]);
            }
        }
    } else {
        for (int i = 0; i < numJoints; i++) {
            if (progress > 1) {
                angles(i) = standing[i];
            } else {
                angles(i) = sitting[i] + progress * (standing[i] - sitting[i]);
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
