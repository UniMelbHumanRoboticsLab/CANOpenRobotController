/**
 * Author: Dillon Chong
 * Date: 29 Sep 2019
 *
 * Editor: Samuel Wong
 * Date: 09/02/2020
 * compiled using cmd g++ -I eigen -std=c++11 *.cpp -o mai
 *
*/

#include "AlexTrajectoryGenerator.h"

/*Test hardcoded trajectory*/
//** test function
std::string AlexTrajectoryGenerator::printName() {
    return "ALEX TG";
}
/**
 * Initialisation Methods
 */
AlexTrajectoryGenerator::AlexTrajectoryGenerator() {
}

AlexTrajectoryGenerator::AlexTrajectoryGenerator(int NumOfJoints) {
    numJoints = NumOfJoints;
}

bool AlexTrajectoryGenerator::initialiseTrajectory() {
    return true;
}
bool AlexTrajectoryGenerator::initialiseTrajectory(RobotMode mvmnt, std::vector<double> qdeg) {
    // Set the trajectory parameters
    jointspace_state jointSpaceState;
    jointSpaceState.q[0] = deg2rad(qdeg[0]);
    jointSpaceState.q[1] = deg2rad(qdeg[1]);
    jointSpaceState.q[2] = deg2rad(qdeg[2]);
    jointSpaceState.q[3] = deg2rad(qdeg[3]);
    jointSpaceState.q[4] = deg2rad(85);
    jointSpaceState.q[5] = deg2rad(85);
    jointSpaceState.time = 0;
    setTrajectoryParameters(movementTrajMap[mvmnt]);
    generateAndSaveSpline(jointSpaceState);
    return true;
}
bool AlexTrajectoryGenerator::initialiseTrajectory(RobotMode mvmnt, Foot stanceFoot, std::vector<double> qdeg) {
    // Set the trajectory parameters
    jointspace_state jointSpaceState;
    jointSpaceState.q[0] = deg2rad(qdeg[0]);
    jointSpaceState.q[1] = deg2rad(qdeg[1]);
    jointSpaceState.q[2] = deg2rad(qdeg[2]);
    jointSpaceState.q[3] = deg2rad(qdeg[3]);
    jointSpaceState.q[4] = deg2rad(85);
    jointSpaceState.q[5] = deg2rad(85);
    jointSpaceState.time = 0;
    setTrajectoryParameters(movementTrajMap[mvmnt]);
    // by default foot stance is right in movementTrajMap, set to left when needed.ß
    if (stanceFoot == Foot::Left) {
        setTrajectoryStanceLeft();
    }
    generateAndSaveSpline(jointSpaceState);
    return true;
}

bool AlexTrajectoryGenerator::initialiseTrajectory(RobotMode mvmnt, double time) {
    // Set the trajectory parameters

    setTrajectoryParameters(movementTrajMap[mvmnt]);
    /*\todo: have this happen getting fed in the intialPose from the robot*/
    return true;
}
/**
 * Action Methods
 */

/*new getSetPoint: almost identical to old exo calcPosition*/
//get the position at any given time
std::vector<double> AlexTrajectoryGenerator::getSetPoint(time_tt time) {
    /*Intialize data*/
    std::vector<double> angles;
    // Discretise/Sample the spline
    time_tt startTime = trajectoryJointSpline.times.front();
    time_tt endTime = trajectoryJointSpline.times.back();
    // Every sample time, compute the value of q1 to q6 based on the time segment / set of NO_JOINTS polynomials
    int numPoints = trajectoryJointSpline.times.size();
    int numPolynomials = numPoints - 1;
    CubicPolynomial currentPolynomial[NUM_JOINTS];
    for (int polynomial_index = 0; polynomial_index < numPolynomials; polynomial_index++) {
        //cout << "[discretise_spline]: pt " << polynomial_index << ":" << endl;
        //if the jointspaceState time is bounded by the section of spline
        if (time >= trajectoryJointSpline.times.at(polynomial_index) &&
            time <= trajectoryJointSpline.times.at(polynomial_index + 1)) {
            //cout << "time " << time << "\t" << trajectoryJointSpline.times.at(polynomial_index)  << endl;
            for (int i = 0; i < NUM_JOINTS; i++) {
                currentPolynomial[i] = trajectoryJointSpline.polynomials[i].at(polynomial_index);
                angles.push_back(evaluate_cubic_polynomial(currentPolynomial[i], time));
            }
            ////force ankles to be in the final position
            //currentPolynomial[LEFT_ANKLE] = trajectoryJointSpline.polynomials[LEFT_ANKLE].at(numPolynomials - 1);
            //positionArray[LEFT_ANKLE] = evaluate_cubic_polynomial(currentPolynomial[LEFT_ANKLE], endTime);
            //currentPolynomial[RIGHT_ANKLE] = trajectoryJointSpline.polynomials[RIGHT_ANKLE].at(numPolynomials - 1);
            //positionArray[RIGHT_ANKLE] = evaluate_cubic_polynomial(currentPolynomial[RIGHT_ANKLE], endTime);
            //make sure the angles are within boundary
            limit_position_against_angle_boundary(angles);
            return angles;
        }
    }
    //spdlog::debug("Time point outside range")
    //if the time point is outside range
    for (int i = 0; i < NUM_JOINTS; i++) {
        currentPolynomial[i] = trajectoryJointSpline.polynomials[i].at(numPolynomials - 1);
        angles.push_back(evaluate_cubic_polynomial(currentPolynomial[i], endTime));
    }
    //make sure the angles are within boundary
    limit_position_against_angle_boundary(angles);
    //else return previous poly
    return angles;
}
// handle error if reached
/***********************************************************************
Methods to Set Trajectory and Pilot Parameters
***********************************************************************/
void AlexTrajectoryGenerator::setTrajectoryParameters(TrajectoryParameters trajectoryParameter) {
    this->trajectoryParameter = trajectoryParameter;
    printTrajectoryParameters();
}

void AlexTrajectoryGenerator::setPilotParameters(PilotParameters pilotParameters) {
    this->pilotParameters = pilotParameters;
    spdlog::debug("Pilot Paramaters set");
}

/**
 * Debugging Methods
 */
void AlexTrajectoryGenerator::printTrajectoryParameters() {
    //std::cout << "Step height:" << ((AlexTrajectoryGenerator *)trajectoryGenerator)->trajectoryParameter.step_height << std::endl;
    //std::cout << "Slope angle: " << ((AlexTrajectoryGenerator *)trajectoryGenerator)->trajectoryParameter.slope_angle << std::endl;

    std::cout << "Step Type: " << StepTypeToString[trajectoryParameter.stepType] << std::endl;
}
/*
void AlexTrajectoryGenerator::setTrajectoryParameters(time_tt step_duration, double step_height, double step_length, double hip_height_slack, double torso_forward_angle, double swing_ankle_down_angle,
                                                      Foot stance_foot, StepType movement, double seat_height, double step_end_height, double slope_angle, bool left_foot_on_tilt, bool right_foot_on_tilt) {
    trajectoryParameter.step_duration = step_duration;
    trajectoryParameter.step_height = step_height;
    trajectoryParameter.step_length = step_length;
    trajectoryParameter.hip_height_slack = hip_height_slack;
    trajectoryParameter.torso_forward_angle = torso_forward_angle;
    trajectoryParameter.swing_ankle_down_angle = swing_ankle_down_angle;
    trajectoryParameter.stance_foot = stance_foot;
    trajectoryParameter.stepType = movement;
    trajectoryParameter.seat_height = seat_height;
    trajectoryParameter.step_end_height = step_end_height;
    trajectoryParameter.slope_angle = slope_angle;
    trajectoryParameter.left_foot_on_tilt = left_foot_on_tilt;
    trajectoryParameter.right_foot_on_tilt = right_foot_on_tilt;
}

void AlexTrajectoryGenerator::setPilotParameters(double lowerleg_length, double upperleg_length, double ankle_height, double foot_length,
                                                 double hip_width, double torso_length, double buttocks_height) {
    pilotParameters.lowerleg_length = lowerleg_length;
    pilotParameters.upperleg_length = upperleg_length;
    pilotParameters.ankle_height = ankle_height;
    pilotParameters.foot_length = foot_length;
    pilotParameters.hip_width = hip_width;
    pilotParameters.torso_length = torso_length;
    pilotParameters.buttocks_height = buttocks_height;
}*/

/**********************************************************************

Functions for taskspace and joint space conversion

**********************************************************************/
std::vector<taskspace_state> AlexTrajectoryGenerator::generate_key_taskspace_states(taskspace_state initialTaskspaceState,
                                                                                    const TrajectoryParameters &trajectoryParameters, const PilotParameters &pilotParameters) {
    std::vector<taskspace_state> keyTaskspaceStates;
    double deltaFootDistance = 0.05;

    // Sit-stand
    if (trajectoryParameters.stepType == StepType::Stand) {
        double seatHeight = trajectoryParameters.seat_height;
        double buttocksHeight = pilotParameters.buttocks_height;
        {
            taskspace_state state0 = initialTaskspaceState;  // init
            state0.time = 0.4;
            state0.left_ankle_position.x = 0;
            state0.right_ankle_position.x = 0;
            state0.left_ankle_position.z = pilotParameters.ankle_height;
            state0.right_ankle_position.z = pilotParameters.ankle_height;
            state0.hip_position.x = -sin(deg2rad(85)) * (pilotParameters.upperleg_length);
            state0.hip_position.z =
                pilotParameters.lowerleg_length + pilotParameters.ankle_height + cos(deg2rad(85)) * pilotParameters.upperleg_length;
            state0.torso_forward_angle = trajectoryParameters.torso_forward_angle * 5;
            state0.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(state0);
        }
        {
            taskspace_state state1 = initialTaskspaceState;  // init
            state1.time = 0.6;
            state1.left_ankle_position.x = 0;
            state1.right_ankle_position.x = 0;
            state1.left_ankle_position.z = pilotParameters.ankle_height;
            state1.right_ankle_position.z = pilotParameters.ankle_height;
            state1.hip_position.x = -sin(deg2rad(60)) * (pilotParameters.upperleg_length);
            state1.hip_position.z =
                pilotParameters.lowerleg_length + pilotParameters.ankle_height + cos(deg2rad(60)) * pilotParameters.upperleg_length;
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle * 5;
            state1.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(state1);
        }
        {
            taskspace_state state2 = initialTaskspaceState;  // init
            state2.time = 0.80;
            state2.left_ankle_position.x = 0;
            state2.right_ankle_position.x = 0;
            state2.left_ankle_position.z = pilotParameters.ankle_height;
            state2.right_ankle_position.z = pilotParameters.ankle_height;
            state2.hip_position.x = -sin(deg2rad(15)) * (pilotParameters.upperleg_length);
            state2.hip_position.z =
                pilotParameters.lowerleg_length + pilotParameters.ankle_height + cos(deg2rad(15)) * pilotParameters.upperleg_length;
            state2.torso_forward_angle = trajectoryParameters.torso_forward_angle * 3;
            state2.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(state2);
        }
        {
            taskspace_state stateEnd = initialTaskspaceState;  // init
            stateEnd.time = 1;
            stateEnd.left_ankle_position.x = 0;
            stateEnd.right_ankle_position.x = 0;
            stateEnd.left_ankle_position.z = pilotParameters.ankle_height;
            stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
            stateEnd.hip_position.x =
                0;
            stateEnd.hip_position.z =
                pilotParameters.ankle_height + pilotParameters.upperleg_length + pilotParameters.lowerleg_length - trajectoryParameters.hip_height_slack;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle * 1;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }

    // Stand-sit
    if (trajectoryParameters.stepType == StepType::Sit) {
        double seatHeight = trajectoryParameters.seat_height;
        double buttocksHeight = pilotParameters.buttocks_height;

        {
            taskspace_state state1 = initialTaskspaceState;  // init
            state1.time = 0.8;
            state1.left_ankle_position.x = 0;
            state1.right_ankle_position.x = 0;
            state1.left_ankle_position.z = pilotParameters.ankle_height;
            state1.right_ankle_position.z = pilotParameters.ankle_height;

            state1.hip_position.x =
                -0.5 * (pilotParameters.upperleg_length);
            state1.hip_position.z =
                (seatHeight + buttocksHeight) + 0.6 * (pilotParameters.ankle_height + pilotParameters.upperleg_length + pilotParameters.lowerleg_length - trajectoryParameters.hip_height_slack - (seatHeight + buttocksHeight));  // how far away from seat height
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle * 5;
            state1.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(state1);
        }

        {
            taskspace_state stateEnd = initialTaskspaceState;  // init
            stateEnd.time = 1.0;
            stateEnd.left_ankle_position.x = 0;
            stateEnd.right_ankle_position.x = 0;
            stateEnd.left_ankle_position.z = pilotParameters.ankle_height;
            stateEnd.right_ankle_position.z = pilotParameters.ankle_height;

            stateEnd.hip_position.x =
                -1 * (pilotParameters.upperleg_length);
            stateEnd.hip_position.z =
                (seatHeight + buttocksHeight);
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle * 5;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }
    // Sitting
    if (trajectoryParameters.stepType == StepType::Sitting) {
        double seatHeight = trajectoryParameters.seat_height;
        double buttocksHeight = pilotParameters.buttocks_height;

        {
            taskspace_state stateEnd = initialTaskspaceState;  // init
            stateEnd.time = 1;
            stateEnd.left_ankle_position.x = 0;
            stateEnd.right_ankle_position.x = 0;
            stateEnd.left_ankle_position.z = pilotParameters.ankle_height;
            stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
            stateEnd.hip_position.x =
                -1 * (pilotParameters.upperleg_length);
            stateEnd.hip_position.z =
                seatHeight + buttocksHeight;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }

    // Walk
    if (trajectoryParameters.stepType == StepType::Walk) {
        Foot inferredStanceFoot = ((initialTaskspaceState.left_ankle_position.x > initialTaskspaceState.right_ankle_position.x)
                                       ? Foot::Left
                                       : Foot::Right);
        if (initialTaskspaceState.stance_foot != inferredStanceFoot)
            std::cout << "[generate_key_taskspace_states] Stance foot isn't in front of swing foot!?!!" << std::endl;
        double ankleDistance = abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x);
        double stepDisplacement = ankleDistance + trajectoryParameters.step_length;
        double legLengthSlacked = pilotParameters.lowerleg_length + pilotParameters.upperleg_length - trajectoryParameters.hip_height_slack;
        double hipHeight = pilotParameters.ankle_height + legLengthSlacked;
        double stanceFoot_x = std::max(initialTaskspaceState.left_ankle_position.x, initialTaskspaceState.right_ankle_position.x);

        // TrajectoryGenerator forming algorithm here
        //  All key states except initial state

        // Middle state
        {
            taskspace_state state1 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance;
                state1.left_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height;
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.right_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.right_ankle_position.x;
            } else {
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance;
                state1.right_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height;
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.left_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.left_ankle_position.x;
            }
            state1.hip_position.z = hipHeight;  // probably should deal with rounding error that makes hipheight slightly larger than leglength?
            state1.time = 0.4;
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state1.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state1);
        }

        {
            taskspace_state state2 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                //if stand together
                if (trajectoryParameters.step_length < 0.1) {
                    state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance + 0.25;
                } else {
                    state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance + sqrt(pow(legLengthSlacked, 2.0) - pow(legLengthSlacked - trajectoryParameters.step_length * trajectoryParameters.step_height, 2.0));
                }
                state2.left_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height;
                state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state2.right_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length / 3.0;
            } else {
                //if stand together
                if (trajectoryParameters.step_length < 0.1) {
                    state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + 0.25;
                } else {
                    state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + sqrt(pow(legLengthSlacked, 2.0) - pow(legLengthSlacked - trajectoryParameters.step_length * trajectoryParameters.step_height, 2.0));
                }
                state2.right_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height;
                state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state2.left_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length / 3.0;
            }
            state2.hip_position.z = pilotParameters.ankle_height + 0.999 * (sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length / 3.0, 2.0)));
            state2.time = 0.7;
            state2.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state2.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state2);
        }

        // Final state
        {
            taskspace_state stateEnd = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                stateEnd.left_ankle_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length;
                stateEnd.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                stateEnd.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length * 1.6 / 3.0;
            }

            else {
                stateEnd.right_ankle_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length;
                stateEnd.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                stateEnd.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length * 1.6 / 3.0;
            }
            stateEnd.left_ankle_position.z = pilotParameters.ankle_height;
            stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
            //if stand together
            if (trajectoryParameters.step_length < 0.1) {
                stateEnd.hip_position.z = pilotParameters.ankle_height + legLengthSlacked;
            } else {
                stateEnd.hip_position.z = pilotParameters.ankle_height + 0.999 * (sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length * 1.6 / 3.0, 2.0)));
            }
            stateEnd.time = 1;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }

    // Back
    if (trajectoryParameters.stepType == StepType::Back)

    {
        Foot backwardStanceFoot = ((initialTaskspaceState.left_ankle_position.x > initialTaskspaceState.right_ankle_position.x)
                                       ? Foot::Right
                                       : Foot::Left);
        if (initialTaskspaceState.stance_foot != backwardStanceFoot)
            std::cout << "[generate_key_taskspace_states] Backward stance foot isn't at the back!?!!" << std::endl;
        double ankleDistance = abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x);
        double stepDisplacement = ankleDistance + trajectoryParameters.step_length;
        double legLengthSlacked = pilotParameters.lowerleg_length + pilotParameters.upperleg_length - trajectoryParameters.hip_height_slack;
        double hipHeight = pilotParameters.ankle_height + legLengthSlacked;

        // TrajectoryGenerator forming algorithm here
        //  All key states except initial state

        // Middle state
        {
            taskspace_state state1 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Left)
            //||	abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x - ankleDistance;
                state1.right_ankle_position.z = initialTaskspaceState.right_ankle_position.z + trajectoryParameters.step_height;
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.left_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.left_ankle_position.x;
            } else {
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x - ankleDistance;
                state1.left_ankle_position.z = initialTaskspaceState.left_ankle_position.z + trajectoryParameters.step_height;
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.right_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.right_ankle_position.x;
            }
            state1.hip_position.z = hipHeight;  // probably should deal with rounding error that makes hipheight slightly larger than leglength?
            state1.time = 0.5;
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state1.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state1);
        }
        //Post Middle step
        {
            taskspace_state state2 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Left) {
                //if stand together
                if (trajectoryParameters.step_length < 0.1) {
                    state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x - ankleDistance + 0.1;
                } else {
                    state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x - ankleDistance - trajectoryParameters.step_length / 2.0;
                }
                state2.right_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height;
                state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state2.left_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.left_ankle_position.x - trajectoryParameters.step_length / 3.0;
            } else {
                //if stand together
                if (trajectoryParameters.step_length < 0.1) {
                    state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x - ankleDistance + 0.1;
                } else {
                    state2.left_ankle_position.x = initialTaskspaceState.right_ankle_position.x - ankleDistance - trajectoryParameters.step_length / 2.0;
                }
                state2.left_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height;
                state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state2.right_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.right_ankle_position.x - trajectoryParameters.step_length / 3.0;
            }
            state2.hip_position.z = pilotParameters.ankle_height + 0.999 * (sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length / 3.0, 2.0)));
            state2.time = 0.7;
            state2.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state2.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state2);
        }
        // Final state
        {
            taskspace_state stateEnd = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Left)
            //||	abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                stateEnd.right_ankle_position.x = initialTaskspaceState.left_ankle_position.x - trajectoryParameters.step_length;
                stateEnd.hip_position.x = initialTaskspaceState.left_ankle_position.x - trajectoryParameters.step_length / 2.0;
            } else {
                stateEnd.left_ankle_position.x = initialTaskspaceState.right_ankle_position.x - trajectoryParameters.step_length;
                stateEnd.hip_position.x = initialTaskspaceState.right_ankle_position.x - trajectoryParameters.step_length / 2.0;
            }
            stateEnd.left_ankle_position.z = pilotParameters.ankle_height;
            stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
            stateEnd.hip_position.z = pilotParameters.ankle_height + sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length / 2.0, 2.0));
            stateEnd.time = 1;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }
    //stairs
    if (trajectoryParameters.stepType == StepType::Stair) {
        Foot inferredStanceFoot = ((initialTaskspaceState.left_ankle_position.x > initialTaskspaceState.right_ankle_position.x)
                                       ? Foot::Left
                                       : Foot::Right);
        if (initialTaskspaceState.stance_foot != inferredStanceFoot)
            std::cout << "[generate_key_taskspace_states] Stance foot isn't in front of swing foot!?!!" << std::endl;
        double ankleDistance = abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x);
        double heightDistance = abs(initialTaskspaceState.left_ankle_position.z - initialTaskspaceState.right_ankle_position.z);
        double stepDisplacement = ankleDistance + trajectoryParameters.step_length;
        double legLengthSlacked = pilotParameters.lowerleg_length + pilotParameters.upperleg_length - trajectoryParameters.hip_height_slack;
        double hipHeight = pilotParameters.ankle_height + legLengthSlacked;
        double stanceFoot_x = std::max(initialTaskspaceState.left_ankle_position.x, initialTaskspaceState.right_ankle_position.x);

        // TrajectoryGenerator forming algorithm here
        //  All key states except initial state

        // Middle state
        {
            taskspace_state state1 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance;
                state1.left_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height;
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.right_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.right_ankle_position.x;
            } else {
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance;
                state1.right_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height;
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.left_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.left_ankle_position.x;
            }
            state1.hip_position.z = hipHeight;  // probably should deal with rounding error that makes hipheight slightly larger than leglength?
            state1.time = 0.4;
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state1.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state1);
        }

        {
            taskspace_state state2 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                //A forward left swing
                state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance + 0.7 * sqrt(pow(legLengthSlacked, 2.0) - pow(legLengthSlacked - trajectoryParameters.step_length * trajectoryParameters.step_height, 2.0));
                state2.left_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height + trajectoryParameters.step_end_height;
                state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state2.right_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length * 0.0 / 3.0;
            } else {
                //if stand together
                state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + 0.35;
                state2.right_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height;
                state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state2.left_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length * 0.0 / 3.0;
            }
            state2.hip_position.z = pilotParameters.ankle_height + 0.999 * (sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length * 0.0 / 3.0, 2.0)));
            state2.time = 0.7;
            state2.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state2.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state2);
        }

        // Final state
        {
            taskspace_state stateEnd = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                stateEnd.left_ankle_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length;
                stateEnd.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                stateEnd.hip_position.x = initialTaskspaceState.right_ankle_position.x;
                stateEnd.left_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_end_height;
                stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.hip_position.z = pilotParameters.ankle_height + legLengthSlacked;
            }

            else {
                stateEnd.right_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                stateEnd.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                stateEnd.hip_position.x = initialTaskspaceState.left_ankle_position.x;
                stateEnd.left_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.hip_position.z = pilotParameters.ankle_height + legLengthSlacked;
            }
            stateEnd.time = 1;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }
    //down stairs
    if (trajectoryParameters.stepType == StepType::DownStair) {
        Foot inferredStanceFoot = ((initialTaskspaceState.left_ankle_position.x > initialTaskspaceState.right_ankle_position.x)
                                       ? Foot::Left
                                       : Foot::Right);
        if (initialTaskspaceState.stance_foot != inferredStanceFoot)
            std::cout << "[generate_key_taskspace_states] Stance foot isn't in front of swing foot!?!!" << std::endl;
        double ankleDistance = abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x);
        double heightDistance = abs(initialTaskspaceState.left_ankle_position.z - initialTaskspaceState.right_ankle_position.z);
        double stepDisplacement = ankleDistance + trajectoryParameters.step_length;
        double legLengthSlacked = pilotParameters.lowerleg_length + pilotParameters.upperleg_length - trajectoryParameters.hip_height_slack;
        double hipHeight = pilotParameters.ankle_height + legLengthSlacked;
        double stanceFoot_x = std::max(initialTaskspaceState.left_ankle_position.x, initialTaskspaceState.right_ankle_position.x);

        // TrajectoryGenerator forming algorithm here
        //  All key states except initial state

        // Middle state
        {
            taskspace_state state1 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
              //A backward left swing
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x - ankleDistance;
                state1.left_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height * .85;
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.right_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.right_ankle_position.x;
            } else {
              //A backward right swing
                // based on eye balling state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x - ankleDistance - 0.1; // with 0.2 seems to just clear the step, to be tested
                // state1.right_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height * 1.05;
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x - ankleDistance - 0.2; // with 0.2 seems to just clear the step, to be tested
                state1.right_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height * 0.85;
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.left_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.left_ankle_position.x;
            }
            state1.hip_position.z = hipHeight;  // probably should deal with rounding error that makes hipheight slightly larger than leglength?
            state1.time = 0.5;
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state1.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state1);
        }

        // Final state
        {
            taskspace_state stateEnd = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
              //A backward left swing, ending in a left foot backward/right foot forward state
                // based on 16/10 eye balling stateEnd.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x - stepDisplacement + 0.05;
                stateEnd.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x - stepDisplacement;
                stateEnd.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                stateEnd.hip_position.x = stateEnd.left_ankle_position.x;
                stateEnd.left_ankle_position.z = pilotParameters.ankle_height - trajectoryParameters.step_end_height;
                stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.hip_position.z = stateEnd.left_ankle_position.z + legLengthSlacked;
            }

            else {
              //A backward right swing, ending in a step tgt
                stateEnd.right_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                stateEnd.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                stateEnd.hip_position.x = stateEnd.right_ankle_position.x;
                stateEnd.left_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.hip_position.z = stateEnd.right_ankle_position.z + legLengthSlacked;
            }
            stateEnd.time = 1;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }

    //Ramp up
    if (trajectoryParameters.stepType == StepType::RampUp) {
        Foot inferredStanceFoot = ((initialTaskspaceState.left_ankle_position.x > initialTaskspaceState.right_ankle_position.x)
            ? Foot::Left
            : Foot::Right);
        if (initialTaskspaceState.stance_foot != inferredStanceFoot)
            std::cout << "[generate_key_taskspace_states] Stance foot isn't in front of swing foot!?!!" << std::endl;
        double ankleDistance = abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x);
        double stepDisplacement = ankleDistance + trajectoryParameters.step_length;
        double legLengthSlacked = pilotParameters.lowerleg_length + pilotParameters.upperleg_length - trajectoryParameters.hip_height_slack;
        double hipHeight = pilotParameters.ankle_height + legLengthSlacked;
        double stanceFoot_x = std::max(initialTaskspaceState.left_ankle_position.x, initialTaskspaceState.right_ankle_position.x);

        // TrajectoryGenerator forming algorithm here
        //  All key states except initial state

        // Middle state
        {
            taskspace_state state1 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
                //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance;
                state1.left_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height + trajectoryParameters.step_length * sin(trajectoryParameters.swing_ankle_down_angle);
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.right_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.right_ankle_position.x;
            }
            else {
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance;
                state1.right_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height + trajectoryParameters.step_length * sin(trajectoryParameters.swing_ankle_down_angle);
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.left_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.left_ankle_position.x;
            }
            state1.hip_position.z = hipHeight;  // probably should deal with rounding error that makes hipheight slightly larger than leglength?
            state1.time = 0.4;
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle * 0;
            state1.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state1);
        }

        {
            taskspace_state state2 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
                //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                //if stand together
                if (trajectoryParameters.step_length < 0.1) {
                    state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance + 0.3;
                }
                else {
                    state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance + sqrt(pow(legLengthSlacked, 2.0) - pow(legLengthSlacked - trajectoryParameters.step_length * trajectoryParameters.step_height, 2.0));
                }
                state2.left_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height + trajectoryParameters.step_end_height;
                state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state2.right_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length / 3.0;
            }
            else {
                //if stand together
                if (trajectoryParameters.step_length < 0.1) {
                    state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + 0.3;
                }
                else {
                    state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + sqrt(pow(legLengthSlacked, 2.0) - pow(legLengthSlacked - trajectoryParameters.step_length * trajectoryParameters.step_height, 2.0));
                }
                state2.right_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height + trajectoryParameters.step_end_height;
                state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state2.left_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length / 3.0;
            }
            state2.hip_position.z = pilotParameters.ankle_height + 0.999 * (sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length / 3.0, 2.0)));
            state2.time = 0.70;
            state2.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state2.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state2);
        }
        // Final state
        {
            taskspace_state stateEnd = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
                //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                stateEnd.left_ankle_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length;
                stateEnd.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                stateEnd.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length / 2.0;
                stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.left_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_end_height;
            }

            else {
                stateEnd.right_ankle_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length;
                stateEnd.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                stateEnd.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length / 2.0;
                stateEnd.left_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.right_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_end_height;
            }

            //if stand together
            if (trajectoryParameters.step_length < 0.1) {
                stateEnd.hip_position.z = pilotParameters.ankle_height + legLengthSlacked;
            }
            else {
                stateEnd.hip_position.z = pilotParameters.ankle_height + 0.999 * (sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length /2.0, 2.0)));
            }
            stateEnd.time = 1;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }

    //Ramp down

    if (trajectoryParameters.stepType == StepType::RampDown) {
        Foot inferredStanceFoot = ((initialTaskspaceState.left_ankle_position.x > initialTaskspaceState.right_ankle_position.x)
            ? Foot::Left
            : Foot::Right);
        if (initialTaskspaceState.stance_foot != inferredStanceFoot)
            std::cout << "[generate_key_taskspace_states] Stance foot isn't in front of swing foot!?!!" << std::endl;
        double ankleDistance = abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x);
        double stepDisplacement = ankleDistance + trajectoryParameters.step_length;
        double legLengthSlacked = pilotParameters.lowerleg_length + pilotParameters.upperleg_length - trajectoryParameters.hip_height_slack;
        double hipHeight = pilotParameters.ankle_height + legLengthSlacked;
        double stanceFoot_x = std::max(initialTaskspaceState.left_ankle_position.x, initialTaskspaceState.right_ankle_position.x);

        // TrajectoryGenerator forming algorithm here
        //  All key states except initial state

        // Middle state
        {
            taskspace_state state1 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
                //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance;
                state1.left_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height - trajectoryParameters.step_length * sin(trajectoryParameters.swing_ankle_down_angle);
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.right_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.right_ankle_position.x;
            }
            else {
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance;
                state1.right_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height - trajectoryParameters.step_length * sin(trajectoryParameters.swing_ankle_down_angle);
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.left_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.left_ankle_position.x;
            }
            state1.hip_position.z = hipHeight;  // probably should deal with rounding error that makes hipheight slightly larger than leglength?
            state1.time = 0.4;
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle * 0;
            state1.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state1);
        }

        {
            taskspace_state state2 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
                //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                //if stand together
                if (trajectoryParameters.step_length < 0.1) {
                    state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance + 0.3;
                }
                else {
                    state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance + sqrt(pow(legLengthSlacked, 2.0) - pow(legLengthSlacked - trajectoryParameters.step_length * trajectoryParameters.step_height, 2.0));
                }
                state2.left_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height + trajectoryParameters.step_end_height;
                state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state2.right_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length / 3.0;
            }
            else {
                //if stand together
                if (trajectoryParameters.step_length < 0.1) {
                    state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + 0.3;
                }
                else {
                    state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + sqrt(pow(legLengthSlacked, 2.0) - pow(legLengthSlacked - trajectoryParameters.step_length * trajectoryParameters.step_height, 2.0));
                }
                state2.right_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height + trajectoryParameters.step_end_height;
                state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state2.left_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length / 3.0;
            }
            state2.hip_position.z = pilotParameters.ankle_height + 0.999 * (sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length / 3.0, 2.0)));
            state2.time = 0.70;
            state2.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state2.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state2);
        }
        // Final state
        {
            taskspace_state stateEnd = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
                //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                stateEnd.left_ankle_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length;
                stateEnd.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                stateEnd.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length / 2.0;
                stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.left_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_end_height;
            }

            else {
                stateEnd.right_ankle_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length;
                stateEnd.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                stateEnd.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length / 2.0;
                stateEnd.left_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.right_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_end_height;
            }

            //if stand together
            if (trajectoryParameters.step_length < 0.1) {
                stateEnd.hip_position.z = pilotParameters.ankle_height + legLengthSlacked;
            }
            else {
                stateEnd.hip_position.z = pilotParameters.ankle_height + 0.999 * (sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length / 2.0, 2.0)));
            }
            stateEnd.time = 1;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }

    //Tilt up
    if (trajectoryParameters.stepType == StepType::TiltUp) {
        Foot inferredStanceFoot = ((initialTaskspaceState.left_ankle_position.x > initialTaskspaceState.right_ankle_position.x)
                                       ? Foot::Left
                                       : Foot::Right);
        if (initialTaskspaceState.stance_foot != inferredStanceFoot)
            std::cout << "[generate_key_taskspace_states] Stance foot isn't in front of swing foot!?!!" << std::endl;
        double ankleDistance = abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x);
        double heightDistance = abs(initialTaskspaceState.left_ankle_position.z - initialTaskspaceState.right_ankle_position.z);
        double legLengthSlacked = pilotParameters.lowerleg_length + pilotParameters.upperleg_length - trajectoryParameters.hip_height_slack;
        double hipHeight = pilotParameters.ankle_height + legLengthSlacked;
        double stanceFoot_x = std::max(initialTaskspaceState.left_ankle_position.x, initialTaskspaceState.right_ankle_position.x);
        // Middle state
        {
            taskspace_state state1 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance;
                state1.left_ankle_position.z = initialTaskspaceState.left_ankle_position.z + trajectoryParameters.step_height + heightDistance + trajectoryParameters.step_length * sin(trajectoryParameters.swing_ankle_down_angle);
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.right_ankle_position.z = initialTaskspaceState.right_ankle_position.z;
                state1.hip_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.hip_position.z = initialTaskspaceState.right_ankle_position.z + hipHeight - pilotParameters.ankle_height;
            } else {
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance;
                state1.right_ankle_position.z = initialTaskspaceState.right_ankle_position.z + trajectoryParameters.step_height + heightDistance + trajectoryParameters.step_length * sin(trajectoryParameters.swing_ankle_down_angle);
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.left_ankle_position.z = initialTaskspaceState.left_ankle_position.z;
                state1.hip_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.hip_position.z = initialTaskspaceState.left_ankle_position.z + hipHeight - pilotParameters.ankle_height;
            }
            state1.time = 0.5;
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state1.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, but zero it out anyways
            keyTaskspaceStates.push_back(state1);
        }
        // Final state
        {
            taskspace_state stateEnd = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                stateEnd.left_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + trajectoryParameters.step_length * cos(trajectoryParameters.slope_angle);
                stateEnd.left_ankle_position.z = initialTaskspaceState.right_ankle_position.z + heightDistance + trajectoryParameters.step_length * sin(trajectoryParameters.slope_angle);
                stateEnd.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length / 2.0;
                stateEnd.hip_position.z = initialTaskspaceState.right_ankle_position.z + hipHeight - pilotParameters.ankle_height;
            }

            else {
                stateEnd.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + trajectoryParameters.step_length * cos(trajectoryParameters.slope_angle);
                stateEnd.right_ankle_position.z = initialTaskspaceState.right_ankle_position.z + heightDistance + trajectoryParameters.step_length * sin(trajectoryParameters.slope_angle);
                stateEnd.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length / 2.0;
                stateEnd.hip_position.z = initialTaskspaceState.left_ankle_position.z + hipHeight - pilotParameters.ankle_height;
            }
            stateEnd.time = 1;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }

    //Tilt down
    if (trajectoryParameters.stepType == StepType::TiltDown) {
        Foot inferredStanceFoot = ((initialTaskspaceState.left_ankle_position.x > initialTaskspaceState.right_ankle_position.x)
            ? Foot::Left
            : Foot::Right);
        if (initialTaskspaceState.stance_foot != inferredStanceFoot)
            std::cout << "[generate_key_taskspace_states] Stance foot isn't in front of swing foot!?!!" << std::endl;
        double ankleDistance = abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x);
        double heightDistance = abs(initialTaskspaceState.left_ankle_position.z - initialTaskspaceState.right_ankle_position.z);
        double legLengthSlacked = pilotParameters.lowerleg_length + pilotParameters.upperleg_length - trajectoryParameters.hip_height_slack;
        double hipHeight = pilotParameters.ankle_height + legLengthSlacked;
        double stanceFoot_x = std::max(initialTaskspaceState.left_ankle_position.x, initialTaskspaceState.right_ankle_position.x);

        // Middle state
        {
            taskspace_state state1 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            {
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance;
                state1.left_ankle_position.z = initialTaskspaceState.left_ankle_position.z + trajectoryParameters.step_height + heightDistance - trajectoryParameters.step_length * sin(trajectoryParameters.swing_ankle_down_angle);
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.right_ankle_position.z = initialTaskspaceState.right_ankle_position.z;
                state1.hip_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.hip_position.z = initialTaskspaceState.right_ankle_position.x + hipHeight - pilotParameters.ankle_height;
            }
            else {
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance;
                state1.right_ankle_position.z = initialTaskspaceState.right_ankle_position.z + trajectoryParameters.step_height + heightDistance - trajectoryParameters.step_length * sin(trajectoryParameters.swing_ankle_down_angle);
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.left_ankle_position.z = initialTaskspaceState.left_ankle_position.z;
                state1.hip_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.hip_position.z = initialTaskspaceState.left_ankle_position.z + hipHeight - pilotParameters.ankle_height;
            }

            state1.time = 0.5;
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state1.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(state1);

        }

        // Final state
        {
            taskspace_state stateEnd = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            {
                stateEnd.left_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + trajectoryParameters.step_length * cos(trajectoryParameters.slope_angle);
                stateEnd.left_ankle_position.z = initialTaskspaceState.right_ankle_position.z + heightDistance + trajectoryParameters.step_length * sin(trajectoryParameters.slope_angle);
                stateEnd.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length / 2.0;
                stateEnd.hip_position.z = initialTaskspaceState.right_ankle_position.z + hipHeight - pilotParameters.ankle_height;
            }
            else {
                stateEnd.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + trajectoryParameters.step_length * cos(trajectoryParameters.slope_angle);
                stateEnd.left_ankle_position.z = initialTaskspaceState.right_ankle_position.z + heightDistance + trajectoryParameters.step_length * sin(trajectoryParameters.slope_angle);
                stateEnd.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length / 2.0;
                stateEnd.hip_position.z = initialTaskspaceState.left_ankle_position.z + hipHeight - pilotParameters.ankle_height;
            }

            stateEnd.time = 1;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }

    }


    //Uneven ground, basically a mixture of walk and stair
    if (trajectoryParameters.stepType == StepType::Uneven) {
        Foot inferredStanceFoot = ((initialTaskspaceState.left_ankle_position.x > initialTaskspaceState.right_ankle_position.x)
                                       ? Foot::Left
                                       : Foot::Right);
        if (initialTaskspaceState.stance_foot != inferredStanceFoot)
            std::cout << "[generate_key_taskspace_states] Stance foot isn't in front of swing foot!?!!" << std::endl;
        double ankleDistance = abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x);
        double stepDisplacement = ankleDistance + trajectoryParameters.step_length;
        double legLengthSlacked = pilotParameters.lowerleg_length + pilotParameters.upperleg_length - trajectoryParameters.hip_height_slack;
        double hipHeight = pilotParameters.ankle_height + legLengthSlacked;
        double stanceFoot_x = std::max(initialTaskspaceState.left_ankle_position.x, initialTaskspaceState.right_ankle_position.x);

        // TrajectoryGenerator forming algorithm here
        //  All key states except initial state

        // Middle state
        {
            taskspace_state state1 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance;
                state1.left_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height;
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state1.right_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.right_ankle_position.x;
            } else {
                state1.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance;
                state1.right_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_height;
                state1.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state1.left_ankle_position.z = pilotParameters.ankle_height;
                state1.hip_position.x = initialTaskspaceState.left_ankle_position.x;
            }
            state1.hip_position.z = hipHeight;  // probably should deal with rounding error that makes hipheight slightly larger than leglength?
            state1.time = 0.4;
            state1.torso_forward_angle = trajectoryParameters.torso_forward_angle * 0;
            state1.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state1);
        }

        {
            taskspace_state state2 = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                //if stand together
                if (trajectoryParameters.step_length < 0.1) {
                    state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance + 0.3;
                } else {
                    state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x + ankleDistance + sqrt(pow(legLengthSlacked, 2.0) - pow(legLengthSlacked - trajectoryParameters.step_length * trajectoryParameters.step_height, 2.0));
                }
                state2.left_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height + trajectoryParameters.step_end_height;
                state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                state2.right_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length / 3.0;
            } else {
                //if stand together
                if (trajectoryParameters.step_length < 0.1) {
                    state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + 0.3;
                } else {
                    state2.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x + ankleDistance + sqrt(pow(legLengthSlacked, 2.0) - pow(legLengthSlacked - trajectoryParameters.step_length * trajectoryParameters.step_height, 2.0));
                }
                state2.right_ankle_position.z = pilotParameters.ankle_height + 0.5 * trajectoryParameters.step_height + trajectoryParameters.step_end_height;
                state2.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                state2.left_ankle_position.z = pilotParameters.ankle_height;
                state2.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length / 3.0;
            }
            state2.hip_position.z = pilotParameters.ankle_height + 0.999 * (sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length / 3.0, 2.0)));
            state2.time = 0.70;
            state2.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            state2.swing_ankle_down_angle = 0.0;  // could be non-zero due to slight issues in forward kinematics/positioning, btu zero it out anyways
            keyTaskspaceStates.push_back(state2);
        }
        // Final state
        {
            taskspace_state stateEnd = initialTaskspaceState;
            if (initialTaskspaceState.stance_foot == Foot::Right)
            //|| abs(initialTaskspaceState.left_ankle_position.x - initialTaskspaceState.right_ankle_position.x) <= deltaFootDistance)
            {
                stateEnd.left_ankle_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length;
                stateEnd.right_ankle_position.x = initialTaskspaceState.right_ankle_position.x;
                stateEnd.hip_position.x = initialTaskspaceState.right_ankle_position.x + trajectoryParameters.step_length * 1.6 / 3.0;
                stateEnd.right_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.left_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_end_height;
            }

            else {
                stateEnd.right_ankle_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length;
                stateEnd.left_ankle_position.x = initialTaskspaceState.left_ankle_position.x;
                stateEnd.hip_position.x = initialTaskspaceState.left_ankle_position.x + trajectoryParameters.step_length * 1.6 / 3.0;
                stateEnd.left_ankle_position.z = pilotParameters.ankle_height;
                stateEnd.right_ankle_position.z = pilotParameters.ankle_height + trajectoryParameters.step_end_height;
            }

            //if stand together
            if (trajectoryParameters.step_length < 0.1) {
                stateEnd.hip_position.z = pilotParameters.ankle_height + legLengthSlacked;
            } else {
                stateEnd.hip_position.z = pilotParameters.ankle_height + 0.999 * (sqrt(pow(legLengthSlacked, 2.0) - pow(trajectoryParameters.step_length * 1.6 / 3.0, 2.0)));
            }
            stateEnd.time = 1;
            stateEnd.torso_forward_angle = trajectoryParameters.torso_forward_angle;
            stateEnd.swing_ankle_down_angle = 0.0;
            keyTaskspaceStates.push_back(stateEnd);
        }
    }
    return keyTaskspaceStates;
}

// Generates discrete trajectory from parameters to use in control system
void AlexTrajectoryGenerator::compute_discrete_trajectory(
    const TrajectoryParameters &trajectoryParameters,
    const PilotParameters &pilotParameters,
    jointspace_state initialJointspaceState) {
    Foot stanceFoot = trajectoryParameters.stance_foot;
    taskspace_state initialTaskspaceState;
    std::vector<taskspace_state> taskspaceStates;
    std::vector<jointspace_state> jointspaceStates;
    jointspace_spline jointspaceSpline;

    initialTaskspaceState = jointspace_state_to_taskspace_state(initialJointspaceState, trajectoryParameters, pilotParameters);

    // Obtain key tTrajectoryParameters //  (output excludes initial state which is already known in jointspace)
    taskspaceStates = generate_key_taskspace_states(initialTaskspaceState, trajectoryParameters, pilotParameters);
    //printing the initial taskspace state
    std::cout << "[compute_trajectory]: Keypoints (taskspace):" << std::endl;
    std::cout << "[compute_trajectory]:"
              << "\t"
              << "time | left_ankle_position | hip_position | right_ankle_position | torso_forward_angle | swing_ankle_down_angle"
              << std::endl;
    std::cout << "[compute_trajectory]:"
              << "\t"
              << initialTaskspaceState.time << "\t"
              << " | "
              << initialTaskspaceState.left_ankle_position.x << "\t"
              << initialTaskspaceState.left_ankle_position.y << "\t"
              << initialTaskspaceState.left_ankle_position.z << "\t"
              << " | "
              << initialTaskspaceState.hip_position.x << "\t"
              << initialTaskspaceState.hip_position.y << "\t"
              << initialTaskspaceState.hip_position.z << "\t"
              << " | "
              << initialTaskspaceState.right_ankle_position.x << "\t"
              << initialTaskspaceState.right_ankle_position.y << "\t"
              << initialTaskspaceState.right_ankle_position.z << "\t"
              << " | "
              << initialTaskspaceState.torso_forward_angle << "\t"
              << " | "
              << initialTaskspaceState.swing_ankle_down_angle
              << std::endl;
    //printing the middle and last task space states
    for (auto currentState : taskspaceStates) {
        std::cout << "[compute_trajectory]:"
                  << "\t"
                  << currentState.time << "\t"
                  << "|"
                  << currentState.left_ankle_position.x << "\t"
                  << currentState.left_ankle_position.y << "\t"
                  << currentState.left_ankle_position.z << "\t"
                  << "|"
                  << currentState.hip_position.x << "\t"
                  << currentState.hip_position.y << "\t"
                  << currentState.hip_position.z << "\t"
                  << "|"
                  << currentState.right_ankle_position.x << "\t"
                  << currentState.right_ankle_position.y << "\t"
                  << currentState.right_ankle_position.z << "\t"
                  << "|"
                  << currentState.torso_forward_angle << "\t"
                  << "|"
                  << currentState.swing_ankle_down_angle
                  << std::endl;
    }

    // Convert key states to jointspace (prepend known initial jointspace state)
    jointspaceStates = taskspace_states_to_jointspace_states(initialJointspaceState, taskspaceStates, trajectoryParameters, pilotParameters);

    std::cout << "[compute_trajectory]: Keypoints (jointspace):"
              << "LEFT_HIP"
              << "\t"
              << "LEFT_KNEE"
              << "\t"
              << "RIGHT_HIP"
              << "\t"
              << "RIGHT_KNEE"
              << "\t"
              << "LEFT_ANKLE"
              << "\t"
              << "RIGHT_ANKLE" << std::endl;
    for (auto state : jointspaceStates) {
        std::cout << "[compute_trajectory]:\t" << state.time << "\t\t\t";
        for (int i = 0; i < NUM_JOINTS; i++)
            std::cout << rad2deg(state.q[i]) << "\t  ";
        std::cout << std::endl;
    }
}

taskspace_state AlexTrajectoryGenerator::jointspace_state_to_taskspace_state(
    jointspace_state jointspaceState,
    TrajectoryParameters trajectoryParameters,
    PilotParameters pilotParameters) {
    taskspace_state taskspaceState{};

    // q's in a reference frame - imagine a 6link thing positioned with all angles at '0' (horizontal), then only PI is
    //  needed to be added at hip/right_leg to get exo
    double *q = jointspaceState.q;  // for readability; will be optimised out by compiler?
    //calculate the torso forward angle from left joints, which is repsect to world space vertical up
    double torso_forward_angle = M_PI_2 + q[LEFT_ANKLE] - q[LEFT_KNEE] - q[LEFT_HIP];
    // Perform Forward Kinematics
    //  Put origin at base of Stance foot, vertically below the ankle
    //  Assume hip has zero width for now
    taskspaceState.left_ankle_position.x = 0.0;
    taskspaceState.left_ankle_position.y = 0.0;
    taskspaceState.left_ankle_position.z = pilotParameters.ankle_height;
    if (trajectoryParameters.left_foot_on_tilt) {
        taskspaceState.hip_position.x = taskspaceState.left_ankle_position.x - pilotParameters.lowerleg_length * (cos(q[LEFT_ANKLE] + deg2rad(trajectoryParameters.slope_angle))) - pilotParameters.upperleg_length * (cos(q[LEFT_ANKLE] + deg2rad(trajectoryParameters.slope_angle - q[LEFT_KNEE])));
        taskspaceState.hip_position.y = 0.0;
        taskspaceState.hip_position.z = taskspaceState.left_ankle_position.z + pilotParameters.lowerleg_length * (sin(q[LEFT_ANKLE] + deg2rad(trajectoryParameters.slope_angle))) + pilotParameters.upperleg_length * (sin(q[LEFT_ANKLE] + deg2rad(trajectoryParameters.slope_angle - q[LEFT_KNEE])));
    } else {
        taskspaceState.hip_position.x = taskspaceState.left_ankle_position.x - pilotParameters.lowerleg_length * (cos(q[LEFT_ANKLE])) - pilotParameters.upperleg_length * (cos(q[LEFT_ANKLE] - q[LEFT_KNEE]));
        taskspaceState.hip_position.y = 0.0;
        taskspaceState.hip_position.z = taskspaceState.left_ankle_position.z + pilotParameters.lowerleg_length * (sin(q[LEFT_ANKLE])) + pilotParameters.upperleg_length * (sin(q[LEFT_ANKLE] - q[LEFT_KNEE]));
    }
    taskspaceState.right_ankle_position.x = taskspaceState.hip_position.x + pilotParameters.upperleg_length * (sin(M_PI - q[RIGHT_HIP] - torso_forward_angle)) - pilotParameters.lowerleg_length * (cos(M_PI_2 + M_PI - q[RIGHT_HIP] - torso_forward_angle - q[RIGHT_KNEE]));
    taskspaceState.right_ankle_position.y = 0.0;
    taskspaceState.right_ankle_position.z = taskspaceState.hip_position.z - pilotParameters.upperleg_length * (cos(M_PI - q[RIGHT_HIP] - torso_forward_angle)) - pilotParameters.lowerleg_length * (sin(M_PI_2 + M_PI - q[RIGHT_HIP] - torso_forward_angle - q[RIGHT_KNEE]));
    taskspaceState.torso_forward_angle = torso_forward_angle;
    taskspaceState.swing_ankle_down_angle = 0;  //TODO: this is called when converting from robot to trajectory calculation. Do i need this information?
    taskspaceState.stance_foot = trajectoryParameters.stance_foot;
    taskspaceState.time = jointspaceState.time;

    return taskspaceState;
}

jointspace_state AlexTrajectoryGenerator::taskspace_state_to_jointspace_state(
    taskspace_state taskspaceState,
    TrajectoryParameters trajectoryParameters,
    PilotParameters pilotParameters) {
    jointspace_state jointspaceState{};
    // Do the bulk of the computations based on each leg
    std::vector<double> LeftTempAngles = triangle_inverse_kinematics(
        taskspaceState.left_ankle_position.x,
        taskspaceState.left_ankle_position.z,
        taskspaceState.hip_position.x,
        taskspaceState.hip_position.z,
        pilotParameters.lowerleg_length,
        pilotParameters.upperleg_length);
    std::vector<double> RightTempAngles = triangle_inverse_kinematics(
        taskspaceState.right_ankle_position.x,
        taskspaceState.right_ankle_position.z,
        taskspaceState.hip_position.x,
        taskspaceState.hip_position.z,
        pilotParameters.lowerleg_length,
        pilotParameters.upperleg_length);
    //[0] angleAtHip,  // CCW angle of upper leg from vertical down
    //[1] angleAtKnee, // CCW angle of upper leg from straight leg config
    //[2] angleAtAnkle //  CW angle of lower leg from vertical up

    jointspaceState.time = taskspaceState.time;
    if (trajectoryParameters.left_foot_on_tilt) {
        jointspaceState.q[LEFT_ANKLE] = M_PI_2 + LeftTempAngles.at(2) - trajectoryParameters.slope_angle;
    } else {
        jointspaceState.q[LEFT_ANKLE] = M_PI_2 + LeftTempAngles.at(2);
    }

    jointspaceState.q[LEFT_ANKLE] = M_PI_2 + LeftTempAngles.at(2);
    jointspaceState.q[LEFT_KNEE] = LeftTempAngles.at(1);
    jointspaceState.q[LEFT_HIP] = M_PI - LeftTempAngles.at(0) - taskspaceState.torso_forward_angle;
    jointspaceState.q[RIGHT_HIP] = M_PI - RightTempAngles.at(0) - taskspaceState.torso_forward_angle;
    jointspaceState.q[RIGHT_KNEE] = RightTempAngles.at(1);

    if (trajectoryParameters.right_foot_on_tilt) {
        jointspaceState.q[RIGHT_ANKLE] = M_PI_2 + RightTempAngles.at(2) - trajectoryParameters.slope_angle;
    } else {
        jointspaceState.q[RIGHT_ANKLE] = M_PI_2 + RightTempAngles.at(2);
    }
    return jointspaceState;
}

std::vector<jointspace_state> AlexTrajectoryGenerator::taskspace_states_to_jointspace_states(
    jointspace_state initialJointspaceState,
    const std::vector<taskspace_state> &taskspaceStates,
    TrajectoryParameters trajectoryParameters,
    PilotParameters pilotParameters) {
    std::vector<jointspace_state> jointspaceStates;
    jointspace_state tempJointspacestate;
    bool containNaN = false;
    jointspaceStates.push_back(initialJointspaceState);
    //First check whether any point in the trajectory contains NaN
    // convert every taskspace state to a jointspace state
    for (auto taskspaceState : taskspaceStates) {
        if(jointspace_NaN_check(taskspace_state_to_jointspace_state(taskspaceState, trajectoryParameters, pilotParameters))){
          containNaN = true;
          spdlog::debug("THE TRAJECTORY CONTAINS NAN");
        }
    }
    //construct the jointspaceStates with respect to whether it contains NaN
    for (auto taskspaceState : taskspaceStates) {
      if (containNaN == false){
        jointspaceStates.push_back(taskspace_state_to_jointspace_state(taskspaceState, trajectoryParameters, pilotParameters));
      }
      //If any part contains NaN, make all points in jointspaceStates just the initial jointstatespace points
      else{
        tempJointspacestate = taskspace_state_to_jointspace_state(taskspaceState, trajectoryParameters, pilotParameters);
        for (int i = 0; i < NUM_JOINTS; i++) {
          tempJointspacestate.q[i] = initialJointspaceState.q[i];
        }
        jointspaceStates.push_back(tempJointspacestate);
      }
    }
    return jointspaceStates;
}

std::vector<double> AlexTrajectoryGenerator::triangle_inverse_kinematics(
    const double xAnkle,
    const double zAnkle,
    const double xHip,
    const double zHip,
    const double Llower,
    const double Lupper) {
    // x is right, z is up; we're looking at the exo from the right side, so it walks rightwards

    const double L = sqrtf((xAnkle - xHip) * (xAnkle - xHip) + (zAnkle - zHip) * (zAnkle - zHip));
    const double angleLAcuteFromVertical = atan2f((xAnkle - xHip), (zHip - zAnkle));
    const double angleInternalHip = acos((Lupper * Lupper + L * L - Llower * Llower) / (2.0 * Lupper * L));
    if (std::isnan(angleInternalHip)) {
        spdlog::debug("acos outside of domain: do not move!");
        /*\todo throw and catch an error from this is calling function*/
    }
    //const double angleInternalAnkle = asin(sin(angleInternalHip)*Lupper/Llower);
    //const double angleInternalAnkle = acos((Llower * Llower + L * L - Lupper * Lupper) / (2.0 * Llower * L));
    const double angleInternalAnkle = asin(sin(angleInternalHip) * Lupper / Llower);
    const double angleInternalKnee = M_PI - angleInternalHip - angleInternalAnkle;

    const double angleAtHip = angleLAcuteFromVertical + angleInternalHip;
    const double angleAtKnee = M_PI - angleInternalKnee;
    const double angleAtAnkle = angleInternalAnkle - angleLAcuteFromVertical;

    return {
        angleAtHip,   // CCW angle of upper leg from vertical down
        angleAtKnee,  // CCW angle of upper leg from straight leg config
        angleAtAnkle  //  CW angle of lower leg from vertical up
    };
}

/**********************************************************************

Functions for Splines

**********************************************************************/
std::vector<CubicPolynomial> AlexTrajectoryGenerator::cubic_spline(
    double x[],
    time_tt t[],
    int numPoints) {
    std::vector<CubicPolynomial> cubicSplinePolynomials;

    // std::cout << "[cubic_spline]: x's: ";
    // for (int i = 0; i < numPoints; i++) std::cout << x[i] << "\t";
    // std::cout << std::endl;

    // Cubic spline
    // Assume boundary vel and acc are zero.
    const int numPointsm1 = numPoints - 1;
    const int numPolynomials = numPointsm1;
    const int numVias = numPoints - 2;
    const int numPolynomialCoefficients = NUM_CUBIC_COEFFICIENTS;
    const int numUnknowns = numPolynomialCoefficients * numPolynomials;

    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(numUnknowns, numUnknowns);
    Eigen::MatrixXf B = Eigen::MatrixXf::Zero(numUnknowns, 1);
    int offset = 0;
    A.block<2, numPolynomialCoefficients>(offset, 0)
        << 1,
        t[0], t[0] * t[0], t[0] * t[0] * t[0],
        0, 1, 2 * t[0], 3 * t[0] * t[0];
    B.block<2, 1>(offset, 0)
        << x[0],
        0;
    offset += 2;

    A.block<2, numPolynomialCoefficients>(offset, numUnknowns - numPolynomialCoefficients)
        << 1,
        t[numPointsm1], t[numPointsm1] * t[numPointsm1], t[numPointsm1] * t[numPointsm1] * t[numPointsm1],
        0, 1, 2 * t[numPointsm1], 3 * t[numPointsm1] * t[numPointsm1];
    B.block<2, 1>(offset, 0)
        << x[numPointsm1],
        0;
    offset += 2;

    for (int i = 0; i < numVias; i++) {
        int j = i + 1;
        A.block<1, numPolynomialCoefficients>(offset, i * numPolynomialCoefficients)
            << 1,
            t[j], t[j] * t[j], t[j] * t[j] * t[j];
        B.block<1, 1>(offset, 0)
            << x[j];
        offset += 1;
        A.block<1, numPolynomialCoefficients>(offset, j * numPolynomialCoefficients)
            << 1,
            t[j], t[j] * t[j], t[j] * t[j] * t[j];
        B.block<1, 1>(offset, 0)
            << x[j];
        offset += 1;
        A.block<2, 2 * numPolynomialCoefficients>(offset, i * numPolynomialCoefficients)
            << 0,
            1, 2 * t[j], 3 * t[j] * t[j], 0, -1, -2 * t[j], -3 * t[j] * t[j],
            0, 0, 2, 6 * t[j], 0, 0, -2, -6 * t[j];
        B.block<2, 1>(offset, 0)
            << 0,
            0;
        offset += 2;
    }

    Eigen::MatrixXf X = A.inverse() * B;

    for (int i = 0; i < numPolynomials; i++) {
        // cubic (t)
        cubicSplinePolynomials.push_back({.coefficients = {
                                              X(NUM_CUBIC_COEFFICIENTS * i + 0, 0),
                                              X(NUM_CUBIC_COEFFICIENTS * i + 1, 0),
                                              X(NUM_CUBIC_COEFFICIENTS * i + 2, 0),
                                              X(NUM_CUBIC_COEFFICIENTS * i + 3, 0)}});
    }

    return cubicSplinePolynomials;
}

jointspace_spline AlexTrajectoryGenerator::cubic_spline_jointspace_states(std::vector<jointspace_state> jointspaceStates) {
    jointspace_spline jointspaceSpline;
    // Get times
    for (auto &jointspaceState : jointspaceStates)
        jointspaceSpline.times.push_back(jointspaceState.time);

    // Take spline for each q
    for (int i = 0, numPoints = jointspaceStates.size(); i < NUM_JOINTS; i++) {
        double q[numPoints];
        for (int j = 0; j < numPoints; j++) {
            q[j] = jointspaceStates.at(j).q[i];
        }
        jointspaceSpline.polynomials[i] = cubic_spline(q, jointspaceSpline.times.data(), numPoints);
    }

    return jointspaceSpline;
}

double AlexTrajectoryGenerator::evaluate_cubic_polynomial(CubicPolynomial cubicPolynomial, time_tt time) {
    return cubicPolynomial.coefficients[0] + cubicPolynomial.coefficients[1] * time + cubicPolynomial.coefficients[2] * time * time + cubicPolynomial.coefficients[3] * time * time * time;
}
double AlexTrajectoryGenerator::evaluate_cubic_polynomial_first_derivative(CubicPolynomial cubicPolynomial, time_tt time) {
    return cubicPolynomial.coefficients[1] + 2 * cubicPolynomial.coefficients[2] * time + 3 * cubicPolynomial.coefficients[3] * time * time;
}
double AlexTrajectoryGenerator::evaluate_cubic_polynomial_second_derivative(CubicPolynomial cubicPolynomial, time_tt time) {
    return 2 * cubicPolynomial.coefficients[2] + 6 * cubicPolynomial.coefficients[3] * time;
}

//Generate and store the trajectory spline into the trajectory object
void AlexTrajectoryGenerator::generateAndSaveSpline(jointspace_state initialJointspaceState) {
    this->trajectoryJointSpline = compute_trajectory_spline(trajectoryParameter, pilotParameters, initialJointspaceState);
}

/**********************************************************************

Functions for Controller

**********************************************************************/

/**
 *
 * @param initialTaskspaceState
 * @param trajectoryParameters
 * @param pilotParameters
 * @return Vector of key taskspace_states of the trajectory (excluding initial state)
 */

// Generates trajectory spline from parameters to use in control system
jointspace_spline AlexTrajectoryGenerator::compute_trajectory_spline(const TrajectoryParameters &trajectoryParameters,
                                                                     const PilotParameters &pilotParameters,
                                                                     jointspace_state initialJointspaceState) {
    Foot stanceFoot = trajectoryParameters.stance_foot;
    taskspace_state initialTaskspaceState;
    std::vector<taskspace_state> taskspaceStates;
    std::vector<jointspace_state> jointspaceStates;
    jointspace_spline jointspaceSpline;
    initialTaskspaceState = jointspace_state_to_taskspace_state(initialJointspaceState, trajectoryParameters, pilotParameters);

    // Obtain key taskspace states
    //  (output excludes initial state which is already known in jointspace)
    taskspaceStates = generate_key_taskspace_states(initialTaskspaceState, trajectoryParameters, pilotParameters);

    // Convert key states to jointspace (prepend known initial jointspace state)
    jointspaceStates = taskspace_states_to_jointspace_states(initialJointspaceState, taskspaceStates, trajectoryParameters, pilotParameters);
    //Print out all joint space states
    // spdlog::debug("---- Joint space via points 'states'-----")
    // for (auto states : jointspaceStates) {
    //     spdlog::debug("TIME: " << states.time)
    //     for (auto q : states.q) {
    //         std::cout << rad2deg(q) << ", ";
    //     }
    //     std::cout << std::endl;
    // }
    // Calculate the spline for the given jointspacestates
    jointspaceSpline = cubic_spline_jointspace_states(jointspaceStates);

    return jointspaceSpline;
}

//get the velocity at any given time
/*void AlexTrajectoryGenerator::calcVelocity(time_tt time, double *velocityArray) {
    time_tt startTime = trajectoryJointSpline.times.front();
    time_tt endTime = trajectoryJointSpline.times.back();
    // Every sample time, compute the value of q1 to q6 based on the time segment / set of NUM_JOINTS polynomials
    int numPoints = trajectoryJointSpline.times.size();
    int numPolynomials = numPoints - 1;
    CubicPolynomial currentPolynomial[NUM_JOINTS];
    jointspace_state_ex temp{};
    for (int polynomial_index = 0; polynomial_index < numPolynomials; polynomial_index++) {
        //cout << "[discretise_spline]: pt " << polynomial_index << ":" << std::endl;
        //if the jointspaceState time is bounded by the section of spline
        if (time >= trajectoryJointSpline.times.at(polynomial_index) &&
            time <= trajectoryJointSpline.times.at(polynomial_index + 1)) {
            //cout << "time " << time << "\t" << trajectoryJointSpline.times.at(polynomial_index)  << std::endl;
            for (int i = 0; i < NUM_JOINTS; i++) {
                currentPolynomial[i] = trajectoryJointSpline.polynomials[i].at(polynomial_index);
                temp.qd[i] = evaluate_cubic_polynomial_first_derivative(currentPolynomial[i], time);
                velocityArray[i] = temp.qd[i];
            }
            return;
        }
    }
    //cout << "[discretise_spline]:\t" << temp.time << "\t";
    //if the time point is outside range
    for (int i = 0; i < NUM_JOINTS; i++) {
        currentPolynomial[i] = trajectoryJointSpline.polynomials[i].at(numPolynomials - 1);
        temp.qd[i] = evaluate_cubic_polynomial_first_derivative(currentPolynomial[i], endTime);
        velocityArray[i] = temp.qd[i];
    }
}
*/

jointspace_state AlexTrajectoryGenerator::compute_position_trajectory_difference(
    jointspace_spline jointspaceSpline,
    jointspace_state currentJointspaceStates) {
    jointspace_state position_diff;
    CubicPolynomial currentPolynomial[NUM_JOINTS];
    int numPoints = jointspaceSpline.times.size();
    int numPolynomials = numPoints - 1;
    time_tt endTime = jointspaceSpline.times.back();
    jointspace_state splinePosition;

    // Get the target position based on time and spline
    // If the time is past the spline time range,
    // Calculate the diff based on last position in spline
    if (currentJointspaceStates.time >= jointspaceSpline.times.back()) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            currentPolynomial[i] = jointspaceSpline.polynomials[i].at(numPolynomials - 1);
        }
        // calculate the position difference at each joint
        for (int i = 0; i < NUM_JOINTS; i++) {
            splinePosition.q[i] = evaluate_cubic_polynomial(currentPolynomial[i], endTime);
            position_diff.q[i] = currentJointspaceStates.q[i] - splinePosition.q[i];
        }
        position_diff.time = currentJointspaceStates.time;
        return position_diff;
    }
    // Else, find the relevant section in spline and calculate the position difference

    else {
        for (int polynomial_index = 0; polynomial_index < numPolynomials; polynomial_index++) {
            //cout << "[discretise_spline]: pt " << polynomial_index << ":" << std::endl;
            //if the jointspaceState time is bounded by the section of spline
            if (currentJointspaceStates.time >= jointspaceSpline.times.at(polynomial_index) &&
                currentJointspaceStates.time <= jointspaceSpline.times.at(polynomial_index + 1)) {
                // capture the polynomial for the current spline
                for (int i = 0; i < NUM_JOINTS; i++) {
                    currentPolynomial[i] = jointspaceSpline.polynomials[i].at(polynomial_index);
                    splinePosition.q[i] = evaluate_cubic_polynomial(currentPolynomial[i], currentJointspaceStates.time);
                    position_diff.q[i] = currentJointspaceStates.q[i] - splinePosition.q[i];
                }
                //set up the position time to prevent weird behavior
                position_diff.time = currentJointspaceStates.time;
                return position_diff;
            }
        }
    }
    // return error if none of the above case return anything
    std::cout << "[compute_position_trajectory_difference] Error: Cannot find the time region in the spline" << std::endl;
    std::cout << "[compute_position_trajectory_difference] Assuming no position tracking error" << std::endl;
    for (int i = 0; i < NUM_JOINTS; i++) {
        position_diff.q[i] = 0;
    }
    //set up the position time to prevent weird behavior
    position_diff.time = currentJointspaceStates.time;
    return position_diff;
}

// Limiting the velocity control to not pushing against angle limit
// use AFTER the current velocity is added to the control velocity
void AlexTrajectoryGenerator::limit_velocity_against_angle_boundary(
    jointspace_state currentJointspaceStates,
    double *velocitySignal) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        int minIndex = i * 2;
        int maxIndex = i * 2 + 1;
        //if we near the boundary and velocity is pushing towards it
        if ((currentJointspaceStates.q[i] < Q_MIN_MAX[minIndex] &&
             velocitySignal < 0) ||
            (currentJointspaceStates.q[i] > Q_MIN_MAX[maxIndex] &&
             velocitySignal > 0)) {
            velocitySignal = 0;
        }
    }
}

//limiting the position array in trajectory class
void AlexTrajectoryGenerator::limit_position_against_angle_boundary(std::vector<double> &positions) {
    for (int i = 0; i < positions.size(); i++) {
        int minIndex = i * 2;
        int maxIndex = i * 2 + 1;
        //if at the boundary
        if (positions[i] < Q_MIN_MAX[minIndex]) {
            positions[i] = Q_MIN_MAX[minIndex];
        }
        if (positions[i] > Q_MIN_MAX[maxIndex]) {
            positions[i] = Q_MIN_MAX[maxIndex];
        }
        // Got another method to properly dealt with error, so get rid of it right now
        // if (std::isnan(positions[i])) {
        //     std::cout << "Joint " << i << "ISNAN now " << std::endl;
        //     positions[i] = Q_MIN_MAX[minIndex]; /*move to smallest joint angle*/
        //     // positions[i] = Q_MIN_MAX[maxIndex] + 10000; why is this +1000 (from old code)
        // }
    }
}
bool AlexTrajectoryGenerator::jointspace_NaN_check(jointspace_state checkJointspaceState){
  bool containNaN  = false;
  //check whether the checkJointspaceState contains NaN
  for (int i = 0; i < NUM_JOINTS; i++){
    //Flag it if it does contains NaN
    if(std::isnan(checkJointspaceState.q[i])) {
      containNaN = true;
    }
  }
  //if any one joint returns a NaN, change the whole jointspace state to the original state
  return containNaN;
}


/*void AlexTrajectoryGenerator::getVelocityAfterPositionCorrection(
    time_tt time, double *robotPositionArray, double *velocityArray) {
    double splinePositionArray[NUM_JOINTS];
    double positionDiff[NUM_JOINTS];
    double positionFeedback[NUM_JOINTS];
    const double FEEDBACKGAIN = 1;
    calcPosition(time, splinePositionArray);
    //the spline position array is in radian, trajectory object perspective jointspace
    //so the robot position array also needs to be converted to trajectory jointspace, numbering and radian
    for (int i = 0; i < NUM_JOINTS; i++) {
        positionDiff[i] = splinePositionArray[i] - robotPositionArray[i];
    }
    //the feedback gain part
    calcVelocity(time, velocityArray);
    for (int i = 0; i < NUM_JOINTS; i++) {
        velocityArray[i] += FEEDBACKGAIN * positionDiff[i];
    }
}*/

/***********************************************************************

Getter and setter

***********************************************************************/

void AlexTrajectoryGenerator::setTrajectoryStanceRight() {
    trajectoryParameter.stance_foot = Foot::Right;
}
void AlexTrajectoryGenerator::setTrajectoryStanceLeft() {
    trajectoryParameter.stance_foot = Foot::Left;
}

double AlexTrajectoryGenerator::getStepDuration() {
    return trajectoryParameter.step_duration;
}

bool AlexTrajectoryGenerator::isTrajectoryFinished(double trajProgress) {
    double fracProgress = trajProgress / (double)trajectoryParameter.step_duration;
    if (fracProgress > 1.05) {
        return true;
    } else {
        return false;
    }
}
