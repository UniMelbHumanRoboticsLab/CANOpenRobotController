/* Header file for calculating trajectory,
*/

#ifndef AlexTrajectoryGenerator_H_INCLUDED
#define AlexTrajectoryGenerator_H_INCLUDED

//using namespace std;
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

#include "RobotParams.h"

/**
 * An enum type.
 * Joint Index for the 4 joints (note, CANopen NODEID = this + 1)
 */


#define deg2rad(deg) ((deg)*M_PI / 180.0)
#define rad2deg(rad) ((rad)*180.0 / M_PI)

//#include <iomanip>
//#include <iostream>
//#include <vector>

#define NUM_CUBIC_COEFFICIENTS (4)

//step parameters
#define STANDTIME 3
#define SITTIME 3
#define STEPTIME 2
#define STAIRTIME 3
#define UNEVENSTEPTIME 3
#define UNEVENTORSO deg2rad(10)
#define STEPLENGTH 0.33
#define HALFSTEPLENGTH STEPLENGTH / 2
#define LONGSTEPLENGTH STEPLENGTH * 1.5
#define BACKLENGTH 0.3
#define STEPHEIGHT 0.4
#define STEPHIGH 0.7
#define STEPTGTLENGTH 0.0
#define LEGSLACK 0.0001
#define TORSOANGLE deg2rad(5)
#define UNEVENSTEP 0.3
#define STAIRSTEP 0.35
#define STAIRHEIGHT 0.25
#define TILTANKLE deg2rad(10) //tbd, 12 deg for tilted path, 20 deg for ramp
#define RAMPANKLE deg2rad(10) //tbd, need to measure when back in the lab

typedef double time_tt;  // time_t is already used

enum class Foot {
    Left,
    Right
};

enum class StepType {
    Walk,
    Sit,
    Stand,
    Stair,
    RampUp,
    RampDown,
    TiltUp,
    TiltDown,
    Back,
    Sitting,
    Uneven,
    DownStair
};

static std::map<StepType, std::string> StepTypeToString = {
    {StepType::Walk, "Walk"},
    {StepType::Sit, "Sit"},
    {StepType::Stand, "Stand"},
    {StepType::Stair, "Up stairs"},
    {StepType::DownStair, "Down stairs"},
    {StepType::RampUp, "Up ramp"},
    {StepType::RampDown, "Down ramp"},
    {StepType::TiltUp, "Up tilt"},
    {StepType::TiltDown, "Down tilt"},
    {StepType::Back, "Backstep"},
    {StepType::Sitting, "Sitting (Fixed)"},
    {StepType::Uneven, "Uneven"}};

typedef struct point {
    double x, y, z;
} point;

/** Structs for Task Space and Joint Space
 */
typedef struct taskspace_state {
    point left_ankle_position, hip_position, right_ankle_position;
    double torso_forward_angle;     // torso angle forward from vertical
    double swing_ankle_down_angle;  // swing_ankle angle down from the horizontal; 0 => horizontal swing_foot
    Foot stance_foot;               // Foot::Left or Foot::Right
    time_tt time;
} taskspace_state;
// void printTaskState(taskspace_state state) {
//     spdlog::debug("---points---")
//     spdlog::debug("LANK" << state.left_ankle_position.x << "," << state.left_ankle_position.y "," << state.left_ankle_position.z)
// }

typedef struct jointspace_state {
    double q[ALEX_NUM_JOINTS];
    time_tt time;
} jointspace_state;

typedef struct jointspace_state_ex {  // extended to include velocity and acceleration
    double q[ALEX_NUM_JOINTS], qd[ALEX_NUM_JOINTS], qdd[ALEX_NUM_JOINTS];
    time_tt time;
} jointspace_state_ex;

/** Structs for Polynomials and Splines
 */
typedef struct CubicPolynomial {
    double coefficients[NUM_CUBIC_COEFFICIENTS];  // starting from const term.
} CubicPolynomial;

typedef struct jointspace_spline {
    std::vector<CubicPolynomial> polynomials[ALEX_NUM_JOINTS];  // polynomial[0] is a vector of cubic polynomials TrajectoryParameterstor<time_tt> times;                            // Start/end times of the polynomials (#times = #polynomials+1)
    std::vector<time_tt> times;
} jointspace_spline;

/** Structs for Trajectory and Pilot Parameters
 */
typedef struct TrajectoryParameters {
    time_tt step_duration;
    double step_height, step_length, hip_height_slack;
    double torso_forward_angle, swing_ankle_down_angle;
    Foot stance_foot;
    StepType stepType;                           //type of movement
    double seat_height;                          // sit-stand
    double step_end_height;                      // stairs and uneven ground
    double slope_angle;                          // tilted path
    bool left_foot_on_tilt, right_foot_on_tilt;  // whether the foot is on tilted ground
} TrajectoryParameters;

typedef struct PilotParameters {
    double lowerleg_length, upperleg_length, ankle_height, foot_length, hip_width, torso_length;
    double buttocks_height;  // sit-stand
} PilotParameters;

/**
 * @brief Map between Movement values for specific trajectory motion paramaters. These paramaters are fed into the
 * TrajectoryGenerator object to create unique trajectories. The map is constructed for ease of loading
 * in new trajectories dictated by an external CAN enabled controller in the exoskeleton State machine. The paramater
 * map is constructed at runtime from trajectoryParam.JSON
 * @param RobotMode Current mode of robot
 * @return TrajectoryGenerator::TrajectoryParameters
 */
static std::map<RobotMode, TrajectoryParameters> movementTrajMap = {
    {RobotMode::INITIAL, {.step_duration = SITTIME, .step_height = STEPHEIGHT, .step_length = STEPLENGTH,
                          .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                          .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                          .swing_ankle_down_angle = 0,
                          .stance_foot = Foot::Right,
                          .stepType = StepType::Sitting,
                          .seat_height = 0.42,     // sit-stand, when checked on July 23rd was 0.45
                          .step_end_height = 0.0,  // stairs
                          .slope_angle = 0.0,      // tilted path
                          .left_foot_on_tilt = false,
                          .right_foot_on_tilt = false}},
    {RobotMode::NORMALWALK, {.step_duration = STEPTIME, .step_height = STEPHEIGHT, .step_length = STEPLENGTH,
                             .hip_height_slack = LEGSLACK,  // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                             .torso_forward_angle = TORSOANGLE,
                             //.torso_forward_angle = UNEVENTORSO,
                             .swing_ankle_down_angle = 0,
                             .stance_foot = Foot::Right,
                             .stepType = StepType::Walk,
                             //.stepType = StepType::Uneven,
                             .seat_height = 0.42,     // sit-stand
                             .step_end_height = 0.0,  // stairs
                             .slope_angle = 0.0,      // tilted path
                             .left_foot_on_tilt = false,
                             .right_foot_on_tilt = false}},
    {RobotMode::UPSTAIR, {.step_duration = STAIRTIME * 2, .step_height = STEPHEIGHT, .step_length = STAIRSTEP,
                          .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                          .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                          .swing_ankle_down_angle = 0,
                          .stance_foot = Foot::Right,
                          .stepType = StepType::Stair, //Stair
                          .seat_height = 0.42,     // sit-stand
                          .step_end_height = STAIRHEIGHT,  // stairs
                          .slope_angle = 0.0,      // tilted path
                          .left_foot_on_tilt = false,
                          .right_foot_on_tilt = false}},
    {RobotMode::DWNSTAIR, {.step_duration = STAIRTIME * 2, .step_height = STEPHEIGHT, .step_length = STAIRSTEP,
                           .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                           .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                           .swing_ankle_down_angle = 0,
                           .stance_foot = Foot::Right,
                           .stepType = StepType::DownStair,
                           .seat_height = 0.42,     // sit-stand
                           .step_end_height = STAIRHEIGHT,  // stairs
                           .slope_angle = 0.0,      // tilted path
                           .left_foot_on_tilt = false,
                           .right_foot_on_tilt = false}},
    {RobotMode::RAMPUP, {.step_duration = 6, .step_height = STEPHEIGHT, .step_length = STEPLENGTH,
                         .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                         .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                         .swing_ankle_down_angle = RAMPANKLE,
                         .stance_foot = Foot::Right,
                         .stepType = StepType::RampUp,
                         .seat_height = 0.42,     // sit-stand
                         .step_end_height = 0.0,  // stairs
                         .slope_angle = 0.0,      // tilted path
                         .left_foot_on_tilt = false,
                         .right_foot_on_tilt = false}},
    {RobotMode::RAMPDOWN, {.step_duration = 6, .step_height = STEPHEIGHT, .step_length = STEPLENGTH,
                         .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                         .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                         .swing_ankle_down_angle = -RAMPANKLE,
                         .stance_foot = Foot::Right,
                         .stepType = StepType::RampDown,
                         .seat_height = 0.42,     // sit-stand
                         .step_end_height = 0.0,  // stairs
                         .slope_angle = 0.0,      // tilted path
                         .left_foot_on_tilt = false,
                         .right_foot_on_tilt = false}},
    {RobotMode::TILTUP, {.step_duration = 6, .step_height = STEPHEIGHT, .step_length = STEPLENGTH,
                         .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                         .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                         .swing_ankle_down_angle = TILTANKLE,
                         .stance_foot = Foot::Right,
                         .stepType = StepType::TiltUp,
                         .seat_height = 0.42,     // sit-stand
                         .step_end_height = 0.0,  // stairs
                         .slope_angle = 0.0,      // tilted path
                         .left_foot_on_tilt = false,
                         .right_foot_on_tilt = false}},
    {RobotMode::TILTDWN, {.step_duration = 6, .step_height = STEPHEIGHT, .step_length = STEPLENGTH,
                          .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                          .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                          .swing_ankle_down_angle = -TILTANKLE,
                          .stance_foot = Foot::Right,
                          .stepType = StepType::TiltDown,
                          .seat_height = 0.42,     // sit-stand
                          .step_end_height = 0.0,  // stairs
                          .slope_angle = 0.0,      // tilted path
                          .left_foot_on_tilt = false,
                          .right_foot_on_tilt = false}},
    {RobotMode::FTTG, {.step_duration = STEPTIME, .step_height = STEPHEIGHT, .step_length = STEPTGTLENGTH,
                       .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                       .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                       .swing_ankle_down_angle = 0,
                       .stance_foot = Foot::Right,
                       .stepType = StepType::Walk,
                       .seat_height = 0.42,     // sit-stand
                       .step_end_height = 0.0,  // stairs
                       .slope_angle = 0.0,      // tilted path
                       .left_foot_on_tilt = false,
                       .right_foot_on_tilt = false}},
    {RobotMode::BKSTEP, {.step_duration = STEPTIME, .step_height = STEPHEIGHT, .step_length = BACKLENGTH,
                         .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                         .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                         .swing_ankle_down_angle = 0,
                         .stance_foot = Foot::Right,
                         .stepType = StepType::Back,
                         .seat_height = 0.42,     // sit-stand
                         .step_end_height = 0.0,  // stairs
                         .slope_angle = 0.0,      // tilted path
                         .left_foot_on_tilt = false,
                         .right_foot_on_tilt = false}},
    {RobotMode::SITDWN, {.step_duration = SITTIME, .step_height = STEPHEIGHT, .step_length = STEPLENGTH,
                         .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                         .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                         .swing_ankle_down_angle = 0,
                         .stance_foot = Foot::Right,
                         .stepType = StepType::Sit,
                         .seat_height = 0.42,     // sit-stand
                         .step_end_height = 0.0,  // stairs
                         .slope_angle = 0.0,      // tilted path
                         .left_foot_on_tilt = false,
                         .right_foot_on_tilt = false}},
    {RobotMode::STNDUP, {.step_duration = STANDTIME, .step_height = STEPHEIGHT, .step_length = STEPLENGTH,
                         .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                         .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                         .swing_ankle_down_angle = 0,
                         .stance_foot = Foot::Right,
                         .stepType = StepType::Stand,
                         .seat_height = 0.42,     // sit-stand
                         .step_end_height = 0.0,  // stairs
                         .slope_angle = 0.0,      // tilted path
                         .left_foot_on_tilt = false,
                         .right_foot_on_tilt = false}},
    {RobotMode::UNEVEN, {.step_duration = UNEVENSTEPTIME, .step_height = STEPHEIGHT, .step_length = STEPLENGTH,
                         .hip_height_slack = LEGSLACK,       // never make this zero, or else it'll probably make a trig/pythag give NaN due to invalid triangle
                         .torso_forward_angle = TORSOANGLE,  // TODO: make this a vector/array?
                         .swing_ankle_down_angle = 0,
                         .stance_foot = Foot::Right,
                         .stepType = StepType::Uneven,
                         .seat_height = 0.42,     // sit-stand
                         .step_end_height = 0.04,  // stairs
                         .slope_angle = 0.0,      // tilted path
                         .left_foot_on_tilt = false,
                         .right_foot_on_tilt = false}}
};



class AlexTrajectoryGenerator {
   private:
    //setter for the parameters
    /*void setTrajectoryParameters(time_tt step_duration, double step_height, double step_length, double hip_height_slack, double torso_forward_angle, double swing_ankle_down_angle,
                                 Foot stance_foot, StepType stepType, double seat_height, double step_end_height, double slope_angle, bool left_foot_on_tilt, bool right_foot_on_tilt);

    void setPilotParameters(double lowerleg_length, double upperleg_length, double ankle_height, double foot_length,
                            double hip_width, double torso_length, double buttocks_height);*/
    /**
     * @brief Parameters which should be constant from construction and never changed - related to the physical
     * pilot/exoskeleton system
     *
     */
    PilotParameters pilotParameters;

    /**
     * @brief Parameters related to the trajectory to be executed
     *
     */
    TrajectoryParameters trajectoryParameter;
    /**
     * @brief Parameters related to joint space splines
     *
     */

    jointspace_spline trajectoryJointSpline;
    /** Methods which are used to generate the Trajectory from the parameters
     */
    // Generates key taskspace states from gait parameters
    std::vector<taskspace_state> generate_key_taskspace_states(taskspace_state initialTaskspaceState,
                                                               const TrajectoryParameters &trajectoryParameters, const PilotParameters &pilotParameters);
    // Generates discrete trajectory from parameteTrajectoryParameters system
    void compute_discrete_trajectory(const TrajectoryParameters &trajectoryParameters,
                                     const PilotParameters &pilotParameters, jointspace_state initialJointspaceState);
    // Converts jointspace to taskspace (Forward Kinematics)
    taskspace_state jointspace_state_to_taskspace_state(jointspace_state jointspaceState, TrajectoryParameters trajectoryParameters,
                                                        PilotParameters pilotParameters);
    // Converts taskspace to jointspace (Inverse Kinematics)
    jointspace_state taskspace_state_to_jointspace_state(taskspace_state taskspaceState, TrajectoryParameters trajectoryParameters,
                                                         PilotParameters pilotParameters);
    // Converts a vector of taskspace states to jointspace (Inverse Kinematics)
    std::vector<jointspace_state> taskspace_states_to_jointspace_states(jointspace_state initialJointspaceState,
                                                                        const std::vector<taskspace_state> &taskspaceStates, TrajectoryParameters trajectoryParameters, PilotParameters pilotParameters);
    // Helper function for Inverse Kinematics
    std::vector<double> triangle_inverse_kinematics(double xAnkle, double zAnkle, double xHip, double zHip, double Llower,
                                                    double Lupper);

    /** Methods which are used to generate and evaluate splines
     */
    // Splines a set of 2D points as an array of polynomials between the points
    std::vector<CubicPolynomial> cubic_spline(double x[], time_tt t[], int numPoints);

    // Evaluate cubic polynomial at a given time
    double evaluate_cubic_polynomial(CubicPolynomial cubicPolynomial, time_tt time);
    double evaluate_cubic_polynomial_first_derivative(CubicPolynomial cubicPolynomial, time_tt time);
    double evaluate_cubic_polynomial_second_derivative(CubicPolynomial cubicPolynomial, time_tt time);

    // Spline the jointspace states (the q's)
    jointspace_spline cubic_spline_jointspace_states(std::vector<jointspace_state> states);

    /**********************************************************************
	Functions for Controllers
	***********************************************************************/
    // Generates trajectory spline from parameters to use in control system
    jointspace_spline compute_trajectory_spline(const TrajectoryParameters &trajectoryParameters,
                                                const PilotParameters &pilotParameters, jointspace_state initialJointspaceState);

    // Compare the provided position to the supposedly spline position
    // A positive values mean the exo is leading/ spline value is lagging
    jointspace_state compute_position_trajectory_difference(jointspace_spline jointspaceSpline,
                                                            jointspace_state currentJointspaceStates);

    // ing the velocity control to not pushing against angle limit
    // use AFTER the current velocity is added to the control velocity
    void limit_velocity_against_angle_boundary(
        jointspace_state currentJointspaceStates,
        double *velocitySignal);

    //limiting the position array in trajectory class
    //void limit_position_against_angle_boundary(std::vector<double>);
    void limit_position_against_angle_boundary(std::vector<double> &positions);

    // gives a velocity array output
    //    void getVelocityAfterPositionCorrection(time_tt time, double *robotPositionArray, double *velocityArray);

    //calculate the velocity at any given time
    //void calcVelocity(time_tt time, double *velocityArray);
    bool jointspace_NaN_check(jointspace_state checkJointspaceState);
   public:
    int numJoints = 6;
    //TEST FUNCTION
    std::string
    printName();
    // Hardware angle limiation in Radians
    const double Q_MIN_MAX[12]{deg2rad(70), deg2rad(210),
                               0, deg2rad(120),
                               deg2rad(70), deg2rad(210),
                               0, deg2rad(120),
                               deg2rad(75), deg2rad(105),
                               deg2rad(75), deg2rad(105)};
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        * Function Declarations                                                                                             *
        * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    AlexTrajectoryGenerator();
    AlexTrajectoryGenerator(int NumOfJoints);

    //Generate and store the trajectory spline into the trajectory object
    virtual void generateAndSaveSpline(jointspace_state initialJointspaceState);
    /**********************************************************************

    Getter and setter
    **********************************************************************/

    bool initialiseTrajectory();
    bool initialiseTrajectory(RobotMode mov);
    /**
     * \brief
     *      initialize the trajectory generators paramaters and generate a spline given current pos
     *      Function overloaded for walking stance leg input or w/o
     * @param mvmnt type
     * @param vector of robot joints qdeg
     * \return true
     * \return false
     */
    bool initialiseTrajectory(RobotMode mvmnt, std::vector<double> qdeg);
    bool initialiseTrajectory(RobotMode mvmnt, Foot stanceFoot, std::vector<double> qdeg);
    bool initialiseTrajectory(RobotMode mvmnt, double time);

    std::vector<double> getSetPoint(time_tt);

    void setPilotParameters(PilotParameters pilotParameters);
    void setTrajectoryParameters(TrajectoryParameters trajectoryParameter);

    void printTrajectoryParameters();

    void setTrajectoryStanceRight();
    /**
     * \brief Set the Trajectory Objects paramaters Foot stance to Left
     *
     */
    void setTrajectoryStanceLeft();

    double getStepDuration();
    /**
     * \brief Check if the trajectory has been completed based on last elapsed time
     * \param take in the progress of the current trajectory progress
     * \return true if trajectory has been completed
     * \return false if trajectory has not been completed
     */
    bool isTrajectoryFinished(double trajProgress);
};
#endif
