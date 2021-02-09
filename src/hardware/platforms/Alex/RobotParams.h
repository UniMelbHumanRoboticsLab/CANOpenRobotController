/**
 * \file RobotParams.h
 * \author William Campbell
 * \brief Specific paramaters and naming definitions for exoskeleton robot class implementetion.
 * 
 * \version 0.1
 * \date 2020-23-12
 * \version 0.2
 * \copyright Copyright (c) 2020
 */
#include "AlexJoint.h"
#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

#define ALEX_NUM_JOINTS 4

/**
 * An enum type.
 * Joint Index for the 4 joints (note, CANopen NODEID = this + 1)
 */
enum X2Joints {
    ALEX_LEFT_HIP = 0,    /**< Left Hip*/
    ALEX_LEFT_KNEE = 1,   /**< Left Knee*/
    ALEX_RIGHT_HIP = 2,   /**< Right Hip*/
    ALEX_RIGHT_KNEE = 3,  /**< Right Knee*/
    ALEX_LEFT_ANKLE = 4,  /**< Right Hip*/
    ALEX_RIGHT_ANKLE = 5, /**< Right Knee*/
};


/**
 * \brief An enum type for robot communication with designed state machine
 * 
 */
enum class AlexState
{
    Init,         /**< 0 */
    InitSitting,  /**< 1 */
    LeftForward,  /**< 2 */
    RightForward, /**< 3 */
    Standing,     /**< 4 */
    Sitting,      /**< 5 */
    SittingDown,  /**< 6 */
    StandingUp,   /**< 7 */
    StepFirstL,   /**< 8 */
    StepFirstR,   /**< 9 */
    StepLastL,    /**< 10 */
    StepLastR,    /**< 11 */
    StepL,        /**< 12 */
    StepR,        /**< 13 */
    BackStepR,    /**< 14 */
    BackStepL,    /**< 15 */
    Error,        /**< 16 */
    Debug         /**< 17 */
};
/**
 * An enum class.
 * Different Robot motion profiles
 */
enum class RobotMode
{
    NORMALWALK,
    SITDWN,
    STNDUP,
    UPSTAIR,
    DWNSTAIR,
    TILTUP,
    TILTDWN,
    RAMPUP,
    RAMPDOWN,
    BKSTEP,
    FTTG,
    UNEVEN,
    INITIAL,
};

#endif /*ROBOT_PARAMS_H*/