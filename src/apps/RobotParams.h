/**
 * \file RobotParams.h
 * \author William Campbell
 * \brief Specific paramaters and naming definitions for exoskeleton robot class implementetion.
 * 
 * \version 0.1
 * \date 2020-04-09
 * \version 0.1
 * \copyright Copyright (c) 2020
 */
#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

//#define _NOANKLES  //w / o ankles
#ifndef _NOANKLES
#define NUM_JOINTS 6
#else
#define NUM_JOINTS 4
#endif
// Macros
#define deg2rad(deg) ((deg)*M_PI / 180.0)
#define rad2deg(rad) ((rad)*180.0 / M_PI)

/**
 * An enum type.
 * Joint Index for the 6 joints (note, NODEID = this + 1)
 */
enum robotJoints {
    LEFT_HIP = 0,   /**< Left Hip*/
    LEFT_KNEE = 1,  /**< Left Knee*/
    RIGHT_HIP = 2,  /**< Right Hip*/
    RIGHT_KNEE = 3, /**< Right Knee*/
    LEFT_ANKLE = 4, /**< Left Ankle*/
    RIGHT_ANKLE = 5 /**< Right Knee*/
};
/**
 * An enum class.
 * Different Robot motion profiles
 */
enum class RobotMode {
    NORMALWALK,
    SITDWN,
    STNDUP,
    UPSTAIR,
    DWNSTAIR,
    TILTUP,
    TILTDWN,
    BKSTEP,
    FTTG,
    UNEVEN,
    INITIAL
};
/**
 * 
 * Paramater definitions: Knee motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
#define KNEE_MOTOR_POS1 (250880)
#define KNEE_MOTOR_DEG1 (90)
#define KNEE_MOTOR_POS2 (0)
#define KNEE_MOTOR_DEG2 (0)
/**
 * 
 * Paramater definitions: Hip motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
#define HIP_MOTOR_POS1 (250880)
#define HIP_MOTOR_DEG1 (90)
#define HIP_MOTOR_POS2 (0)
#define HIP_MOTOR_DEG2 (180)
/**
 * 
 * Paramater definitions: Ankle motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
#define ANKLE_MOTOR_POS1 (0)
#define ANKLE_MOTOR_DEG1 (90)
#define ANKLE_MOTOR_POS2 (-800000)
#define ANKLE_MOTOR_DEG2 (115)

#endif /*ROBOT_PARAMS_H*/