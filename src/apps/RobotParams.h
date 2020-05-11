/**
 * Specific paramaters and naming definitions for exoskeleton robot class implementetion.
 * 
 * Version 0.1
 * Date: 07/04/2020
 *
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
//Node ID for the 6 joints
enum robotJoints {
    LEFT_HIP = 0,
    LEFT_KNEE = 1,
    RIGHT_HIP = 2,
    RIGHT_KNEE = 3,
    LEFT_ANKLE = 4,
    RIGHT_ANKLE = 5
};

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

const double Q_MIN_MAX[12]{deg2rad(70), deg2rad(210),
                           0, deg2rad(120),
                           deg2rad(70), deg2rad(210),
                           0, deg2rad(120),
                           deg2rad(75), deg2rad(105),
                           deg2rad(75), deg2rad(105)};

//Params for specific robt
//Knee motor reading and corresponding angle. Used for mapping between degree and motor values.
#define KNEE_MOTOR_POS1 (250880)
#define KNEE_MOTOR_DEG1 (90)
#define KNEE_MOTOR_POS2 (0)
#define KNEE_MOTOR_DEG2 (0)
//Hip motor reading and corresponding angle. Used for mapping between degree and motor values.
#define HIP_MOTOR_POS1 (250880)
#define HIP_MOTOR_DEG1 (90)
#define HIP_MOTOR_POS2 (0)
#define HIP_MOTOR_DEG2 (180)
//Ankle motor reading and corresponding angle. Used for mapping between degree and motor values.
#define ANKLE_MOTOR_POS1 (0)
#define ANKLE_MOTOR_DEG1 (90)
#define ANKLE_MOTOR_POS2 (-800000)
#define ANKLE_MOTOR_DEG2 (115)

#endif /*ROBOT_PARAMS_H*/