/**
 * \file ALEXCrutchController.h
 * \author William Campbell
 * \brief The <code>ALEXCrutchController</code> class implements the pocket beagles 
 * selection screen of next motions and selection of the next motion.
 * It takes in keyboard input and outputs an int value related to the Alex Robots
 * Motion enum: RobotMode (found in RobotParams.h)
 * \version 0.1
 * \date 2020-04-26
 * \version 0.1
 * \copyright Copyright (c) 2020
 */

#ifndef PB_H_INCLUDED
#define PB_H_INCLUDED
#include <iostream>
#include <string>

#include "RobotParams.h"
#include "InputDevice.h"

#define NUMMODES 4
/**
 * @brief Abstract class representing any input device to be used in a Robot object 
 * 
 */
class ALEXCrutchController : public InputDevice {
   private:
    // hard coded to only sit stand walk, todo is add the others to the list as deemed important.
    RobotMode nextMotion[NUMMODES] = {RobotMode::NORMALWALK, RobotMode::SITDWN, RobotMode::STNDUP, RobotMode::UNEVEN};
    // track index in nextMotion[]
    int currentMode = 0;

    RPDO *goRPDO;
    RPDO *nextMovementRPDO;

    // UNSIGNED8 HB; // Heartbeat - to implement

   public:
    ALEXCrutchController();

    // These should probably be private...TODO
    UNSIGNED8 nextMovement;
    UNSIGNED8 goButton;

    /**
    * \brief update the current controller menu and print to the screen, if select is
    * pressed, return the RobotMode value for that motion to the caller.
    * 
    * @param up 
    * @param dwn 
    * @param select 
    * \return int value from RobotMode enum defining movement types
    */
    RobotMode updateController(bool up, bool dwn, bool select);
    /**
     * \brief print current menu of motions to the screen -> next Motion on crutch
     * 
     */
    void printMenu();
    /**
     * \brief print current int value for Robot Mode
     * 
     */
    int intRobotMode(RobotMode mode);
    /**
 * \brief print the current robot mode to screen
 * 
 * @param mode 
 * \return std::string 
 */
    std::string printRobotMode(RobotMode mode);

    void updateGO(bool go);

    bool getGo();

    void updateInput();
    bool configureMasterPDOs();
};

#endif