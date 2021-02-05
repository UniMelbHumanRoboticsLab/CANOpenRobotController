/**
 * \file M3Chai.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-07-04
 * \copyright Copyright (c) 2020
 *
 * /brief The <code>M3Chai</code> class is a state machine/app providing support of M3 for Chai3D.
 *
 */
#ifndef M3_SM_H
#define M3_SM_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <csignal> //For raise()

#include "RobotM3.h"
#include "StateMachine.h"

// State Classes
#include "M3ChaiStates.h"

/**
 * @brief StateMachine for the M3Robot providing support for Chai3D.
 *
 */
class M3Chai : public StateMachine {
   public:
    bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M3Chai();
    ~M3Chai();
    void init();
    void end();

    void hwStateUpdate();
    bool configureMasterPDOs();
    
    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    M3CalibState *calibState;
    M3ChaiWaitForCommunication *waitForCommunicationState;
    M3ChaiCommunication *communicationState;

   protected:
    RobotM3 *robot; /*<!Pointer to the Robot*/
    server *chaiServer; /*<! Pointer to communication server for Chai*/

   private:
    EventObject(EndCalib) * endCalib;
    EventObject(IsConnected) * isConnected;
    EventObject(IsDisconnected) * isDisconnected;
};

#endif /*M3_SM_H*/
