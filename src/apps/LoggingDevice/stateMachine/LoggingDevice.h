/**
 * \file LoggingDevice.h
 * \author William Campbell
 * \version 0.1
 * \date 2019-09-24
 * \copyright Copyright (c) 2020
 *
 *
 */
#ifndef LOGGINGDEVICE_SM_H
#define LOGGINGDEVICE_SM_H

#include "StateMachine.h"
#include "LoggingRobot.h"

// State Classes
#include "InitState.h"
#include "IdleState.h"
#include "CalibrateState.h"
#include "RecordState.h"

/**
 * @brief Example implementation of a StateMachine for the ExoRobot class. States should implemented ExoTestState
 *
 */
class LoggingDevice : public StateMachine {
   public:
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    LoggingDevice();
    void init();
    
    bool trajComplete;

    LoggingRobot *robot() { return static_cast<LoggingRobot*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)
};

#endif
