/**
 * \file CANDevice.h
 * \author Justin Fong
 * \brief  Generic CANDevice Class
 *
 * \version 0.1
 * \date 2021-01-12
 * \version 0.1
 *
 * \copyright Copyright (c) 2021
 *
 */



// TODO - THIS IS NOT FINISHED YET, OR USED. 


#ifndef CANDEVICE_H_INCLUDED
#define CANDEVICE_H_INCLUDED

#include <CANopen.h>
#include <CO_command.h>
#include <string.h>

#include <vector>

#include "logging.h"

/**
 * @ingroup Robot
 * \brief Abstract class describing a CANOpen Device
 *
 */
class CANDevice {
   protected:
    /**
        * \brief The CAN Node ID used to address this particular device on the CAN bus
        *
        */
    int NodeID;


#endif
