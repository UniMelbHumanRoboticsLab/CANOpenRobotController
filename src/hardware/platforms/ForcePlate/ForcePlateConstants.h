/**
 * @file ForcePlateConstants.h
 * @author Justin Fong
 * @brief 
 * @version 0.1
 * @date 2021-02-23
 * 
 * @copyright Copyright (c) 2021
 * 
 * This file defines constants which are to be used for a force plate. 
 */
#ifndef FORCEPLATECONSTANTS_H_INCLUDED
#define FORCEPLATECONSTANTS_H_INCLUDED

enum ForcePlateCommand {
    NONE = 0,
    CALIBRATE = 1,
    STARTSTREAM = 2,
    RECORD = 3,
    STOP = 4,
};

#endif