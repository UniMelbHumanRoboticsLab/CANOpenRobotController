/**
 * \file DebugMacro.h
 * \author Justin Fong
 * \brief Defines some macros for debugging output
 * \version 0.1
 * \date 2020-04-09
 * \author William Campbell (you@domain.com)
 * \version 0.1
 * \date 2020-05-12
 * \copyright Copyright (c) 2020
 * 
 */
#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED
#include <iostream>
//#define NOROBOT
#define DEBUG
#ifdef DEBUG
#define DEBUG_OUT(x) (std::cout << x << std::endl);
#else
#define DEBUG_OUT(x) \
    do {             \
    } while (0);
#endif
#endif