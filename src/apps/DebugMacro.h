/**
 * @brief Defines some macros for debugging output
 * 
 */
#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED
#include <iostream>
#define NOROBOT
#define DEBUG
#ifdef DEBUG
#define DEBUG_OUT(x) (std::cout << x << std::endl);
#else
#define DEBUG_OUT(x) \
    do {             \
    } while (0);
#endif
#endif