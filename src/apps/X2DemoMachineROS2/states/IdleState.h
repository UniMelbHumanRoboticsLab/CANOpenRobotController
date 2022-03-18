/**
 * /file IdleState.h
 * /author Justin Fong, Emek Baris Kucuktabak
 * /brief Concrete implementation of IdleState
 * /version 0.1
 * /date 2020-07-06
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef INITSTATE_H_INCLUDED
#define INITSTATE_H_INCLUDED

#include "State.h"
#include "X2Robot.h"

/**
 * \brief Idle State for the X2DemoMachineROS2
 *
 * State holds until event triggers its exit, and runs initPositionControl on exit
 * Control of transition is independent of this class and is defined in X2DemoMachineROS2.
 *
 */
class IdleState : public State {
    X2Robot *robot;

public:
    void entry(void);
    void during(void);
    void exit(void);
    IdleState(StateMachine *m, X2Robot *exo, const char *name = NULL) : State(m, name), robot(exo){};
};

#endif