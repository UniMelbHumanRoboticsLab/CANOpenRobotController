/**
 * \file Event.h
 * \author William Campbell 
 * \version 0.1
 * \date 2019-09-24
 * For more detail on the architecture and mechanics of the state machine class see: https://embeded.readthedocs.io/en/latest/StaeMachines/.
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef EXO_EVENT_H
#define EXO_EVENT_H
//#include <cstddef>

#include "StateMachine.h"

/**
 *  @ingroup stateMachine
 * \brief Abstract class for events used as StateMachine triggers to transition between states.
 * Events must be explicitly tied to a current State and state to transition to once the event has been triggered. 
 * This is done using a <code>Transition</code> object in a designed StateMachine.
 * 
 */
class Event {
   public:
    StateMachine *owner; /*<!Pointer to the owner state machine for this event*/
    /* constructor */
    Event(StateMachine *p, const char n[] = NULL) {
        owner = p;
        name = n;
    };
    /**
     * \brief Virtual check function Must be implemented for each event. 
     * The check function is called each event loop to determine if a transition
     * has been triggered from the current state to the transition defined next state.
     *
    */
    virtual bool check(void) = 0;
    const char *getName(void);

   private:
    const char *name; /*<! Pointer to the name of this event*/
};

#endif  //EXO_EVENT_H
