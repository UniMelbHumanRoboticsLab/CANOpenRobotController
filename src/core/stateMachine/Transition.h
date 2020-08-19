/**
 * \file Transition.h
 * \author William Campbell 
 * \version 0.1
 * \date 2019-09-24
 * For more detail on the architecture and mechanics of the state machine class see: https://embeded.readthedocs.io/en/latest/StaeMachines/.
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef TRANSITION_H
#define TRANSITION_H

#include "Event.h"
#include "State.h"
#include "StateMachine.h"

/* Forward declarations*/
class State;
class Event;

/**
 * @ingroup stateMachine
 * \brief Represents possible transitions linking two State objects with an Event. 
 * 
 */
class Transition {
    friend class State;
    friend class StateMachine;

   public:
    /* Constructor: set state this arc targets (points towards) and the event which triggers it */
    Transition(State* targ, Event* e) {
        target = targ;
        ev = e;
    };
    State* getTarget(void);

   private:
    Event* ev;     /*<! pointer to tranisitions event object - triggering a tranistion to the target state*/
    State* target; /*<! target State of the transition*/
};
#endif  //EXO_TRANSITION_H
