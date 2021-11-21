/**
 * \file StateMachine.h
 * \author William Campbell
 * \version 0.1
 * \date 2019-09-24
 * For more detail on the architecture and mechanics of the state machine class see: https://embeded.readthedocs.io/en/latest/StaeMachines/.
 * \copyright Copyright (c) 2019
 *
 */
/**
 *  @defgroup stateMachine State Machine module
 * A group of abstract classes, used to build event-driven state machines.
 */
#ifndef STATEMACHINE_H
#define STATEMACHINE_H

class State;

#include "logging.h"
#include "State.h"
#include "Robot.h"
#include "LogHelper.h"

/**
 * @ingroup stateMachine
 * \brief Abstract class representing a state machine. Includes a number of State and Transition objects
 *
 */
class StateMachine {
   public:
    /**
     * \brief Construct a new State Machine object
     *
     */
    StateMachine(void);
    /**
     * \brief Default destructor
     *
     */
    virtual ~StateMachine(){};
    /**
     * \brief Sets the current state. Note: No check made
     *
     * \param i Pointer to the desired current state.
     */
    void initialize(State *i);//TODO: overload: with int i for state id and default 0 and string state_name

    /**
     * \brief Returns a pointer to the current state
     *
     * \return State* Pointer to the current state
     */
    State *getCurState(void);

    /**
     * \brief Calls the entry method of the current state
     *
     */
    void activate(void);

    /**
     * \brief Processes the state machine. For each possible transition, checks if that transition should be made
     *  If no, calls during() on the current state
     *  If yes, calls exit() on the current state, entry() and then during() on the new state.
     *
     */
    virtual void update(void);

    /**
     * \brief Custom initialisation of the state machine
     *
     */
    virtual void init(void) = 0;

    /**
     * \brief End the state machine execution state
     *
     */
    virtual void end(void) = 0;

    /**
     * \brief Hardware update method called every loop (first thing) to update robot state...
     *
     */
    virtual void hwStateUpdate() = 0;

    //template<class S=State> //TODO???
    void addState(std::string state_name, std::shared_ptr<State> s_ptr) {
       states[state_name]=s_ptr;
    };

    //TODO: addTransition method either by state name or index
    //for state name use a states.find(name) to check if exists.

   protected:
    /**
     * \brief Pointer to the current state
     *
     */
    State *currentState;
    std::map<std::string, std::shared_ptr<State>> states;

    bool initialised = false;

    //TODO: to template, somehow?
    Robot *robot;

    /**
     * \brief Custom spdlogger allowing to conveniently log Eigen Vectors (among other things)
     * Required to be initialised in the derived state machine init()
     */
    LogHelper logHelper;

};

#endif  //STATEMACHINE_H
