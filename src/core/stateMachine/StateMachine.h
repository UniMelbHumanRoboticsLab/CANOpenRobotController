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

#include <csignal> //For raise()
#include "logging.h"
#include "State.h"
#include "Robot.h"
#include "LogHelper.h"

class StateMachine;

typedef std::function<bool(const StateMachine &)> TransitionCb_t; //TODO: make w/ template and pointer on specialised stateMachine?
typedef std::pair<TransitionCb_t, std::string> Transition_t;


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
    StateMachine();
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
    void setInitState(std::string state_name);//TODO: overload: with int i for state id and default 0 and string state_name


    /**
     * \brief Calls the entry method of the current state
     *
     */
    void activate();

    /**
     * \brief Processes the state machine. For each possible transition, checks if that transition should be made
     *  If no, calls during() on the current state
     *  If yes, calls exit() on the current state, entry() and then during() on the new state.
     *
     */
    virtual void update();

    /**
     * \brief Custom initialisation of the state machine
     *
     */
    virtual void init() = 0;

    /**
     * \brief End the state machine execution state
     *
     */
    virtual void end() = 0;

    //TODO: doc. If setInitState() is not used to define first execution state, the first added state is used instead.
    void addState(std::string state_name, std::shared_ptr<State> s_ptr) {
       states[state_name]=s_ptr;
       //Set first added state as default first for execution
       if(states.size()==1) {
          currentState=state_name;
       }
    }

    //TODO: doc. If setInitState() is not used to define first execution state, the first added state is used instead.
    void addTransition(std::string from, TransitionCb_t t_cb, std::string to) {
        if(states.count(from)>0 && states.count(to)>0) {
            transitions[from].push_back(Transition_t(t_cb, to));
        }
        else {
            spdlog::error("State {} or {} do not exist. Cannot create requested transition.", from, to);
        }
    }

    //TODO: doc.
    void addTransitionFromAny(TransitionCb_t t_cb, std::string to) {
        if(states.count(to)>0) {
            //Add transitions to all states (but target)
            for(const auto& [key, s]: states) {
                if(key!=to) {
                    transitions[key].push_back(Transition_t(t_cb, to));
                }
            }
        }
        else {
            spdlog::error("State {} do not exist. Cannot create requested transitions.", to);
        }
    }

    bool isRunning() { return running; }


    std::shared_ptr<State> & getCurrentState();
    std::shared_ptr<State> & getState(std::string state_name);

   private:
    std::vector<Transition_t> & getCurrentStateTransitions();


    /**
     * \brief Hardware update method called every loop (first thing) to update robot state...
     *
     */
    virtual void hwStateUpdate() = 0;

    /**
     * \brief Pointer to the current state
     *
     */
    std::string currentState;
    std::map<std::string, std::shared_ptr<State>> states; //Map of states

    std::map<std::string, std::vector<Transition_t>> transitions; //Map holding for each state a vector of possible std::pair transistions.

    bool running;

   protected:
    //TODO: to template, somehow?
    //Robot *robot;

    /**
     * \brief Custom spdlogger allowing to conveniently log Eigen Vectors (among other things)
     * Required to be initialised in the derived state machine init()
     */
    LogHelper logHelper;
};

#endif  //STATEMACHINE_H
