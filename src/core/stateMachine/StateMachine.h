/**
 * \file StateMachine.h
 * \author William Campbell, Vincent Crocher
 * \version 0.2
 * \date 2021-12-17
 * \copyright Copyright (c) 2019 - 2021
 *
 */
/**
 *  @defgroup stateMachine State Machine module
 * A group of abstract classes, used to build event-driven state machines.
 */
#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <csignal> //For raise()
#include "State.h"
#include "Robot.h"
#include "LogHelper.h"

class StateMachine; //Forward declaration for following type definitions

typedef std::function<bool(StateMachine &)> TransitionCb_t; //!< Type of Transition function callbacks to register using AddTransition()
typedef std::pair<TransitionCb_t, std::string> Transition_t;


/**
 * @ingroup stateMachine
 * \brief A generic (abstract class) state machine, managing States and Transitions between them as well as a Robot object. To be derived for each specific app (see examples and main documentation).
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
     * \brief Sets the initial (starting) state. If not used the first added state is used instead.
     *
     * \param state_name Registered name of the state.
     */
    void setInitState(std::string state_name);

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
     * \brief End the state machine execution (call current State exit() and disable logger and Robot).
     *
     */
    virtual void end();

    /**
     * \brief Default configureMasterPDOs() which call configureMasterPDOs() of set robot if one is set (using setRobot) in the derived stateMachine.
     * Can be overloaded if more configuration is required.
     *
     */
    virtual bool configureMasterPDOs();

    /**
     * \brief Assign a Robot to the stateMachine (optional). Typically called in derived stateMachine constructor at robot instantiation.
     * \param r A unique_ptr to a Robot object (e.g. setRobot(std::make_unique<RobotM3>("ROBOT_NAME")) )
     *
     */
    void setRobot(std::unique_ptr<Robot> r);

     /**
     * \brief Add a State instance to the stateMachine.
     * If setInitState() is not used to define first execution state, the first added state is used instead.
     * \param state_name A unique state name, used to access the state
     * \param s_ptr A shared_ptr to the State to be added.
     */
    void addState(std::string state_name, std::shared_ptr<State> s_ptr);
    /**
     * \brief Add a Transition (as a cb function) between two states registered using AddState().
     * If multiple transitions from the same state are active (true) simultaneously, the first registered one will be used.
     * \param t_cb A callback function (of type TransitionCb_t) to return true if transition is active and false otherwise
     * \param from Name of the state to allow Transition from
     * \param to Name of the state to allow Transition to
     */
    void addTransition(std::string from, TransitionCb_t t_cb, std::string to);
    /**
     * \brief Add a Transition (as a cb function) from the last State to which a transition has been added to (i.e. last 'std::string to' parameter) to another State.
     * If multiple transitions from the same state are active (true) simultaneously, the first registered one will be used.
     * \param t_cb A callback function (of type TransitionCb_t) to return true if transition is active and false otherwise
     * \param to Name of the state to allow Transition to
     */
    void addTransitionFromLast(TransitionCb_t t_cb, std::string to);
    /**
     * \brief Add a Transition (as a cb function) from any state already registered to one state.
     * Typically usefull for a "standby" state or clean "exit" state.
     * \param t_cb A callback function (of type TransitionCb_t) to return true if transition is active and false otherwise
     * \param to Name of the state to allow Transition to
     */
    void addTransitionFromAny(TransitionCb_t t_cb, std::string to);

    /**
     * \brief Return pointer to the State specified by its state_name
     *
     */
    std::shared_ptr<State> state(std::string state_name) { return _states[state_name]; }
    /**
     * \brief Return pointer to the State specified by its name with specicialised State type
     *
     */
    template <typename S>
    std::shared_ptr<S> state(std::string state_name) { return std::static_pointer_cast<S>(_states[state_name]); }
    /**
     * \brief Return pointer to current active State of the stateMachine
     *
     */
    std::shared_ptr<State> state() { return _states[_currentState]; }

    /**
     * \brief Is stateMachine running?
     *
     */
    bool running() { return _running; }

    /**
     * \brief Return stateMachine running time in [s]
     *
     */
    double & runningTime() { return _time_running; }


   protected:
    /**
     * \brief Hardware update method called every loop (first thing) to update the hardware (robot)
     * This default (base) version update the robot registered with the StateMachine (using setRobot).
     */
    virtual void hwStateUpdate();

    std::unique_ptr<Robot> _robot = nullptr;        //!< Set using setRobot method. Can be left null.

    /**
     * \brief Custom spdlogger allowing to conveniently log Eigen Vectors (among other things)
     * Required to be initialised in the derived state machine init()
     */
    LogHelper logHelper;

   private:
    std::string _currentState;                                      //!< Current active State name
    std::map<std::string, std::shared_ptr<State>> _states;          //!< Map of states, indexed and accessed by their names (string)
    std::map<std::string, std::vector<Transition_t>> _transitions;  //!< Map holding for each state a vector of possible std::pair transistions.
    std::string _lastToState;                                       //!< Hold the last state to which a transition has been registered to. Used for addTransitionFromLast().

    bool _running;                                      //!< running flag (set true at activate() stage)
    std::chrono::steady_clock::time_point _time_init;   //!< Initial time that machine started
    double _time_running=0;                             //!< Time elapsed since initialisation in [s]
};

#endif  //STATEMACHINE_H
