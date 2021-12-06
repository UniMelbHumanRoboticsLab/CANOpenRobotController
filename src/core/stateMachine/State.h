/**
 * \file State.h
 * \author William Campbell
 * \version 0.1
 * \date 2019-09-24
 * For more detail on the architecture and mechanics of the state machine class see: https://embeded.readthedocs.io/en/latest/StaeMachines/.
 * \copyright Copyright (c) 2019
 *
 */

#ifndef STATE_H
#define STATE_H

#include <iostream>
#include <string>
#include <Eigen/Dense>


/**
 *  @ingroup stateMachine
 * \brief Abstract class representing a state in a StateMachine
 *
 */
class State {
    //friend class StateMachine; //TODO remove
    // A State machine class can access the private and protected members of a state class  */
   public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Required to use eigen fixed size vectors/objects in states. See first section of http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html.

    /**
     * \brief Construct a new State object
     *
     * \param n Name of the state machine
     */
    State(std::string n=""): _name(n) {};
    ~State() {};


    void doEntry() {
        _time_init = std::chrono::steady_clock::now();
        _time_running = 0;
        _iterations = 0;
        _time_dt = 0;
        _active = true;
        spdlog::debug("Entering {} state...", _name);
        entry();
    };

    void doDuring() {
        _iterations++;
        double tmp = (std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now() - _time_init).count()) / 1e6;
        _time_dt = tmp - _time_running;
        _time_running = tmp;
        during();
    };

    void doExit() {
        double tmp = (std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now() - _time_init).count()) / 1e6;
        _time_dt = tmp - _time_running;
        _time_running = tmp;
        exit();
        _active = false;
        spdlog::debug("Exited {} state...", _name);
    };


    /**
     * \brief Returns the name of the state
     *
     * \return The name of the state
     */
    const std::string &getName() {
        return _name;
    }

    /**
     * \brief Prints the name of the state
     *
     */
    void printName() {
        std::cout << _name << std::endl;
    }

    const unsigned long int & iterations() { return _iterations; }
    const double & dt() { return _time_dt; }
    const double & running() { return _time_running; }
    bool active() { return _active; }                               //!< True if state currently active, false otherwise.

   protected:
    /**
     * \brief Called once when the state is entered. Pure virtual function, must be overwritten by each state
     *
     */
    virtual void entry() = 0;

    /**
     * \brief Called continuously whilst in that state. Pure virtual function, must be overwritten by each state
     *
     */
    virtual void during() = 0;

    /**
     * \brief Called once when the state exits. Pure virtual function, must be overwritten by each state
     *
     */
    virtual void exit() = 0;

    std::string _name;                                  //!< Name of this State
    bool _active = false;
    unsigned long int _iterations = 0;                  //!< Number of iterations (running loops) of the state
    std::chrono::steady_clock::time_point _time_init;   //!< Initial time that state started
    double _time_dt = 0;                                //!< Last loop time in [s]
    double _time_running = 0;                           //!< Time elapsed since state entry in [s]
};

#endif  //STATE_H
