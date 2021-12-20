/**
 * \file State.h
 * \author William Campbell, Vincent Crocher
 * \version 0.2
 * \date 2021-12-17
 * \copyright Copyright (c) 2019 - 2021
 *
 */

#ifndef STATE_H
#define STATE_H

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "logging.h"

/**
 *  @ingroup stateMachine
 * \brief Abstract class representing a State in a StateMachine. To be derived for each State type to be implemented within a specific StateMachine.
 *
 */
class State {
    friend class StateMachine;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Required to use eigen fixed size vectors/objects in states. See first section of http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html.

    /**
     * \brief Construct a new State object
     *
     * \param n Name of the State
     */
    State(std::string n=""): _name(n) {};
    ~State() {};


    /**
     * \brief Returns the name of the state (for back compatibility)
     *
     * \return The name of the state
     */
    const std::string &getName() {
        return _name;
    }
    /**
     * \brief Returns the name of the state
     *
     * \return The name of the state
     */
    const std::string &name() {
        return _name;
    }

    /**
     * \brief Prints the name of the state
     *
     */
    void printName() {
        std::cout << _name << std::endl;
    }

    const unsigned long int & iterations() { return _iterations; }  //!< Number of State iterations (of during() calls) since last entry())
    const double & dt() { return _time_dt; }                        //!< Last iteration running time if State is active, in [s]
    const double & running() { return _time_running; }              //!< Running time of the State (0 if not active yet) in [s]
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
    bool _active = false;                               //!< Is State currently active?
    unsigned long int _iterations = 0;                  //!< Number of iterations (running loops) of the state
    std::chrono::steady_clock::time_point _time_init;   //!< Initial time that state started
    double _time_dt = 0;                                //!< Last loop time in [s]
    double _time_running = 0;                           //!< Time elapsed since state entry in [s]


   private:
    /**
     * \brief For internal use only. Not to overload.
     *
     */
    void doEntry() {
        _time_init = std::chrono::steady_clock::now();
        _time_running = 0;
        _iterations = 0;
        _time_dt = 0;
        _active = true;
        spdlog::debug("Entering {} state...", _name);
        entry();
    };
    /**
     * \brief For internal use only. Not to overload.
     *
     */
    void doDuring() {
        _iterations++;
        double tmp = (std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now() - _time_init).count()) / 1e6;
        _time_dt = tmp - _time_running;
        _time_running = tmp;
        during();
    };
    /**
     * \brief For internal use only. Not to overload.
     *
     */
    void doExit() {
        double tmp = (std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now() - _time_init).count()) / 1e6;
        _time_dt = tmp - _time_running;
        _time_running = tmp;
        exit();
        _active = false;
        spdlog::debug("Exited {} state...", _name);
    };
};

#endif  //STATE_H
