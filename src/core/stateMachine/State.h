/**
 * \file State.h
 * \author William Campbell
 * \version 0.1
 * \date 2019-09-24
 * For more detail on the architecture and mechanics of the state machine class see: https://embeded.readthedocs.io/en/latest/StaeMachines/.
 * \copyright Copyright (c) 2019
 *
 */

#ifndef EXO_STATE_H
#define EXO_STATE_H

#include <cstddef>
#include <iostream>
#include <string>
#include <Eigen/Dense>

class StateMachine;
class Transition;

#include "StateMachine.h"
#include "Transition.h"
#define MAXARCS 10 /*<!Define the max number of arcs (transitions) any state can have*/
/**
 *  @ingroup stateMachine
 * \brief Abstract class representing a state in a StateMachine
 *
 */
class State {
    friend class StateMachine;
    // A State machine class can access the private and protected members of a state class  */
   public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Required to use eigen fixed size vectors/objects in states. See first section of http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html.

    /**
     * \brief Pointer to the owning state machine
     *
     */
    StateMachine *owner;

    /**
     * \brief Construct a new State object
     *
     * \param p Pointer to the owning state machine
     * \param n Name of the state machine
     */
    State(StateMachine *p, const char n[] = NULL): owner(p), numarcs(0) {
        if(n==NULL)
            name = "";
        else
            name = n;
    };
    ~State();
    // Arc creating and accessing functions
    bool addArc(Transition *t);
    Transition *getActiveArc(void);

    /**
     * \brief Called once when the state is entered. Pure virtual function, must be overwritten by each state
     *
     */
    virtual void entry(void) = 0;

    /**
     * \brief Called continuously whilst in that state. Pure virtual function, must be overwritten by each state
     *
     */
    virtual void during(void) = 0;

    /**
     * \brief Called once when the state exits. Pure virtual function, must be overwritten by each state
     *
     */
    virtual void exit(void) = 0;

    /**
     * \brief Returns the name of the state
     *
     * \return The name of the state
     */
    const std::string &getName(void);

    /**
     * \brief Prints the name of the state
     *
     */
    void printName(void);

   private:
    /**
    * \brief List of possible transitions
    *
    */
    Transition *arclist[MAXARCS];  /*<!Array of transition objects this state can transition to on exit*/
    std::string name;              /*<!Pointer to the name of this State*/
    int numarcs;
};

#endif  //EXO_STATE_H
