#include "X2DemoMachine.h"

#define OWNER ((X2DemoMachine *)owner)

X2DemoMachine::X2DemoMachine() {
    robot = new ExoRobot();
    // Create PRE-DESIGNED State Machine events and state objects.
    startExo = new StartExo(this);
    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    idleState = new IdleState(this, robot);
    std::cout<<"[X2DemoMachine::X2DemoMachine()]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    NewTransition(idleState, startExo, idleState);
    std::cout<<"[X2DemoMachine::X2DemoMachine()]222222222222222222222222"<<std::endl;
    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(idleState);

}
/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void X2DemoMachine::init() {
    DEBUG_OUT("ExoTestMachine::init()")
    robot->initialise();
    running = true;
}

void X2DemoMachine::end() {
    if(initialised) {
        currentState->exit();
        robot->~ExoRobot();
    }
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool X2DemoMachine::StartExo::check(void) {
    if (OWNER->robot->keyboard->getS() == true) {
        std::cout << "Pressed S!" << std::endl;
        return true;
    }
    return false;
}
/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void X2DemoMachine::hwStateUpdate(void) {
    robot->updateRobot();
}