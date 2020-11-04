#include "M1DemoMachine.h"

#define OWNER ((M1DemoMachine *)owner)

M1DemoMachine::M1DemoMachine() {
//    std::cout << "M1DemoMachine::constructed!" << std::endl;
    robot = new RobotM1();

    // Create PRE-DESIGNED State Machine state objects.
    demoState = new M1DemoState(this, robot);
    idleState = new IdleState(this, robot);
    monitorState = new Monitoring( this, robot);
    positionTracking = new M1PositionTracking(this, robot);

    // Create PRE-DESIGNED State Machine events objects.
    event2Demo = new Event2Demo(this);
    event2Monitor = new Event2Monitor(this);
    event2Idle = new Event2Idle(this);
    event2Pos = new Event2Pos(this);

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    NewTransition(idleState, event2Demo, demoState);
    NewTransition(demoState, event2Idle, idleState);

    NewTransition(idleState, event2Monitor, monitorState);
    NewTransition(monitorState, event2Idle, idleState);

    NewTransition(idleState, event2Pos, positionTracking);
    NewTransition(positionTracking, event2Idle, idleState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(idleState);
}

M1DemoMachine::~M1DemoMachine() {
    delete demoState;
    delete idleState;
    delete monitorState;
    delete positionTracking;
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */

void M1DemoMachine::init() {
//    std::cout << "M1DemoMachine::init()" << std::endl;
    if(robot->initialise()) {
        initialised = true;
    }
    else {
        initialised = false;
        std::cout /*cerr is banned*/ << "Failed robot initialisation. Exiting..." << std::endl;
        std::raise(SIGTERM); //Clean exit
    }
    running = true;
}

void M1DemoMachine::end() {
    if(initialised) {
        currentState->exit();
        robot->stop();
    }
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M1DemoMachine::hwStateUpdate(void) {
//    spdlog::debug("hw/**/StateUpdate!");
//    std::cout << "M1DemoMachine::hwStateUpdate()" << std::endl;
    robot->updateRobot();
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool M1DemoMachine::Event2Demo::check(void) {
    if (OWNER->robot->keyboard->getS() == true) {
        std::cout << "Pressed S!" << std::endl;
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool M1DemoMachine::Event2Monitor::check(void) {
    if (OWNER->robot->keyboard->getX() == true) {
        std::cout << "Pressed S!" << std::endl;
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool M1DemoMachine::Event2Idle::check(void) {
    if (OWNER->robot->keyboard->getQ() == true) {
        std::cout << "Pressed Q!" << std::endl;
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool M1DemoMachine::Event2Pos::check(void) {
    if (OWNER->robot->keyboard->getA() == true) {
//        std::cout << "Pressed A!" << std::endl;
        return true;
    }
    return false;
}