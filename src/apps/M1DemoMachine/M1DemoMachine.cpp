#include "M1DemoMachine.h"

#define OWNER ((M1DemoMachine *)owner)

M1DemoMachine::M1DemoMachine() {
    robot = new RobotM1();

    // Create PRE-DESIGNED State Machine state objects.
    demoState = new M1DemoState(this, robot);
    //calibState = new M1CalibState(this, robot);
    //standbyState = new M1MassCompensation(this, robot);
    //endEffDemoState = new M1EndEffDemo(this, robot);
    //impedanceState = new M1DemoImpedanceState(this, robot);
    //timingState = new M1SamplingEstimationState(this, robot);
    //endCalib = new EndCalib(this);
    idleState = new IdleState(this, robot);
    monitorState = new Monitoring( this, robot);

    // Create PRE-DESIGNED State Machine events objects.
    event2Demo = new Event2Demo(this);
    event2Monitor = new Event2Monitor(this);
    event2Idle = new Event2Idle(this);

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    //NewTransition(calibState, endCalib, timingState);
    //NewTransition(calibState, endCalib, standbyState);
    //NewTransition(calibState, endCalib, endEffDemoState);
    NewTransition(idleState, event2Demo, demoState);
    NewTransition(idleState, event2Monitor, monitorState);
    NewTransition(demoState, event2Idle, idleState);
    NewTransition(monitorState, event2Idle, idleState);
//    NewTransition(idleState, monitorExo, monitorState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(idleState);
}

M1DemoMachine::~M1DemoMachine() {
    delete demoState;
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */

void M1DemoMachine::init() {
    DEBUG_OUT("M1DemoMachine::init()")
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
