#include "M1DemoMachine.h"

#define OWNER ((M1DemoMachine *)owner)

M1DemoMachine::M1DemoMachine() {
    robot = new RobotM1();

    // Create PRE-DESIGNED State Machine events and state objects.
    demoState = new M1DemoState(this, robot);
    //calibState = new M1CalibState(this, robot);
    //standbyState = new M1MassCompensation(this, robot);
    //endEffDemoState = new M1EndEffDemo(this, robot);
    //impedanceState = new M1DemoImpedanceState(this, robot);
    //timingState = new M1SamplingEstimationState(this, robot);
    //endCalib = new EndCalib(this);
    idleState = new IdleState(this, robot);
    monitorState = new Monitoring( this, robot);

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    //NewTransition(calibState, endCalib, timingState);
    //NewTransition(calibState, endCalib, standbyState);
    //NewTransition(calibState, endCalib, endEffDemoState);
    NewTransition(idleState, startExo, demoState);
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
bool M1DemoMachine::StartExo::check(void) {
    if (OWNER->robot->keyboard->getS() == true) {
        std::cout << "Pressed S!" << std::endl;
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool M1DemoMachine::MonitorExo::check(void) {
    if (OWNER->robot->keyboard->getX() == true) {
        std::cout << "Pressed S!" << std::endl;
        return true;
    }
    return false;
}
