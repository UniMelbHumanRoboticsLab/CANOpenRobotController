#include "M3Chai.h"

#define OWNER ((M3Chai *)owner)

M3Chai::M3Chai() {
    robot = new RobotM3();
    chaiServer = new server(3, 3);

    // Create PRE-DESIGNED State Machine events and state objects.
    calibState = new M3CalibState(this, robot);
    communicationState = new M3ChaiCommunication(this, robot, chaiServer);
    endCalib = new EndCalib(this);

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    NewTransition(calibState, endCalib, communicationState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(calibState);
}
M3Chai::~M3Chai() {
    delete chaiServer;
    delete calibState;
    delete communicationState;
    delete endCalib;
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */

void M3Chai::init() {
    DEBUG_OUT("M3Chai::init()")
    if(robot->initialise()) {
        initialised = true;
        if(chaiServer->Connect("192.168.6.2")!=0) {
            std::cout /*cerr is banned*/ << "M3ChaiCommunication: Unable to initialise socket... Quitting." <<std::endl;
            raise(SIGTERM); //Clean exit
        }
    }
    else {
        initialised = false;
        std::cout /*cerr is banned*/ << "Failed robot initialisation. Exiting..." << std::endl;
        std::raise(SIGTERM); //Clean exit
    }
    running = true;
}

void M3Chai::end() {
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
void M3Chai::hwStateUpdate(void) {
    robot->updateRobot();
}



bool M3Chai::EndCalib::check() {
    return OWNER->calibState->isCalibDone();
}
