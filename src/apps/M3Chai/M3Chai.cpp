#include "M3Chai.h"

#define OWNER ((M3Chai *)owner)


#define IP_ADDRESS "192.168.6.2" //Local IP address to use

M3Chai::M3Chai() {
    robot = new RobotM3();
    chaiServer = new server(6, 3);

    // Create PRE-DESIGNED State Machine events and state objects.
    calibState = new M3CalibState(this, robot);
    waitForCommunicationState = new M3ChaiWaitForCommunication(this, robot, chaiServer);
    communicationState = new M3ChaiCommunication(this, robot, chaiServer);
    endCalib = new EndCalib(this);
    isConnected = new IsConnected(this);
    isDisconnected = new IsDisconnected(this);

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    NewTransition(calibState, endCalib, waitForCommunicationState);
    NewTransition(waitForCommunicationState, isConnected, communicationState);
    NewTransition(communicationState, isDisconnected, waitForCommunicationState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(calibState);
}
M3Chai::~M3Chai() {
    delete chaiServer;
    delete calibState;
    delete communicationState;
    delete waitForCommunicationState;
    delete endCalib;
    delete isConnected;
    delete isDisconnected;
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */

void M3Chai::init() {
    spdlog::debug("M3Chai::init()");
    if(robot->initialise()) {
        initialised = true;
        if(chaiServer->Connect(IP_ADDRESS)!=0) {
            spdlog::critical("M3ChaiCommunication: Unable to initialise socket... Quitting.");
            raise(SIGTERM); //Clean exit
        }
    }
    else {
        initialised = false;
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
    running = true;
}

void M3Chai::end() {
    if(initialised) {
        currentState->exit();
        robot->disable();
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

bool M3Chai::IsConnected::check() {
    return OWNER->chaiServer->IsConnected();
}

bool M3Chai::IsDisconnected::check() {
    return !OWNER->chaiServer->IsConnected();
}
