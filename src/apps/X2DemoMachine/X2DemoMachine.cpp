#include "X2DemoMachine.h"

#define OWNER ((X2DemoMachine *)owner)

X2DemoMachine::X2DemoMachine() {
    robot = new X2Robot();
    // Create PRE-DESIGNED State Machine events and state objects.
    startExo = new StartExo(this);
    startNextMove = new StartNextMove(this);
    finishedSitting = new FinishedSitting(this);
    finishedStanding = new FinishedStanding(this);

    /**f
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    idleState = new IdleState(this, robot);
    //x2DemoState = new X2DemoState(this, robot);
    sittingState = new Sitting(this, robot);
    sittingDwnState = new SittingDwn(this, robot);
    standingState = new Standing(this, robot);
    standingUpState = new StandingUp(this, robot);

    NewTransition(idleState, startExo, standingState);
    NewTransition(sittingState, startNextMove, sittingDwnState);
    NewTransition(sittingDwnState, finishedSitting, standingState);
    NewTransition(standingState, startNextMove, standingUpState);
    NewTransition(standingUpState, finishedStanding, sittingState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(idleState);

    // Create ros object
    x2DemoMachineRos_ = new X2DemoMachineROS(robot);
}
/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void X2DemoMachine::init(int argc, char *argv[]) {
    DEBUG_OUT("X2DemoMachine::init()")
    initialised = robot->initialise();
    x2DemoMachineRos_->initialize(argc, argv);
    running = true;
}

void X2DemoMachine::end() {
    if (initialised) {
        currentState->exit();
        x2DemoMachineRos_->~X2DemoMachineROS();
        robot->~X2Robot();
    }
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool X2DemoMachine::StartExo::check(void) {
    if (OWNER->robot->keyboard.getS() == true) {
        std::cout << "Pressed S!" << std::endl;
        return true;
    }
    return false;
}

bool X2DemoMachine::StartNextMove::check(void) {
    if (OWNER->robot->keyboard.getS() == true) {
        std::cout << "Pressed S!" << std::endl;
        return true;
    }
    return false;
}

bool X2DemoMachine::FinishedStanding::check(void) {
    return (OWNER->standingUpState->trajFinished);
}

bool X2DemoMachine::FinishedSitting::check(void) {
    return (OWNER->sittingDwnState->trajFinished);
}

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void X2DemoMachine::hwStateUpdate(void) {
    robot->updateRobot();
}

void X2DemoMachine::update() {
    StateMachine::update();
    x2DemoMachineRos_->update();
}