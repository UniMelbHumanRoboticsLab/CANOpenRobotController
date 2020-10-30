#include "M3DemoMachine.h"

#define OWNER ((M3DemoMachine *)owner)

M3DemoMachine::M3DemoMachine() {
    robot = new RobotM3();

    // Create PRE-DESIGNED State Machine events and state objects.
    testState = new M3DemoState(this, robot);
    calibState = new M3CalibState(this, robot);
    standbyState = new M3MassCompensation(this, robot);
    endEffDemoState = new M3EndEffDemo(this, robot);
    impedanceState = new M3DemoImpedanceState(this, robot);
    pathState = new M3DemoPathState(this, robot);
    minJerkState = new M3DemoMinJerkPosition(this, robot);
    timingState = new M3SamplingEstimationState(this, robot);

    endCalib = new EndCalib(this);
    goToState1 = new GoToState1(this);
    goToState2 = new GoToState2(this);
    goToState3 = new GoToState3(this);
    goToState4 = new GoToState4(this);


    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
     NewTransition(calibState, endCalib, standbyState);
     NewTransition(standbyState, goToState2, pathState);
     NewTransition(pathState, goToState2, minJerkState);
     NewTransition(minJerkState, goToState2, timingState);
     NewTransition(timingState, goToState2, endEffDemoState);
     NewTransition(endEffDemoState, goToState2, impedanceState);
     NewTransition(impedanceState, goToState2, pathState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(calibState);
    //StateMachine::initialize(testState);
}
M3DemoMachine::~M3DemoMachine() {
    delete testState;
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void M3DemoMachine::init() {
    spdlog::debug("M3DemoMachine::init()");
    if(robot->initialise()) {
        initialised = true;
        logHelper.initLogger("M3DemoMachineLog", "logs/M3DemoMachine.csv", LogFormat::CSV, true);
        logHelper.add(time_running, "Time (s)");
        logHelper.add(robot->getPosition(), "JointPositions");
        logHelper.add(robot->getVelocity(), "JointVelocities");
        logHelper.add(robot->getTorque(), "JointTorques");
        logHelper.startLogger();
    }
    else {
        initialised = false;
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
    running = true;
    time_init = std::chrono::steady_clock::now();
    time_running = 0;
}

void M3DemoMachine::end() {
    if(initialised) {
        if(logHelper.isStarted())
            logHelper.endLog();
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
void M3DemoMachine::hwStateUpdate(void) {
    time_running = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time_init).count()) / 1e6;
    robot->updateRobot();
}



bool M3DemoMachine::EndCalib::check() {
    return OWNER->calibState->isCalibDone();
}


bool M3DemoMachine::GoToState1::check() {
    return (OWNER->robot->joystick->isButtonPressed(0) || OWNER->robot->keyboard->getNb()==0);
}
bool M3DemoMachine::GoToState2::check() {
    return (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==1);
}
bool M3DemoMachine::GoToState3::check() {
    return (OWNER->robot->joystick->isButtonPressed(2) || OWNER->robot->keyboard->getNb()==2);
}
bool M3DemoMachine::GoToState4::check() {
    return (OWNER->robot->joystick->isButtonPressed(3) || OWNER->robot->keyboard->getNb()==3);
}

