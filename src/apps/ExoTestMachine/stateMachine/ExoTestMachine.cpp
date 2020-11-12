
#include "ExoTestMachine.h"

#define OWNER ((ExoTestMachine *)owner)

ExoTestMachine::ExoTestMachine() {
    trajectoryGenerator = new DummyTrajectoryGenerator(6);
    robot = new X2Robot();

    // Create PRE-DESIGNED State Machine events and state objects.
    isAPressed = new IsAPressed(this);
    endTraj = new EndTraj(this);
    startButtonsPressed = new StartButtonsPressed(this);
    startExo = new StartExo(this);
    startSit = new StartSit(this);
    startStand = new StartStand(this);
    initState = new InitState(this, robot, trajectoryGenerator);
    standing = new Standing(this, robot, trajectoryGenerator);
    sitting = new Sitting(this, robot, trajectoryGenerator);
    standingUp = new StandingUp(this, robot, trajectoryGenerator);
    sittingDwn = new SittingDwn(this, robot, trajectoryGenerator);

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    NewTransition(initState, startExo, sitting);
    NewTransition(sitting, startStand, standingUp);
    NewTransition(standingUp, endTraj, standing);
    NewTransition(standing, startSit, sittingDwn);
    NewTransition(sittingDwn, endTraj, sitting);
    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(initState);
}
/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void ExoTestMachine::init() {
    spdlog::debug("ExoTestMachine::init()");
    initialised = robot->initialise();
    running = true;

    // Initialising the data logger
    time0 = std::chrono::steady_clock::now();
    dataLogger.initLogger("test_logger", "logs/testLog.csv", LogFormat::CSV, true);
    dataLogger.add(time, "time");
    dataLogger.add(robot->getPosition(), "JointPositions");
    dataLogger.startLogger();
}

void ExoTestMachine::end() {
    spdlog::debug("Ending ExoTestMachine");
    dataLogger.endLog();
    delete robot;
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
/**
     * \brief poll the trajectory Generators flag to see if the currently loaded motion is complete
     *
     */
bool ExoTestMachine::EndTraj::check() {
    return OWNER->trajectoryGenerator->isTrajectoryFinished();
}
bool ExoTestMachine::IsAPressed::check(void) {
    if (OWNER->robot->keyboard->getA() == true) {
        return true;
    }
    return false;
}
bool ExoTestMachine::StartButtonsPressed::check(void) {
    if (OWNER->robot->keyboard->getW() == true) {
        return true;
    }
    return false;
}
bool ExoTestMachine::StartExo::check(void) {
    if (OWNER->robot->keyboard->getS() == true) {
        spdlog::info("LEAVING INIT and entering Sitting");
        return true;
    }
    return false;
}
bool ExoTestMachine::StartStand::check(void) {
    if (OWNER->robot->keyboard->getW() == true) {
        return true;
    }
    return false;
}

bool ExoTestMachine::StartSit::check(void) {
    if (OWNER->robot->keyboard->getW()) {
        return true;
    }
    return false;
}
/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void ExoTestMachine::hwStateUpdate(void) {
    robot->updateRobot();
}

/**
 * \brief Statemachine update: overloaded to include logging
 *
 */
void ExoTestMachine::update() {
    // Update time (used for log)
    time = (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - time0)
                .count()) /
           1e6;
    spdlog::debug("Update()");
    StateMachine::update();
    dataLogger.recordLogData();
}
