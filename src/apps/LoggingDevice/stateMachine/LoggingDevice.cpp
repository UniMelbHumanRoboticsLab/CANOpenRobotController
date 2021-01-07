
#include "LoggingDevice.h"

#define OWNER ((LoggingDevice *)owner)

LoggingDevice::LoggingDevice() {
    robot = new LoggingRobot();

    // Events
    isAPressed = new IsAPressed(this);

    // States
    initState = new InitState(this, robot);

    // Transitions
    NewTransition(initState, isAPressed, initState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(initState);
}

void LoggingDevice::init() {
    spdlog::debug("LoggingDevice::init()");
    initialised = robot->initialise();
    running = true;

    // Initialising the data logger
    time0 = std::chrono::steady_clock::now();
    dataLogger.initLogger("test_logger", "logs/testLog.csv", LogFormat::CSV, true);
    dataLogger.add(time, "time");
    dataLogger.add(robot->getPosition(), "JointPositions");
    dataLogger.startLogger();
}

void LoggingDevice::end() {
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

bool LoggingDevice::IsAPressed::check(void) {
    if (OWNER->robot->keyboard->getA() == true) {
        return true;
    }
    return false;
}

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void LoggingDevice::hwStateUpdate(void) {
    robot->updateRobot();
}

/**
 * \brief Statemachine update: overloaded to include logging
 *
 */
void LoggingDevice::update() {
    // Update time (used for log)
    time = (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - time0)
                .count()) /
           1e6;
    spdlog::debug("Update()");
    StateMachine::update();
    dataLogger.recordLogData();
}
