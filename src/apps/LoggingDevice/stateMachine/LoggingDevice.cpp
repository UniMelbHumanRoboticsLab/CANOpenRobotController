
#include "LoggingDevice.h"

#define OWNER ((LoggingDevice *)owner)

LoggingDevice::LoggingDevice() {
    robot = new LoggingRobot();

    // Events
    isAPressed = new IsAPressed(this);
    isSPressed = new IsSPressed(this);
    isCalibrationFinished = new IsCalibrationFinished(this);

    // States
    initState = new InitState(this, robot);
    idleState = new IdleState(this, robot);
    calibrateState = new CalibrateState(this, robot);
    recordState = new RecordState(this, robot);

    // Transitions
    NewTransition(initState, isAPressed, idleState);
    NewTransition(idleState, isAPressed, calibrateState);
    NewTransition(calibrateState, isCalibrationFinished, idleState);
    NewTransition(idleState, isSPressed, recordState);
    NewTransition(recordState, isSPressed, idleState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(initState);
}

void LoggingDevice::init() {
    spdlog::info("LoggingDevice::init()");
    initialised = robot->initialise();
    running = true;

    // Initialising the data logger
    time0 = std::chrono::steady_clock::now();
    dataLogger.initLogger("test_logger", "logs/testLog.csv", LogFormat::CSV, true);
    dataLogger.add(time, "time");
    dataLogger.add(robot->getCrutchReadings(), "CrutchReadings");
    dataLogger.startLogger();
}

void LoggingDevice::end() {
    spdlog::debug("Ending Logging Device");
    dataLogger.endLog();
    delete robot;
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
/**
     * \brief Keyboard States
     *
     */

bool LoggingDevice::IsAPressed::check(void) {
    if (OWNER->robot->keyboard->getA() == true) {

        return true;
    }
    return false;
}
bool LoggingDevice::IsSPressed::check(void) {
    if (OWNER->robot->keyboard->getS() == true) {
        return true;
    }
    return false;
}

bool LoggingDevice::IsCalibrationFinished::check(void) {
    if (OWNER->calibrateState->getCurrReading() < NUM_CALIBRATE_READINGS) {
        return false;
    }
    return true;
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
    StateMachine::update();
    dataLogger.recordLogData();
}

void LoggingDevice::configureMasterPDOs() {
    spdlog::debug("LoggingDevice::configureMasterPDOs()");
    robot->configureMasterPDOs();
}

