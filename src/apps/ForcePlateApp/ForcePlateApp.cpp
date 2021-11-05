
#include "ForcePlateApp.h"

#define OWNER ((ForcePlateApp *)owner)

ForcePlateApp::ForcePlateApp() {
    //robot = new ForcePlate4();
    robot = new ForcePlate();
    // Events
    startCalibrate = new StartCalibrate(this);
    startRecord = new StartRecord(this);
    stopRecord = new StopRecord(this);
    isCalibrationFinished = new IsCalibrationFinished(this);
    completeInit = new CompleteInit(this);

    // States
    initState = new InitState(this, robot);
    idleState = new IdleState(this, robot);
    calibrateState = new CalibrateState(this, robot);
    recordState = new RecordState(this, robot);

    // Transitions
    NewTransition(initState, completeInit, idleState);
    NewTransition(idleState, startCalibrate, calibrateState);
    NewTransition(calibrateState, isCalibrationFinished, idleState);
    NewTransition(idleState, startRecord, recordState);
    NewTransition(recordState, stopRecord, idleState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(initState);
}

void ForcePlateApp::init() {
    spdlog::info("ForcePlateApp::init()");
    initialised = robot->initialise();
    running = true;

    // Initialising the data logger
    time0 = std::chrono::steady_clock::now();
    dataLogger.initLogger("test_logger", "logs/testLog.csv", LogFormat::CSV, true);
    dataLogger.add(time, "time");
    dataLogger.add(robot->getStrainReadings(), "StrainReadings");

    dataLogger.startLogger();
}

void ForcePlateApp::end() {
    spdlog::debug("Ending Force Plate App");
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

bool ForcePlateApp::CompleteInit::check(void) {
    return true;
}

bool ForcePlateApp::StartCalibrate::check(void) {
    if (OWNER->robot->keyboard->getA() == true) {
        return true;
    } else if (OWNER->robot->getCommand() == CALIBRATE) {
        OWNER->robot->resetCommand();
        return true;
    }
    return false;
}
bool ForcePlateApp::StartRecord::check(void) {
    if (OWNER->robot->keyboard->getS() == true) {
        return true;
    } else if (OWNER->robot->getCommand() == RECORD) {
        OWNER->robot->resetCommand();
        return true;
    }
    return false;
}

bool ForcePlateApp::StopRecord::check(void) {
    if (OWNER->robot->keyboard->getS() == true) {
        return true;
    } else if(OWNER->robot->getCommand() == STOP){
        OWNER->robot->resetCommand();
        return true;
    }
    return false;
}
bool ForcePlateApp::IsCalibrationFinished::check(void) {
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
void ForcePlateApp::hwStateUpdate(void) {
    robot->updateRobot();
}

/**
 * \brief Statemachine update: overloaded to include logging
 *
 */
void ForcePlateApp::update() {
    // Update time (used for log)
    time = (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - time0)
                .count()) /
           1e6;
    StateMachine::update();
    dataLogger.recordLogData();
}

void ForcePlateApp::configureMasterPDOs() {
    spdlog::debug("ForcePlateApp::configureMasterPDOs()");
    robot->configureMasterPDOs();
}
