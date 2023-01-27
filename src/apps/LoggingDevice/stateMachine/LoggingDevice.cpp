#include "LoggingDevice.h"



////////////////////////////////////////////////////////////////
// Transitions--------------------------------------------------
///////////////////////////////////////////////////////////////
bool isAPressed(StateMachine & sm) {
    LoggingDevice & SM = static_cast<LoggingDevice &>(sm);
    spdlog::trace("IsAPressed");
    if (SM.robot()->keyboard->getA() == true) {

        return true;
    }
    return false;
}
bool isSPressed(StateMachine & sm) {
    LoggingDevice & SM = static_cast<LoggingDevice &>(sm);
    if (SM.robot()->keyboard->getS() == true) {
        return true;
    }
    return false;
}

bool isCalibrationFinished(StateMachine & sm) {
    LoggingDevice & SM = static_cast<LoggingDevice &>(sm);
    if (SM.state<CalibrateState>("calibrateState")->getCurrReading() < NUM_CALIBRATE_READINGS) {
        return false;
    }
    return true;
}


LoggingDevice::LoggingDevice() {
    setRobot(std::make_unique<LoggingRobot>());

    // States
    addState("initState", std::make_shared<InitState>(robot()));
    addState("idleState", std::make_shared<IdleState>(robot()));
    addState("calibrateState", std::make_shared<CalibrateState>(robot()));
    addState("recordState", std::make_shared<RecordState>(robot()));

    // Transitions
    addTransition("initState", isAPressed, "idleState");
    addTransition("idleState", isAPressed, "calibrateState");

    addTransition("calibrateState", isCalibrationFinished, "idleState");
    addTransition("idleState", isSPressed, "recordState");
    addTransition("recordState", isSPressed, "idleState");

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    setInitState("initState");
}

void LoggingDevice::init() {
    spdlog::info("LoggingDevice::init()");
    robot()->initialise();

    // Initialising the data logger
    logHelper.initLogger("test_logger", "logs/testLog.csv", LogFormat::CSV, true);
    logHelper.add(runningTime(), "time");
    logHelper.add(robot()->getCrutchReadings(), "CrutchReadings");
    //logHelper.add(robot()->getMotorPositions(), "MotorPositions");
    //logHelper.add(robot()->getMotorVelocities(), "MotorVelocities");
    //logHelper.add(robot()->getMotorTorques(), "MotorTorques");
    //logHelper.add(robot()->getGoButton(), "GoButton");
    //logHelper.add(robot()->getCurrentState(), "CurrentState");
    //logHelper.add(robot()->getCurrentMovement(), "CurrentMovement");
}
