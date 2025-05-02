#include "FITHVExoDemoMachine.h"

using namespace std;

bool endCalib(StateMachine & sm) {
    return (sm.state<CalibState>("CalibState"))->isCalibDone();
}

bool goToNextState(StateMachine & SM) {
    FITHVExoDemoMachine & sm = static_cast<FITHVExoDemoMachine &>(SM); //Cast to specific StateMachine type

    //keyboard
    if ( (sm.robot()->keyboard->getNb()==1) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("GTNS") ) {
        sm.UIserver->sendCmd(string("OK"));
        return true;
    }

    //Otherwise false
    return false;
}

bool standby(StateMachine & SM) {
    FITHVExoDemoMachine & sm = (FITHVExoDemoMachine &)SM; //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getNb()==0) {
        return true;
    }
    return false;
}


FITHVExoDemoMachine::FITHVExoDemoMachine() {
    //Create a Robot and set it to generic state machine
    setRobot(std::make_unique<RobotFITHVExo>("RobotFITHVExo"));

    //Create state instances and add to the State Machine
    addState("Standby", std::make_shared<StandbyState>(robot()));
    addState("WallAssist", std::make_shared<WallAssistState>(robot()));
    addState("Test", std::make_shared<TestState>(robot()));
    addState("Calib", std::make_shared<CalibState>(robot()));

    //Define transitions between states
    addTransition("Calib", &endCalib, "Standby");
    addTransition("WallAssist", &goToNextState, "Standby");
    addTransition("Standby", &goToNextState, "WallAssist");
    addTransition("Calib", &endCalib, "StandbyState");
    addTransitionFromAny(&standby, "Standby");

    //Initialize the state machine with first state of the designed state machine
    //setInitState("Calib");
    setInitState("Test");
}
FITHVExoDemoMachine::~FITHVExoDemoMachine() {
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void FITHVExoDemoMachine::init() {
    spdlog::debug("FITHVExoDemoMachine::init()");
    if(robot()->initialise()) {
        logHelper.initLogger("FITHVExoDemoMachineLog", "logs/FITHVExoDemoMachine.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getPosition(), "q");
        logHelper.add(robot()->getVelocity(), "dq");
        logHelper.add(robot()->getTorque(), "tau");
        UIserver = std::make_shared<FLNLHelper>("192.168.7.2");
        UIserver->registerState(runningTime());
        UIserver->registerState(robot()->getPosition());
        UIserver->registerState(robot()->getVelocity());
        UIserver->registerState(robot()->getTorque());
    }
    else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}

void FITHVExoDemoMachine::end() {
    if(running())
        UIserver->closeConnection();
    StateMachine::end();
}


/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void FITHVExoDemoMachine::hwStateUpdate() {
    StateMachine::hwStateUpdate();
    //Also send robot state over network
    UIserver->sendState();
}
