#include "M2DemoMachine.h"

bool endCalib(StateMachine & sm) {
    return (sm.state<M2CalibState>("CalibState"))->isCalibDone();
}


bool goToNextState(StateMachine & SM) {
    M2DemoMachine & sm = static_cast<M2DemoMachine &>(SM); //Cast to specific StateMachine type

    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==1) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GTNS") { //Go To Next State command received
            //Acknowledge
            sm.UIserver->sendCmd(string("OK"));

            return true;
        }
    }

    //Otherwise false
    return false;
}


M2DemoMachine::M2DemoMachine() {
    //Create an M2 Robot and set it to generic state machine
    setRobot(std::make_unique<RobotM2>("M2_MELB"));

    //Create state instances and add to the State Machine
    addState("TestState", std::make_shared<M2DemoState>(robot()));
    addState("CalibState", std::make_shared<M2CalibState>(robot()));
    addState("StandbyState", std::make_shared<M2Transparent>(robot()));
    addState("EndEffDemoState", std::make_shared<M2EndEffDemo>(robot()));
    addState("PathState", std::make_shared<M2DemoPathState>(robot()));
    addState("MinJerkState", std::make_shared<M2DemoMinJerkPosition>(robot()));

    //Define transitions between states
    addTransition("CalibState", &endCalib, "StandbyState");
    addTransitionFromLast(&goToNextState, "MinJerkState");
    addTransitionFromLast(&goToNextState, "EndEffDemoState");
    addTransitionFromLast(&goToNextState, "PathState");
    addTransitionFromLast(&goToNextState, "StandbyState");

    //Initialize the state machine with first state of the designed state machine
    setInitState("CalibState");
}
M2DemoMachine::~M2DemoMachine() {
}

/**
 * \brief start function for running any designed statemachine specific functions
 *
 */
void M2DemoMachine::init() {
    spdlog::debug("M2DemoMachine::init()");
    if(robot()->initialise()) {
        logHelper.initLogger("M2DemoMachineLog", "logs/M2DemoMachine.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getEndEffPosition(), "Position");
        logHelper.add(robot()->getEndEffVelocity(), "Velocity");
        logHelper.add(robot()->getEndEffForce(), "Force");
        logHelper.startLogger();
        UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.7.2");
    }
    else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}

void M2DemoMachine::end() {
    if(running())
        UIserver->closeConnection();
    StateMachine::end();
}


////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////

/**
 * \brief Statemachine to hardware interface method.
 *
 */
void M2DemoMachine::hwStateUpdate(void) {
    StateMachine::hwStateUpdate();
    //Also send robot state over network
    UIserver->sendState();
}
