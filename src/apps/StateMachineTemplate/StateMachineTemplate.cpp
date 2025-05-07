#include "StateMachineTemplate.h"

using namespace std;

bool endCalib(StateMachine & sm) {
    return (sm.state<CalibState>("CalibState"))->isCalibDone();
}

bool goToNextState(StateMachine & SM) {
    StateMachineTemplate & sm = static_cast<StateMachineTemplate &>(SM); //Cast to specific StateMachine type

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
    StateMachineTemplate & sm = (StateMachineTemplate &)SM; //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getQ()==1) {
        return true;
    }
    return false;
}


StateMachineTemplate::StateMachineTemplate() {
    //Create a Robot and set it to generic state machine
    setRobot(std::make_unique<PlatformName>("PlatformName"));

    //Create state instances and add to the State Machine
    addState("TestState", std::make_shared<StandbyState>(robot()));
    addState("CalibState", std::make_shared<CalibState>(robot()));


    //Define transitions between states
    addTransition("CalibState", &endCalib, "StandbyState");
    addTransitionFromAny(&standby, "StandbyState");

    //Initialize the state machine with first state of the designed state machine
    setInitState("CalibState");
}
StateMachineTemplate::~StateMachineTemplate() {
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void StateMachineTemplate::init() {
    spdlog::debug("StateMachineTemplate::init()");
    if(robot()->initialise()) {
        logHelper.initLogger("StateMachineTemplateLog", "logs/StateMachineTemplate.csv", LogFormat::CSV, true);
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

void StateMachineTemplate::end() {
    if(running())
        UIserver->closeConnection();
    StateMachine::end();
}


/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void StateMachineTemplate::hwStateUpdate() {
    StateMachine::hwStateUpdate();
    //Also send robot state over network
    UIserver->sendState();
}
