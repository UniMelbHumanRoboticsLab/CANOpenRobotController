#include "FITHVExoDemoMachine.h"

using namespace std;

bool endCalib(StateMachine & sm) {
    return (sm.state<CalibState>("Calib"))->isCalibDone();
}

bool goToNextState(StateMachine & sm_) {
    FITHVExoDemoMachine & sm = static_cast<FITHVExoDemoMachine &>(sm_); //Cast to specific StateMachine type

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

bool standby(StateMachine & sm_) {
    FITHVExoDemoMachine & sm = (FITHVExoDemoMachine &)sm_; //Cast to specific StateMachine type

    //keyboard press
    if ( (sm.robot()->keyboard->getNb()==0) ) {
        return true;
    }

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("GOST") ) {
        sm.UIserver->sendCmd(string("OK"));
        return true;
    }

    return false;
}


//Go to wall assistance
bool assist(StateMachine & sm_) {
    FITHVExoDemoMachine & sm = (FITHVExoDemoMachine &)sm_; //Cast to specific StateMachine type

    //keyboard press
    if ( (sm.robot()->keyboard->getNb()==1) ) {
        return true;
    }

    //Check incoming command requesting assist state
    std::vector<double> params;
    if ( sm.UIserver->isCmd("GOAS", params) ) {
        if(params.size()==2) {
            //Assign parameters to state.
            std::shared_ptr<WallAssistState> s = sm.state<WallAssistState>("WallAssist");
            if(!s->active()) {
                if(s->setParameters(params[0], params[1])) {
                    spdlog::debug("wall assist OK ({}, {})", params[0], params[1]);
                    sm.UIserver->sendCmd(string("OK"));
                    return true;
                }
                spdlog::warn("wall assist error: wrong parameters.");
                sm.UIserver->sendCmd(string("ER1"));
            }
            else {
                spdlog::warn("wall assist error: state running.");
                sm.UIserver->sendCmd(string("ER2"));
            }
        }
        else {
            spdlog::warn("wall assist error: number of command parameters.");
            sm.UIserver->sendCmd(string("ER3"));
        }
    }

    return false;
}


//Fake transition (always return false) updating wall assist parameters
bool updateAssist(StateMachine & sm_) {
    FITHVExoDemoMachine & sm = (FITHVExoDemoMachine &)sm_; //Cast to specific StateMachine type

    //Check incoming command requesting new force control state
    std::vector<double> params;
    if ( sm.UIserver->isCmd("UPAS", params) ) {
        if(params.size()==2) {
            //Assign parameters to state
            std::shared_ptr<WallAssistState> s = sm.state<WallAssistState>("WallAssist");
            if(s->setParameters(params[0], params[1])) {
                spdlog::debug("wall assist OK ({}, {})", params[0], params[1]);
                sm.UIserver->sendCmd(string("OK"));
                return false;
            }
            spdlog::warn("wall assist error: wrong parameters.");
            sm.UIserver->sendCmd(string("ER1"));
        }
        else {
            spdlog::warn("wall assist error: number of command parameters.");
            sm.UIserver->sendCmd(string("ER3"));
        }
    }

    return false;
}



//Exit CORC cleanly
bool quit(StateMachine & sm_) {
    FITHVExoDemoMachine & sm = (FITHVExoDemoMachine &)sm_; //Cast to specific StateMachine type

    //keyboard press
    if ( sm.robot()->keyboard->getKeyUC()=='Q' ) {
        std::raise(SIGTERM); //Clean exit
        return true;
    }

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("QUIT") ) {
        sm.UIserver->sendCmd(string("OK"));
        spdlog::debug("Quit");
        std::raise(SIGTERM); //Clean exit
        return true;
    }

    return false;
}



FITHVExoDemoMachine::FITHVExoDemoMachine() {
    //Create a Robot and set it to generic state machine
    setRobot(std::make_unique<RobotFITHVExo>("RobotFITHVExo"));

    //Create state instances and add to the State Machine
    addState("Calib", std::make_shared<CalibState>(robot()));
    addState("Standby", std::make_shared<StandbyState>(robot()));
    addState("WallAssist", std::make_shared<WallAssistState>(robot()));
    addState("Test", std::make_shared<TestState>(robot()));

    //Define transitions between states
    addTransition("Calib", &endCalib, "Standby");
    addTransition("WallAssist", &standby, "Standby");
    addTransition("Standby", &updateAssist, "WallAssist");
    addTransition("WallAssist", &updateAssist, "WallAssist");
    addTransition("Standby", &assist, "WallAssist");
    addTransitionFromAny(&standby, "Standby");
    addTransitionFromAny(&quit, "Standby");

    //Initialize the state machine with first state of the designed state machine
    //setInitState("Test");
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
        UIserver->registerState(state<WallAssistState>("WallAssist")->getk());
        UIserver->registerState(state<WallAssistState>("WallAssist")->getq0());
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
