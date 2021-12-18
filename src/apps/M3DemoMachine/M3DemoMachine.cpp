#include "M3DemoMachine.h"

bool endCalib(StateMachine & sm) {
    return (sm.state<M3CalibState>("CalibState"))->isCalibDone();
}

bool goToNextState(StateMachine & SM) {
    M3DemoMachine & sm = static_cast<M3DemoMachine &>(SM);

    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonTransition(3)>0 || sm.robot()->keyboard->getNb()==1) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GTNS") { //Go To Next State command received
            sm.UIserver->clearCmd();
            //Acknowledge
            sm.UIserver->sendCmd(string("OK"));

            return true;
        }
    }

    //Otherwise false
    return false;
}

bool standby(StateMachine & SM) {
    M3DemoMachine & sm = (M3DemoMachine &)SM; //annoying....

    if (sm.robot()->keyboard->getQ()==1) {
        return true;
    }
    return false;
}


M3DemoMachine::M3DemoMachine() {
    //Create a Robot and set it to generic state machine
    setRobot(std::make_unique<RobotM3>("EMU_FOURIER", "M3_params.yaml"));

    //Create state instances and add to the State Machine
    addState("TestState", std::make_shared<M3DemoState>(robot()));
    addState("CalibState", std::make_shared<M3CalibState>(robot()));
    addState("StandbyState", std::make_shared<M3MassCompensation>(robot()));
    addState("MinJerkState", std::make_shared<M3DemoMinJerkPosition>(robot()));
    addState("EndEffState", std::make_shared<M3EndEffDemo>(robot()));
    addState("TimingState", std::make_shared<M3SamplingEstimationState>(robot()));
    addState("PathState", std::make_shared<M3DemoPathState>(robot()));
    addState("ImpedanceState", std::make_shared<M3DemoImpedanceState>(robot()));

    //Define transitions between states
    addTransition("CalibState", &endCalib, "StandbyState");
    addTransition("StandbyState", &goToNextState, "MinJerkState");
    addTransition("MinJerkState", &goToNextState, "ImpedanceState");
    addTransition("ImpedanceState", &goToNextState, "PathState");
    addTransition("PathState", &goToNextState, "EndEffState");
    addTransition("EndEffState", &goToNextState, "TimingState");
    addTransition("TimingState", &goToNextState, "StandbyState");
    addTransitionFromAny(&standby, "StandbyState");

    //Initialize the state machine with first state of the designed state machine
    setInitState("CalibState");
}
M3DemoMachine::~M3DemoMachine() {
    if(UIserver) {
        delete UIserver;
    }
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void M3DemoMachine::init() {
    spdlog::debug("M3DemoMachine::init()");
    if(robot()->initialise()) {
        logHelper.initLogger("M3DemoMachineLog", "logs/M3DemoMachine.csv", LogFormat::CSV, true);
        /*logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getEndEffPosition(), "X");
        logHelper.add(robot()->getEndEffVelocity(), "dX");
        logHelper.add(robot()->getInteractionForce(), "F");
        logHelper.add(robot()->getEndEffAcceleration(), "ddX");
        logHelper.add(robot()->getEndEffVelocityFiltered(), "dXFilt");*/
        UIserver = new FLNLHelper(*robot(), "192.168.6.2");
    }
    else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}

void M3DemoMachine::end() {
    if(running())
        UIserver->closeConnection();
    StateMachine::end();
}


/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M3DemoMachine::hwStateUpdate() {
    StateMachine::hwStateUpdate();
    //Also send robot state over network
    UIserver->sendState();
}
