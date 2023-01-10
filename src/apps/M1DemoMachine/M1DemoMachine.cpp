#include "M1DemoMachine.h"


////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool Event2Demo(StateMachine & SM) {
    M1DemoMachine & sm = static_cast<M1DemoMachine &>(SM); //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getA() == true) {
        std::cout << "Pressed A!" << std::endl;
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool Event2Monitor(StateMachine & SM) {
    M1DemoMachine & sm = static_cast<M1DemoMachine &>(SM); //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getX() == true) {
        std::cout << "Pressed X!" << std::endl;
        return true;
    }
    return false;
}

bool Event2Idle(StateMachine & SM) {
    M1DemoMachine & sm = static_cast<M1DemoMachine &>(SM); //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getQ() == true) {
        std::cout << "Pressed Q!" << std::endl;
        return true;
    }
    return false;
}

bool Event2Cali(StateMachine & SM) {
    M1DemoMachine & sm = static_cast<M1DemoMachine &>(SM); //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getS() == true) {
//        std::cout << "Pressed S!" << std::endl;
        return true;
    }
    return false;
}




M1DemoMachine::M1DemoMachine() {

    setRobot(std::make_unique<RobotM1>());

    // Create State Machine state objects.
    addState("idleState", std::make_shared<IdleState>(robot()));
    addState("calibrationState", std::make_shared<Calibration>(robot()));
    addState("monitorState", std::make_shared<Monitoring>(robot()));
    addState("demoState", std::make_shared<M1DemoState>(robot()));


    // Create State Machine events objects.
    addTransition("idleState", &Event2Monitor, "monitorState");
    addTransitionFromLast(&Event2Idle, "idleState");

    addTransition("idleState", &Event2Cali, "calibrationState");
    addTransitionFromLast(&Event2Idle, "idleState");

    addTransition("idleState", &Event2Demo, "demoState");
    addTransitionFromLast(&Event2Idle, "idleState");

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    setInitState("idleState");
}

M1DemoMachine::~M1DemoMachine() {
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void M1DemoMachine::init() {
    spdlog::debug("M1DemoMachine::init()");
    if(robot()->initialise()) {
        logHelper.initLogger("M1DemoMachineLog", "logs/M1DemoMachine.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "time (s)");
        logHelper.add(robot()->getPosition(), "JointPositions");
        logHelper.add(robot()->getVelocity(), "JointVelocities");
        logHelper.add(robot()->getTorque(), "MotorTorques");
        logHelper.add(robot()->getJointTor_s(), "JointTorques_s");
//        logHelper.add(robot()->getJointTor_cmd(), "JointTorques_cmd");
        logHelper.add(robot()->mode, "control_mode");
    }
    else {
        std::cout /*cerr is banned*/ << "Failed robot initialisation. Exiting..." << std::endl;
        std::raise(SIGTERM); //Clean exit
    }
}
