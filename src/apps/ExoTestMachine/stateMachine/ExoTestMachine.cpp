
#include "ExoTestMachine.h"

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
/**
     * \brief poll the trajectory Generators flag to see if the currently loaded motion is complete
     *
     */
bool endTraj(StateMachine & SM) {
    ExoTestMachine & sm = static_cast<ExoTestMachine &>(SM); //Cast to specific StateMachine type

    return sm.trajectoryGenerator->isTrajectoryFinished();
}

bool startExo(StateMachine & SM) {
    ExoTestMachine & sm = static_cast<ExoTestMachine &>(SM); //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getS() == true) {
        spdlog::info("LEAVING INIT and entering Sitting");
        return true;
    }
    return false;
}
bool startExoCal(StateMachine & SM) {
    ExoTestMachine & sm = static_cast<ExoTestMachine &>(SM); //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getA() == true) {
        #ifndef NOROBOT
            spdlog::info("LEAVING INIT and entering Sitting");
            spdlog::info("Homing");

            sm.robot()->disable();
            sm.robot()->homing();
            spdlog::info("Finished");
        #else
            spdlog::warn("Calibration Not Avaiable in Simulation (NoRobot) Mode");
        #endif
        return true;
    }
    return false;
}

bool startStand(StateMachine & SM) {
    ExoTestMachine & sm = static_cast<ExoTestMachine &>(SM); //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getW() == true) {
        return true;
    }
    return false;
}

bool startSit(StateMachine & SM) {
    ExoTestMachine & sm = static_cast<ExoTestMachine &>(SM); //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getW()) {
        return true;
    }
    return false;
}

ExoTestMachine::ExoTestMachine() {
    trajectoryGenerator = new DummyTrajectoryGenerator(X2_NUM_JOINTS);
    setRobot(std::make_unique<X2Robot>());

    spdlog::info("Test");

    // Create PRE-DESIGNED State Machine events and state objects.
    addState("initState", std::make_shared<InitState>(robot(), trajectoryGenerator));
    addState("standing", std::make_shared<Standing>(robot(), trajectoryGenerator));
    addState("sitting", std::make_shared<Sitting>(robot(), trajectoryGenerator));
    addState("standingUp", std::make_shared<StandingUp>(robot(), trajectoryGenerator));
    addState("sittingDwn", std::make_shared<SittingDwn>(robot(), trajectoryGenerator));

    addTransition("initState", &startExo, "standing");
    addTransition("initState", &startExoCal, "standing");
    addTransition("sitting", &startStand, "standingUp");
    addTransition("standingUp", &endTraj, "standing");
    addTransition("standing", &startSit, "sittingDwn");
    addTransition("sittingDwn", &endTraj, "sitting");

    setInitState("initState"); 
}
/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void ExoTestMachine::init() {
    spdlog::debug("ExoTestMachine::init()");
    robot()->initialise();
    //running = true;

    // Initialising the data logger
    logHelper.initLogger("test_logger", "logs/testLog.csv", LogFormat::CSV, true);
    logHelper.add(runningTime(), "time");
    logHelper.add(robot()->getPosition(), "JointPositions");
    logHelper.startLogger();

    spdlog::info("InitFinished");
}

void ExoTestMachine::end() {
    spdlog::debug("Ending ExoTestMachine");
    logHelper.endLog();
}


/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void ExoTestMachine::hwStateUpdate(void) {
    robot()->updateRobot();
}