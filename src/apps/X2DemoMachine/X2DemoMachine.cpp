#include "X2DemoMachine.h"

#define OWNER ((X2DemoMachine *)owner)

X2DemoMachine::X2DemoMachine() {
    robot_ = new X2Robot();
    // Create PRE-DESIGNED State Machine events and state objects.
    startExo = new StartExo(this);
    /**f
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    idleState = new IdleState(this, robot_);
    x2DemoState = new X2DemoState(this, robot_);

    NewTransition(idleState, startExo, x2DemoState);
    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(idleState);

    // Create ros object
    x2DemoMachineRos_ = new X2DemoMachineROS(robot_);
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void X2DemoMachine::init(int argc, char *argv[]) {
    spdlog::debug("X2DemoMachine::init()");

    ros::init(argc, argv, "x2_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle;

    // Pass nodeHandle to the classes that use ROS features
    x2DemoMachineRos_->setNodeHandle(nodeHandle);

#ifdef SIM
    robot_->setNodeHandle(nodeHandle);
#endif

    initialised = robot_->initialise();
    x2DemoMachineRos_->initialize();
    running = true;
    time0 = std::chrono::steady_clock::now();

    logHelper_.initLogger("test_logger", "logs/helperTrial.csv", LogFormat::CSV, true);
    logHelper_.add(time, "time");
    logHelper_.add(robot_->getPosition(), "JointPositions");
    logHelper_.add(robot_->getTorque(), "JointTorques");
    logHelper_.startLogger();
}

void X2DemoMachine::end() {
    if(initialised) {
        logHelper_.endLog();
        currentState->exit();
        robot_->disable();
        delete x2DemoMachineRos_;
        delete robot_;
    }
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool X2DemoMachine::StartExo::check(void) {
    if (OWNER->robot_->keyboard->getS() == true) {
        std::cout << "Pressed S!" << std::endl;
        return true;
    }
    return false;
}
/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void X2DemoMachine::hwStateUpdate(void) {
    robot_->updateRobot();
}

void X2DemoMachine::update() {
    time = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time0).count()) / 1e6;

    StateMachine::update();
    x2DemoMachineRos_->update();
    logHelper_.recordLogData();
    ros::spinOnce();
}
