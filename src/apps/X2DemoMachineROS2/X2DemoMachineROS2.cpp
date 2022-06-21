#include "X2DemoMachineROS2.h"

#define OWNER ((X2DemoMachineROS2 *)owner)

X2DemoMachineROS2::X2DemoMachineROS2(int argc, char *argv[]) {

    rclcpp::install_signal_handlers();
    auto rosInit = rclcpp::InitOptions();
    rosInit.shutdown_on_sigint = false;
    rclcpp::init(argc, argv, rosInit);
    // node = rclcpp::Node::make_shared("~");
    node = rclcpp::Node::make_shared("x2");

    // Get robot name from the node name
    robotName_ = node->get_name();
    robotName_.erase(0,1); // erase the first character which is '/'

#ifdef SIM
    robot_ = new X2Robot(node, robotName_);
#else
    robot_ = new X2Robot(robotName_);
#endif

    // Create PRE-DESIGNED State Machine events and state objects.
    startExo = new StartExo(this);
    /**f
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */

    // Create ros object
    X2DemoMachineRos_ = new X2DemoMachineROS(robot_, node);

    // Pass MachineRos to the State to use ROS features
    x2DemoState_ = new X2DemoState(this, robot_, X2DemoMachineRos_);
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void X2DemoMachineROS2::init() {
    spdlog::debug("X2DemoMachineROS2::init()");

    // Create states with ROS features // This should be created after ros::init()
    StateMachine::initialize(x2DemoState_);

    initialised = robot_->initialise();
    running = true;
    time0 = std::chrono::steady_clock::now();

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream logFileName;

    logFileName << "spdlogs/" << robotName_<< std::put_time(&tm, "/%d-%m-%Y_%H-%M-%S") << ".csv";

    logHelper.initLogger("test_logger", logFileName.str(), LogFormat::CSV, true);
    logHelper.add(time, "time");
    logHelper.add(x2DemoState_->controller_mode_, "mode");
    logHelper.add(robot_->getPosition(), "JointPositions");
    logHelper.add(robot_->getVelocity(), "JointVelocities");
    logHelper.add(robot_->getTorque(), "JointTorques");
    logHelper.add(x2DemoState_->getDesiredJointTorques(), "DesiredJointTorques");
    logHelper.add(robot_->getInteractionForce(), "InteractionForces");
    //    logHelper.add(x2DemoState_->virtualMassRatio_, "virtualMassRatio");

    logHelper.startLogger();
}

void X2DemoMachineROS2::end() {
    if(initialised) {
        logHelper.endLog();
        currentState->exit();
        robot_->disable();
        delete X2DemoMachineRos_;
        delete robot_;
    }
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool X2DemoMachineROS2::StartExo::check(void) {
    if (OWNER->robot_->keyboard->getS() == true || OWNER->X2DemoMachineRos_->startExoTriggered_) {
        spdlog::info("Exo started!");
        OWNER->X2DemoMachineRos_->startExoTriggered_ = false;
        return true;
    }
    return false;
}
/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void X2DemoMachineROS2::hwStateUpdate(void) {
    robot_->updateRobot();
}

void X2DemoMachineROS2::update() {
    time = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time0).count()) / 1e6;

    StateMachine::update();
    X2DemoMachineRos_->update();
    rclcpp::spin_some(node);
}

bool X2DemoMachineROS2::configureMasterPDOs() {
    spdlog::debug("X2DemoMachineROS2::configureMasterPDOs()");
    return robot_->configureMasterPDOs();
}
