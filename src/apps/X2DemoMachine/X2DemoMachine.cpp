#include "X2DemoMachine.h"

#define OWNER ((X2DemoMachine *)owner)

X2DemoMachine::X2DemoMachine(int argc, char *argv[]) {

    ros::init(argc, argv, "x2", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle("~");

    // Get robot name from the node name
    robotName_ = ros::this_node::getName();
    robotName_.erase(0,1); // erase the first character which is '/'

#ifdef SIM
    robot_ = new X2Robot(nodeHandle, robotName_);
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

    // Create state objet
    x2DemoState_ = new X2DemoState(this, robot_);

    // Create ros object
    x2DemoMachineRos_ = new X2DemoMachineROS(robot_, x2DemoState_, nodeHandle);

}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void X2DemoMachine::init() {
    spdlog::debug("X2DemoMachine::init()");

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
    logHelper.add(x2DemoState_->getDesiredJointVelocities(), "DesiredJointVelocities");
    logHelper.add(robot_->getJointTorquesViaStrainGauges(), "JointTorquesViaGauges");
    logHelper.add(robot_->getGravitationTorque(), "gravitationTorque");
    logHelper.add(robot_->getFrictionTorque(), "FrictionTorque");
    logHelper.add(robot_->getInteractionForce(), "InteractionForces");
    logHelper.add(robot_->getSmoothedInteractionForce(), "SmoothedInteractionForces");
    logHelper.add(robot_->getEstimatedGeneralizedAcceleration(), "EstimatedAcceleration");
    logHelper.add(robot_->getBackPackAngleOnMedianPlane(), "BackPackAngle");
    logHelper.add(robot_->getBackPackAngularVelocityOnMedianPlane(), "BackPackAngularVelocity");
    logHelper.add(x2DemoState_->enableJoints, "enableJoints");
    logHelper.add(robot_->getButtonValue(ButtonColor::GREEN), "greenButton");
    logHelper.add(robot_->getButtonValue(ButtonColor::RED), "redButton");
    logHelper.add(robot_->getButtonValue(ButtonColor::YELLOW), "yellowButton");
    logHelper.add(robot_->getButtonValue(ButtonColor::BLUE), "blueButton");
    logHelper.add(robot_->getGroundReactionForces(), "GroundReactionForces");
    logHelper.add(robot_->getGRFSensorThresholds(), "GRFThresholds");
    logHelper.add(robot_->getJointVelDerivativeCutOffFrequency(), "JointVelDerivativeCutOffFreq");
    logHelper.add(robot_->getBackpackVelDerivativeCutOffFrequency(), "BackpackVelDerivativeCutOffFreq");
    logHelper.add(robot_->getDynamicParametersCutOffFrequency(), "DynamicParametersCutOffFreq");

    logHelper.startLogger();
}

void X2DemoMachine::end() {
    if(initialised) {
        logHelper.endLog();
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
        spdlog::info("Exo started!");
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
    ros::spinOnce();
}

bool X2DemoMachine::configureMasterPDOs() {
    spdlog::debug("X2DemoMachine::configureMasterPDOs()");
    return robot_->configureMasterPDOs();
}
