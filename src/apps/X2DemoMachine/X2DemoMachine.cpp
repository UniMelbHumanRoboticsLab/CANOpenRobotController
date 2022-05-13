#include "X2DemoMachine.h"

X2DemoMachine::X2DemoMachine(int argc, char *argv[]) {

    ros::init(argc, argv, "x2", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle("~");

    // Get robot name from the node name
    robotName_ = ros::this_node::getName();
    robotName_.erase(0,1); // erase the first character which is '/'

#ifdef SIM
    setRobot(std::make_unique<X2Robot>(nodeHandle, "x2"));
#else
    setRobot(std::make_unique<X2Robot>("x2"));
#endif

    //Create state instances and add to the State Machine
    addState("X2DemoState", std::make_shared<X2DemoState>(robot()));

    setInitState("X2DemoState");

    // Create ros object
    x2DemoMachineRos_ = new X2DemoMachineROS(robot(), state<X2DemoState>("X2DemoState"), nodeHandle);
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void X2DemoMachine::init() {
    spdlog::debug("X2DemoMachine::init()");

    time0 = std::chrono::steady_clock::now();

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream logFileName;

    logFileName << "spdlogs/" << robotName_<< std::put_time(&tm, "/%d-%m-%Y_%H-%M-%S") << ".csv";

    if(robot()->initialise()) {
        logHelper.initLogger("test_logger", logFileName.str(), LogFormat::CSV, true);
        logHelper.add(time, "time");
        logHelper.add(state<X2DemoState>("X2DemoState")->controller_mode_, "mode");
        logHelper.add(robot()->getPosition(), "JointPositions");
        logHelper.add(robot()->getVelocity(), "JointVelocities");
        logHelper.add(robot()->getTorque(), "JointTorques");
        logHelper.add(state<X2DemoState>("X2DemoState")->getDesiredJointTorques(), "DesiredJointTorques");
        logHelper.add(state<X2DemoState>("X2DemoState")->getDesiredJointVelocities(), "DesiredJointVelocities");
        logHelper.add(robot()->getJointTorquesViaStrainGauges(), "JointTorquesViaGauges");
        logHelper.add(robot()->getGravitationTorque(), "gravitationTorque");
        logHelper.add(robot()->getFrictionTorque(), "FrictionTorque");
        logHelper.add(robot()->getInteractionForce(), "InteractionForces");
        logHelper.add(robot()->getSmoothedInteractionForce(), "SmoothedInteractionForces");
        logHelper.add(robot()->getEstimatedGeneralizedAcceleration(), "EstimatedAcceleration");
        logHelper.add(robot()->getBackPackAngleOnMedianPlane(), "BackPackAngle");
        logHelper.add(robot()->getBackPackAngularVelocityOnMedianPlane(), "BackPackAngularVelocity");
        logHelper.add(state<X2DemoState>("X2DemoState")->enableJoints, "enableJoints");
        logHelper.add(robot()->getButtonValue(ButtonColor::GREEN), "greenButton");
        logHelper.add(robot()->getButtonValue(ButtonColor::RED), "redButton");
        logHelper.add(robot()->getButtonValue(ButtonColor::YELLOW), "yellowButton");
        logHelper.add(robot()->getButtonValue(ButtonColor::BLUE), "blueButton");
        logHelper.add(robot()->getGroundReactionForces(), "GroundReactionForces");
        logHelper.add(robot()->getGRFSensorThresholds(), "GRFThresholds");
        logHelper.add(robot()->getJointVelDerivativeCutOffFrequency(), "JointVelDerivativeCutOffFreq");
        logHelper.add(robot()->getBackpackVelDerivativeCutOffFrequency(), "BackpackVelDerivativeCutOffFreq");
        logHelper.add(robot()->getDynamicParametersCutOffFrequency(), "DynamicParametersCutOffFreq");

        logHelper.startLogger();
    }
}

void X2DemoMachine::end() {
    robot()->initTorqueControl();
    // setting 0 torque for safety.
    robot()->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
}

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void X2DemoMachine::hwStateUpdate(void) {
    robot()->updateRobot();
}

void X2DemoMachine::update() {
    time = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time0).count()) / 1e6;

    StateMachine::update();
    x2DemoMachineRos_->update();
    ros::spinOnce();
}
