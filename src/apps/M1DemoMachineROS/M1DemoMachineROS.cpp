#include "M1DemoMachineROS.h"


M1DemoMachineROS::M1DemoMachineROS(int argc, char *argv[]) {
    spdlog::debug("M1DemoMachineROS::constructed!");

    ros::init(argc, argv, "m1", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle("~");

    // Get robot name from the node name
    robotName_ = ros::this_node::getName();
    robotName_.erase(0,1); // erase the first character which is '/'

    // create robot
    setRobot(std::make_unique<RobotM1>(robotName_));

    // Create ros object
    M1MachineRos_ = new M1MachineROS(robot());

    // Pass nodeHandle to the classes that use ROS features
    M1MachineRos_->setNodeHandle(nodeHandle);
    M1MachineRos_->initialize();

    // Create states with ROS features // This should be created after ros::init()
    addState("multiControllerState", std::make_shared<MultiControllerState>(robot(), M1MachineRos_));

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    setInitState("multiControllerState");
}

M1DemoMachineROS::~M1DemoMachineROS() {
    delete M1MachineRos_;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */

void M1DemoMachineROS::init() {

    if(!robot()->initialise()) {
        std::cout /*cerr is banned*/ << "Failed robot initialisation. Exiting..." << std::endl;
        std::raise(SIGTERM); //Clean exit
    }

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream logFileName;

//    std::string robotName_ = ros::this_node::getName();

    logFileName << "spdlogs/" << robotName_<< std::put_time(&tm, "/%d-%m-%Y_%H-%M-%S") << ".csv";

    logHelper.initLogger("test_logger", logFileName.str(), LogFormat::CSV, true);
    logHelper.add(runningTime(), "time");
    logHelper.add(state<MultiControllerState>("multiControllerState")->controller_mode_, "mode");

    logHelper.add(robot()->getPosition(), "JointPositions");
    logHelper.add(robot()->getVelocity(), "JointVelocities");
    logHelper.add(robot()->getTorque(), "JointTorques");
    logHelper.add(robot()->getJointTor_s(), "SensorTorques");
    logHelper.add(state<MultiControllerState>("multiControllerState")->tau_raw, "SensorTorques_raw");
    logHelper.add(state<MultiControllerState>("multiControllerState")->tau_filtered, "SensorTorques_filtered");

    logHelper.add(state<MultiControllerState>("multiControllerState")->q_raw, "q_raw");
    logHelper.add(state<MultiControllerState>("multiControllerState")->q_filtered, "q_filtered");

    logHelper.add(state<MultiControllerState>("multiControllerState")->spk_, "SpringStiffness");
    logHelper.add(state<MultiControllerState>("multiControllerState")->spring_tor, "SpringTorque");
    logHelper.add(state<MultiControllerState>("multiControllerState")->tau_cmd, "CommandTorque");      // motor_torque = command_torque + compensation_torque
    logHelper.add(robot()->tau_motor, "MotorTorque");

    logHelper.add(M1MachineRos_->jointTorqueCommand_, "MM1_DesiredJointTorques");
    logHelper.add(M1MachineRos_->jointPositionCommand_, "MM1_DesiredJointPositions");
    logHelper.add(M1MachineRos_->interactionTorqueCommand_, "MM1_DesiredInteractionTorques");

    logHelper.add(state<MultiControllerState>("multiControllerState")->digitalInValue_, "digitalIn");
    logHelper.add(state<MultiControllerState>("multiControllerState")->digitalOutValue_, "digitalOut");

}

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M1DemoMachineROS::hwStateUpdate(void) {
    StateMachine::update();
    M1MachineRos_->update();
    ros::spinOnce();
}
