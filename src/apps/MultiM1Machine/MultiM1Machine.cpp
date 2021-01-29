#include "MultiM1Machine.h"

#define OWNER ((MultiM1Machine *)owner)

MultiM1Machine::MultiM1Machine() {
    spdlog::debug("MultiM1Machine::constructed!");

    // create robot
    robot_ = new RobotM1();

    // Create ros object
    multiM1MachineRos_ = new MultiM1MachineROS(robot_);

}

MultiM1Machine::~MultiM1Machine() {
    currentState->exit();
    robot_->disable();
    delete multiM1MachineRos_;
    delete robot_;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */

void MultiM1Machine::init(int argc, char *argv[]) {
    ros::init(argc, argv, "m1", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle("~");

    // Pass nodeHandle to the classes that use ROS features
    multiM1MachineRos_->setNodeHandle(nodeHandle);

    // Create states with ROS features // This should be created after ros::init()
    multiControllerState_ = new MultiControllerState(this, robot_, multiM1MachineRos_);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(multiControllerState_);

    if(robot_->initialise()) {
        initialised = true;
    }
    else {
        initialised = false;
        std::cout /*cerr is banned*/ << "Failed robot initialisation. Exiting..." << std::endl;
        std::raise(SIGTERM); //Clean exit
    }
    running = true;

    multiM1MachineRos_->initialize();
    time0_ = std::chrono::steady_clock::now();

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream logFileName;

    std::string robotName_ = ros::this_node::getName();

    logFileName << "spdlogs/" << robotName_<< std::put_time(&tm, "/%d-%m-%Y_%H-%M-%S") << ".csv";

    logHelper.initLogger("test_logger", logFileName.str(), LogFormat::CSV, true);
    logHelper.add(time_, "time");
    logHelper.add(multiControllerState_->controller_mode_, "mode");

    logHelper.add(robot_->getPosition(), "JointPositions");
    logHelper.add(robot_->getVelocity(), "JointVelocities");
    logHelper.add(robot_->getTorque(), "JointTorques");
    logHelper.add(robot_->getJointTor_s(), "SensorTorques");

    logHelper.add(multiControllerState_->spk_, "SpringStiffness");
    logHelper.add(multiControllerState_->spring_tor, "SpringTorque");
    logHelper.add(multiControllerState_->tau_cmd, "CommandTorque");      // motor_torque = command_torque + compensation_torque
    logHelper.add(robot_->tau_motor, "MotorTorque");

    logHelper.add(multiM1MachineRos_->jointTorqueCommand_, "MM1_DesiredJointTorques");
    logHelper.add(multiM1MachineRos_->jointPositionCommand_, "MM1_DesiredJointPositions");
    logHelper.add(multiM1MachineRos_->interactionTorqueCommand_, "MM1_DesiredInteractionTorques");
    logHelper.startLogger();
}

void MultiM1Machine::end() {
    if(initialised) {
        currentState->exit();
        robot_->stop();
//        logHelper.endLog();
        delete multiM1MachineRos_;
        delete robot_;
    }
}

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void MultiM1Machine::hwStateUpdate(void) {
    robot_->updateRobot();
    multiM1MachineRos_->update();
//    logHelper.recordLogData();
    time_ = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time0_).count()) / 1e6;
    ros::spinOnce();
}