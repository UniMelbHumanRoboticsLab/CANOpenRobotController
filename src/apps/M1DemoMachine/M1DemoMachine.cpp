#include "M1DemoMachine.h"

#define OWNER ((M1DemoMachine *)owner)

M1DemoMachine::M1DemoMachine() {
    std::cout << "M1DemoMachine::constructed!" << std::endl;

    // create robot
    robot_ = new RobotM1();

    // Create ros object
    m1DemoMachineRos_ = new M1DemoMachineROS(robot_);

}

M1DemoMachine::~M1DemoMachine() {
    currentState->exit();
    robot_->disable();
    delete m1DemoMachineRos_;
    delete robot_;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */

void M1DemoMachine::init(int argc, char *argv[]) {
    std::cout << "M1DemoMachine::init()" << std::endl;

    ros::init(argc, argv, "m1", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle("~");

    // Pass nodeHandle to the classes that use ROS features
    m1DemoMachineRos_->setNodeHandle(nodeHandle);

    // Create states with ROS features // This should be created after ros::init()
    multiControllerState_ = new MultiControllerState(this, robot_, m1DemoMachineRos_);

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

    m1DemoMachineRos_->initialize();
}

void M1DemoMachine::end() {
    if(initialised) {
        currentState->exit();
        robot_->stop();
        delete m1DemoMachineRos_;
        delete robot_;
    }
}

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M1DemoMachine::hwStateUpdate(void) {
//    spdlog::debug("hw/**/StateUpdate!");
//    std::cout << "M1DemoMachine::hwStateUpdate()" << std::endl;
    robot_->updateRobot();
    m1DemoMachineRos_->update();
    ros::spinOnce();
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
//bool M1DemoMachine::Event2Demo::check(void) {
//    if (OWNER->robot->keyboard->getS() == true) {
//        std::cout << "Pressed S!" << std::endl;
//        return true;
//    }
//    return false;
//}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
//bool M1DemoMachine::Event2Monitor::check(void) {
//    if (OWNER->robot->keyboard->getX() == true) {
//        std::cout << "Pressed S!" << std::endl;
//        return true;
//    }
//    return false;
//}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
//bool M1DemoMachine::Event2Idle::check(void) {
//    if (OWNER->robot->keyboard->getQ() == true) {
//        std::cout << "Pressed Q!" << std::endl;
//        return true;
//    }
//    return false;
//}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
//bool M1DemoMachine::Event2Pos::check(void) {
//    if (OWNER->robot->keyboard->getA() == true) {
////        std::cout << "Pressed A!" << std::endl;
//        return true;
//    }
//    return false;
//}