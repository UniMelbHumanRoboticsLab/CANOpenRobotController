////////// STATE ////////////////////
//-------  Sitting ------------/////
////////////////////////////////////
#include "Sitting.h"
void Sitting::entry() {
    std::cout << "Sitting State Entered " << std::endl
#ifdef KEYBOARD
              << "=======================" << std::endl
              << " A ->> Stand up" << std::endl
#endif
              << "=======================" << std::endl
              << std::endl;
    robot->setCurrentState(AlexState::Sitting);
    //robot->pb.printMenu();
    // entry flag must be set to true by a green button release
    robot->setResetFlag(false);
}
void Sitting::during() {
    // RobotMode modeSelected = robot->pb.updateController(robot->keyboard.getE(), robot->keyboard.getW(), robot->keyboard.getX());
    // if (modeSelected != RobotMode::INITIAL) {
    //     std::cout << "Selected mode: " << robot->pb.printRobotMode(modeSelected) << std::endl;
    //     robot->setNextMotion(modeSelected);
    // }
    //robot->pb.updateGO(robot->keyboard.getD());
    updateCrutch();
    updateFlag();
}
void Sitting::exit() {
}