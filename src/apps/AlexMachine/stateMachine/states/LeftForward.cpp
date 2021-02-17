#include "LeftForward.h"

void LeftForward::entry(void) {
    spdlog::info(" Left FORWARD STATE ");
#ifdef KEYBOARD
     spdlog::info(" S ->> WALK or A ->> FEET TOGETHER ");
#endif
    robot->setCurrentState(AlexState::LeftForward);
    //robot->pb.printMenu();
    // entry flag must be set to true by a green button release
    robot->setResetFlag(false);
}
void LeftForward::during(void) {
    // Virtual crutch menu + OD.nm
    // RobotMode modeSelected = robot->pb.updateController(robot->keyboard.getE(), robot->keyboard.getW(), robot->keyboard.getX());
    // if (modeSelected != RobotMode::INITIAL) {
    //     std::cout << "Selected mode: " << robot->pb.printRobotMode(modeSelected) << std::endl;
    // }
    updateCrutch();
    updateFlag();
}
void LeftForward::exit(void) {
    spdlog::debug("Left Forward state exited");
}
