#include "RightForward.h"

void RightForward::entry(void) {
    spdlog::info(" RIGHT FORWARD STATE ");
#ifdef KEYBOARD
        spdlog::info(" S ->> WALK of A ->> FEET TOGETHER ");
#endif
    robot->setCurrentState(AlexState::RightForward);
    //robot->pb.printMenu();
    // entry flag must be set to true by a green button release
    robot->setResetFlag(false);
}
void RightForward::during(void) {
    // RobotMode modeSelected = robot->pb.updateController(robot->keyboard.getE(), robot->keyboard.getW(), robot->keyboard.getX());
    // if (modeSelected != RobotMode::INITIAL) {
    //     std::cout << "Selected mode: " << robot->pb.printRobotMode(modeSelected) << std::endl;
    //     ;
    // }
    updateCrutch();
    updateFlag();
}
void RightForward::exit(void) {
    spdlog::debug("RightForward state exited");
}
