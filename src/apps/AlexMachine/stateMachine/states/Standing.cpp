////////// STATE ////////////////////
//-------  Standing ------------/////
////////////////////////////////////
#include "Standing.h"

void Standing::entry(void) {
    spdlog::info("Standing State Entered ");

#ifdef KEYBOARD
    spodlog::info(" A ->> sit down or S ->> start Walk");
#endif
    robot->setCurrentState(AlexState::Standing);
    //robot->pb.printMenu();
    // entry flag must be set to true by a green button release
    robot->setResetFlag(false);
}

void Standing::during(void) {
    // RobotMode modeSelected = robot->pb.updateController(robot->keyboard.getE(), robot->keyboard.getW(), robot->keyboard.getX());
    // if (modeSelected != RobotMode::INITIAL) {
    //     std::cout << "Selected mode: " << robot->pb.printRobotMode(modeSelected) << std::endl;
    //     ;
    // }
    updateCrutch();
    updateFlag();
}

void Standing::exit(void) {
    spdlog::debug("Standing State Exited");
}
