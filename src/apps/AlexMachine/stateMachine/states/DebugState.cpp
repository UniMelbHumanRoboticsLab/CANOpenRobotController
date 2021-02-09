////////// STATE ////////////////////
//-------  Debug ------------/////
////////////////////////////////////
#include "DebugState.h"

#ifdef VIRTUAL

void DebugState::entry(void) {
    std::cout << "Debug State entered" << std::endl
              << "=======================" << std::endl
              << " 0-9 ->> Write angle for joints" << std::endl
              << " return ->> Send input angle" << std::endl
              << "=======================" << std::endl
              << std::endl;
    std::cout << "========================" << endl
              << " Press S to leave debug state" << endl
              << "========================" << endl;

    robot->setCurrentState(AlexState::Debug);

    // entry flag must be set to true by a green button release
    robot->setResetFlag(false);
}

void DebugState::during(void) {
    if (robot->keyboard.getNum() != '\0') {                   // Append the pressed number to angle value (shift current number left: x10, then add new number)
        angle = 10 * angle + robot->keyboard.getNum() - '0';  // Subtract '0' to convert from ASCII id to value ('0' starts at ASCII 48)
    }
    if (robot->keyboard.getEnter()) {
        std::cout << "SENDING " << angle << "degrees TO JOINTS" << std::endl;
        robot->setVirtualPosition(angle);
        angle = 0;  // reset angle
        std::cout
            << "=======================" << std::endl
            << " 0-9 ->> Write angle for joints" << std::endl
            << " return ->> Send input angle" << std::endl
            << "=======================" << std::endl;
    }

    updateCrutch();
    updateFlag();
}

void DebugState::exit(void) {
    robot->printStatus();
    std::cout
        << "Debug State Exited" << endl;
}

#endif 