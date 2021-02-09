#include "ALEXCrutchController.h"

ALEXCrutchController::ALEXCrutchController() {
    spdlog::debug("ALEXCrutchController created");
}

void ALEXCrutchController::updateGO(bool go) {
    if (go) {
        goButton = 1;
    } else {
        goButton = 0;
    }
}

bool ALEXCrutchController::getGo() {
    if (goButton == 1) {
        return true;
    } else {
        return false;
    }
}

void ALEXCrutchController::updateInput(){

}
bool ALEXCrutchController::configureMasterPDOs(){
        UNSIGNED16 dataSize[1] = {1};
        void *dataGo[1] = {
            (void *)&goButton,
        };

        goRPDO = new RPDO(0x192, 0xff, dataGo, dataSize, 1);

        void *dataNextMove[1] = {
            (void *)&nextMovement,
        };

        nextMovementRPDO = new RPDO(0x191, 0xff, dataNextMove, dataSize, 1);

        return true;
}

RobotMode ALEXCrutchController::updateController(bool up, bool dwn, bool select) {
    RobotMode returnValue = RobotMode::INITIAL;
    if (up) {
        if (currentMode == (NUMMODES - 1)) {
            //end of list go to start
            currentMode = 0;
        } else {
            currentMode++;
        }
        printMenu();
    } else if (dwn) {
        if (currentMode == 0) {
            //endcurrentMode of list go to start
            currentMode = NUMMODES - 1;
        } else {
            currentMode--;
        }
        printMenu();
    } else if (select) {
        printRobotMode(nextMotion[currentMode]);
        returnValue = nextMotion[currentMode];
    }
    //printMenu();
    return returnValue;
}
/**
     * \brief print current menu of motions to the screen -> next Motion on crutch
     * 
     */
void ALEXCrutchController::printMenu() {
    int n = sizeof(nextMotion) / sizeof(nextMotion[0]);
    // print from ind-th index to (n+i)th index.
    for (int i = currentMode; i < n + currentMode; i++) {
        std::cout << printRobotMode(nextMotion[(i % n)]) << " , ";
    }
    std::cout << std::endl;
}
std::string ALEXCrutchController::printRobotMode(RobotMode mode) {
    switch (mode) {
        case RobotMode::NORMALWALK:
            return "Normal Walk";
            break;
        case RobotMode::SITDWN:
            return "Sitting Down";
            break;
        case RobotMode::STNDUP:
            return "Standing Up";
            break;
        case RobotMode::UNEVEN:
            return "Uneven Walk";
            break;
        case RobotMode::FTTG:
            return "Feet Together";
            break;
        case RobotMode::INITIAL:
            return "Initial Sit ";
            break;
        case RobotMode::UPSTAIR:
            return "Upstairs";
            break;
        case RobotMode::DWNSTAIR:
            return "DWNSTAIR";
            break;
        case RobotMode::TILTUP:
            return "TILTUP";
            break;
        case RobotMode::TILTDWN:
            return "TILTDWN";
            break;
        case RobotMode::BKSTEP:
            return "Backstep";
            break;
        default:
            return "No SELECTA";
            break;
    }
}
int ALEXCrutchController::intRobotMode(RobotMode mode) {
    switch (mode) {
        case RobotMode::NORMALWALK:
            return 1;
            break;
        case RobotMode::SITDWN:
            return 2;
            break;
        case RobotMode::STNDUP:
            return 3;
            break;
        default:
            return 0;
            break;
    }
}