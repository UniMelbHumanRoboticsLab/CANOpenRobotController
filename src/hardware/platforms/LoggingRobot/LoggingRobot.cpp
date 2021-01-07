#include "LoggingRobot.h"

LoggingRobot::LoggingRobot() {
    inputs.push_back(keyboard = new Keyboard());
};

LoggingRobot::~LoggingRobot() {
    spdlog::debug("Delete LoggingRobot object begins");
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    joints.clear();
    delete keyboard;
    inputs.clear();
    spdlog::debug("LoggingRobot deleted");
}
