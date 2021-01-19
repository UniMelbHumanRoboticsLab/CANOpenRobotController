#include "LoggingRobot.h"

LoggingRobot::LoggingRobot() {
    spdlog::info("New Logging Robot");

    inputs.push_back(keyboard = new Keyboard());
    
    // Add two crutch sensors
    crutchSensors.push_back(new RobotousRFT(0xf8, 0xf9, 0xfa));
    crutchSensors.push_back(new RobotousRFT(0xf0, 0xf1, 0xf2));

    // Add to input stack
    for (int i = 0; i < 2; i++) {
        inputs.push_back(crutchSensors[i]);
    }
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

Eigen::VectorXd& LoggingRobot::getCrutchSensors() {

    if ((unsigned int)crutchForces.size() != crutchSensors.size()) {
        crutchForces = Eigen::VectorXd::Zero(3*crutchSensors.size()); // 3 Forces per sensor
    }

    //Update values
    for (int i = 0; i < (int) crutchSensors.size(); i++) {
        Eigen::VectorXd forces = crutchSensors[i]->getForces();
        for (int j =0; j < 3; j++){
            crutchForces[i * 3 + j] = forces[j];
        }
    }
    spdlog::info("crutchForces {}", crutchForces[0]);
    return crutchForces;
}