#include "LoggingRobot.h"

LoggingRobot::LoggingRobot() {
    spdlog::info("New Logging Robot");

    initialiseJoints();
    initialiseInputs();
}


bool LoggingRobot::initialiseInputs() {
    spdlog::info("test");

    inputs.push_back(keyboard = new Keyboard());
    spdlog::info("test");

    inputs.push_back(forcePlate = new ForcePlateSensor(0x3f0, 0x3f1, 0x3f2));

    // Add two crutch sensors
    crutchSensors.push_back(new RobotousRFT(0xf8, 0xf9, 0xfa));
    crutchSensors.push_back(new RobotousRFT(0xf0, 0xf1, 0xf2));

    // Add to input stack
    for (int i = 0; i < 2; i++) {
        inputs.push_back(crutchSensors[i]);
    }

    motorPositions = Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1>::Zero(numJoints);
    motorVelocities = Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1>::Zero(numJoints);
    motorTorques = Eigen::Matrix<INTEGER16, Eigen::Dynamic, 1>::Zero(numJoints);
    motorStatusWords = Eigen::Matrix<INTEGER16, Eigen::Dynamic, 1>::Zero(numJoints);
    goButton = -1;
    nextMotion = -1;

    state = -1;
    currentMotion = -1;
    return true;
}

LoggingRobot::~LoggingRobot() {
    spdlog::debug("Delete LoggingRobot object begins");

    // Shouldn't be needed
    for (auto p : joints) {
        spdlog::info("Delete Joint ID: {}", p->getId());
        delete p;
    }

    joints.clear();

    // Delete the inputs
    delete keyboard;
    for (auto cs : crutchSensors) {
        spdlog::info("Delete Crutch Sensor with CommandID: 0x{0:x}", cs->getCommandID());
        delete cs;
    }
    delete forcePlate;
    inputs.clear();

    spdlog::debug("LoggingRobot deleted");
}

bool LoggingRobot::configureMasterPDOs() {

    // Position/Velocity PDOs
    for (int i = 0; i < numJoints; i++){
        void *dataEntryMotor[2] = {(void *)&motorPositions(i), (void *)&motorVelocities(i)};
        UNSIGNED16 dataSize[2] = {4,4};
        rpdos.push_back(new RPDO(0x281+i, 0xff, dataEntryMotor, dataSize, 2));
    }
    // Motor Torque PDOs
    for (int i = 0; i < numJoints; i++){
        void *dataEntryMotor[1] = {(void *)&motorTorques(i)};
        UNSIGNED16 dataSize[1] = {2};
        rpdos.push_back(new RPDO(0x381+i, 0xff, dataEntryMotor, dataSize, 1));
    }
    // Motor Status words (NOT CURRENTLY USED)
    // for (int i = 0; i < numJoints; i++){
    //     void *dataEntryMotor[1] = {(void *)&motorStatusWords(i)};
    //     UNSIGNED16 dataSize[1] = {2};
    //     UNSIGNED8 lengthData = 1;
    //     rpdos.push_back(new RPDO(0x481+i, 0xff, dataEntryMotor, dataSize, lengthData));
    // }

    // PDOs for Status Variables
    void *tempDataEntryPointer[1] = {(void *)&goButton};
    UNSIGNED16 dataSize[1] = {1};
    rpdos.push_back(new RPDO(0x192, 0xff, tempDataEntryPointer, dataSize, 1));

    tempDataEntryPointer[0] = {(void *)&state};
    rpdos.push_back(new RPDO(0x211, 0xff, tempDataEntryPointer, dataSize, 1));

    tempDataEntryPointer[0] = {(void *)&currentMotion};
    rpdos.push_back(new RPDO(0x212, 0xff, tempDataEntryPointer, dataSize, 1));

    return Robot::configureMasterPDOs();
}

Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1>& LoggingRobot::getMotorPositions() {
    return motorPositions;
}
Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1>& LoggingRobot::getMotorVelocities() {
    return motorVelocities;
}
Eigen::Matrix<INTEGER16, Eigen::Dynamic, 1>& LoggingRobot::getMotorTorques() {
    return motorTorques;
}
INTEGER16& LoggingRobot::getGoButton() {
    return goButton;
}
INTEGER16&  LoggingRobot::getCurrentState() {
    return state;
}
INTEGER16&  LoggingRobot::getCurrentMovement() {
    return currentMotion;
}


Eigen::VectorXd& LoggingRobot::getCrutchReadings() {
    updateCrutchReadings();
    return crutchReadings;
}

Eigen::VectorXi& LoggingRobot::getForcePlateReadings() {
    return forcePlate->getForces();
}

void LoggingRobot::updateCrutchReadings(){
    if ((unsigned int)crutchReadings.size() != 6 * crutchSensors.size()) {
        crutchReadings = Eigen::VectorXd::Zero(6 * crutchSensors.size());  // 6 Forces per sensor
    }
    //Update values
    for (int i = 0; i < (int)crutchSensors.size(); i++) {
        Eigen::VectorXd forces = crutchSensors[i]->getForces();
        Eigen::VectorXd torques = crutchSensors[i]->getTorques();
        for (int j = 0; j < 3; j++) {
            crutchReadings[i * 6 + j] = forces[j];
            crutchReadings[i * 6 + 3 + j] = torques[j];
        }
        crutchReadings = Eigen::VectorXd::Zero(6 * crutchSensors.size());
        sensorsOn = false;
        return true;
    } else {
        return false;
    }
}

void LoggingRobot::setCrutchOffsets(Eigen::VectorXd offsets) {
    for (unsigned int i = 0; i < crutchSensors.size(); i++) {
        crutchSensors[i]->setOffsets(offsets.segment(i * 6, 3), offsets.segment(i * 6+3, 3));
    }
}

void LoggingRobot::zeroForcePlate() {
    spdlog::debug("Zeroing Force Plate Sensor");
    forcePlate->zero();
}

bool LoggingRobot::startSensors() {
    if (sensorsOn){
        //do nothing
        return false;
    } else
    {
        for (unsigned int i = 0; i < crutchSensors.size(); i++) {
            crutchSensors[i]->startStream();
        }
        forcePlate->startStream();
        sensorsOn = true; 
        return true;
    }
}

bool LoggingRobot::stopSensors() {
    if (sensorsOn) {
        for (unsigned int i = 0; i < crutchSensors.size(); i++) {
            crutchSensors[i]->stopStream();
        }
        forcePlate->stopStream();
        crutchReadings = Eigen::VectorXd::Zero(6 * crutchSensors.size());
        sensorsOn = false;
        return true;
    } else {
        return false;
    }
}
bool LoggingRobot::startCrutchSensors() {
    if (sensorsOn) {
        //do nothing
        return false;
    } else {
        for (unsigned int i = 0; i < crutchSensors.size(); i++) {
            crutchSensors[i]->startStream();
        }
        sensorsOn = true;
        return true;
    }
}

bool LoggingRobot::stopCrutchSensors() {
    if (sensorsOn) {
        for (unsigned int i = 0; i < crutchSensors.size(); i++) {
            crutchSensors[i]->stopStream();
        }
        crutchReadings = Eigen::VectorXd::Zero(6 * crutchSensors.size());
        sensorsOn = false;
        return true;
    } else {
        return false;
    }
}
