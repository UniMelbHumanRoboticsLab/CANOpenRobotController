#include "LoggingRobot.h"

LoggingRobot::LoggingRobot() {
    spdlog::info("New Logging Robot");

    initialiseJoints();
    initialiseInputs();
}


bool LoggingRobot::initialiseInputs() {

    inputs.push_back(keyboard = new Keyboard());

    // Force Plates
    forcePlates.push_back(new ForcePlateSensor(0x3f0, 0x3f1, 0x3f2));
    forcePlates.push_back(new ForcePlateSensor(0x3f0, 0x3f3, 0x3f4));
    forcePlates.push_back(new ForcePlateSensor(0x3f0, 0x3f5, 0x3f6));
    forcePlates.push_back(new ForcePlateSensor(0x3f0, 0x3f7, 0x3f8));

    // Add to input stack
    for (int i = 0; i < forcePlates.size(); i++) {
        inputs.push_back(forcePlates[i]);
    }

    // Foot Sensors
    footSensors.push_back(new ForcePlateSensor(0x3e0, 0x3e1, 0x3e2));
    footSensors.push_back(new ForcePlateSensor(0x3e3, 0x3e4, 0x3e5));

    // Add to input stack
    for (int i = 0; i < footSensors.size(); i++) {
        inputs.push_back(footSensors[i]);
    }

    // Add two crutch sensors
    crutchSensors.push_back(new RobotousRFT(0xf8, 0xf9, 0xfa));
    crutchSensors.push_back(new RobotousRFT(0xf0, 0xf1, 0xf2));

    // Add to input stack
    for (int i = 0; i < crutchSensors.size(); i++) {
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
    for (auto fp : forcePlates){
        delete fp;
    }
    for (auto fs : footSensors) {
        delete fs;
    }
    inputs.clear();

    spdlog::debug("LoggingRobot deleted");
}

bool LoggingRobot::configureMasterPDOs() {
/*
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
    UNSIGNED16 dataSize[1] = {2};
    rpdos.push_back(new RPDO(0x192, 0xff, tempDataEntryPointer, dataSize, 1));

    dataSize[0] = 1;
    tempDataEntryPointer[0] = {(void *)&state};
    rpdos.push_back(new RPDO(0x211, 0xff, tempDataEntryPointer, dataSize, 1));

    tempDataEntryPointer[0] = {(void *)&currentMotion};
    rpdos.push_back(new RPDO(0x212, 0xff, tempDataEntryPointer, dataSize, 1));
*/
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
INTEGER8&  LoggingRobot::getCurrentState() {
    return state;
}
INTEGER8&  LoggingRobot::getCurrentMovement() {
    return currentMotion;
}


Eigen::VectorXd& LoggingRobot::getCrutchReadings() {
    updateCrutchReadings();
    return crutchReadings;
}

Eigen::VectorXi& LoggingRobot::getForcePlateReadings() {
    forcePlateForces = Eigen::VectorXi::Zero(forcePlates.size() * 4);
    int i = 0;
    for (auto fp : forcePlates) {
        forcePlateForces.segment<4>(i * 4) = fp->getForces();
        i++;
    }

    return forcePlateForces;
}

Eigen::VectorXi& LoggingRobot::getFootSensorReadings() {
    footSensorForces = Eigen::VectorXi::Zero(footSensors.size() * 4);
    int i = 0;
    for (auto fs : footSensors) {
        footSensorForces.segment<4>(i * 4) = fs->getForces();
        i++;
    }

    return footSensorForces;
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
    spdlog::debug("Zeroing Force Plate Sensors");
    for (auto fp :forcePlates){
        fp->zero();
    }
}

void LoggingRobot::zeroLeftFoot() {
    spdlog::debug("Zeroing Left Foot");
    footSensors[0]->zero();
    spdlog::info("{}", (void*)footSensors[0]);
}

void LoggingRobot::zeroRightFoot() {
    spdlog::debug("Zeroing Right Foot");
    footSensors[1]->zero();
    spdlog::info("{}", (void*)footSensors[1]);
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
        for (auto fp : forcePlates) {
            fp->startStream();
        }
        for (auto fs : footSensors) {
            fs->startStream();
            spdlog::info("{}", (void*)fs);
        }
        sensorsOn = true; 
        return true;
    }
}

bool LoggingRobot::stopSensors() {
    if (sensorsOn) {
        for (unsigned int i = 0; i < crutchSensors.size(); i++) {
            crutchSensors[i]->stopStream();
        }
        for (auto fp : forcePlates) {
            fp->stopStream();
        }
        for (auto fs : footSensors) {
            fs->stopStream();
        }
        crutchReadings = Eigen::VectorXd::Zero(6 * crutchSensors.size());
        forcePlateForces =Eigen::VectorXi::Zero(forcePlates.size() * 4);
        footSensorForces = Eigen::VectorXi::Zero(footSensors.size() * 4);
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
