#include "UBORobot.h"

UBORobot::UBORobot(std::string robot_name, std::string yaml_config_file) :  Robot(robot_name, yaml_config_file) {
    spdlog::info("New Logging Robot");

    //Check if YAML file exists and contain robot parameters
    initialiseFromYAML(yaml_config_file);

    initialiseJoints();
    initialiseInputs();
}

bool UBORobot::initialiseInputs() {

    inputs.push_back(keyboard = new Keyboard());

    UBO_FTSensors.push_back(new RobotousRFT(0xf0, 0xf1, 0xf2));
    // UBO_FTSensors.push_back(new RobotousRFT(0xf8, 0xf9, 0xf10));
    UBO_FTSensors.push_back(new RobotousRFT(0xe6, 0xe7, 0xe8));
    UBO_FTSensors.push_back(new RobotousRFT(0xec, 0xed, 0xee));

    // Add to input stack
    for (uint i = 0; i < UBO_FTSensors.size(); i++) {
        inputs.push_back(UBO_FTSensors[i]);
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

UBORobot::~UBORobot() {
    spdlog::debug("Delete UBORobot object begins");

    // Shouldn't be needed
    for (auto p : joints) {
        spdlog::info("Delete Joint ID: {}", p->getId());
        delete p;
    }
    joints.clear();

    // Delete the inputs
    delete keyboard;
    for (auto cs : UBO_FTSensors) {
        spdlog::info("Delete UBO Sensor with CommandID: 0x{0:x}", cs->getCommandID());
        delete cs;
    }
    inputs.clear();

    spdlog::debug("UBORobot deleted");
}

bool UBORobot::configureMasterPDOs() {
    return Robot::configureMasterPDOs();
}

Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1>& UBORobot::getMotorPositions() {
    return motorPositions;
}
Eigen::Matrix<INTEGER32, Eigen::Dynamic, 1>& UBORobot::getMotorVelocities() {
    return motorVelocities;
}
Eigen::Matrix<INTEGER16, Eigen::Dynamic, 1>& UBORobot::getMotorTorques() {
    return motorTorques;
}
INTEGER16& UBORobot::getGoButton() {
    return goButton;
}
INTEGER8&  UBORobot::getCurrentState() {
    return state;
}
INTEGER8&  UBORobot::getCurrentMovement() {
    return currentMotion;
}

Eigen::VectorXd& UBORobot::getUBO_readings() {
    updateUBO_readings();
    return UBO_readings;
}
void UBORobot::updateUBO_readings(){
    if ((unsigned int)UBO_readings.size() != 6 * UBO_FTSensors.size()) {
        UBO_readings = Eigen::VectorXd::Zero(6 * UBO_FTSensors.size());  // 6 Forces per sensor
    }
    //Update values
    for (int i = 0; i < (int)UBO_FTSensors.size(); i++) {
        Eigen::VectorXd forces = UBO_FTSensors[i]->getForces();
        Eigen::VectorXd torques = UBO_FTSensors[i]->getTorques();
        
        for (int j = 0; j < 3; j++) {
            UBO_readings[i * 6 + j] = forces[j];
            UBO_readings[i * 6 + 3 + j] = torques[j];
        }
    }
}
void UBORobot::printUBO_readings(Eigen::VectorXd readings) {
    for (int i = 0; i < (int)UBO_FTSensors.size(); i++) {
        std::cout << std::setprecision(3) << std::fixed << std::showpos;
        std::cout << "F" << i << "=[ " << readings.segment(i * 6, 6).transpose() << " ]\t";
        std::cout << "Stat" << i << "=[ " << UBO_FTSensors[i]->getOverload() << " ]\t";
        std::cout <<  std::endl;
        std::cout <<  std::noshowpos;
    }
}

Eigen::VectorXd&  UBORobot::getCorrectedUBO_readings(){
    updateUBO_readings();
    // correct the readings here
    {
        Eigen::VectorXd faulty_force = UBO_readings.segment(0, 3);
        Eigen::VectorXd faulty_torque = UBO_readings.segment(3, 3);

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix = Eigen::AngleAxisd(-M_PI*50/180, Eigen::Vector3d::UnitZ());  // Rotate 50 degrees around Z-axis
        Eigen::VectorXd corrected_force = rotation_matrix * faulty_force;
        Eigen::VectorXd corrected_torque = rotation_matrix * faulty_torque;

        UBO_readings[0] = corrected_force[0];
        UBO_readings[1] = corrected_force[1];
        UBO_readings[2] = corrected_force[2];

        UBO_readings[3] = corrected_torque[0];
        UBO_readings[4] = corrected_torque[1];
        UBO_readings[5] = corrected_torque[2];
    }
    return UBO_readings;
}

Eigen::VectorXd& UBORobot::getUBO_OLStatus(){
    if ((unsigned int)UBO_olStats.size() != 6 * UBO_FTSensors.size()) {
        UBO_olStats = Eigen::VectorXd::Zero(UBO_FTSensors.size());  // 6 Forces per sensor
    }
    //Update values
    for (int i = 0; i < (int)UBO_FTSensors.size(); i++) {
        UBO_olStats[i] = UBO_FTSensors[i]->getOverload();
    }
    return UBO_olStats;
}

void UBORobot::setUBOOffsets(Eigen::VectorXd offsets) {
    for (unsigned int i = 0; i < UBO_FTSensors.size(); i++) {
        UBO_FTSensors[i]->setOffsets(offsets.segment(i * 6, 3), offsets.segment(i * 6+3, 3));
    }
}
bool UBORobot::startUBO_FTSensors() {
    if (sensorsOn) {
        //do nothing
        return false;
    } else {
        for (unsigned int i = 0; i < UBO_FTSensors.size(); i++) {
            UBO_FTSensors[i]->startStream();
        }
        sensorsOn = true;
        return true;
    }
}
bool UBORobot::stopUBO_FTSensors() {
    if (sensorsOn) {
        for (unsigned int i = 0; i < UBO_FTSensors.size(); i++) {
            UBO_FTSensors[i]->stopStream();
        }
        UBO_readings = Eigen::VectorXd::Zero(6 * UBO_FTSensors.size());
        sensorsOn = false;
        return true;
    } else {
        return false;
    }
}

bool UBORobot::setUBO_FTSensorsFilter() {
    for (unsigned int i = 0; i < UBO_FTSensors.size(); i++) {
        UBO_FTSensors[i]->setFilter();
    }
    return true;
}
