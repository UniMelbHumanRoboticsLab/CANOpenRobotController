#include "FT_RobotM3.h"
using namespace Eigen;
using namespace std;

FT_RobotM3::FT_RobotM3(string robot_name, string yaml_config_file) :  RobotM3(robot_name, yaml_config_file) {
    FT_Sensors.push_back(new RobotousRFT(0xf0, 0xf1, 0xf2));
//     FT_Sensors.push_back(new RobotousRFT(0xf8, 0xf9, 0xf10));
//     FT_Sensors.push_back(new RobotousRFT(0xe6, 0xe7, 0xe8));
//     FT_Sensors.push_back(new RobotousRFT(0xec, 0xed, 0xee));

    // Add to input stack
    for (uint i = 0; i < FT_Sensors.size(); i++) {
        inputs.push_back(FT_Sensors[i]);
    }
    
    // initialise all the wrench readings first depending on number of sensors
    wrenches = Eigen::VectorXd::Zero(6 * FT_Sensors.size());  // 6 Forces per sensor
    correctedWrenches = Eigen::VectorXd::Zero(6 * FT_Sensors.size());  // 6 Forces per sensor
    olStatWrenches = Eigen::VectorXd::Zero(FT_Sensors.size());  // 1 per sensor
}

FT_RobotM3::~FT_RobotM3() {
    spdlog::debug("Delete FT_RobotM3 object begins");
    for (auto ft : FT_Sensors) {
        spdlog::info("FT Sensor CommandID: 0x{0:x} deleted", ft->getCommandID());
        delete ft;
    }
    inputs.clear();
    spdlog::debug("FT_RobotM3 deleted");
}
void FT_RobotM3::updateRobot(){
    RobotM3::updateRobot();
    spdlog::trace("FT_RobotM3::updateRobot()");
    for (int i = 0; i < (int)FT_Sensors.size(); i++) {
        Eigen::VectorXd forces = FT_Sensors[i]->getForces();
        Eigen::VectorXd torques = FT_Sensors[i]->getTorques();
        
        for (int j = 0; j < 3; j++) {
            wrenches[i * 6 + j] = forces[j];
            wrenches[i * 6 + 3 + j] = torques[j];
        }
        olStatWrenches[i] = FT_Sensors[i]->getOverload();
    }
    correctWrenches();
}
void  FT_RobotM3::correctWrenches(){
    // correct the readings here
    Eigen::VectorXd faulty_force = wrenches.segment(0, 3);
    Eigen::VectorXd faulty_torque = wrenches.segment(3, 3);

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(-M_PI*50/180, Eigen::Vector3d::UnitZ());  // Rotate 50 degrees around Z-axis
    Eigen::VectorXd corrected_force = rotation_matrix * faulty_force;
    Eigen::VectorXd corrected_torque = rotation_matrix * faulty_torque;
    
    correctedWrenches[0] = corrected_force[0];
    correctedWrenches[1] = corrected_force[1];
    correctedWrenches[2] = corrected_force[2];
    correctedWrenches[3] = corrected_torque[0];
    correctedWrenches[4] = corrected_torque[1];
    correctedWrenches[5] = corrected_torque[2];
}

Eigen::VectorXd& FT_RobotM3::getWrenches() {
    return correctedWrenches;
}
Eigen::VectorXd& FT_RobotM3::getOlStatWrenches(){
    return olStatWrenches;
}
void FT_RobotM3::printStatus() {
    std::cout << std::setprecision(3) << std::fixed << std::showpos;
    std::cout << "X=[ " << getEndEffPosition().transpose() << " ]\t";
    std::cout << "dX=[ " << getEndEffVelocity().transpose() << " ]\t";
    std::cout <<  std::endl;
    std::cout << "F_int=[ " << getInteractionForce().transpose() << " ]\t";
    std::cout << "|F_int|=[ " << getInteractionForce().transpose().norm() << " ]\t";
    std::cout <<  std::endl;
    std::cout <<  std::noshowpos;
}
void FT_RobotM3::printWrenches() {
    Eigen::VectorXd curSensorWrenches = getWrenches();
    for (int i = 0; i < (int)FT_Sensors.size(); i++) {
        Eigen::VectorXd curSensorWrench = curSensorWrenches.segment(i * 6, 6);
        std::cout << std::setprecision(3) << std::fixed << std::showpos;
        std::cout << "F" << i << "=[ " << curSensorWrench.transpose() << " ]\t";
        std::cout << "|F" << i << "|=[ " << curSensorWrench.segment(0, 3).norm() << " ]\t";
        std::cout << "|M" << i << "|=[ " << curSensorWrench.segment(3, 3).norm() << " ]\t";
        std::cout << "Stat" << i << "=[ " << olStatWrenches[i] << " ]\t";
        std::cout <<  std::endl;
        std::cout <<  std::noshowpos;
    }
}

/**
 * @ Configure FT sensors
 *
 */
void FT_RobotM3::setWrenchesOffset(Eigen::VectorXd offsets) {
    for (unsigned int i = 0; i < FT_Sensors.size(); i++) {
        FT_Sensors[i]->setOffsets(offsets.segment(i * 6, 3), offsets.segment(i * 6+3, 3));
    }
}
bool FT_RobotM3::startFT_Sensors() {
    if (sensorsOn) {
        //do nothing
        return false;
    } else {
        for (unsigned int i = 0; i < FT_Sensors.size(); i++) {
            FT_Sensors[i]->startStream();
        }
        sensorsOn = true;
        return true;
    }
}
bool FT_RobotM3::stopFT_Sensors() {
    if (sensorsOn) {
        for (unsigned int i = 0; i < FT_Sensors.size(); i++) {
            FT_Sensors[i]->stopStream();
        }
        wrenches = Eigen::VectorXd::Zero(6 * FT_Sensors.size());
        correctedWrenches = Eigen::VectorXd::Zero(6 * FT_Sensors.size());
        sensorsOn = false;
        return true;
    } else {
        return false;
    }
}

bool FT_RobotM3::setFT_SensorsFilter() {
    for (unsigned int i = 0; i < FT_Sensors.size(); i++) {
        FT_Sensors[i]->setFilter();
    }
    return true;
}

bool FT_RobotM3::configureMasterPDOs() {
    return Robot::configureMasterPDOs();
}
