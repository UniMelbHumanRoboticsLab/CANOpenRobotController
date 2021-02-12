/**
 * @file Robot.cpp
 * @author William Campbell, Justin Fong, Vincent Crocher
 * @brief Generic Abstract Robot class, which includes joints and a trajectory generator, to be used
 *          with a CAN-based robot device
 * @version 0.2
 * @date 2020-10-14
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "Robot.h"

short int sign(double val) { return (val > 0) ? 1 : ((val < 0) ? -1 : 0); }

Robot::Robot(){
    spdlog::debug("Robot object created");
    //initialiseJoints();
    //initialiseInputs();
}

Robot::~Robot() {
    spdlog::debug("Robot object deleted");
}

bool Robot::initialise() {
    if (initialiseNetwork()) {
            return true;
    }
    return false;
}


bool Robot::disable() {
    spdlog::info("Disabling robot...");
    for (auto p : joints) {
        p->disable();
    }
    return true;
}

void Robot::updateRobot() {

    //Retrieve latest values from hardware
    for (auto joint : joints)
        joint->updateValue();
    for (auto input : inputs ){
        input->updateInput();
    }

    //Update local copies of joint values
    if((unsigned int)jointPositions_.size()!=joints.size()) {
        jointPositions_ = Eigen::VectorXd::Zero(joints.size());
    }
    if((unsigned int)jointVelocities_.size()!=joints.size()) {
        jointVelocities_ = Eigen::VectorXd::Zero(joints.size());
    }
    if((unsigned int)jointTorques_.size()!=joints.size()) {
        jointTorques_ = Eigen::VectorXd::Zero(joints.size());
    }
    unsigned int i = 0;
    for (auto joint : joints) {
        jointPositions_[i] = joint->getPosition();
        jointVelocities_[i] = joint->getVelocity();
        jointTorques_[i] = joint->getTorque();
        i++;
    }
}

Eigen::VectorXd& Robot::getPosition() {
    //Initialise vector if not already done
    if((unsigned int)jointPositions_.size()!=joints.size()) {
        jointPositions_ = Eigen::VectorXd::Zero(joints.size());
    }

    //Update values
    unsigned int i = 0;
    for (auto j : joints) {
        jointPositions_[i] = j->getPosition();
        i++;
    }
    return jointPositions_;
}

Eigen::VectorXd& Robot::getVelocity() {
    //Initialise vector if not already done
    if((unsigned int)jointVelocities_.size()!=joints.size()) {
        jointVelocities_ = Eigen::VectorXd::Zero(joints.size());
    }

    //Update values
    unsigned int i = 0;
    for (auto j : joints) {
        jointVelocities_[i] = j->getVelocity();
        i++;
    }
    return jointVelocities_;
}

Eigen::VectorXd& Robot::getTorque() {
    //Initialise vector if not already done
    if((unsigned int)jointTorques_.size()!=joints.size()) {
        jointTorques_ = Eigen::VectorXd::Zero(joints.size());
    }

    //Update values
    unsigned int i = 0;
    for (auto j : joints) {
        jointTorques_[i] = j->getTorque();
        i++;
    }
    return jointTorques_;
}

void Robot::printStatus() {
    std::cout << "q=[ " << jointPositions_.transpose() * 180 / M_PI << " ]\t";
    std::cout << "dq=[ " << jointVelocities_.transpose() * 180 / M_PI << " ]\t";
    std::cout << "tau=[ " << jointTorques_.transpose() << " ]\t";
    std::cout << std::endl;
}

void Robot::printJointStatus(int J_i) {
    joints[J_i]->printStatus();
}

bool Robot::configureMasterPDOs() {
    for (auto j : joints) {
        j->configureMasterPDOs();
    }
    for (auto i : inputs) {
        i->configureMasterPDOs();
    }
    return true;
}