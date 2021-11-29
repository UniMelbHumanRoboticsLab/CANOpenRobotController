#include "Robot.h"

short int sign(double val) { return (val > 0) ? 1 : ((val < 0) ? -1 : 0); }

Robot::Robot(std::string robot_name, std::string yaml_config_file): robotName(robot_name) {
    spdlog::debug("Robot ({}) object created", robotName);
}

Robot::~Robot() {
    spdlog::debug("Robot object deleted");
}


bool Robot::initialiseFromYAML(std::string yaml_config_file) {
    if(yaml_config_file.size()>0) {
        // need to use address of base directory because when run with ROS, working directory is ~/.ros
        std::string baseDirectory = XSTR(BASE_DIRECTORY);
        std::string relativeFilePath = "/config/";
        try {
            YAML::Node params = YAML::LoadFile(baseDirectory + relativeFilePath + yaml_config_file);

            if(!params[robotName]){
                spdlog::error("Parameters of {} couldn't be found in {} !", robotName, baseDirectory + relativeFilePath + yaml_config_file);
                spdlog::error("Default parameters used !");

                return false;
            }
            else {
                spdlog::info("Loading robot parameters from {}.", baseDirectory + relativeFilePath + yaml_config_file);
                //Attempt to load parameters from YAML file (delegated to each custom robot implementation)
                return loadParametersFromYAML(params);
            }

        } catch (...) {
            spdlog::error("Failed loading parameters from {}. Using default parameters instead.", baseDirectory + relativeFilePath + yaml_config_file);
            return false;
        }
    }
    else {
        spdlog::info("Using default robot parameters (no YAML file specified).");
        return false;
    }
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
    //std::cout << "dq=[ " << jointVelocities_.transpose() * 180 / M_PI << " ]\t";
    //std::cout << "tau=[ " << jointTorques_.transpose() << " ]\t";
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
