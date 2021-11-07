#include "ForcePlate.h"

ForcePlate::ForcePlate(std::string robot_name, std::string yaml_config_file) :  Robot(robot_name, yaml_config_file) {
    spdlog::info("New ForcePlate Robot");

    //Check if YAML file exists and contain robot parameters
    initialiseFromYAML(yaml_config_file);

    initialiseJoints();
    initialiseInputs();
}

bool ForcePlate::initialiseInputs() {
    // Useful for testing, not really required but doesn't hurt to have it, even without a keyboard
    inputs.push_back(keyboard = new Keyboard());

    // There are 4 Strain guage objects on this
    Eigen::Matrix<int, 4, 2> inputPins;
    inputPins(0,0) = 2; // Sensor 1
    inputPins(0,1) = 4;
    inputPins(1,0) = 2; // Sensor 2
    inputPins(1,1) = 6;
    inputPins(2, 0) = 2;  // Sensor 3
    inputPins(2, 1) = 8;
    inputPins(3, 0) = 2;  // Sensor 4
    inputPins(3, 1) = 10;

    Eigen::Vector2i clock = {2,2}; // Clock Pin

    inputs.push_back(strainGauge = new HX711(inputPins, clock));
    strainGauge->begin(128);

    return true;
}

ForcePlate::~ForcePlate() {
    spdlog::debug("Delete ForcePlate object begins");

    // Delete any joints (there shouldn't be any)
    for (auto p : joints) {
        spdlog::info("Delete Joint ID: {}", p->getId());
        delete p;
    }
    joints.clear();

    // Delete the Inputs
    delete keyboard;
    delete strainGauge;
    inputs.clear();

    spdlog::debug("ForcePlate deleted");
}


Eigen::VectorXd& ForcePlate::getStrainReadings() {
    strainForces = strainGauge->getAllForces();
    updatePDOs();
    return strainForces;
}

Eigen::VectorXi ForcePlate::getRawStrainReadings() {
    return strainGauge->getAllRawData();
}

void ForcePlate::setStrainOffsets(Eigen::VectorXi offsets) {
    for (int i = 0; i < offsets.size(); i++) {
        strainGauge->set_offset(i, offsets(i));
    }
}

bool ForcePlate::configureMasterPDOs(){
    Robot::configureMasterPDOs();
    strainForcesTPDO = Eigen::VectorXi(4);

    UNSIGNED16 dataSize[2] = {4,4};
    UNSIGNED16 RPDO_CMD = FP_CMDRPDO;
    UNSIGNED16 TPDOStart = FP_STARTTPDO;

    // Create TPODs for the measurements
    for (int i = 0; i < 2; i++) {
        void *dataPointer[] = {(void *)&strainForcesTPDO(2 * i), (void *)&strainForcesTPDO(2 * i + 1)};
        tpdos.push_back(new TPDO(TPDOStart + i, 0xff, dataPointer, dataSize, 2));
    }

    UNSIGNED16 dataCmdSize[2] = {4};
    void *cmdPointer[] = {(void *)&currCommand};
    rpdoCmd = new RPDO(RPDO_CMD, 0xff, cmdPointer, dataCmdSize, 1);

    return true;
}

void ForcePlate::updateRobot(){
    Robot::updateRobot();

    // Uncomment the line below if you want updates on the CANbus all the time
    // getStrainReadings();
    // updatePDOs();
}

void ForcePlate::updatePDOs() {
    // Uncomment the line below if you want updates on the CANbus all the time
    for (int i = 0; i < strainForces.size(); i++){
        strainForcesTPDO(i) = strainForces(i);
    }
}

ForcePlateCommand ForcePlate::getCommand() {
    return currCommand;
}

void ForcePlate::resetCommand(){
    currCommand = NONE;
}
