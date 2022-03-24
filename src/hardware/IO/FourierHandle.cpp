#include "FourierHandle.h"

FourierHandle::FourierHandle(int sensor_can_node_ID): InputDevice(),
                                                      sensorNodeID_(sensor_can_node_ID){

    buttonValues_ = Eigen::VectorXd::Zero(4);
}

bool FourierHandle::configureMasterPDOs() {
    UNSIGNED16 dataSize[4] = {1, 1, 1, 1};
    void *dataEntry[4] = {(void *)&rawData_[0],
                          (void *)&rawData_[1],
                          (void *)&rawData_[2],
                          (void *)&rawData_[3],};

    rpdo_ = new RPDO(0x180+sensorNodeID_, 0xff, dataEntry, dataSize, 4);

    return true;
}

void FourierHandle::updateInput() {

    for(int i = 0; i<4; i++){
        buttonValues_[i] = rawData_[i];
    }
}

Eigen::VectorXd& FourierHandle::getButtonValues() {
    return buttonValues_;
}