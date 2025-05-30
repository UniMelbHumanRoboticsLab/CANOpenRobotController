#include "CANIMU.h"

#define INT16_MAX 32767

// Define the zero default return values
Eigen::VectorXd CANIMU::defaultVector3d_ = Eigen::VectorXd::Zero(3);
Eigen::VectorXd CANIMU::defaultVector4d_ = Eigen::VectorXd::Zero(4);

CANIMU::CANIMU(IMUParameters IMUParameters1) {
    numIMUs_ = 1;
    IMUParameters1_ = IMUParameters1;
    IMUData1_ = IMUData();
    spdlog::info("CANIMU Object Created with 1 IMU");
}

CANIMU::CANIMU(IMUParameters IMUParameters1, IMUParameters IMUParameters2) {
    numIMUs_ = 2;
    IMUParameters1_ = IMUParameters1;
    IMUParameters2_ = IMUParameters2;
    IMUData1_ = IMUData();
    IMUData2_ = IMUData();
    spdlog::info("CANIMU Object Created with 2 IMUs");
}

bool CANIMU::configureMasterPDOForOneIMU(const IMUParameters& IMUParameters, UNSIGNED8 rawCanBusData[]) {    
    // Store the 1st can message to the 0-7 index of the temporary storagerawCanBusData
    void *canMessage1[8] = {(void *)&rawCanBusData[0],
                                (void *)&rawCanBusData[1],
                                (void *)&rawCanBusData[2],
                                (void *)&rawCanBusData[3],
                                (void *)&rawCanBusData[4],
                                (void *)&rawCanBusData[5],
                                (void *)&rawCanBusData[6],
                                (void *)&rawCanBusData[7]};
    
    // Store the 2nd can message to the 8-15 index of the temporary storage rawCanBusData
    void *canMessage2[8] = {(void *)&rawCanBusData[8],
                                (void *)&rawCanBusData[9],
                                (void *)&rawCanBusData[10],
                                (void *)&rawCanBusData[11],
                                (void *)&rawCanBusData[12],
                                (void *)&rawCanBusData[13],
                                (void *)&rawCanBusData[14],
                                (void *)&rawCanBusData[15]};
    
    // Store the 3rd can message to the 16-23 index of the temporary storage rawCanBusData
    void *canMessage3[8] = {(void *)&rawCanBusData[16],
                              (void *)&rawCanBusData[17],
                              (void *)&rawCanBusData[18],
                              (void *)&rawCanBusData[19],
                              (void *)&rawCanBusData[20],
                              (void *)&rawCanBusData[21],
                              (void *)&rawCanBusData[22],
                              (void *)&rawCanBusData[23]};

    // Create the RPDO objects for the 3 can messages
    rpdo1 = new RPDO(IMUParameters.COBID1, 0xff, canMessage1, dataSize, lengthData);
    rpdo2 = new RPDO(IMUParameters.COBID2, 0xff, canMessage2, dataSize, lengthData);
    rpdo3 = new RPDO(IMUParameters.COBID3, 0xff, canMessage3, dataSize, lengthData);
    
    return true;
}

void CANIMU::updateInputForOneIMU(IMUData& IMUData, const IMUParameters& IMUParameters, UNSIGNED8 rawCanBusData[]) {
    // Extract the raw data from the can message
    int16_t rawRawAccX = rawCanBusData[0] * 256 + rawCanBusData[1];
    int16_t rawRawAccY = rawCanBusData[2] * 256 + rawCanBusData[3];
    int16_t rawRawAccZ = rawCanBusData[4] * 256 + rawCanBusData[5];

    int16_t rawLinAccX = rawCanBusData[8] * 256 + rawCanBusData[9];
    int16_t rawLinAccY = rawCanBusData[10] * 256 + rawCanBusData[11];
    int16_t rawLinAccZ = rawCanBusData[12] * 256 + rawCanBusData[13];

    int16_t rawQuatW = rawCanBusData[16] * 256 + rawCanBusData[17];
    int16_t rawQuatX = rawCanBusData[18] * 256 + rawCanBusData[19];
    int16_t rawQuatY = rawCanBusData[20] * 256 + rawCanBusData[21];
    int16_t rawQuatZ = rawCanBusData[22] * 256 + rawCanBusData[23];

    // Convert the data to its original unit (e.g. m/s^2, etc.)
    IMUData.rawAcc[0] = range_mapping(rawRawAccX, IMUParameters.acclRange, INT16_MAX);
    IMUData.rawAcc[1] = range_mapping(rawRawAccY, IMUParameters.acclRange, INT16_MAX);
    IMUData.rawAcc[2] = range_mapping(rawRawAccZ, IMUParameters.acclRange, INT16_MAX);

    IMUData.linAcc[0] = range_mapping(rawLinAccX, IMUParameters.acclRange, INT16_MAX);
    IMUData.linAcc[1] = range_mapping(rawLinAccY, IMUParameters.acclRange, INT16_MAX);
    IMUData.linAcc[2] = range_mapping(rawLinAccZ, IMUParameters.acclRange, INT16_MAX);

    IMUData.quat[0] = range_mapping(rawQuatW, IMUParameters.orenQuantRange, INT16_MAX);
    IMUData.quat[1] = range_mapping(rawQuatX, IMUParameters.orenQuantRange, INT16_MAX);
    IMUData.quat[2] = range_mapping(rawQuatY, IMUParameters.orenQuantRange, INT16_MAX);
    IMUData.quat[3] = range_mapping(rawQuatZ, IMUParameters.orenQuantRange, INT16_MAX);
}

bool CANIMU::configureMasterPDOs() {
    configureMasterPDOForOneIMU(IMUParameters1_, rawDataIMU1);
    if (numIMUs_ >= 2) {
        configureMasterPDOForOneIMU(IMUParameters2_, rawDataIMU2);
    }
    spdlog::debug("The RPDOs of the IMU-MCU are initialised");
    return true;
}

void CANIMU::updateInput() {
    updateInputForOneIMU(IMUData1_, IMUParameters1_, rawDataIMU1);
    if (numIMUs_ >= 2) {
        updateInputForOneIMU(IMUData2_, IMUParameters2_, rawDataIMU2);
    }
}

float CANIMU::range_mapping(float msgVal, float sensorRange, float msgMax) {
    return (msgVal / (msgMax /  sensorRange));
}

Eigen::VectorXd& CANIMU::getRawAccFromTheFirstIMU() {
    return IMUData1_.rawAcc;
}

Eigen::VectorXd& CANIMU::getLinAccFromTheFirstIMU() {
    return IMUData1_.linAcc;
}

Eigen::VectorXd& CANIMU::getQuatFromTheFirstIMU() {
    return IMUData1_.quat;
}

Eigen::VectorXd& CANIMU::getRawAccFromTheSecondIMU() {
    if (numIMUs_ != 2) {
        spdlog::error("The second IMU is not configured");
        return defaultVector3d_;
    }
    return IMUData2_.rawAcc;
}

Eigen::VectorXd& CANIMU::getLinAccFromTheSecondIMU() {
    if (numIMUs_ != 2) {
        spdlog::error("The second IMU is not configured");
        return defaultVector3d_;
    }
    return IMUData2_.linAcc;
}

Eigen::VectorXd& CANIMU::getQuatFromTheSecondIMU() {
    if (numIMUs_ != 2) {
        spdlog::error("The second IMU is not configured");
        return defaultVector4d_;
    }
    return IMUData2_.quat;
}
