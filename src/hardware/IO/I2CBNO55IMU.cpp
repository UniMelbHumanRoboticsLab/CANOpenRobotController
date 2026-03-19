#include "I2CBNO55IMU.h"


I2CBNO55IMU::I2CBNO55IMU(unsigned char _DeviceAddress, int _BusId) {
    setDeviceAddress(_DeviceAddress);
    setBusId(_BusId);
    #ifndef NOROBOT
    initDevice();
    #endif
    spdlog::debug("Created I2C BNO055 IMU on {}.", _DeviceAddress);
    #ifndef NOROBOT
    setMode();//Set in default 9DoF fusion
    setSIUnits();
    #endif
};


void I2CBNO55IMU::readQuat() {
    //Read all 4 values as consecutive registers (one block) for faster access
    int16_t val[4];
    readBlockValues16RdwrS(BNO055_QUATERNION_DATA_W_LSB_ADDR, 4, val);

    //Scale everything:
    const double scale = (1.0 / (1 << 14));
    data.quat[0]=scale * val[0];
    data.quat[1]=scale * val[1];
    data.quat[2]=scale * val[2];
    data.quat[3]=scale * val[3];
}

void I2CBNO55IMU::readLinAcc() {
    //Read all 3 values as consecutive registers (one block) for faster access
    int16_t val[3];
    readBlockValues16RdwrS(BNO055_ACCEL_DATA_X_LSB_ADDR, 3, val);

    /* 1m/s^2 = 100 LSB */
    data.linAcc[0] = ((double)val[0]) / 100.0;
    data.linAcc[1] = ((double)val[1]) / 100.0;
    data.linAcc[2] = ((double)val[2]) / 100.0;
}

void I2CBNO55IMU::readGrav() {
    int16_t val[3];
    readBlockValues16RdwrS(BNO055_GRAVITY_DATA_X_LSB_ADDR, 3, val);
    /* 1m/s^2 = 100 LSB */
    data.grav[0] = ((double)val[0]) / 100.0;
    data.grav[1] = ((double)val[1]) / 100.0;
    data.grav[2] = ((double)val[2]) / 100.0;
}

void I2CBNO55IMU::updateInput() {
    //readQuat();
    readGrav();
    //readLinAcc();
    std::cout << "Q = " << getQuat().transpose() << "\t G = " << getGravityVec().transpose() << "A = " << getLinAcc().transpose() << "\n";
}


