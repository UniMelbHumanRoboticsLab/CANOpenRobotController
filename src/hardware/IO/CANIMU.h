/*/**
 * \file CANIMU.h
 * \author Mingrui Sun
 * \brief  Class representing a MCU with a CAN bus module) which converts its connected IMU reading to CANOpen messages. 
 *         The MCU can be Teensy, Arduino, Xiao, etc. but need to have a CAN bus module so that they can send CAN messages to the CAN bus.
 * 
 *    NOTE: this is not a CANOpen Device, and the PDO-like messages are send on non-standard COB-IDs
 *    
 *    This class support either one or two IMUs connected to the MCU, which can be initialised in one of the two ways:
 *    - one IMU: CANIMU(IMUParameters IMUParameters1);
 *    - two IMUs: CANIMU(IMUParameters IMUParameters1, IMUParameters IMUParameters2);

 *    The MCU should quantise the sensor readings as signed 16-bit integers (2 bytes) and send CAN message in the following format:
 *    - If one IMU is connected:
 *      - CAN message 1 (8 bytes): Acceleration-X (2 bytes), Acceleration-Y (2 bytes), Acceleration-Z (2 bytes), anything else (2 bytes, which will be ignored)
 *      - CAN message 2 (8 bytes): LinearAcceleration-X (2 bytes), LinearAcceleration-Y (2 bytes), LinearAcceleration-Z (2 bytes), anything else (2 bytes, which will be ignored)
 *      - CAN message 3 (8 bytes): Quaternion-W (2 bytes), Quaternion-X (2 bytes), Quaternion-Y (2 bytes), Quaternion-Z (2 bytes)
 *    - If two IMUs are connected:
 *      [of the 1st IMU]
 *      - CAN message 1 (8 bytes): Acceleration-X (2 bytes), Acceleration-Y (2 bytes), Acceleration-Z (2 bytes), anything else (2 bytes, which will be ignored)
 *      - CAN message 2 (8 bytes): LinearAcceleration-X (2 bytes), LinearAcceleration-Y (2 bytes), LinearAcceleration-Z (2 bytes), anything else (2 bytes, which will be ignored)
 *      - CAN message 3 (8 bytes): Quaternion-W (2 bytes), Quaternion-X (2 bytes), Quaternion-Y (2 bytes), Quaternion-Z (2 bytes)
 *      [of the 2nd IMU]
 *      - CAN message 4 (8 bytes): Acceleration-X (2 bytes), Acceleration-Y (2 bytes), Acceleration-Z (2 bytes), anything else (2 bytes, which will be ignored)
 *      - CAN message 5 (8 bytes): LinearAcceleration-X (2 bytes), LinearAcceleration-Y (2 bytes), LinearAcceleration-Z (2 bytes), anything else (2 bytes, which will be ignored)
 *      - CAN message 6 (8 bytes): Quaternion-W (2 bytes), Quaternion-X (2 bytes), Quaternion-Y (2 bytes), Quaternion-Z (2 bytes)
 *
 *    The CANIMU class will receive the CAN messages and convert them to RPDOs, which are then used to update the input for the robot.
 * 
 *    Example MCU firmware code:
 *    - Teensy with 1 IMU (BNO055): https://github.com/MingruiSun2019/open_source_knee_orthosis/blob/master/firmware/Teensy_1x_BNO055_CORC_Example/Teensy_1x_BNO055_CORC_Example.ino
 *    - Teensy with 2 IMUs (BNO055): https://github.com/MingruiSun2019/open_source_knee_orthosis/blob/master/firmware/Teensy_2x_BNO055_CORC_Example/Teensy_2x_BNO055_CORC_Example.ino
 * 
 *    Note: Make sure the IMUParameters in the MCU firmware and here are the same.
 *
 * \version 0.1
 * \date 2025-05-30
 * \copyright Copyright (c) 2025
 */

#ifndef CANIMU_H_INCLUDED
#define CANIMU_H_INCLUDED

#include <CANopen.h>
#include <CO_command.h>
#include <string.h>
#include <Eigen/Dense>

#include "InputDevice.h"
#include "logging.h"
#include "RPDO.h"
#include "TPDO.h"


struct IMUParameters {
    // Each IMU has 3 COB-IDs, one for each of the 3 PDOs
    // block 1 for acceleration (with gravity)
    // block 2 for linear acceleration (without gravity)
    // block 3 for quaternion
    int COBID1;
    int COBID2;
    int COBID3;

    // Range of variables from the IMU:
    // On the MCU, the sensor readings are mapped as [-limit, limit] to [-32767, 32767] (int16)
    // Here, the int16 is converted back to [-limit, limit]
    int acclRange;   // acceleration range, e.g., 100 m/s^2 (10g)
    int gyroRange;   // angular velocity range, e.g. 2000 deg/s
    int orenRollRange;   // orientation roll range, e.g. 180 deg
    int orenPitchRange;   // orientation pitch range, e.g. 90 deg
    int orenYawRange;   // orientation yaw range, e.g. 360 deg
    int orenQuantRange;   // orientation quaternion range, e.g. 1 (unitless)
};

struct IMUData {
    Eigen::VectorXd rawAcc = Eigen::VectorXd::Zero(3);  // 3D vector for x,y,z acceleration
    Eigen::VectorXd linAcc = Eigen::VectorXd::Zero(3);  // 3D vector for x,y,z linear acceleration
    Eigen::VectorXd quat = Eigen::VectorXd::Zero(4);    // 4D vector for w,x,y,z quaternion
};

class CANIMU : public InputDevice {
    private:
        // Parameters for the IMUs
        IMUParameters IMUParameters1_;
        IMUParameters IMUParameters2_;

        // Data from the IMUs
        IMUData IMUData1_;
        IMUData IMUData2_;

        // Static members for default return values
        static Eigen::VectorXd defaultVector3d_;
        static Eigen::VectorXd defaultVector4d_;

        // Number of IMUs connected to the MCU
        int numIMUs_;

        // Objects representing the RPDOs (used to create the RPDOs in the OD)
        RPDO *rpdo1;
        RPDO *rpdo2;
        RPDO *rpdo3;
        RPDO *rpdo4;
        RPDO *rpdo5;
        RPDO *rpdo6;

        /// Raw data - these variables are linked to the RPDOs
        UNSIGNED8 rawDataIMU1[24] = {0};  // 24 bytes for 3 RPDOs from IMU 1, 8 bytes each
        UNSIGNED8 rawDataIMU2[24] = {0};  // 24 bytes for 3 RPDOs from IMU 2, 8 bytes each

        // Length of the data for each RPDO
        UNSIGNED8 lengthData = 8;

        // Data size for each RPDO
        UNSIGNED16 dataSize[8] = {1,1,1,1,1,1,1,1};
 
        /**
        * \brief Maps the CANOpen data value to the original value in the original unit.
        *
        * \param msgVal the CANOpen data value
        * \param sensorRange the range of the sensor reading
        * \param msgMax the maximum value of the CANOpen data, e.g. 32767 for int16
        */
        float range_mapping(float msgVal, float sensorRange, float msgMax);

        /**
         * \brief Sets up the receiving PDOs for the one connected IMU
         * 
         */
        bool configureMasterPDOForOneIMU(const IMUParameters& IMUParameters, UNSIGNED8 rawCanBusData[]);

        /**
         * \brief Updates the input for the one connected IMU
         * 
         */
        void updateInputForOneIMU(IMUData& IMUData, const IMUParameters& IMUParameters, UNSIGNED8 rawCanBusData[]);

       public:
        /**
        * \brief Sets up the CANIMU object with the COB-IDs of 3 PDOs for one IMU
        *
        * \param IMUParameters1 the parameters of the first IMU
        */
        CANIMU(IMUParameters IMUParameters1);

        /**
        * \brief Sets up the CANIMU object with the COB-IDs of 6 PDOs for two IMUs
        *
        * \param IMUParameters1 the parameters of the first IMU
        * \param IMUParameters2 the parameters of the second IMU
        */
        CANIMU(IMUParameters IMUParameters1, IMUParameters IMUParameters2);

        /**
         * \brief Sets up the receiving PDOs for all connected IMUs
         * 
         */
        bool configureMasterPDOs();

        /**
         * \brief Updates the input from all connected IMUs
         * 
         */
        void updateInput();

        /**
         * \brief Get the acceleration readings from the first connected IMU of the MCU
         * 
         * \return Eigen::VectorXd x,y,z axis of acceleration
         */
        Eigen::VectorXd& getRawAccFromTheFirstIMU();

        /**
         * \brief Get the linear acceleration readings from the first connected IMU of the MCU
         * 
         * \return Eigen::VectorXd x,y,z axis of linear acceleration
         */
        Eigen::VectorXd& getLinAccFromTheFirstIMU();

        /**
         * \brief Get the quaternion readings from the first connected IMU of the MCU
         * 
         * \return Eigen::VectorXd w,x,y,z quaternion
         */
        Eigen::VectorXd& getQuatFromTheFirstIMU();

        /**
         * \brief Get the acceleration readings from the second connected IMU of the MCU
         * 
         * \return Eigen::VectorXd x,y,z axis of acceleration
         */
        Eigen::VectorXd& getRawAccFromTheSecondIMU();

        /**
         * \brief Get the linear acceleration readings from the second connected IMU of the MCU
         * 
         * \return Eigen::VectorXd x,y,z axis of linear acceleration
         */
        Eigen::VectorXd& getLinAccFromTheSecondIMU();

        /**
         * \brief Get the quaternion readings from the second connected IMU of the MCU
         * 
         * \return Eigen::VectorXd w,x,y,z quaternion
         */
        Eigen::VectorXd& getQuatFromTheSecondIMU();
};
#endif