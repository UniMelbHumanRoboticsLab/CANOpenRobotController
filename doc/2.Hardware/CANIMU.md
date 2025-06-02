# CANIMU Module Documentation

## 1. Overview

The `CANIMU` module for CORC (CANOpen Robot Controller) allows for the integration of custom Inertial Measurement Unit (IMU) setups into your robotic system. It is designed for scenarios where an IMU is connected to a microcontroller (MCU) (e.g., Teensy, Arduino, ESP32, Xiao) which, in turn, communicates the IMU data over a CAN bus.

This module listens for specific CAN messages from the MCU, interprets them, and makes the IMU data (acceleration, linear acceleration, orientation) available to the CORC application/state machine as if it were from standard RPDOs.

> Note: The `CANIMU` module facilitates communication with MCUs sending IMU data via CAN messages using *non-standard* COB-IDs. It is not a fully compliant CANOpen slave device itself but rather a bridge to make custom IMU data easily accessible within CORC. User can choose to use CAN messages IDs that does not clash with pre-occupied CANOpen COB-IDs.

## 2. Principle of Operation

The workflow is as follows:

1.  **MCU and IMU**: Your IMU (or IMUs, up to two per `CANIMU` instance) is connected to an MCU that has CAN bus capabilities.
2.  **MCU Firmware**: The firmware on the MCU is responsible for:
    *   Reading data from the IMU(s).
    *   Quantizing the sensor readings into signed 16-bit integers (ranging from -32767 to 32767). The scaling factor for this quantization must be known (see `IMUParameters`).
    *   Transmitting this data as a series of CAN messages. Each message typically has a specific COB-ID.
3.  **`CANIMU` Module (CORC side)**:
    *   The `CANIMU` class is configured with the COB-IDs and sensor scaling parameters (`IMUParameters`) that match those used by the MCU.
    *   It registers to receive CAN messages with these COB-IDs.
    *   Upon receiving messages, it extracts the 16-bit integer data.
    *   It then converts these integers back to physical units (e.g., m/s^2, unitless for quaternions) using the `range_mapping` function.
    *   The processed data is stored internally and can be accessed via getter methods.

### CAN Message Format

The MCU must send CAN messages in the following format. All data values are signed 16-bit integers (2 bytes).

*   **If one IMU is connected to the MCU:**
    *   **CAN Message 1 (8 bytes):**
        *   Bytes 0-1: Acceleration-X
        *   Bytes 2-3: Acceleration-Y
        *   Bytes 4-5: Acceleration-Z
        *   Bytes 6-7: Anything else (ignored by `CANIMU`)
    *   **CAN Message 2 (8 bytes):**
        *   Bytes 0-1: Linear Acceleration-X (gravity removed)
        *   Bytes 2-3: Linear Acceleration-Y
        *   Bytes 4-5: Linear Acceleration-Z
        *   Bytes 6-7: Anything else (ignored)
    *   **CAN Message 3 (8 bytes):**
        *   Bytes 0-1: Quaternion-W
        *   Bytes 2-3: Quaternion-X
        *   Bytes 4-5: Quaternion-Y
        *   Bytes 6-7: Quaternion-Z
*   **If two IMUs are connected to the MCU:**
    *   **IMU 1:** Uses three CAN messages as described above (e.g., Message 1, 2, 3).
    *   **IMU 2:** Uses another three CAN messages with *different COB-IDs* (e.g., Message 4, 5, 6), following the same data layout for its respective acceleration, linear acceleration, and quaternion data.

## 3. MCU Firmware Requirements

*   The MCU must have a CAN controller and be programmed to send CAN messages according to the format specified above.
*   **Data Quantization**: Sensor readings must be mapped to the `int16_t` range \[-32767, 32767\]. For example, if an accelerometer axis has a range of ±100 m/s^2, then +100 m/s^2 should be mapped to 32767 and -100 m/s^2 to -32767.
*   **COB-IDs**: You will define a unique COB-ID for each of the 3 (or 6) CAN messages. These COB-IDs must match the configuration provided to the `CANIMU` class in your CORC application.
*   **MSB/LSB first?**: The data is expected in the format of "most significant byte first". For example, `(value_high_byte * 256 + value_low_byte)` or `(byte0 * 256 + byte1)` for acceleration-x. Nevertheless, you can customised this by making corresponding changes in the MCU firmware. 

### Example MCU Firmware:

*   Teensy with 1 IMU (BNO055): [Teensy_1x_BNO055_CORC_Example.ino](https://github.com/MingruiSun2019/open_source_knee_orthosis/blob/master/firmware/Teensy_1x_BNO055_CORC_Example/Teensy_1x_BNO055_CORC_Example.ino)
*   Teensy with 2 IMUs (BNO055): [Teensy_2x_BNO055_CORC_Example.ino](https://github.com/MingruiSun2019/open_source_knee_orthosis/blob/master/firmware/Teensy_2x_BNO055_CORC_Example/Teensy_2x_BNO055_CORC_Example.ino)

**Crucial**: Ensure the `IMUParameters` (COB-IDs, sensor ranges) defined in your MCU firmware are identical to those used when creating the `CANIMU` object in your C++ CORC application.

## 4. Using `CANIMU` in your CORC Application

Here's how to integrate and use the `CANIMU` module within your CORC `Robot` derived class or state machine.

### Step 1: Define `IMUParameters`

The `IMUParameters` struct holds the configuration for each IMU processed by the `CANIMU` module.

```cpp
#include "CANIMU.h" // Make sure this is included

// ...

IMUParameters myImuSetupParams;

// Assign the COB-IDs for the 3 CAN messages from your MCU
myImuSetupParams.COBID1 = 0x282; // Example: For Acceleration data
myImuSetupParams.COBID2 = 0x283; // Example: For Linear Acceleration data
myImuSetupParams.COBID3 = 0x284; // Example: For Quaternion data

// Define the sensor measurement ranges.
// These values represent the physical value corresponding to 32767 in the int16_t message.
myImuSetupParams.acclRange = 100;      // e.g., 100 m/s^2 (approx 10g) for acceleration values
// myImuSetupParams.gyroRange = 2000;  // e.g., 2000 deg/s (used if gyro data was part of the messages)
// myImuSetupParams.orenRollRange = 180; // ... and so on for other specific orientation types if applicable
// myImuSetupParams.orenPitchRange = 90;
// myImuSetupParams.orenYawRange = 360;
myImuSetupParams.orenQuantRange = 1;   // For quaternion components (W, X, Y, Z), which are typically in the range [-1, 1]
```

### Step 2 & 3: Define and Integrate CANIMU

#### In your Robot header file (.h):
```cpp
class MyRobot : public Robot {
private:
    // For single IMU setup
    CANIMU* imu1;
    
    // Or for dual IMU setup
    CANIMU* imu1;  // Will handle both IMUs in one object
};
```

#### In your Robot implementation file (.cpp):
```cpp
MyRobot::MyRobot(...) {
    // For single IMU setup
    IMUParameters imu1Params;
    imu1Params.COBID1 = ...;  // For Acceleration data
    imu1Params.COBID2 = ...;  // For Linear Acceleration data
    imu1Params.COBID3 = ...;  // For Quaternion data
    imu1Params.acclRange = ...;     // e.g., 100 m/s^2
    imu1Params.orenQuantRange = ...;  // For quaternion components [-1, 1]
    
    inputs.push_back(imu1 = new CANIMU(imu1Params));
    
    // Or for dual IMU setup
    IMUParameters imu1Params;  // Configure for first IMU as above
    IMUParameters imu2Params;  // Configure for second IMU with different COB-IDs
    imu2Params.COBID1 = ...;  // Different COB-IDs for second IMU
    imu2Params.COBID2 = ...;
    imu2Params.COBID3 = ...;
    imu2Params.acclRange = ...;
    imu2Params.orenQuantRange = ...;
    
    inputs.push_back(imu1 = new CANIMU(imu1Params, imu2Params));
    
    return true;
}
```

When your main application calls `robot->initialise()`, the `Robot` base class will iterate through all `InputDevice` objects in the `inputs` vector (including your `CANIMU` instance) and call their `configureMasterPDOs()` method. This sets up the necessary RPDOs within `CANIMU` to listen for the specified CAN messages.

### Step 4: Updating and Accessing IMU Data

1.  **Data Update**: The `Robot::updateRobot()` method, which should be called cyclically in your main control loop, will automatically call the `updateInput()` method for all registered input devices, including your `CANIMU` instance. This reads the latest data from the internal CAN buffers and performs the range mapping.

2.  **Data Access**: You can retrieve the processed IMU data using the getter methods of the `CANIMU` object.

```cpp
// Assuming 'myIMU' is a pointer to your CANIMU object

// For the first IMU
Eigen::VectorXd& rawAcceleration1 = myIMU->getRawAccFromTheFirstIMU();       // Returns 3D vector (x,y,z)
Eigen::VectorXd& linearAcceleration1 = myIMU->getLinAccFromTheFirstIMU();  // Returns 3D vector (x,y,z)
Eigen::VectorXd& quaternion1 = myIMU->getQuatFromTheFirstIMU();          // Returns 4D vector (w,x,y,z)

// If two IMUs were configured:
// Eigen::VectorXd& rawAcceleration2 = myIMU->getRawAccFromTheSecondIMU();
// Eigen::VectorXd& linearAcceleration2 = myIMU->getLinAccFromTheSecondIMU();
// Eigen::VectorXd& quaternion2 = myIMU->getQuatFromTheSecondIMU();

// Example usage (e.g., in a state machine's 'during' method or your Robot's update logic):
// spdlog::info("IMU1 Accel: ({}, {}, {})", rawAcceleration1(0), rawAcceleration1(1), rawAcceleration1(2));
// spdlog::info("IMU1 Quat: ({}, {}, {}, {})", quaternion1(0), quaternion1(1), quaternion1(2), quaternion1(3));
```

## 5. Example Usage in a Demo State Machine

To use the IMU data within a state of your `DemoMachine`, you would typically access it through your `Robot` object.

```cpp
// In your M1DemoStates.cpp or similar 'during()' method:
void MyDemoState::during(void) {
    // Assuming 'myRobot' is a pointer to your Robot-derived class,
    // and it has public pointers to the CANIMU objects myIMU.

    Eigen::VectorXd& accl1 = myRobot->myIMU->getRawAccFromTheFirstIMU();
    Eigen::VectorXd& linAcc1 = myRobot->myIMU->getLinAccFromTheFirstIMU();
    Eigen::VectorXd& quat1 = myRobot->myIMU->getQuatFromTheFirstIMU();

    // If you have a 2nd IMU
    Eigen::VectorXd& accl1 = myRobot->myIMU->getRawAccFromTheSecondIMU();
    Eigen::VectorXd& linAcc1 = myRobot->myIMU->getLinAccFromTheSecondIMU();
    Eigen::VectorXd& quat1 = myRobot->myIMU->getQuatFromTheSecondIMU();

    // Happy days :)
}
```
