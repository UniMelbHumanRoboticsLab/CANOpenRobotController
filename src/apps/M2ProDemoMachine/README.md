# set the driver address for the M2 pro
add/replace the following codes into CANOpenRobotController/src/hardware/drives/KincoDrive.cpp
```bash
OD_Addresses[DIGITAL_IN] = {0x2010, 0x0A};
OD_Addresses[DIGITAL_OUT] = {0x2010, 0x0E};
OD_DataSize[DIGITAL_IN] = 2;
OD_DataSize[DIGITAL_OUT] = 2;
```

# set up the network
```bash
ifconfig
UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.1.11")
```

## build the demo app
if no build folder
```bash
mkdir build && cd build && cmake ..
make -j8
```

if existing a build folder
```bash
rm -r build && mkdir build && cd build && cmake ../CMakeLists.txt
make -j8
```

## run the demo app
Initialize the CAN device (No need to repeat this step, unless you restart your PC):

```bash
sudo ../script/initCAN0.sh # or initCAN1.sh
```

Run the compiled executable file:
```bash
sudo ./M2ProDemoMachine_APP
```

## check the configuration of the device CAN port
```bash
ifconfig -a
```