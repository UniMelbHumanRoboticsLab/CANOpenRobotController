# Note on USB-CAN adapters

CORC can be used running on a Linux computer/laptop/SBC and using a USB-CAN adapter. Currently, CORC has been tested with two different USB-CAN adapters:

- the [PCAN-USB](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1)
- the [inno-maker USB CAN Module](https://www.inno-maker.com/product/usb-can/)

## Recommendations

We strongly recommend to use the inno-maker one: cheaper and has better performance natively. A simple case to be 3D printed is available to download [here](https://github.com/UniMelbHumanRoboticsLab/CORC-UI-Demo/blob/14f7208c58703624cce523ccba70a3ac1c9fef7e/innoMakerUSBCANEnclosure.zip).

In case of the use of the PCAN device, you must check if a communication delay exists. In this case, use the following commands to reduce this delay (below the ms):

- Edit `/etc/modprobe.d/pcan.conf` and add the line: `options pcan fast_fwd=1`.
- Then reload the driver:
```
$sudo rmmod pcan
$sudo modprobe pcan
```
