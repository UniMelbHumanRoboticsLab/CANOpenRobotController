# Setting up CAN device on BeagleBone AI

The BeagleBone AI doesn't natively expose its can device. To do so an update of the Device Tree Source (DTS) is required.

## Instructions

### Short version using binary (compiled) DTB file
1. Download the updated [DTB file](am5729-beagleboneai-custom-can.dtb)
2. Copy it into the '/boot/dtbs' folder of your BB AI: `/boot/dtbs/am5729-beagleboneai-custom-can.dtb`
3. On the BB AI edit the uEnv file: `$ sudo editor /boot/uEnv.txt` and add the line: `dtb=am5729-beagleboneai-custom-can.dtb`. Save the file (ctrl+x).
4. Reboot the BB AI (`$ sudo reboot`).
5. Test: `$ ip link`


### Longer version using DTS file
See [StackOverflow post](https://stackoverflow.com/questions/62207737/beaglebone-ai-how-to-setup-can-bus).


### Note 
This will expose the CAN port of pins P9_24 and P9_26 as CAN0 device as opposed to the BeagleBone Black where it corresponds to CAN1 devcie. To use it you should thus either use the iniCANBBAI.sh script or manually configure the can0 port:

```bash
$ sudo ip link set can0 up type can bitrate 1000000
$ sudo ifconfig can0 up
```

## Source
See [StackOverflow post](https://stackoverflow.com/questions/62207737/beaglebone-ai-how-to-setup-can-bus).