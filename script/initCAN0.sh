#!/bin/bash

#initialisation of CAN interface on BBAI
#assumes proper configuration on CAN0 (see https://stackoverflow.com/questions/62207737/beaglebone-ai-how-to-setup-can-bus)

echo "Enabling CAN0"
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 up
#sudo ifconfig can0 txqueuelen 1000

#echo "can0 up. Dumping (ctrl+c to close):"
#candump -c -t z can0,080~111111 #Filter out 080 sync messages