#!/bin/bash

echo "Configure CAN1 pins"
sudo config-pin p9.24 can
sudo config-pin p9.26 can
echo "Enabling CAN1"
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can1 up
#sudo ifconfig can1 txqueuelen 1000

#echo "can0 up. Dumping (ctrl+c to close):"
#candump -c -t z can1,080~111111 #Filter out 080 sync messages