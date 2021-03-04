#!/bin/bash

#initialisation of CAN interfaces on BB

echo "Enabling CAN0"
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 up

echo "Enabling CAN1"
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can1 up

#echo "can0 up. Dumping (ctrl+c to close):"
#candump -c -t z can0,080~111111 #Filter out 080 sync messages
