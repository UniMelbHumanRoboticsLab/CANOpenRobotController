#!/bin/bash

#initialisation of CAN interface for peakcan
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000 #required for peakcan

echo "can0 up. Dumping (ctrl+c to close):"

candump -c -t z can0,080~111111 #Filter out 080 sync messages