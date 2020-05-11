#!/bin/bash

#initialisation of Virtual CAN interface and CANopen nodes for 
#testing of socket connections.

#setting up CAN1 interface on BBB.
sudo config-pin p9.24 can
sudo config-pin p9.26 can
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can1 up

#setting up GPIO pin input for buttons (PULL UP MODE) 
sudo config-pin -a p8.7 in+
sudo config-pin -a p8.8 in+
sudo config-pin -a p8.9 in+
sudo config-pin -a p8.10 in+

#Setting up slave nodes
cd /home/debian/Sit_stand/canopend
echo - > od4_storage
echo - > od4_storage_auto