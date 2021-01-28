#Initializes can0 and can1. Set the queue length top 1000

sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 up

sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can1 up

sudo ifconfig can0 txqueuelen 1000
sudo ifconfig can1 txqueuelen 1000