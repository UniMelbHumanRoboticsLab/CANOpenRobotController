echo "enabling VCAN device on vcan0"
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
echo "starting candump of vcan0"
candump vcan0