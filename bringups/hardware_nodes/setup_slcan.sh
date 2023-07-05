sudo modprobe can
sudo modprobe can-raw
sudo modprobe slcan

sudo slcan_attach -f -s5 -o /dev/robast/robast_can
sudo slcand robast/robast_can can0
sudo ifconfig can0 up

DOCKERPID=$(docker inspect -f '{{ .State.Pid }}' hardware_nodes_drawer)

sudo ip link add vxcan0 type vxcan peer name vxcan1 netns $DOCKERPID

sudo modprobe can-gw

sudo cangw -A -s can0 -d vxcan0 -e

sudo cangw -A -s vxcan0 -d can0 -e

sudo ip link set vxcan0 up

sudo ip link set can0 up

sudo nsenter -t $DOCKERPID -n ip link set vxcan1 up
