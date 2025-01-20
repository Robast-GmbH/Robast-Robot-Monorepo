#!/bin/bash

node_list="/robot/zed/zed_node"


# If no nodes are passed return 0
if [[ -z ${node_list} ]]
then
    echo "No nodes to check"
    exit 0
fi

# List to save pids of async processes
declare -a pid_exec

main() {
    echo "ROS2 healthcheck"
    echo "Node list:"
    source install/setup.bash
    
    local good_nodes=0
    local nodes_running=$(schedtool -n +19 -p 1 -e ros2 node list --no-daemon --spin-time 1 -a)
    for node_name in ${node_list}; do
        eval "node_name=${node_name}"
        echo "${nodes_running}" | grep -q "${node_name}"
        if [ $? -eq 0 ]; then
            echo "  OK   - ${node_name}"
        else
            echo "  FAIL - ${node_name}"
            good_nodes=1
        fi
    done
    if [ ${good_nodes} -eq 0 ]; then
        echo "All nodes are running"
        exit 0
    else
        echo "At least one node is not running"
        exit 1
    fi
}

source /opt/ros/humble/setup.bash
main