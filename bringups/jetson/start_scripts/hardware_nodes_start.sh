#!/bin/sh

. /robast/humble/setup.sh
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0 &
ros2 launch drawer_bridge drawer_bridge_launch.py &

for module_id in $(seq 1 8); do
    ros2 topic pub /led_cmd communication_interfaces/msg/LedCmd '{drawer_address: {module_id: '${module_id}', drawer_id: 0}, leds: [{red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}, {red: 0, green: 255, blue: 255, brightness: 20}], start_index: 0, fade_time_in_ms: 1000}' --once
done