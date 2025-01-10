#!/bin/bash

. /robast/humble/setup.sh
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0 &
ros2 launch drawer_bridge drawer_bridge_launch.py &
ros2 launch nfc_bridge nfc_bridge_launch.py &
ros2 run qos_bridge error_qos_bridge &
ros2 run zed_bridge person_distance_pub &
ros2 run audio_bridge sound_playback &

# 1376257 1310721 1114113 1114114 1179649 1245185 1441793 1507329
module_ids=(1376257 1310721 1114113 1114114 1179649 1245185 1441793 1507329)
for module_id in "${module_ids[@]}"; do
    ros2 topic pub /led_cmd communication_interfaces/msg/LedCmd '{drawer_address: {module_id: '${module_id}', drawer_id: 0}, leds: [{red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}, {red: '${RED}', green: '${GREEN}', blue: '${BLUE}', brightness: '${BRIGHTNESS}'}], start_index: 0, fade_time_in_ms: 1000}' --once
done

ros2 service call /robot/safety_module/enable_buzzer std_srvs/srv/SetBool data:\ false

while sleep 1000; do :; done
