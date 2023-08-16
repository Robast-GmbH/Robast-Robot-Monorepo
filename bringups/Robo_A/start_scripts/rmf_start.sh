#!/bin/sh

. /robast/humble/setup.sh
ros2 run free_fleet_client_direct client_direct &
ros2 launch api_connector web_api.launch.py