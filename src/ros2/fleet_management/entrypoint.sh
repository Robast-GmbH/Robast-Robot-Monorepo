#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths /workspace/src/open_rmf --ignore-src -r -y
rosdep install --from-paths /workspace/src/communication_interfaces --ignore-src -r -y

# RMF-Web Setup
curl -fsSL https://get.pnpm.io/install.sh > install_pnpm.sh
. install_pnpm.sh
rm install_pnpm.sh
export PNPM_HOME="/home/robast/.local/share/pnpm"
export PATH="$PNPM_HOME:$PATH"
source /home/robast/.bashrc
pnpm env use --global 20
export PATH=$PATH:/home/robast/.local/bin
source ~/.bashrc

cd /workspace && colcon build
source install/setup.bash

cd /rmf-web-workspace/rmf-web && pnpm install

cd packages/api-server/api_server
sed -i "s/host=app_config.host,/host='0.0.0.0',/" __main__.py

cd ..
pnpm start &

ros2 run dispenser_ingestor_mock dispenser_ingestor_mock &

sleep 5
ros2 launch fleet_adapter_rb_theron rmf_launch.py