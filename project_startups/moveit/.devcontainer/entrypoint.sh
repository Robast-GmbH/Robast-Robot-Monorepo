cd ~/../../workspace
sudo apt-get update && sudo apt-get upgrade -y
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install PlotJuggler as debugging tool to view controller_state data and so forth
sudo apt-get update && sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros -y