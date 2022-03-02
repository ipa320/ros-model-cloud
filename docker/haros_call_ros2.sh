#/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash

cd /root/ws/src/
git clone $1

sudo apt-get update && rosdep update && rosdep install -y -i -r --from-path /root/ws/src
source /root/ws/install/setup.bash

cd /root/ws
colcon build

# A BETTER OPTION IS GIVE THE INDEX FILE AS INPUT
#echo "" > /root/.haros/index.yaml
#echo -en $2 | while IFS= read -r line ; do echo $line >> /root/.haros/index.yaml ; done 

haros full --server-host 0.0.0.0:4000
