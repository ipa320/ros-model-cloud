#!/bin/bash

# Arguments:
#   1: Package name
#   2: Node name or launch file name
#   3: Type of the request: either 'launch' or 'node'
#   4: Path to the folder where the resulting model files should be stored
#   5: Path to the ROS workspace 
#   (optional) 6: Http address link of the Git repository 
# Returns:
#   (None)

# scripts_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

source /root/ws/install/setup.bash

if [ "$#" -eq 6 ]; then
   cd "${5}"/src
   git clone "${6}"
fi

cd "${5}"
rosdep install -y -i -r --from-path src
colcon build

source /root/ws/install/setup.bash


haros init
# python "${scripts_dir}"/ros_model_extractor.py --package "$1" --name "$2" --"${3}" --model-path "${4}"
python3 /ros_model_extractor_ros2.py --package "$1" --name "$2" --"${3}" --model-path "${4}" --ws "${5}"

cat "${4}"/"$2".ros
