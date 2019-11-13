#!/bin/bash

workspace_path=$1
model_path=$2
package_list=$(sed 's/[^ ]* *//' <<< $@)
package_list=$(sed 's/[^ ]* *//' <<< $package_list)
distro=$(echo $ROS_DISTRO)

for pkg in $package_list
do
    pkg=$(sed 's/_/-/g' <<< $pkg)    
    sudo apt install ros-$distro-$pkg
done

scripts_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $workspace_path || exit # path to the catkin workspace
source devel/setup.bash
rosdep install -y -i -r --from-path src
catkin_make
source devel/setup.bash
bash $scripts_dir/generate_messages_model_helper.sh $package_list > $model_path
