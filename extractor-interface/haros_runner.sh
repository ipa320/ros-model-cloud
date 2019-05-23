#/bin/bash

current_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

source /root/haros/devel/setup.bash
rosdep install -y -i -r --from-path /root/haros/src

if [[ ${4} = 'launch' ]];
then
   cd /root/haros/src/
   ${current_dir}/setup_ws_depends.sh $2
   launch_path=$( find /root/haros/src -name $3 )

   # Get the output of roslaunch-dump
   dump=$( python ${current_dir}/roslaunch-dump ${launch_path} )
   
   # Replace the launch file by the output of roslaunch-dump
   echo ${dump} > ${launch_path}
else
   cd /root/haros
   catkin_make
fi

python ${current_dir}/ros_model_extractor.py --package $2 --name $3 --${4} --model-path $5
