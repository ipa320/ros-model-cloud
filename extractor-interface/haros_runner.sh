#!/bin/bash

current_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd ${6}
source devel/setup.bash

rosdep install -y -i -r --from-path ${6}/src

if [[ ${4} = 'launch' ]];
then
   cd ${6}/src
   ${current_dir}/setup_ws_depends.sh $2
   
   cd ..
   ${current_dir}/catkin/bin/catkin_make_isolated --only-pkg-with-deps $2 --continue-on-failure -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-3.8 --merge

   mkdir build && touch build/compile_commands.json
   cat build_isolated/*/compile_commands.json >> build/compile_commands.json
   sed -i -e 's/\]\[/\,/g' build/compile_commands.json
   launch_path="$( find ${6}/src -name $3 )"

   # Get the output of roslaunch-dump
   dump="$( python ${current_dir}/roslaunch-dump ${launch_path} )"

   # Replace the launch file by the output of roslaunch-dump
   echo ${dump} > ${launch_path}

else
   cd ${6}
   catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1
fi

haros init
python ${current_dir}/ros_model_extractor.py --package $2 --name $3 --${4} --model-path $5
