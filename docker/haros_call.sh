#/bin/bash
source /root/catkin_ws/devel/setup.bash
cd /root/catkin_ws/src/
git clone https://github.com/ipa320/cob_driver

rosdep install -y -i -r --from-path /root/catkin_ws/src
cd /root/catkin_ws
catkin_make

# A BETTER OPTION IS GIVE THE INDEX FILE AS INPUT
#echo "" > /root/.haros/index.yaml
#echo -en $2 | while IFS= read -r line ; do echo $line >> /root/.haros/index.yaml ; done 

haros init
python /ros_model_extractor.py --package cob_sick_s300 --name cob_sick_s300 --node --model-path .

cat cob_sick_s300.ros
