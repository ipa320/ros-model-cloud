#/bin/bash
source /root/haros/devel/setup.bash
cd /root/haros/src/
git clone $1

rosdep install -y -i -r --from-path /root/haros/src
cd /root/haros
catkin_make

python /ros_model_extractor.py --package $2 --name $3 --${4} --model-path $5
