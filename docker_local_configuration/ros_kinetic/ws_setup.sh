#/bin/bash
source /root/haros/devel/setup.bash
cd /root/haros/src/
git clone $1

rosdep install -y -i -r --from-path /root/haros/src
cd /root/haros
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1

touch /root/$3.ros
/root/haros_runner.sh $2 $3 node /root/ /root/haros
python /web_test.py $3 /root/$3.ros 

