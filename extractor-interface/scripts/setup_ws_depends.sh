#!/bin/bash
dependencies=$(rospack depends-indent $1)

declare -A depends_uniq
for d in $dependencies
do 
    depends_uniq[$d]=1
done

for d in ${!depends_uniq[@]}
do
    case $d in catkin|genmsg|cpp_common|std_msgs|genpy|rostime|message_runtime|rosconsole|rosbuild|roscpp_traits|xmlrpcpp|roscpp*|rosgraph_msgs|rosmaster|roslib|message_generation|gencpp|genlisp|rospack|rosunit|rosparam|realtime_tools|gennodejs|rosgraph|rospy|rossservice|ros_environment|topic_tools|pluginlib|rosbag*|geneus|std_srvs|actionlib|*_msgs|*_srvs|rostopic|roslaunch|rosclean|rosout|rviz|cmake*|rqt*|tf|tf2) 
        true
    ;;
  *)
        info=$(roslocate info $d)
        tmp=${info#*uri: }
        repo=${tmp%version*}
        version=${info#*version: }
        if [ -z "$version" ]
        then
            git clone $repo
        else
            git clone $repo --branch $version
        fi
    ;;
    esac
done


