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

if [ "$#" -eq 6 ]; then
   cd "${5}"/src
   git clone "${6}"
   cd "${5}"
fi

cd "${5}"

if [ -n $ROS_VERSION ]
then
  if [ $ROS_VERSION == "1" ]
  then
    source devel/setup.bash
    rosdep install -y -i -r --from-path src
    catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1
  elif [ $ROS_VERSION == "2" ]
  then
    source install/setup.bash
    rosdep install -y -i -r --from-path src
    colcon build
  else
    echo "ROS version not supported"
    exit
  fi
else
  echo "ROS installation not found"
fi

haros init

if [ -n $PYTHON_VERSION ]
then
  if [ $PYTHON_VERSION == "2" ]
  then
    python /ros_model_extractor.py --package "$1" --name "$2" --"${3}" --model-path "${4}" --ws "${5}"
  elif [ $PYTHON_VERSION == "3" ]
  then
    python3 /ros_model_extractor.py --package "$1" --name "$2" --"${3}" --model-path "${4}" --ws "${5}"
  else
    echo "Python version not supported"
    exit
  fi
else
  echo "Python setup not found"
fi

echo "###########"
echo "~~~~~~~~~~~"
echo "Print of the model: $2.ros:"
echo "~~~~~~~~~~~"
cat "${4}"/"$2".ros
echo ""
echo "~~~~~~~~~~~"
echo "###########"
