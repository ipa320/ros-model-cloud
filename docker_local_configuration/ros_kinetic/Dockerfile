# Use an official Python runtime as a parent image
FROM ros:kinetic

RUN apt-get update && apt-get install -y \
    cppcheck \
    cccc \
    clang-3.8 \
    libclang-3.8-dev \
    python-pip \
    llvm-3.8-dev \
    wget \
    netbase \
    libmagic-dev

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/lib/llvm-3.8/lib

RUN pip install --upgrade pip
RUN pip install -Iv clang==3.8
RUN pip install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor
RUN pip install haros
RUN pip install bonsai-code

RUN apt-get update && apt-get install -y ros-kinetic-desktop

SHELL ["bash", "-c"]

RUN mkdir -p ~/haros/src
RUN source /opt/ros/kinetic/setup.bash

# Set the working directory
WORKDIR ~/haros

# Copy the current directory contents into the container
COPY . ~/haros

ENV CMAKE_CXX_COMPILER /usr/lib/llvm-3.8/bin/clang++

RUN source /opt/ros/kinetic/setup.bash;\
 cd ~/haros/src;\
 catkin_init_workspace;\
 cd ~/haros;\
 catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1;\
 source ~/haros/devel/setup.bash; \
 haros init

RUN wget -O ~/haros_runner.sh https://raw.githubusercontent.com/ipa320/ros-model-cloud/kinetic/extractor-interface/scripts/haros_runner.sh 
RUN wget -O ~/ros_model_extractor.py  https://raw.githubusercontent.com/ipa320/ros-model-cloud/kinetic/extractor-interface/scripts/ros_model_extractor.py
RUN chmod +x ~/haros_runner.sh
RUN chmod +x ~/ros_model_extractor.py

COPY ws_setup.sh /root/ws_setup.sh
#COPY scripts/ros_model_extractor.py /
#COPY scripts/haros_runner.sh /

# Just for testing, can be removed
COPY web_test.py /
RUN pip install Flask\
    Redis


CMD []

