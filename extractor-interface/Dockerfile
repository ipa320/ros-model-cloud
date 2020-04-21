# Use an official Python runtime as a parent image
FROM ros:melodic-ros-base

RUN apt-get update && apt-get install -y \
    cppcheck \
    cccc \
    clang-4.0 \
    libclang-4.0-dev \
    python-pip \
    llvm-4.0-dev \
    wget \
    netbase

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/lib/llvm-4.0/lib

RUN pip install --upgrade pip
RUN apt-get update && apt-get install -y ros-melodic-desktop

RUN curl -sL https://deb.nodesource.com/setup_10.x | bash - \
&& apt-get install -y nodejs

SHELL ["bash", "-c"]
RUN source /opt/ros/melodic/setup.bash

RUN mkdir -p ~/extractor-interface

# Set the working directory
WORKDIR /root/extractor-interface

# Python requirements
COPY requirements.txt .
RUN pip install -Ir requirements.txt

# Node requirements
COPY package*.json ./
RUN npm install -g npm

ENV CMAKE_CXX_COMPILER /usr/lib/llvm-4.0/bin/clang++

COPY . ./

# Build the front-end
RUN npm run build

EXPOSE 4000
CMD ["gunicorn", "--bind", "0.0.0.0:4000", "-k", "flask_sockets.worker", "wsgi:app"]
