# Use an official Python runtime as a parent image
FROM ros:kinetic-ros-base

RUN apt-get update && apt-get install -y \
    cppcheck \
    cccc \
    clang-3.8 \
    libclang-3.8-dev \
    python-pip \
    llvm-3.8-dev \
    wget \
    netbase

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/lib/llvm-3.8/lib

RUN pip install --upgrade pip
RUN pip install -Iv clang==3.8
RUN pip install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor
RUN pip install haros==3.1.0
RUN pip install bonsai-code

RUN apt-get update && apt-get install -y ros-kinetic-desktop

RUN curl -sL https://deb.nodesource.com/setup_10.x | bash - \
&& apt-get install -y nodejs

SHELL ["bash", "-c"]
RUN source /opt/ros/kinetic/setup.bash

RUN mkdir -p ~/haros
RUN mkdir -p ~/extractor-interface

# Set the working directory
WORKDIR /root/extractor-interface

# Web app requirements
COPY requirements.txt .
RUN pip install -r requirements.txt

COPY package*.json ./
RUN npm install

ENV CMAKE_CXX_COMPILER /usr/lib/llvm-3.8/bin/clang++

COPY . ./

RUN npm run build

ENV HAROS_RUNNER /root/extractor-interface/scripts/haros_runner.sh
ENV MODEL_PATH /root/
ENV HAROS_SRC /root/haros

EXPOSE 4000
CMD ["gunicorn", "--bind", "0.0.0.0:4000", "-k", "flask_sockets.worker", "wsgi:app"]
