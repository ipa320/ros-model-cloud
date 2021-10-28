#!/bin/bash

git clone https://github.com/ipa320/ros-model-extractors

docker build --tag=haros_melodic -f ros-model-extractors/melodic/Dockerfile --build-arg enable_ssh=true --build-arg path_to_scripts="ros-model-extractors/" . 
docker build --tag=haros_noetic -f ros-model-extractors/noetic/Dockerfile --build-arg enable_ssh=true --build-arg path_to_scripts="ros-model-extractors/" .
docker build --tag=haros_foxy -f ros-model-extractors/foxy/Dockerfile --build-arg enable_ssh=true --build-arg path_to_scripts="ros-model-extractors/" .

docker build --tag=extractor_frontend -f extractor-interface/Dockerfile .
