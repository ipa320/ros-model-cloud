# Use the docker container to run the ros-model plugin for HAROS

Install docker https://docs.docker.com/install/linux/docker-ce/ubuntu/

Build the HAROS docker image, for your desired ROS distro version:
```
cd path-to-ros-model-cloud-repo/docker
[sudo] docker build --tag=haros_foxy -f foxy/Dockerfile .
```

Call the ros-model extractor plugin, remember you have to also clone the repository to be analysed:

```
[sudo] docker run -it haros_foxy:latest /haros_runner_ros2.sh *package_name* *node_name* *type* *path_to_resulted_model* *workspace_path* *github_repository* *branch*
```

For example:

```
[sudo] docker run -it haros_foxy:latest /haros_runner.sh sick_scan2 sick_generic_caller node . /root/ws https://github.com/SICKAG/sick_scan2


[sudo] docker run -it haros_foxy:latest /haros_runner.sh ros2_ouster_drivers ouster_driver node . /root/ws  https://github.com/ros-drivers/ros2_ouster_drivers foxy-devel
```

