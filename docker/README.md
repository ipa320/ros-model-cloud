# Use the docker container to run HAROS

Install docker https://docs.docker.com/install/linux/docker-ce/ubuntu/

Build the HAROS docker image, for your desired ROS distro version:
```
cd path-to-haros-repo/docker/ROS_DISTRO
[sudo] docker build --tag=haros .
```

Run the image with the docker extractor script givig as argument the url of the GitHub repository that holds the source code and mounting the `index.yaml` file to its filesystem:

Either
```
[sudo] docker run -it --mount type=bind,source=*Path to index.yaml*,target=/root/.haros/index.yaml -p 4000:4000 haros:latest /haros_call.sh *Repo URL*
```

or

```
[sudo] docker run -it -v *Path to index.yaml*:/root/.haros/index.yaml -p 4000:4000 haros:latest /haros_call.sh *Repo URL*
```

can be used.

### Examples

#### ROS

```
cd path-to-haros-repo/docker/ROS_DISTRO
[sudo] docker run -it --mount type=bind,source="$(pwd)"/index.yaml,target=/root/.haros/index.yaml -p 4000:4000 haros:latest /haros_call.sh https://github.com/ros2/examples -b foxy
```

or

```
cd path-to-haros-repo/docker/ROS_DISTRO
[sudo] docker run -it -v "$(pwd)"/index.yaml:/root/.haros/index.yaml -p 4000:4000 haros:latest /haros_call.sh https://github.com/ros2/examples -b foxy
```
Open on your browser the page: http://localhost:4000/

