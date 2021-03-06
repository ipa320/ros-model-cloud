# Cloning the repository

The repository has to be cloned using submodules:

```
git clone --recursive https://github.com/ipa320/ros-model-cloud
```

If it has already been cloned, the submodules have to be initialized and updated:

```
git submodule init
git submodule update
```

# Use the docker container

```
sudo docker build --tag=haros .
```

Run the docker container
```
sudo docker run -p 4000:4000 -ti haros:latest
```

Open on your browser the page: http://localhost:4000/ where the Git repository, node and package names can be set
