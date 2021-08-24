# ros-model-cloud

### Clone the repository

The repository has to be cloned using submodules:

```
git clone --recursive https://github.com/ipa320/ros-model-cloud
```

If it has already been cloned, the submodules have to be initialized and updated:

```
git submodule init
git submodule update
```

### Setup and start the web application

#### Using a docker container


```
./setup_helper.sh
docker-compose up

```

--------

For the old version: [https://github.com/ipa320/ros-model-cloud/tree/v1.0](https://github.com/ipa320/ros-model-cloud/tree/v1.0)


### Setup and start the web application

#### Using a docker container

Instructions on installing Docker can be found here:[https://docs.docker.com/install/linux/docker-ce/ubuntu/](https://docs.docker.com/install/linux/docker-ce/ubuntu/)

Build the docker image:
```shell
cd extractor-interface
[sudo] docker build --tag=haros .
```

Run the container:
```shell
[sudo] docker run -p 4000:4000 -ti haros:latest
```

Open on your browser the page: http://localhost:4000/ where the Git repository, node and package names can be set

#### Using a docker container on a server as a web service

```shell
[sudo] docker run -d --restart always -p 4000:4000 -ti haros:latest
```
where *-d* means in the background and *--restart always* that the daemon job will be automatically started if the system is rebooted.

#### Local testing

The app requires `python 2.7` and a local installation of `nodejs`. It assumes that the terminal shell, in which it is started, has a source `/opt/ros/<distro>/setup.bash`. The setup described below has been tested on Ubuntu 16.04 with ROS Kinetic.

Install node (detailed instructions can be found [here](https://gist.github.com/d2s/372b5943bce17b964a79)):

```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.34.0/install.sh | bash
nvm install 10
```

The dependencies needed for [HAROS](https://github.com/git-afsantos/haros) need to be set up:

```shell
[sudo] apt-get install cppcheck
[sudo] apt-get install cccc
[sudo] apt-get install libclang-3.8-dev
```

Additionally, install `clang-3.8`

```shell
[sudo] apt-get install clang-3.8
```

Set the `LD_LIBRARY_PATH` environmental variable:
```shell
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/llvm-3.8/lib
```

To start the app in a virtual enviroment, install virtualenv:

```shell
sudo apt-get install python-virtualenv
```

Create a venv directory and install the requirements:

```shell
cd extractor-interface
virtualenv --system-site-packages -p python2.7 venv
source venv/bin/activate
pip install -r requirements.txt 
```

Build the frontend:

```
npm install
npm run build
```

Run the app:

```shell
gunicorn -k flask_sockets.worker wsgi:app
```

To start the frontend in watch-mode (will rebuild automatically if one of the files in the `static` folder changes), run in a new terminal:
```shell
npm run watch
```

Open http://127.0.0.1:8000 in a browser.

All generated models will be saved in the folder `extractor-interface/models`. Temporary workspaces used for the extraction are created in the folder `extractor-interface/workspaces`.
