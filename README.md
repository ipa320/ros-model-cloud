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

#### Local testing

The app required `python 2.7` and a local installation of `nodejs`. It assumes that the terminal shell, in which it is started, has a source `/opt/ros/<distro>/setup.bash`. 

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

To start the frontend in watch-mode, run:
```shell
npm run watch
```

Open http://127.0.0.1:8000 in a browser.
