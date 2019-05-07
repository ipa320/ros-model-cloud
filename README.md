# ros-model-cloud

## Start the flask app

Install virtualenv

```shell
     sudo apt-get install python-virtualenv
```

Create venv and install the requirements:

```shell
     cd extractor-interface
     virtualenv --python=python2.7 venv
     . venv/bin/activate
     pip install -r requirements.txt

     npm install
     npm run build
```

Run the app:

```shell
     export MODEL_PATH={Path to the location where the file with the model is saved}
     export HAROS_RUNNER={Path to the location of the haros runner script}
     export HAROS_SRC={Path to the HAROS workspace src folder}
     gunicorn -k flask_sockets.worker wsgi:app
```

Open http://127.0.0.1:8000 in a browser.
