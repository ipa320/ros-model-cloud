# ros-model-cloud

## Start the flask app

Create venv and install the requirements:

```shell
     cd extractor-interface
     virtualenv --python=python2.7 venv
     . venv/bin/activate
     pip install -r requirements.txt
```

Run the app:

```shell
     export MODEL_PATH={Path to the location where the file with the model is saved}
     export HAROS_RUNNER={Path to the location of the haros runner script}
     python wsgi.py
     // OR
     gunicorn --worker-class eventlet -w 1 wsgi:app
```

Open http://127.0.0.1:5000 in a browser.
