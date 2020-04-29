import os

from flask import Flask
from flask_sockets import Sockets


def create_app(test_config=None):
    # create and configure the app
    app = Flask(__name__, instance_relative_config=True)

    if test_config is None:
        # load the instance config, if it exists, when not testing
        app.config.from_pyfile('config.py', silent=True)
    else:
        # load the test config if passed in
        app.config.from_mapping(test_config)

    # ensure the instance folder exists
    try:
        os.makedirs(app.instance_path)
    except OSError:
        pass

    os.environ['GIT_TERMINAL_PROMPT'] = '0'

    from . import extractor
    app.register_blueprint(extractor.bp)
    app.debug = True
    app.config['DEBUG'] = os.environ.get('DEBUG', True)
    sockets_app = Sockets(app)
    sockets_app.register_blueprint(extractor.ws)

    return app

