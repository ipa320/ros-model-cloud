from flask import Blueprint

bp = Blueprint(r'extractor', __name__)
ws = Blueprint(r'extractor_ws', __name__)

from flaskr.extractor import routes