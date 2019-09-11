from flask import jsonify


def template(message, status_code=500, payload=''):
    return {'message': {'errors': {'body': message, 'payload': payload}}, 'status_code': status_code}

class BaseInvalidUsage(Exception):

    status_code = 500

    def __init__(self, message, status_code=None, payload=None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.payload = payload

    def to_json(self):
        rv = self.message
        return jsonify(rv)





