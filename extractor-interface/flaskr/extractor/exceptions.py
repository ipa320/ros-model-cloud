from flaskr.exceptions import BaseInvalidUsage, template
import json

FILE_NOT_FOUND = template('FILE_NOT_FOUND', 404)
NO_FILES_SPECIFIED = template('NO_FILES_SPECIFIED', 422)
REPOSITORY_NOT_FOUND = template('REPOSITORY_NOT_FOUND', 400)
MISSING_FIELD = template('MISSING_FIELD', 400)
NO_MODEL_GENERATED = template('NO_MODEL_GENERATED', 500)
LAUNCH_FILE_NOT_FOUND = template('LAUNCH_FILE_NOT_FOUND', 400)
FAILED_PACKAGES = template('FAILED_PACKAGES', 500)


class ExtractorInvalidUsage(BaseInvalidUsage):

    def __init__(self, **kwargs):
        BaseInvalidUsage.__init__(self, kwargs)

    def to_ws_json(self):
        from flaskr.extractor.routes import extractor_ws_template
        rv = self.message
        return json.dumps(extractor_ws_template('error', rv))

    @classmethod
    def file_not_found(cls):
        return cls(**FILE_NOT_FOUND)

    @classmethod
    def no_files_specified(cls):
        return cls(**NO_FILES_SPECIFIED)

    @classmethod
    def repository_not_found(cls, payload=''):
        return cls(payload=payload, **REPOSITORY_NOT_FOUND)

    @classmethod
    def missing_field(cls):
        return cls(**MISSING_FIELD)

    @classmethod
    def no_model_generated(cls):
        return cls(**NO_MODEL_GENERATED)

    @classmethod
    def launch_file_not_found(cls, payload=''):
        return cls(payload=payload, **LAUNCH_FILE_NOT_FOUND)

    @classmethod
    def failed_packages(cls, payload):
        return cls(payload=payload, **FAILED_PACKAGES)
