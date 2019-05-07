from flaskr.exceptions import BaseInvalidUsage, template

FILE_NOT_FOUND = template('FILE_NOT_FOUND', 404)
NO_FILES_SPECIFIED = template('NO_FILES_SPECIFIED', 422)


class ExtractorInvalidUsage(BaseInvalidUsage):

    def __init__(self, **kwargs):
        BaseInvalidUsage.__init__(self, kwargs)

    @classmethod
    def file_not_found(cls):
        return cls(**FILE_NOT_FOUND)

    @classmethod
    def no_files_specified(cls):
        return cls(**NO_FILES_SPECIFIED)

