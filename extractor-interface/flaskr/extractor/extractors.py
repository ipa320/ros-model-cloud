import os
import shlex
import shutil
import subprocess
from abc import ABCMeta, abstractmethod

from git import GitCommandError, Repo, cmd


class ExtractorRunner(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self, repository, package, request_id):
        self.repository = repository
        self.package = package
        self.id = request_id

        self.model_path = os.path.join(os.environ['MODEL_PATH'], self.id)
        self.repo_path = os.path.join(os.environ['HAROS_SRC'], self._get_repo_basename())

    def _get_repo_basename(self):

        repo_name = self.repository[self.repository.rfind('/') + 1:]

        if not repo_name:
            url = self.repository[:-1]
            if url.endswith('/'):
                url = url[:-1]
            repo_name = url[url.rfind('/') + 1:]

        if repo_name.find('.git') != -1:
            repo_name = repo_name[:repo_name.rfind('.')]

        return repo_name

    # use git ls-remote to check if the link to the repository is valid and the repository is public
    def _check_remote_repository(self):
        try:
            g = cmd.Git()
            g.ls_remote(self.repository)
            return True
        except GitCommandError:
            return False

    def _run_event(self, message):
        return {'type': 'run_event', 'data': {'id': self.id, 'message': message}}

    def _error_event(self, message):
        return {'type': 'error_event', 'data': {'id': self.id, 'message': message}}

    def _model_event(self, model, file_):
        return {'type': 'model_event', 'data': {'id': self.id, 'message': {'model': model, 'file': file_}}}

    # check if some of the fields are empty and if the repository is available
    @abstractmethod
    def validate(self):
        if not self.repository or not self.package:
            return self._error_event('Please fill out all fields')

        if not self._check_remote_repository():
            return self._error_event(
                'The repository could not be found. Please ensure that the link is valid and the repository is public')

        return None

    @abstractmethod
    def run_analysis(self):
        pass


class NodeExtractorRunner(ExtractorRunner):

    def __init__(self, node, **kwargs):
        ExtractorRunner.__init__(self, **kwargs)
        self.node = node

    def validate(self):
        error = super(NodeExtractorRunner, self).validate()
        if error:
            return error

        if not self.node:
            return self._error_event('Please fill out all fields')

        return None

    def run_analysis(self):

        os.mkdir(self.model_path)

        shell_command = '/bin/bash ' + \
                        os.environ['HAROS_RUNNER'] + ' ' + \
                        self.repository + ' ' + self.package + ' ' + self.node + ' node ' + self.model_path

        extractor_process = subprocess.Popen(shlex.split(shell_command), stdout=subprocess.PIPE,
                                             stderr=subprocess.STDOUT,
                                             bufsize=1)

        for line in iter(extractor_process.stdout.readline, ''):
            yield self._run_event(line)
            print line

        extractor_process.wait()

        model = None

        try:
            shutil.rmtree(self.repo_path)
        except (OSError, IOError):
            pass

        try:
            model_file = open(os.path.join(self.model_path, self.node + '.ros'), 'r+')
            model = model_file.read()
            yield self._model_event(model, self.node + '.ros')
        except (OSError, IOError):
            yield self._error_event('There was a problem with the model generation')

        if not model:
            yield self._error_event('There was a problem with the model generation')