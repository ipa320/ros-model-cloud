import os
import shutil
import subprocess
import re
import paramiko
from abc import ABCMeta, abstractmethod
from git import GitCommandError, cmd, Repo
from flaskr.extractor.exceptions import ExtractorInvalidUsage


class ExtractorRunner(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self, repository, package, request_id, branch_optional, ros_version):
        # Common for the launch & node extractors
        self.repository = repository.strip()
        self.package = package.strip()
        self.branch_optional = branch_optional.strip()
        self.id = request_id
        self.ros_version = ros_version
        self.home_path = '/home/extractor'
        self.results_path= os.path.join(self.home_path, 'results', self.ros_version)
        self.haros_runner_script = '/haros_runner.sh'
        self.messages_extractor_script = '/messages_generator_runner.sh'
        self.workspace_path = os.path.join(self.home_path, 'ws')

        # Path where the model files are stored
        #if not os.path.exists(self.results_path):
        #    os.mkdir(self.results_path)

    # use git ls-remote to check if the link to the repository is valid and the repository is public
    def _check_remote_repository(self):
        try:
            g = cmd.Git()
            g.ls_remote(self.repository)
            return True
        except GitCommandError:
            return False

    # check if selected Ros version is supported
    def _check_ros_version(self):
        versions = ['melodic','noetic','foxy']
        if self.ros_version in versions:
          return True
        else:
          return False


    def _create_ssh_client(self):
        self.client=paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.load_system_host_keys()
        mykey = paramiko.RSAKey.from_private_key_file('/keys/ssh_key')
        self.client.connect(hostname=self.ros_version, username='extractor',pkey = mykey)
        self.output=""

    # Template for the events sent by the websocket
    def _event_template(self, event_type, **kwargs):

        from flaskr.extractor.routes import extractor_ws_template

        data = {'request_id': self.id}
        data.update(kwargs)
        return extractor_ws_template(event_type, data)

    def _log_event(self, message):
        return self._event_template('log', message=message)

    def _model_event(self, model, file_name, warning=''):
        return self._event_template('model', model=model, file=file_name, warning=warning)

    # check if some of the fields are empty and if the repository is available
    @abstractmethod
    def validate(self):
        if not self.repository:
            raise ExtractorInvalidUsage.missing_field("Repository")
        if not self.package:
            raise ExtractorInvalidUsage.missing_field("Package")
        if not self._check_remote_repository():
            raise ExtractorInvalidUsage.repository_not_found(payload=self.repository)
        if not self._check_ros_version():
            raise ExtractorInvalidUsage.wrong_ros_version(payload=self.ros_version)

class MsgsExtractorRunner(ExtractorRunner):

    def __init__(self, **kwargs):
        ExtractorRunner.__init__(self, **kwargs)

    def validate(self):
        if not self.package:
            raise ExtractorInvalidUsage.missing_field("Package")
        # Path where the model files are stored
        if self.repository!="" and not super(MsgsExtractorRunner, self)._check_remote_repository():
            raise ExtractorInvalidUsage.repository_not_found(payload=self.repository)
        if not self._check_ros_version():
            raise ExtractorInvalidUsage.wrong_ros_version(payload=self.ros_version)
        if self.ros_version == "foxy":
            raise ExtractorInvalidUsage.ros_version_not_supported(payload=self.ros_version) 

    def run_analysis(self):
        msgs_models_path = os.path.join(self.results_path, 'msgs')

        model_full_path = os.path.join(msgs_models_path, self.package + '.ros')

        shell_command = '/bin/bash ' + self.messages_extractor_script +' '+ self.package +' '+ msgs_models_path+' '+self.workspace_path+' "'+self.repository+'"'
        if self.branch_optional != "":
          shell_command = shell_command[:-1]
          shell_command+=' -b '+self.branch_optional+'"'

        yield self._log_event("... Processing the command: ")
        yield self._log_event(shell_command)

        self._create_ssh_client()
        stdin, stdout, stderr = self.client.exec_command(shell_command)

        stdoutLines = stdout.readlines()
        for line in stdoutLines:
            yield self._log_event(line)
            print(line)

        self.client.close()

        # Read the file with the model
        try:
            model_file = open(model_full_path, 'r+')
            model = model_file.read().strip()
            model_file.close()
            yield self._model_event(model, self.package + '.ros')
        except (OSError, IOError):
            raise ExtractorInvalidUsage.no_model_generated()

class NodeExtractorRunner(ExtractorRunner):

    def __init__(self, node, **kwargs):
        ExtractorRunner.__init__(self, **kwargs)
        self.node = node.strip()

    def validate(self):
        super(NodeExtractorRunner, self).validate()

        if not self.node:
            raise ExtractorInvalidUsage.missing_field("Node name")
        if not self._check_ros_version():
            raise ExtractorInvalidUsage.wrong_ros_version(payload=self.ros_version)

    def run_analysis(self):
        if  (self.ros_version=='melodic'):
          python_version=2
        else:
          python_version=3
        nodes_models_path = os.path.join(self.results_path, 'nodes')

        model_full_path = os.path.join(nodes_models_path, self.node + '.ros')

        shell_command='export PYTHON_VERSION='+str(python_version)+' && /haros_runner.sh '+self.package+' '+self.node+' node ' + nodes_models_path +' '+ self.workspace_path +' "'+self.repository+'"'

        if self.branch_optional != "":
          shell_command = shell_command[:-1]
          shell_command+=' -b '+self.branch_optional+'"'

        yield self._log_event("... Processing the command: ")
        yield self._log_event(shell_command)

        self._create_ssh_client()
        stdin, stdout, stderr = self.client.exec_command(shell_command)

        stdoutLines = stdout.readlines()
        for line in stdoutLines:
            yield self._log_event(line)
            print(line)

        self.client.close()


        # Read the file with the model
        try:
            model_file = open(model_full_path, 'r+')
            model = model_file.read().strip()
            model_file.close()
            yield self._model_event(model, self.node + '.ros')
        except (OSError, IOError):
            raise ExtractorInvalidUsage.no_model_generated()

class LaunchExtractorRunner(ExtractorRunner):

    def __init__(self, launch, **kwargs):
        ExtractorRunner.__init__(self, **kwargs)
        self.launch = launch.strip()

    def validate(self):
        super(LaunchExtractorRunner, self).validate()

        if not self.launch:
            raise ExtractorInvalidUsage.missing_field("Launch file name")
        if not self._check_ros_version():
            raise ExtractorInvalidUsage.wrong_ros_version(payload=self.ros_version)

    # check if the launch file is contained in the repository
    def _launch_file_exists(self):
        for root, dirs, files in os.walk(self.workspaces_path):
            for name in files:
                if name == self.launch:
                    return True

        return False

    def run_analysis(self):
        for message in super(LaunchExtractorRunner, self).run_analysis():
            yield message

        if not self._launch_file_exists():
            raise ExtractorInvalidUsage.launch_file_not_found(payload=self.launch)

        shell_command = ['/bin/bash', self.haros_runner_path, self.package, self.launch, 'launch', self.models_path,
                         self.workspaces_path]
        extractor_process = subprocess.Popen(shell_command, stdout=subprocess.PIPE,
                                             stderr=subprocess.STDOUT,
                                             bufsize=1)

        failed_packages_regex = r"Failed to process package '(.+?)'"
        failed_packages = []

        for line in iter(extractor_process.stdout.readline, ''):
            failed_package = re.search(failed_packages_regex, line)
            if failed_package:
                failed_packages.append(failed_package.group(1))
            yield self._log_event(line)
            print(line)

        extractor_process.wait()

        for file_ in os.listdir(self.models_path):
            if file_.endswith(".ros") or file_.endswith(".rossystem"):
                model_file = open(os.path.join(self.models_path, file_), 'r+')
                model = model_file.read().strip()
                model_file.close()
                yield self._model_event(model, file_)

        
        if failed_packages:
            raise ExtractorInvalidUsage.failed_packages(payload=failed_packages)
