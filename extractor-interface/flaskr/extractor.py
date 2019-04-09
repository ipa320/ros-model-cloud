from flask import (
    Blueprint, render_template
)

import subprocess
import shlex
import shutil
import os
import json
import git
from git import GitCommandError


bp = Blueprint('extractor', __name__)
ws = Blueprint(r'extractor_ws', __name__)


def get_repo_basename(repository):
    repo_name = repository[repository.rfind('/') + 1:]

    if not repo_name:
        url = repository[:-1]
        repo_name = url[url.rfind('/') + 1:]

    if repo_name.find('.git') != -1:
        repo_name = repo_name[:repo_name.rfind('.')]

    return repo_name


# create a message that should be sent via the websocket connection
def create_message(message_type, message):
    return json.dumps({'type': message_type, 'data': message})


# use git ls-remote to check if the link to the repository is valid and the repository is public
def check_remote_repository(repository):
    try:
        g = git.cmd.Git()
        g.ls_remote(repository)
        return True
    except GitCommandError:
        return False


@ws.route('/')
def websocket(ws):
    while not ws.closed:
        message = ws.receive()
        if message:
            parsed = json.loads(message)
            print parsed

            repository = parsed['repository']
            package = parsed['package']
            name = parsed['name']
            error = None

            if not repository:
                error = 'Please enter a valid repository name.'
            elif not package:
                error = 'Please enter a valid package name.'
            elif not name:
                error = 'Please enter a valid node name.'
            elif not check_remote_repository(repository):
                error = 'The repository could not be found. Please ensure that the link is valid and the repository is public'

            if error:
                error_message = create_message('error_event', error)
                ws.send(error_message)
            else:
                shell_command = '/bin/bash ' + os.environ['HAROS_RUNNER'] + ' ' + repository + ' ' + package + ' ' + name

                extractor_process = subprocess.Popen(shlex.split(shell_command), stdout=subprocess.PIPE,
                                                     stderr=subprocess.STDOUT,
                                                     bufsize=1)

                for line in iter(extractor_process.stdout.readline, ''):
                    run_message = create_message('run_event', str(line))
                    ws.send(run_message)
                    print line

                extractor_process.wait()

                model = None

                try:
                    repo_name = get_repo_basename(repository)
                    shutil.rmtree(os.path.join(os.environ['HAROS_SRC'], repo_name))
                except OSError or IOError:
                    pass

                try:
                    model_file = open(os.path.join(os.environ['MODEL_PATH'], name + '.ros'))
                    model = model_file.read()
                except OSError or IOError:
                    error_message = create_message('error_event', 'There was a problem with the model generation.')
                    ws.send(error_message)

                if model:
                    model_message = create_message('model_event', model)
                    ws.send(model_message)
                else:
                    error_message = create_message('error_event', 'There was a problem with the model generation')
                    ws.send(error_message)


@bp.route('/', methods=['GET'])
def get_extractor():
    return render_template('/extractor.html')