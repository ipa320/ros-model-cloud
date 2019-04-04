from flask import (
    Blueprint, flash, g, redirect, render_template, request, session, url_for
)

from threading import Lock

import subprocess
import shlex
from flaskr import socketio
import eventlet
import shutil
import os

thread = None
thread_lock = Lock()

bp = Blueprint('extractor', __name__, url_prefix='/')

def get_repo_basename(repository):
    repo_name = repository[repository.rfind('/') + 1:]

    if not repo_name:
        url = repository[:-1]
        repo_name = url[url.rfind('/') + 1:]

    if repo_name.find('.git') != -1:
        repo_name = repo_name[:repo_name.rfind('.')]

    return repo_name


@bp.route('/', methods=('GET', 'POST'))
def submit():
    model = None

    if request.method == 'POST':
        repository = request.form['repository']
        package = request.form['package']
        name = request.form['name']
        error = None

        if not repository:
            error = 'Please enter a valid repository name.'
        elif not package:
            error = 'Please enter a valid package name.'
        elif not name:
            error = 'Please enter a valid node name.'

        if error:
            flash(error)
            return render_template('/extractor.html', async_mode=socketio.async_mode)

        shell_command = '/bin/bash ' + os.environ['HAROS_RUNNER'] + ' ' + repository + ' ' + package + ' ' + name


        extractor_process = subprocess.Popen(shlex.split(shell_command), stdout=subprocess.PIPE,
                                             stderr=subprocess.STDOUT,
                                             bufsize=1)

        for line in iter(extractor_process.stdout.readline, ''):
            eventlet.sleep(1)
            socketio.emit('run_event', {'data': line})
            print line

        extractor_process.wait()

        try:
            repo_name = get_repo_basename(repository)
            shutil.rmtree(os.path.join(os.environ['HAROS_SRC'], repo_name))
            model_file = open(os.path.join(os.environ['MODEL_PATH'], name + '.ros'))
            model = model_file.read()
        except:
            flash('There was a problem with the model generation')

    return render_template('/extractor.html', model=model, async_mode=socketio.async_mode)