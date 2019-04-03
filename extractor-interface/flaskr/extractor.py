from flask import (
    Blueprint, flash, g, redirect, render_template, request, session, url_for
)

import subprocess
import shlex

bp = Blueprint('extractor', __name__, url_prefix='/')


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
            return render_template('/extractor.html')

        shell_command = '/bin/bash /haros_runner.sh ' + repository + ' ' + package + ' ' + name
        extractor = subprocess.Popen(shlex.split(shell_command))
        extractor.wait()

        try:
            model_file = open('/root/' + name + '.ros')
            model = model_file.read()
        except:
            flash('There was a problem with the model generation')

    return render_template('/extractor.html', model=model)