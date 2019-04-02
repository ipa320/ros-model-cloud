import functools

from flask import (
    Blueprint, flash, g, redirect, render_template, request, session, url_for
)

from git import Repo, GitCommandError
import shutil
import os
import ros_model_extractor

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
        else:
            repo_name = repository[repository.rfind('/') + 1:]

            if repo_name.find('.git') != -1:
                repo_name = repo_name[:repo_name.rfind('.')]

            shutil.rmtree(os.environ['WORKSPACE_DIR'] + '/src')

            # TODO: os.environ.get("ROS_PACKAGE_PATH") ?

            try:
                Repo.clone_from(repository, os.environ['WORKSPACE_DIR'] + '/src/' + repo_name)
            except GitCommandError:
                error = 'The repository could not be found'

            model = ros_model_extractor.main(['--package', str(package), '--name', str(name), '--node'])

        if error:
            flash(error)

    return render_template('/extractor.html', model=model)