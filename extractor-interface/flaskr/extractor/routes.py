import json
import os
import zipfile
from io import BytesIO

from flask import render_template, send_file, request

from flaskr.extractor import bp, ws
from flaskr.extractor.errors import ExtractorInvalidUsage
from flaskr.extractor.extractors import NodeExtractorRunner


@bp.route('/', methods=['GET'])
def get_extractor():
    return render_template('/extractor.html')


@bp.errorhandler(ExtractorInvalidUsage)
def handle_invalid_usage(error):
    response = error.to_json()
    response.status_code = error.status_code

    return response


@bp.route('/download')
def download_file():
    memory_file = BytesIO()

    if not request.args:
        raise ExtractorInvalidUsage.no_files_specified()

    with zipfile.ZipFile(memory_file, 'w') as zf:
        for key, value in request.args.iteritems():
            try:
                full_path = os.path.join(os.environ['MODEL_PATH'], value)
                ros_files = [x for x in os.listdir(full_path) if x.endswith('.ros') or x.endswith('.rossystem')]
                for ros_file in ros_files:
                    zf.write(os.path.join(full_path, ros_file), ros_file)
            except OSError:
                raise ExtractorInvalidUsage.file_not_found()
    memory_file.seek(0)
    return send_file(memory_file, as_attachment=True, attachment_filename='ros-models.zip')


@ws.route('/')
def websocket(ws):
    while not ws.closed:
        message = ws.receive()

        # if the message is None, it means that the connection has been closed by the client
        if message is not None:
            parsed_message = json.loads(message)
            extractor_runners = []
            for request in parsed_message:
                print request
                runner = NodeExtractorRunner(**request)
                extractor_runners.append(runner)

            validation_errors = []
            for runner in extractor_runners:
                error = runner.validate()
                if error:
                    validation_errors.append(error['data'])

            if validation_errors:
                ws.send(json.dumps({'type': 'error_event', 'data': validation_errors}))
                return

            runner_errors = []
            models = []

            for runner in extractor_runners:
                for message in runner.run_analysis():
                    if message['type'] == 'run_event':
                        ws.send(json.dumps(message))
                    elif message['type'] == 'model_event':
                        models.append(message['data'])
                    elif message['type'] == 'error_event':
                        runner_errors.append(message['data'])

            ws.send(json.dumps({'type': 'model_event', 'data': models}))
            ws.send(json.dumps({'type': 'error_event', 'data': runner_errors}))
