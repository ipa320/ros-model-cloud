import json
import os
import zipfile
from io import BytesIO

from flask import render_template, send_file, request

from flaskr.extractor import bp, ws
from flaskr.extractor.exceptions import ExtractorInvalidUsage
from flaskr.extractor.extractors import NodeExtractorRunner, LaunchExtractorRunner


@bp.route('/', methods=['GET'])
def get_extractor():
    return render_template('/base.html')


@bp.errorhandler(ExtractorInvalidUsage)
def handle_invalid_usage(error):
    response = error.to_json()
    response.status_code = error.status_code

    return response


@bp.route('/download')
def download_file():

    file_paths = []
    
    # Should be called with a list of the request ids
    if not request.args or not request.args.getlist('id'):
        raise ExtractorInvalidUsage.no_files_specified()
     
    for value in request.args.getlist('id'):
        try:
            full_path = os.path.join(os.environ['MODEL_PATH'], value)
            ros_files = [os.path.join(full_path, x) for x in os.listdir(full_path) if x.endswith('.ros') or x.endswith('.rossystem')]
            file_paths += ros_files
        except OSError:
            raise ExtractorInvalidUsage.file_not_found()

    # Send a zip file if more than one file exists, otherwise send only the text file
    if len(file_paths) > 1:
        memory_file = BytesIO()
        with zipfile.ZipFile(memory_file, 'w') as zf:
            for ros_file in file_paths:
                zf.write(ros_file, os.path.basename(ros_file))
        memory_file.seek(0)
        return send_file(memory_file, as_attachment=True, attachment_filename='ros-models.zip')
    else:
        return send_file(file_paths[0], as_attachment=True, attachment_filename=os.path.basename(file_paths[0]))


def ws_template(type, data=None):
    return {'type': type, 'data': data}

def is_launch_request(request):
    return 'launch' in request

@ws.route('/')
def websocket(ws):
    while not ws.closed:
        message = ws.receive()
        # if the message is None, it means that the connection has been closed by the client
        if message is not None:
            parsed_message = json.loads(message)
            print parsed_message
            extractor_runners = []

            for request in parsed_message:
                if is_launch_request(request):
                    runner = LaunchExtractorRunner(**request)
                else:
                    runner = NodeExtractorRunner(**request)
                extractor_runners.append(runner)

            error = None
            
            for runner in extractor_runners:
                error = runner.validate()
                if error:
                    ws.send(json.dumps(error))
                    ws.send(json.dumps(ws_template('extraction_done')))

            if not error:
                for runner in extractor_runners:
                    for message in runner.run_analysis():
                        ws.send(json.dumps(message))

                ws.send(json.dumps(ws_template('extraction_done')))
