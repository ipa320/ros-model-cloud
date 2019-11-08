import json
import os
import zipfile
from io import BytesIO
from flask import render_template, send_file, request
from flaskr.extractor import bp, ws
from flaskr.extractor.exceptions import ExtractorInvalidUsage
from flaskr.extractor.extractors import NodeExtractorRunner, LaunchExtractorRunner, MsgsExtractorRunner


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
    # e.g. http://localhost:8000/download?id=cjznzv95f000a2460o09eytuo&id=cjznzv42f000a2460o09eytue
    if not request.args or not request.args.getlist('id'):
        raise ExtractorInvalidUsage.no_files_specified()

    for value in request.args.getlist('id'):
        try:
            full_path = os.path.join(os.path.join(os.getcwd(), 'models'), value)
            ros_files = [os.path.join(full_path, x) for x in os.listdir(full_path) if
                         x.endswith('.ros') or x.endswith('.rossystem')]
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


def extractor_ws_template(type, data=None):
    return {'type': type, 'data': data}


@ws.route('/')
def websocket(ws):
    while not ws.closed:
        message = ws.receive()
        # if the message is None, it means that the connection has been closed by the client
        if message is not None:
            extractor_runners = []
            for extraction_request in json.loads(message):
                if 'launch' in extraction_request:
                    runner = LaunchExtractorRunner(**extraction_request)
                elif 'node' in extraction_request:
                    runner = NodeExtractorRunner(**extraction_request)
                else:
                    runner = MsgsExtractorRunner(**extraction_request)
                extractor_runners.append(runner)

            try:
                for runner in extractor_runners:
                    runner.validate()

                for runner in extractor_runners:
                    for message in runner.run_analysis():
                        ws.send(json.dumps(message))

            except ExtractorInvalidUsage as error:
                print error.message
                ws.send(error.to_ws_json())

            for runner in extractor_runners:
                runner.clean_up()

            ws.send(json.dumps(extractor_ws_template('extraction_done')))
