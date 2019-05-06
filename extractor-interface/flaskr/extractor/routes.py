import json

from flask import render_template

from flaskr.extractor import bp, ws
from flaskr.extractor.extractors import NodeExtractorRunner


@bp.route('/', methods=['GET'])
def get_extractor():
    return render_template('/extractor.html')


@ws.route('/')
def websocket(ws):

    while not ws.closed:
        message = ws.receive()
        print message
        if message: # if the message is None, it means that the connection was closed by the client
            if isinstance(message, basestring):
                parsed_message = json.loads(message)
                extractor_runners = []
                for request in parsed_message:
                    runner = NodeExtractorRunner(request['name'], request['repository'], request['package'],
                                                 request['id'])
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
