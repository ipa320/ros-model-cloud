from flaskr import create_app
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler

app = create_app()

if __name__ == "__main__":
    server = pywsgi.WSGIServer(('', 8000), app, handler_class=WebSocketHandler)
    server.serve_forever()
