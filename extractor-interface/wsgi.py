from flaskr import create_app, socketio

import eventlet
eventlet.monkey_patch()
create_thread_func = lambda f: f
start_thread_func = lambda f: eventlet.spawn(f)

app = create_app()

if __name__ == "__main__":
    socketio.run(app=app)