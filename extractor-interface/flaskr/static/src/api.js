
export const eventTypes = {
    SOCKET_ON_MESSAGE_LOG: 'SOCKET_ON_MESSAGE_LOG',
    SOCKET_ON_MESSAGE_MODELS: 'SOCKET_ON_MESSAGE_MODELS',
    SOCKET_ON_MESSAGE_ERRORS: 'SOCKET_ON_MESSAGE_ERRORS',
    SOCKET_ON_ERROR: 'SOCKET_ON_ERROR',
    SOCKET_ON_OPEN: 'SOCKET_ON_OPEN',
    SOCKET_ON_CLOSE:'SOCKET_ON_CLOSE',
    SOCKET_ON_MESSAGE_EXTRACTION_DONE: 'SOCKET_ON_MESSAGE_EXTRACTION_DONE'
};

export const errorMessages = {
    REPOSITORY_NOT_FOUND: () => 'The repository could not be found. Please ensure that the link is valid and the repository is public',
    INVALID_FIELDS: () => 'Please fill out all fields.',
    NO_MODEL_GENERATED: () => 'There was a problem with the model generation.',
    SOCKET_NOT_CONNECTED: () => 'There was a problem connecting to the server.',
    LAUNCH_FILE_NOT_FOUND: () => 'The launch file could not be found in the repository',
    FAILED_PACKAGES: (packages) => packages.length === 1 ? 
    `The package '${packages[0]}' could not be built.` : 
    `The packages ${packages.slice(0, packages.length - 1).map(p => `'${p}'`).join(', ')} and '${packages[packages.length - 1]}' could not be built.`
};

class API {

    constructor() {
        this.subscriptionsObserver = {};

        for (let type in eventTypes) {
            this.subscriptionsObserver[type] = []
        }

        this.socket = null
    }

    subscribe = (event, callback) => {
        this.subscriptionsObserver[event].push(callback);
    };

    _execute = (event, data) => {
        this.subscriptionsObserver[event].map((callback) => {
            return callback(data)
        })
    };

    unsubscribe = (event, callback) => {
        this.subscriptionsObserver[event] = this.subscriptionsObserver[event].filter((cb) => cb !== callback)
    };

    connect = () => {
        return new Promise((resolve, reject) => {
            this.socket = new WebSocket('ws://' + document.domain + ':' + location.port);

            this.socket.onopen = () => {
                this._execute(eventTypes.SOCKET_ON_OPEN);
                resolve(true);
            };

            this.socket.onerror = (err) => {
                this._execute(eventTypes.SOCKET_ON_ERROR, {message: errorMessages.SOCKET_NOT_CONNECTED});
                reject(false);
            };

            this.socket.onclose = (event) => {
                if (!event.wasClean) {
                    this._execute(eventTypes.SOCKET_ON_ERROR, {message: errorMessages.SOCKET_NOT_CONNECTED})
                } else {
                    this._execute(eventTypes.SOCKET_ON_CLOSE, event)
                }
            };

            this.socket.onmessage = (event) => {
                const message = JSON.parse(event.data);
                const data = message.data;
                const type = message.type;

                switch (type) {
                    case 'log':
                        this._execute(eventTypes.SOCKET_ON_MESSAGE_LOG, data);
                        break;
                    case 'model':
                        this._execute(eventTypes.SOCKET_ON_MESSAGE_MODELS, data);
                        break;
                    case 'error':
                        this._execute(eventTypes.SOCKET_ON_MESSAGE_ERRORS, {...data, message: errorMessages[data.message](data.payload)});
                        break;
                    case 'extraction_done':
                        this._execute(eventTypes.SOCKET_ON_MESSAGE_EXTRACTION_DONE, data);
                        this.socket.close();
                        break;
                }
            }
        })
    };

    sendMessage = (message) => {
        if (this.socket && this.socket.readyState === 1) {
            this.socket.send(JSON.stringify(message))
        } else {
            throw new Error()
        }
    };


    downloadModelFiles = (ids) => {
        return new Promise((resolve, reject) => {

            let request_str = 'http://' + document.domain + ':' + location.port + '/download?';

            ids.forEach((id, index) => {
                request_str = request_str + 'id=' + id;
                if (index < ids.length - 1) {
                    request_str = request_str + '&';
                }
            });

            console.log(request_str)
            const request = new XMLHttpRequest();
            request.open('get', request_str);
            request.send();

            request.onreadystatechange = function () {
                if (request.readyState === 2 && request.status === 200) { 
                    request.responseType = "blob";
                }
                if (request.readyState === 4 && request.status === 200) {
                    
                    const contentDisposition = request.getResponseHeader('Content-Disposition');
                    
                    const regex = /filename=(.+)/;
                    const fileName = contentDisposition.match(regex)[1];

                    const type = fileName.endsWith('.zip') ? 'application/zip' : 'text/plain';

                    const newBlob = new Blob([request.response],{type});
                    if (window.navigator && window.navigator.msSaveOrOpenBlob) {
                        window.navigator.msSaveOrOpenBlob(newBlob);
                        return;
                    }

                    const data = window.URL.createObjectURL(newBlob);
                    const link = document.createElement('a');
                    link.href = data;
                    link.download = fileName;
                    document.body.appendChild(link);
                    link.click();
                    link.parentNode.removeChild(link);
                    return resolve();

                } else if (request.readyState === 4) {
                    return reject(request.response)
                }
            };
        })

    }
}


export default new API();
