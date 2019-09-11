import {errorMessages, eventTypes} from "./constants";
import Observer from './observer';


class API {

    constructor() {
        this.socket = null
    }

    connect = () => {
        return new Promise((resolve, reject) => {
            this.socket = new WebSocket('ws://' + document.domain + ':' + location.port);

            this.socket.onopen = () => {
                Observer.execute(eventTypes.SOCKET_ON_OPEN);
                resolve(true);
            };

            this.socket.onerror = (err) => {
                Observer.execute(eventTypes.SOCKET_ON_ERROR, {message: errorMessages.SERVER_ERROR});
                reject(false);
            };

            this.socket.onclose = (event) => {
                // this is not really reliable
                if (!event.wasClean) {
                    Observer.execute(eventTypes.SOCKET_ON_ERROR, {message: errorMessages.SERVER_ERROR})
                } else {
                    Observer.execute(eventTypes.SOCKET_ON_CLOSE, event)
                }
            };

            this.socket.onmessage = (event) => {
                const message = JSON.parse(event.data);
                const data = message.data;
                const type = message.type;

                switch (type) {
                    case 'log':
                        Observer.execute(eventTypes.SOCKET_ON_MESSAGE_LOG, data);
                        break;
                    case 'model':
                        Observer.execute(eventTypes.SOCKET_ON_MESSAGE_MODELS, data);
                        break;
                    case 'error':
                        Observer.execute(eventTypes.SOCKET_ON_MESSAGE_ERRORS, {...data, message: errorMessages[data.message.errors.body](data.payload)});
                        break;
                    case 'extraction_done':
                        Observer.execute(eventTypes.SOCKET_ON_MESSAGE_EXTRACTION_DONE, data);
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