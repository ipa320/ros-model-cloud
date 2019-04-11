define(function () { 
    return {
        connect: function(messageHandlers, errorHandler, request) {
            return new Promise(function (resolve, reject) {    
                var socket = new WebSocket('ws://' + document.domain + ':' + location.port);  
                socket.onopen = function () {
                    socket.send(JSON.stringify(request));
                    resolve(socket);
                };

                socket.onerror = function (err) {
                    console.log(err);
                    errorHandler();
                    reject(false);
                };

                socket.onmessage = function (event) {
                    var message = JSON.parse(event.data);
                    var data = message.data;
                    var type = message.type;

                    if (type === 'run_event') {
                        messageHandlers.runMessageHandler(data);
                    } else if (type === 'model_event') {
                        messageHandlers.modelMessageHandler(data);
                    } else if (type === 'error_event'){
                        messageHandlers.errorMessageHandler(data);
                        socket.close();
                    }
                };
            });
        }
    }
});

