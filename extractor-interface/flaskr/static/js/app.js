var submitButton = document.querySelector('button[type="submit"]');
var form = document.querySelector('form');
var preloader = document.querySelector('.preloader-wrapper');
var modelCode = document.getElementById("model-code");
var pre = document.querySelector("pre");
var log = document.querySelector(".log");
var flash = document.querySelector('.flash');

function checkRequiredFields(fields) {
    var shouldShowPreloader = true;
    fields.forEach(function (field) {
        if (field.value === '') {
            shouldShowPreloader = false
        }
    });

    return shouldShowPreloader;
}

function stopLoading() {
    form.classList.remove('loading');
    preloader.setAttribute("style", "display:none;");
}

function showError(errorMessage) {
    var errorTextNode = document.createTextNode(errorMessage);
    flash.appendChild(errorTextNode);
}

function connect() {
    return new Promise(function (resolve, reject) {
        var socket = new WebSocket('ws://' + document.domain + ':' + location.port);
        socket.onopen = function () {
            resolve(socket);
        };
        socket.onerror = function (err) {
            reject(false);
        };
    });
}

submitButton.onclick = function (e) {
    e.preventDefault();
    var shouldShowPreloader = checkRequiredFields(document.querySelectorAll("[required]"));

    if (shouldShowPreloader) {
        flash.innerHTML = "";

        var repository = document.getElementById("repository").value;
        var package = document.getElementById("package").value;
        var name = document.getElementById("name").value;

        var request = JSON.stringify({ repository: repository, package: package, name: name });
        connect()
            .then(function (socket) {
                socket.send(request);

                modelCode.innerHTML = " ";
                form.classList.add('loading');
                preloader.setAttribute("style", "display:block;");
                pre.setAttribute("style", "height:0;");

                socket.addEventListener('error', function () {
                    showError('There was an error connecting to the server');
                    stopLoading();
                    socket.close();
                });

                socket.addEventListener('message', function (event) {
                    var message = JSON.parse(event.data);

                    if (message.type === 'run_event') {
                        var span = document.createElement("span");
                        var content = document.createTextNode(message.data);
                        span.appendChild(content);
                        log.appendChild(span);
                        log.appendChild(document.createElement("br"));
                        flash.innerHTML = "";

                    } else if (message.type === 'model_event') {
                        var model = message.data;
                        stopLoading();
                        modelTextNode = document.createTextNode(model);
                        modelCode.appendChild(modelTextNode);
                        pre.setAttribute("style", "height:auto;")
                        log.innerHTML = "";
                        flash.innerHTML = "";
                        socket.close();

                    } else if (message.type === 'error_event') {
                        showError(message.data);
                        stopLoading();
                        socket.close();
                    }
                });
            }).catch(function () {
                showError('There was an error connecting to the server')
            });
    } else {
        showError('Please fill out all form fields.')
    }
}