requirejs(["formControl", "loaderControl", "websocket"], function (formControl, loaderControl, websocket) {

    var submitButton = document.querySelector('button[type="submit"]');
    var modelCode = document.getElementById("model-code");
    var log = document.querySelector(".log");
    var commonError = document.getElementById("error-common");
    var formsWrapper = document.getElementById("forms-wrapper");
    var addFormButton = document.getElementById('add-form');

    function showCommonError(errorMessage) {
        var errorTextNode = document.createTextNode(errorMessage);
        commonError.appendChild(errorTextNode);
    }

    // incremented when new forms are added
    var formIdx = 0;

    // Add the initial form
    formControl.addForm(formsWrapper, formIdx);

    // Add additional ones
    addFormButton.onclick = function () {
        formIdx++;
        formControl.addForm(formsWrapper, formIdx);
    };

    function errorHandler() {
        showCommonError('There was an error connecting to the server');
        loaderControl.stopLoading();
    }

    function errorMessageHandler(errors) {
        for (var key in errors) {
            if (errors.hasOwnProperty(key)) {
                if (errors[key.toString()]) {
                    var flashContainer = document.getElementById(key).querySelector('.flash');
                    var errorTextNode = document.createTextNode(errors[key]);
                    flashContainer.appendChild(errorTextNode);
                }
            }
        }
        loaderControl.stopLoading();

    }

    function runMessageHandler(logString) {
        var span = document.createElement("span");
        var content = document.createTextNode(logString.message);
        span.appendChild(content);
        log.appendChild(span);
        log.appendChild(document.createElement("br"));
    }

    function modelMessageHandler(models) {

        console.log(models);

        var modelText = "";
        var showLogs = false;

        for (var i = 0; i < models.length; i++) {
            model = models[i];
            if (model.message.model) {
                modelText += model.message.model;
                modelText += '\n'
            } else {
                // keeps the logs if at least one model had an error
                showLogs = true;
            }
        }

        loaderControl.stopLoading();

        if (modelText) {
            var modelTextNode = document.createTextNode(modelText);
            modelCode.appendChild(modelTextNode);
            if (!showLogs) {
                log.innerHTML = "";
            }
        }

        var downloadButton = document.getElementById('download');
        var ids = [];

        models.forEach(function (model) {
            ids.push(model.id)
        });

        var request_str = 'http://' + document.domain + ':' + location.port + '/download?';

        ids.forEach(function (id, index) {
            request_str = request_str + 'id=' + id;
            if (index < ids.length - 1) {
                request_str = request_str + '&';
            }
        });

        console.log(request_str);
        // downloadButton.setAttribute('href', request_str);

        downloadButton.onclick = function () {
            var request = new XMLHttpRequest();

            request.open('get', request_str);

            request.send();

            request.onreadystatechange = function () {
                if (request.readyState === 2) {
                    console.log(request);
                    if (request.status === 200) {
                        request.responseType = "blob";
                    }
                }
                if (request.readyState === 4 && request.status === 200) {
                    const url = window.URL.createObjectURL(new Blob([request.response]));
                    const link = document.createElement('a');
                    link.href = url;
                    link.setAttribute('download', 'ros-models.zip');
                    document.body.appendChild(link);
                    link.click();
                } else if (request.readyState === 4) {
                    console.log(request);
                }
            };
        }

    }

    submitButton.onclick = function (e) {
        e.preventDefault();

        var errorBlocks = document.querySelectorAll('.flash');
        errorBlocks.forEach(function (errorBlock) {
            errorBlock.innerHTML = "";
        });

        var forms = document.querySelectorAll('form');

        if (forms.length === 0) {
            return showCommonError('Please add at least one node.')
        }

        var shouldSendRequest = formControl.checkRequiredFields();

        if (shouldSendRequest) {
            var values = [];

            forms.forEach(function (form) {
                var repository = form.querySelector('input[name="repository"]').value;
                var packageName = form.querySelector('input[name="package"]').value;
                var name = form.querySelector('input[name="name"]').value;

                values.push({repository: repository, package: packageName, node: name, request_id: cuid()});
            });

            var messageHandlers = {
                errorMessageHandler: errorMessageHandler,
                modelMessageHandler: modelMessageHandler,
                runMessageHandler: runMessageHandler
            };

            websocket.connect(messageHandlers, errorHandler, values)
                .then(function () {
                    loaderControl.startLoading();
                    modelCode.innerHTML = "";
                    log.innerHTML = "";
                }).catch(function (err) {
                console.log(err);
                showCommonError('There was an error connecting to the server')
            });
        } else {
            showCommonError('Please fill out all form fields.')
        }
    }
});

