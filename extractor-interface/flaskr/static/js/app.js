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
        var content = document.createTextNode(logString);
        span.appendChild(content);
        log.appendChild(span);
        log.appendChild(document.createElement("br"));
    }

    function modelMessageHandler(models) {
        var modelsSorted = Object.keys(models).sort(function (a, b) {
            return a - b
        });

        var modelText = "";
        var showLogs = false;

        for (var i = 0; i < modelsSorted.length; i++) {
            modelKey = modelsSorted[i];
            if (models.hasOwnProperty(modelKey)) {
                if (models[modelKey]) {
                    modelText += models[modelKey];
                    modelText += '\n'
                } else {
                    // keeps the logs if at least one model had an error
                    showLogs = true;
                }
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
            var values = {};

            forms.forEach(function (form) {
                var repository = form.querySelector('input[name="repository"]').value;
                var packageName = form.querySelector('input[name="package"]').value;
                var name = form.querySelector('input[name="name"]').value;

                values[form.id] = { repository: repository, package: packageName, name: name };
            });

            var messageHandlers = { errorMessageHandler: errorMessageHandler, modelMessageHandler: modelMessageHandler, runMessageHandler: runMessageHandler };

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

