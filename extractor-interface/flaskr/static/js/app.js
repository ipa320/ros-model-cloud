var submitButton = document.querySelector('button[type="submit"]');
var requiredFields = document.querySelectorAll("[required]")

function checkRequiredFields (fields) {
    var shouldShowPreloader = true;
    fields.forEach(function (field) {
        if (field.value === '') {
            shouldShowPreloader = false
        }
    });

    return shouldShowPreloader;
}

namespace = '';

var log = document.querySelector(".log")

var socket = io.connect('http://localhost' + ':' + location.port + namespace);

socket.on('run_event', function(message) {
     var span = document.createElement("span");
     var content = document.createTextNode(message.data);
     span.appendChild(content);
     log.appendChild(span)
     log.appendChild(document.createElement("br"))
});

submitButton.onclick = function (e) {
    var shouldShowPreloader = checkRequiredFields(requiredFields);

    if (shouldShowPreloader) {
        document.querySelector('form').classList.add('loading');
        document.querySelector('.preloader-wrapper').setAttribute("style", "display:block;");
        document.querySelector('pre').remove()
    }
}