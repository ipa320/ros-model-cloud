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

submitButton.onclick = function (e) {
    var shouldShowPreloader = checkRequiredFields(requiredFields);

    if (shouldShowPreloader) {
        document.querySelector('form').classList.add('loading');
        document.querySelector('.preloader-wrapper').setAttribute("style", "display:block;");
    }
}