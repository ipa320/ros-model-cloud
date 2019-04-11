define(function () {
    var preloader = document.querySelector('.preloader-wrapper');

    return {
        startLoading: function() {           
            var forms = document.querySelectorAll('form');
            forms.forEach(function (form) {
                form.classList.add('loading');
            });
    
            var buttons = document.querySelectorAll('.btn');
            buttons.forEach(function (button) {
                button.classList.add('disabled');
            });
    
            preloader.setAttribute("style", "display:block;");
        },

        stopLoading: function () {
            var forms = document.querySelectorAll('form');
            forms.forEach(function (form) {
                form.classList.remove('loading');
            });
   
            var buttons = document.querySelectorAll('.btn');
            buttons.forEach(function (button) {
                if (button.classList.contains('remove-node') && document.querySelectorAll('form').length === 1) {
                   return;
                }
                button.classList.remove('disabled');
            });

            preloader.setAttribute("style", "display:none;");

        }   
    }
});
