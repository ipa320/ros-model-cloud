define(function () {
    
    function createInputField(name, idx, placeholder) {
        return '<div class="row">' +
            '<div class="input-field col s12">' +
            '<input name="' + name + '" id="' + name + idx + '"' +
            ' class="validate"' +
            ' type="text"' +
            ' required' +
            '>' +
            '<label for="' + name + idx + '">' + placeholder + '</label>' +
            '</div>' +
            '</div>'
    }

    function removeForm(event) {
        var form = event.currentTarget.parentNode.parentNode;
        form.parentNode.removeChild(form);

        var forms = document.querySelectorAll('form');
        if (forms.length === 1) {
            forms.forEach(function (form) {
                form.querySelector('.remove-node').classList.add('disabled');
            });
        }
    }

    function createForm(idx) {
        var repositoryInput = createInputField('repository', idx, 'Git repository');
        var packageInput = createInputField('package', idx, 'Package');
        var nodeInput = createInputField('name', idx, 'Node name');

        var form = '<form id="' + idx + '">' +
            '<div class="flash"></div>' +
            '<div class="float-right">' +
            '   <a class="btn btn-small btn-flat grey lighten-4 remove-node">' +
            '       <i class="material-icons">clear</i>' +
            '   </a>' +
            '</div>' +
            repositoryInput +
            packageInput +
            nodeInput +
            '</form>';

        return form
    }


    return {
        checkRequiredFields: function () {
            var fields = document.querySelectorAll("[required]")

            var areValid = true;
            fields.forEach(function (field) {
                if (field.value === '') {
                    areValid = false;
                }
            });

            return areValid;
        },

        addForm: function (parentElement, formIdx) {
            parentElement.insertAdjacentHTML('beforeend', createForm(formIdx));
            var removeButton = document.getElementById(formIdx.toString())
                .querySelector('.remove-node');

            removeButton.onclick = function (e) {
                removeForm(e);
            }

            var forms = document.querySelectorAll('form');
            if (forms.length > 1) {
                forms.forEach(function (form) {
                    form.querySelector('.remove-node').classList.remove('disabled');
                });
            } else {
                removeButton.classList.add('disabled');
            }
        }
    }
});