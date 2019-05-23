import {h, Component} from 'preact';
import Form from "./Form";
import {Button, Icon, Preloader, Row} from "react-materialize";
import cuid from 'cuid';
import API, {eventTypes, errorMessages} from '../../api';


export default class Forms extends Component {

    constructor(props) {
        super(props);

        this.state = {
            loading: false,
            forms: [],
            errors: []
        }
    }

    defaultFormState = () => {
        const fieldsState = this.props.fields.reduce((prev, curr) => {
            return Object.assign({}, prev, {[curr.name]: ''}) 
        }, {});

        return {
            ...fieldsState,
            request_id: cuid(),
        }
    };

    addForm = () => {
        this.setState(prevState => {
            return {
                forms: [...prevState.forms, this.defaultFormState()]
            }
        })
    };

    removeForm = (request_id) => {
        this.setState(prevState => {
            return {
                forms: prevState.forms.filter(form => form.request_id !== request_id)
            }
        })
    };

    setValue = (request_id, event) => {
        this.setState(prevState => {
            return {
                forms: prevState.forms.map(form => {
                    if (form.request_id !== request_id) {
                        return form
                    }

                    return {...form, [event.target.name]: event.target.value}
                })
            }
        })
    };

    dismissErrors = () => {
        this.setState({errors: []})
    };

    sendRequest = async (forms) => {
        try {
            await API.connect();
            API.sendMessage(forms)
        } catch (e) {
            console.log(e);
        }
    };

    handleSubmit = () => {

        const errors = [];

        // check if some the values are empty
        for (let form of this.state.forms) {
            if (Object.values(form).some(value => !value)) {
                errors.push({request_id: form.request_id, message: errorMessages.INVALID_FIELDS});
            }
        }

        if (errors.length > 0) {
            this.setState({errors});
            return
        }


        this.dismissErrors();

        const forms = [];

        this.state.forms.map(form => {
            const newId = cuid();
            forms.push({...form, request_id: newId})
        });

        this.sendRequest(forms);
        this.setState({forms, errors})
    };

    startLoading = () => {
        this.setState({loading: true})
    };

    stopLoading = () => {
        this.setState({loading: false})
    };

    onErrorMessage = (error) => {
        this.setState(prevState => ({errors: [...prevState.errors, error]}))
    };

    componentDidMount() {
        this.addForm();

        API.subscribe(eventTypes.SOCKET_ON_OPEN, this.startLoading);
        API.subscribe(eventTypes.SOCKET_ON_ERROR, this.stopLoading);
        API.subscribe(eventTypes.SOCKET_ON_CLOSE, this.stopLoading);
        API.subscribe(eventTypes.SOCKET_ON_MESSAGE_ERRORS, this.onErrorMessage);
    }

    componentWillUnmount() {
        API.unsubscribe(eventTypes.SOCKET_ON_OPEN, this.startLoading);
        API.unsubscribe(eventTypes.SOCKET_ON_ERROR, this.stopLoading);
        API.unsubscribe(eventTypes.SOCKET_ON_CLOSE, this.stopLoading);
        API.unsubscribe(eventTypes.SOCKET_ON_MESSAGE_ERRORS, this.onErrorMessage);
    }

    render({fields}, {forms, loading, errors}) {

        const removeDisabled = forms.length < 2;

        return <div>
            {loading && <Preloader size="big" flashing/>}
            {forms.map(form => {
                const error = errors.find(formError => formError.request_id === form.request_id);
                return <Form
                    removeDisabled={removeDisabled}
                    removeForm={this.removeForm}
                    values={form}
                    error={error && error.message}
                    setValue={this.setValue}
                    loading={loading}
                    fields={fields}
                />
            })}
            <Row className='float-right'>
                <Button disabled={loading} onClick={this.addForm}> <Icon> add </Icon> </Button>
            </Row>
            <Row>
                <Button waves="light" disabled={loading} onClick={this.handleSubmit}> Submit <Icon
                    right> send </Icon></Button>
            </Row>
        </div>
    }
}