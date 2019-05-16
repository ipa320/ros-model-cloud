import {h, Component} from 'preact';
import ExtractNodeForm from "./ExtractNodeForm";
import {Button, Icon, Preloader, Row} from "react-materialize";
import Error from "../Error";
import cuid from 'cuid';
import API, {eventTypes, errorMessages} from '../../api';


export default class Forms extends Component {

    constructor(props) {
        super(props);

        this.state = {
            loading: false,
            extractNodeForms: [],
            errors: []
        }
    }

    defaultFormState = () => {
        return {
            request_id: cuid(),
            package: '',
            repository: '',
            node: ''
        }
    };

    addForm = () => {
        this.setState(prevState => {
            return {
                extractNodeForms: [...prevState.extractNodeForms, this.defaultFormState()]
            }
        })
    };

    removeForm = (request_id) => {
        this.setState(prevState => {
            return {
                extractNodeForms: prevState.extractNodeForms.filter(form => form.request_id !== request_id)
            }
        })
    };

    setValue = (request_id, event) => {
        this.setState(prevState => {
            return {
                extractNodeForms: prevState.extractNodeForms.map(form => {
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
        for (let form of this.state.extractNodeForms) {
            if (Object.values(form).some(value => !value)) {
                errors.push({request_id: form.request_id, message: errorMessages.INVALID_FIELDS});
            }
        }

        if (errors.length > 0) {
            this.setState({errors});
            return
        }


        this.dismissErrors();

        const extractNodeForms = [];

        this.state.extractNodeForms.map(form => {
            const newId = cuid();
            extractNodeForms.push({...form, request_id: newId})
        });

        this.sendRequest(extractNodeForms);
        this.setState({extractNodeForms, errors})
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

    render(props, {extractNodeForms, loading, errors}) {

        const removeDisabled = extractNodeForms.length < 2;

        return <div>
            <Error/>
            {loading && <Preloader size="big" flashing/>}
            {extractNodeForms.map(form => {
                const error = errors.find(formError => formError.request_id === form.request_id);
                return <ExtractNodeForm
                    request_id={form.request_id}
                    removeDisabled={removeDisabled}
                    removeForm={this.removeForm}
                    values={form}
                    error={error && error.message}
                    setValue={this.setValue}
                    loading={loading}
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