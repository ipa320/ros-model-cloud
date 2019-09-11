import {h, Component} from "preact";
import { Row } from 'react-materialize';
import Observer from '../observer';
import {eventTypes} from "../constants";


export default class Error extends Component {

    constructor(props){
        super(props);

        this.state = {
            error: ''
        }
    }

    showError = (error) => {
        this.setState({error: error.message})
    };

    dismissError = () => {
        this.setState({error: ''});
    };

    componentDidMount() {
        Observer.subscribe(eventTypes.VALIDATION_ERROR, this.showError);
        Observer.subscribe(eventTypes.SOCKET_ON_ERROR, this.showError);
        Observer.subscribe(eventTypes.SOCKET_ON_OPEN, this.dismissError);
        Observer.subscribe(eventTypes.SOCKET_ON_MESSAGE_ERRORS, this.showError);
        Observer.subscribe(eventTypes.FILE_DOWNLOAD_ERROR, this.showError);
    }

    componentWillUnmount() {
        Observer.unsubscribe(eventTypes.VALIDATION_ERROR, this.showError);
        Observer.unsubscribe(eventTypes.SOCKET_ON_ERROR, this.showError);
        Observer.unsubscribe(eventTypes.SOCKET_ON_OPEN, this.dismissError);
        Observer.unsubscribe(eventTypes.SOCKET_ON_MESSAGE_ERRORS, this.showError);
        Observer.unsubscribe(eventTypes.FILE_DOWNLOAD_ERROR, this.showError);
    }

    render(props, {error}) {
        return <Row>
            <div className="flash">{error}</div>
        </Row>
    }
}