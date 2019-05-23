import {h, Component} from "preact";
import { Row } from 'react-materialize';
import API, {eventTypes, errorMessages} from '../api';


export default class Error extends Component {

    constructor(props){
        super(props);

        this.state = {
            error: ''
        }
    }

    showError = () => {
        this.setState({error: errorMessages.SOCKET_NOT_CONNECTED})
    };

    dismissError = () => {
        this.setState({error: ''});
    };

    componentDidMount() {
        API.subscribe(eventTypes.SOCKET_ON_ERROR, this.showError);
        API.subscribe(eventTypes.SOCKET_ON_OPEN, this.dismissError);
    }

    componentWillUnmount() {
        API.unsubscribe(eventTypes.SOCKET_ON_ERROR, this.showError);
        API.unsubscribe(eventTypes.SOCKET_ON_OPEN, this.dismissError);
    }

    render(props, {error}) {
        return <Row>
            <div className="flash">{error}</div>
        </Row>
    }
}