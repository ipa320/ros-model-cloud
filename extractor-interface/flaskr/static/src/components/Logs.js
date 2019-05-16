import {h, Component} from "preact";
import API, {eventTypes} from './../api';

export default class Logs extends Component {

    constructor(props){
        super(props);

        this.state = {
            logs: [],
            errors: []
        }
    }

    addLog = (log) => {
        this.setState(prevState => {
            return {
                logs: [...prevState.logs, log.message]
            }
        })
    };

    clearLogs = () => {
        this.setState({logs: [], errors: []})
    };

    onErrorMessage = (error) => {
         this.setState(prevState => {
            return {
                errors: [...prevState.errors, error]
            }
        })

    };

    onExtractionDone = () => {
        if (this.state.errors.length > 0) {
            return
        }

        this.clearLogs();
    };

    componentDidMount() {
        API.subscribe(eventTypes.SOCKET_ON_MESSAGE_LOG, this.addLog);
        API.subscribe(eventTypes.SOCKET_ON_MESSAGE_ERRORS, this.onErrorMessage);
        API.subscribe(eventTypes.SOCKET_ON_OPEN, this.clearLogs);
        API.subscribe(eventTypes.SOCKET_ON_MESSAGE_EXTRACTION_DONE, this.onExtractionDone)
    }

    componentWillUnmount() {
        API.unsubscribe(eventTypes.SOCKET_ON_MESSAGE_LOG, this.addLog);
        API.unsubscribe(eventTypes.SOCKET_ON_OPEN, this.clearLogs);
        API.unsubscribe(eventTypes.SOCKET_ON_MESSAGE_ERRORS, this.onErrorMessage);
        API.unsubscribe(eventTypes.SOCKET_ON_MESSAGE_EXTRACTION_DONE, this.onErrorMessage);
    }

    render(props, {logs}) {
        return <div class="log">
            {logs.map((log) => <div><span>{log}</span></div>)}
        </div>
    }
}