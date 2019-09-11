import {h, Component} from "preact";
import Observer from '../observer';
import {eventTypes} from "../constants";

export default class Logs extends Component {

    constructor(props){
        super(props);

        this.state = {
            logs: [],
            error: false
        }
    }

    addLog = (log) => {
        this.setState(prevState => {
            return {
                logs: [...prevState.logs.slice(-100), log.message]
            }
        })
    };


    clearLogs = () => {
        this.setState({logs: [], error: false})
    };

    onErrorMessage = (error) => {
        console.log(error)
        this.setState({error: true})
    };

    onExtractionDone = () => {
        // this keeps the log in the view in case the extraction produced an error (as a possible explanation)
        if (this.state.error) {
            return
        }

        this.clearLogs();
    };

    componentDidMount() {
        Observer.subscribe(eventTypes.SOCKET_ON_MESSAGE_LOG, this.addLog);
        Observer.subscribe(eventTypes.SOCKET_ON_OPEN, this.clearLogs);
        Observer.subscribe(eventTypes.SOCKET_ON_MESSAGE_EXTRACTION_DONE, this.onExtractionDone);
        Observer.subscribe(eventTypes.SOCKET_ON_MESSAGE_ERRORS, this.onErrorMessage);
    }

    componentWillUnmount() {
        Observer.unsubscribe(eventTypes.SOCKET_ON_MESSAGE_LOG, this.addLog);
        Observer.unsubscribe(eventTypes.SOCKET_ON_OPEN, this.clearLogs);
        Observer.unsubscribe(eventTypes.SOCKET_ON_MESSAGE_EXTRACTION_DONE, this.onErrorMessage);
        Observer.unsubscribe(eventTypes.SOCKET_ON_MESSAGE_ERRORS, this.onErrorMessage);
    }

    render(props, {logs}) {
        return <div class="log">
            {logs.map((log) => <div><span>{log}</span></div>)}
        </div>
    }
}