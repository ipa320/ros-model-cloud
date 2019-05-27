import {h, Component} from "preact";
import API, {eventTypes} from './../api';
import {Button, Row} from 'react-materialize';

export default class Models extends Component {

    constructor(props) {
        super(props);

        this.state = {
            models: [],
            showModels: false
        }
    }

    addModel = (model) => {
        this.setState(prevState => ({models: [...prevState.models, model]}))
    };

    showModels = () => {
        this.setState({showModels: true});
    };

    clearModels = () => {
        this.setState({models: [], showModels: false})
    };

    downloadFiles = async () => {
        try {
            await API.downloadModelFiles([...new Set(this.state.models.map(model => model.request_id))])
        } catch (e) {
            console.log(e)
        }
    };

    componentDidMount() {
        API.subscribe(eventTypes.SOCKET_ON_MESSAGE_MODELS, this.addModel);
        API.subscribe(eventTypes.SOCKET_ON_OPEN, this.clearModels);
        API.subscribe(eventTypes.SOCKET_ON_MESSAGE_EXTRACTION_DONE, this.showModels);
    }

    render(props, {models, showModels}) {
        return showModels ? <div class="model-code">
            {models.length > 0 && <Row>
                <Button onClick={this.downloadFiles}>Download the files</Button>
            </Row>}
            {models.map(model => <div>
                <h6>{model.file}</h6>
                <pre>{model.model}</pre>
            </div>)}
        </div> : null
    }
}