import {h, Component} from "preact";
import API from './../api';
import {Button, Row} from 'react-materialize';
import Observer from '../observer'
import { eventTypes, errorMessages } from "../constants";
import React from 'react';

export default class Models extends React.Component {

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
        // shows the models only after all extractions are done
        this.setState({showModels: true});
    };

    clearModels = () => {
        // clear the models when a new request is sent
        this.setState({models: [], showModels: false})
    };

    downloadFiles = async () => {
        try {
            await API.downloadModelFiles([...new Set(this.state.models.map(model => model.request_id))])
        } catch (e) {
            Observer.execute(eventTypes.FILE_DOWNLOAD_ERROR, errorMessages.SERVER_ERROR)
        }
    };

    componentDidMount() {
        Observer.subscribe(eventTypes.SOCKET_ON_MESSAGE_MODELS, this.addModel);
        Observer.subscribe(eventTypes.SOCKET_ON_OPEN, this.clearModels);
        Observer.subscribe(eventTypes.SOCKET_ON_MESSAGE_EXTRACTION_DONE, this.showModels);
    }

    componentWillUnmount() {
        Observer.unsubscribe(eventTypes.SOCKET_ON_MESSAGE_MODELS, this.addModel);
        Observer.unsubscribe(eventTypes.SOCKET_ON_OPEN, this.clearModels);
        Observer.unsubscribe(eventTypes.SOCKET_ON_MESSAGE_EXTRACTION_DONE, this.showModels);
    }

    render(props) {
        return this.state.showModels ? <div class="model-code">
            {this.state.models.length > 0 && <Row>
                <Button onClick={this.downloadFiles}>Download the files</Button>
            </Row>}
            {this.state.models.map(model => <div>
                <h6>{model.file}</h6>
                <pre>{model.model}</pre>
            </div>)}
        </div> : null
    }
}