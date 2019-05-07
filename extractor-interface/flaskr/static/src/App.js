import { h, Component } from "preact";
import Header from "./components/Header";
import 'materialize-css/dist/js/materialize';
import 'materialize-css/dist/css/materialize.css';
import './styles/style.css';
import Forms from './components/forms/Forms';
import { Row, Col } from 'react-materialize';
import Models from './components/Models';
import Logs from './components/Logs';


export default class App extends Component {

    render(props) {
        return <div>
            <Header />
            <Row>
                <Col s={12} m={4} l={3}>
                    <Forms />
                </Col>
                <Col s={12} m={8} l={9}>
                    <Models />
                    <Logs />
                </Col>
            </Row>

        </div>
    }
}