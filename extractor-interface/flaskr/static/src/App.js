import { h, Component } from "preact";
import Header from "./components/Header";
import 'materialize-css/dist/js/materialize';
//import 'materialize-css/dist/css/materialize.css';
import './styles/style.css';
import FormTabs from './components/forms/FormTabs';
import { Row, Col } from 'react-materialize';
import Models from './components/Models';
import Logs from './components/Logs';
import Error from './components/Error';


export default class App extends Component {

    render(props) {
        return <div>
            <Header />
            <Row>
                <Col s={12} m={5} l={4}>
                    <Error/>
                    <FormTabs />
                </Col>
                <Col s={12} m={7} l={8}>
                    <Models />
                    <Logs />
                </Col>
            </Row>

        </div>
    }
}
