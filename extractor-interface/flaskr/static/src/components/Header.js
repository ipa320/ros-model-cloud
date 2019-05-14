import {h, Component} from "preact";
import seronet from '../images/SeRoNet_Logo.png';
import haros from '../images/Haros_Logo.png';

export default class Header extends Component {

    render() {
        return <header className="header">
            <h1>ROS Model Extractor</h1>
            <img src={seronet} alt='SeRoNet'/>
            <img src={haros} alt='HAROS'/>
        </header>
    }
}