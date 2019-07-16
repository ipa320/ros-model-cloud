import {h, Component} from "preact";
import seronet from '../images/SeRoNet_Logo.png';
import haros from '../images/Haros_Logo.png';

export default class Header extends Component {

    render() {
        return <header className="header">
            <h1>ROS Model Extractor</h1>
            <a href="https://www.seronet-projekt.de" target="_blank"><img src={seronet} alt='SeRoNet'/> </a> 
            <a href="https://github.com/git-afsantos/haros" target="_blank"><img src={haros} alt='HAROS'/> </a>
        </header>
    }
}