import React from 'react';
import { Row, Icon, Button, TextInput } from 'react-materialize';
import setValue from "./FormsWrapper";


export default class Form extends React.Component {

    render() {
        return <form>
            <Row className="float-right">
                <Button small flat 
                    className="grey lighten-4"
                    disabled={this.props.removeDisabled || this.props.loading} 
                    onClick={(event) => {
                        event.preventDefault(); this.props.removeForm(this.props.values.request_id)}} > 
                    <Icon>clear</Icon> 
                </Button>
            </Row>
            
            {this.props.fields.map((field) => {
                return <Row>
                    <TextInput 
                    name={field.name} 
                    label={field.label} 
                    id={`${this.props.values.request_id}_${field.name}`} 
                    value={this.props.values[field.name]} 
                    onChange={(event) => this.props.setValue(this.props.values.request_id, event.target)} 
                    disabled={this.props.loading}/>
                </Row>
            })}
        </form>
    }
}
