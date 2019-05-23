import { h, Component } from "preact";
import { Row, Icon, Button, TextInput } from 'react-materialize';


export default class Form extends Component {

    render({ removeDisabled, removeForm, values, error, setValue, loading, fields }) {
        return <form>
            <Row className="float-right">
                <Button small flat className="grey lighten-4"
                    disabled={removeDisabled || loading} 
                    onClick={(event) => {event.preventDefault(); removeForm(values.request_id)}} > <Icon>clear</Icon> </Button>
            </Row>
            <Row className='flash'>{error}</Row>
            
            {fields.map((field) => {
                return <Row>
                    <TextInput name={field.name} id={`${values.request_id}_${field.name}`} label={field.label} value={values[field.name]} onChange={(event) => setValue(values.request_id, event)} disabled={loading}/>
                </Row>
            })}
        </form>
    }
}