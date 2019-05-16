import { h, Component } from "preact";
import { Row, Icon, Button, TextInput } from 'react-materialize';


export default class ExtractNodeForm extends Component {

    render({ request_id, removeDisabled, removeForm, values, error, setValue, loading }) {
        return <form>
            <Row className="float-right">
                <Button small flat className="grey lighten-4"
                    disabled={removeDisabled || loading} 
                    onClick={() => removeForm(request_id)} > <Icon>clear</Icon> </Button>
            </Row>
            <Row className='flash'>{error}</Row>
            <Row>
                <TextInput name='repository' id={`${request_id}_repository`} label='Git repository' value={values.repository} onChange={(event) => setValue(request_id, event)} disabled={loading} />
            </Row>
            <Row>
                <TextInput name='package' id={`${request_id}_package`} label='Package name' value={values.package} onChange={(event) => setValue(request_id, event)} disabled={loading}/></Row>
            <Row>
                <TextInput name='node' id={`${request_id}_node`} label='Node name' value={values.node} onChange={(event) => setValue(request_id, event)} disabled={loading} /></Row>
        </form>
    }
}