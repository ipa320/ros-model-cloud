import { Tabs, Tab } from "react-materialize";
import FormsWrapper from "./FormsWrapper";
import React from 'react';

export default class FormTabs extends React.Component {

    commonFormFields = [
        {name: 'repository', label: 'Git repository'},
        {name: 'branch_optional', label: 'Branch (optional)'},
        {name: 'package', label: 'Package name'},
        {name: 'ros_version', label: 'Ros version (i.e melodic, noetic or foxy)'}

    ];
    
    nodeFormFields = [
        ...this.commonFormFields,
        {name: 'node', label: 'Node name'}
    ];
    
    launchFormFields = [
        ...this.commonFormFields,
        {name: 'launch', label: 'Launch file name'}
    ];

    msgFormFields = [
        {name: 'repository_optional', label: 'Git repository (optional)'},
        {name: 'branch_optional', label: 'Branch (optional)'},
        {name: 'package', label: 'Package name'},
        {name: 'ros_version', label: 'Ros version (i.e melodic, noetic or foxy)'}
    ];

    render() {
        return <Tabs>
            <Tab title="Node analysis">
                <FormsWrapper fields={this.nodeFormFields}/>
            </Tab>
            <Tab title="Specifications analysis">
                <FormsWrapper fields={this.msgFormFields}/>
            </Tab>
            <Tab title="Launch analysis">
                <FormsWrapper fields={this.launchFormFields}/>
            </Tab>
        </Tabs>
    }
}
