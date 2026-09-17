/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_rui) repo
# (see https://github.com/nepi-engine/nepi_rui)
#
# License: NEPI RUI repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import Toggle from "react-toggle"
import Button, { ButtonMenu } from "./Button"
import AsyncToggle from "./AsyncToggle"
import Label from "./Label"
import BooleanIndicator from "./BooleanIndicator"
import Select, { Option } from "./Select"
import { Column, Columns } from "./Columns"

import NepiIFControls from "./Nepi_IF_Controls"
import Nepi_IF_Data from "./Nepi_IF_Data"
import NepiIFConfig from "./Nepi_IF_Config"
import { onChangeSwitchStateValue} from "./Utilities"

@inject("ros")
@observer

// Component that contains the Process controls
class Nepi_IF_ConnectProcess extends Component {
  constructor(props) {
    super(props)

    // these states track the values through  Status messages
    this.state = {

      processNamespace: 'None',
      status_msg: null,

      show_results: (this.props.show_results !== undefined) ? this.props.show_results : true,
      show_controls: (this.props.show_controls !== undefined) ? this.props.show_controls : true,

      processListener: null,
      needs_update: false,
    }

    this.getProcessNamespace = this.getProcessNamespace.bind(this)
    this.updateProcessListener = this.updateProcessListener.bind(this)
    this.processListener = this.processListener.bind(this)

    // renderControls / renderData are not bound here: they no longer exist.
    // renderProcess() mounts Nepi_IF_Data and Nepi_IF_Controls inline. Binding
    // a method that does not exist throws in the constructor, which kills the
    // whole page the moment this component mounts.
    this.renderProcessSelector = this.renderProcessSelector.bind(this)
    this.onProcessSelected = this.onProcessSelected.bind(this)
    this.renderProcess = this.renderProcess.bind(this)
  }

  // Callback for handling ROS Process Status messages.
  //
  // No namespace equality guard here. The node reports ProcessStatus.namespace
  // as the fully-resolved process namespace, and this component may have been
  // pointed at the same namespace by a different string; the listener is
  // already scoped to one topic, so anything arriving on it belongs here.
  processListener(message) {
    this.setState({ status_msg: message })
  }

  // Function for configuring and subscribing to Process Status
  updateProcessListener(processNamespace) {
    if (this.state.processListener != null ) {
      this.state.processListener.unsubscribe()
           this.setState({processListener: null})
            this.setState({ status_msg: null })
    }
    if (processNamespace !== '' &&  processNamespace !== 'None'){
      var processListener = this.props.ros.setupProcessListener(
            processNamespace + '/status',
            this.processListener
          )
      this.setState({ processNamespace: processNamespace, updateNamespace: null})
      this.setState({ processListener: processListener})
    }
  }


    componentDidMount() {
      this.setState({needs_update: true})
    }

  // Lifecycle method cAlled when compnent updates.
  // Used to track changes in the topic
  // The namespace prop is what every caller passes; processNamespace is kept as
  // an accepted alias. Reading only processNamespace is what left this component
  // permanently unsubscribed and blank.
  getProcessNamespace() {
    const ns = (this.props.namespace !== undefined && this.props.namespace !== null) ?
                  this.props.namespace : this.props.processNamespace
    if (ns === undefined || ns === null || ns === '' || ns === 'None') {
      return 'None'
    }
    return ns
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const processNamespace = this.getProcessNamespace()
    const needs_update = ((this.state.processNamespace !== processNamespace))

    if (needs_update) {
      this.setState({processNamespace: processNamespace})
      this.updateProcessListener(processNamespace)
    }
  }

  // Lifecycle method cAlled just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.processListener) {
      this.state.processListener.unsubscribe()
    }
    this.setState({processListener: null, 
                  status_msg: null})
  }



  // Handler for the device Select. Changes the connected topic by publishing a
  // std_msgs/String to the connect namespace select_topic topic.
  onProcessSelected(event) {
    const namespace = this.getProcessNamespace()
    const value = event.target.value
    if (namespace != null && namespace !== 'None') {
      this.props.ros.sendStringMsg(namespace + '/set_process', value)
    }
  }

  // Device selector, backed by ConnectIFStatus. Populated from
  // available_processes/available_names, shows the selected_name and a connected
  // BooleanIndicator, and changes the connection by publishing a
  // std_msgs/String to the connect namespace select_topic topic.
  renderProcessSelector() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return (
        <Columns>
          <Column>

          </Column>
        </Columns>
      )
    }

    const available_processes = status_msg.available_processes
    const available_names = status_msg.available_processes
    const selected_process = status_msg.selected_process

    var items = []

    for (var i = 0; i < available_processes.length; i++) {
      const device_name = (available_names[i] !== undefined) ? available_names[i] : available_processes[i]
      items.push(<Option value={available_processes[i]}>{device_name}</Option>)
    }
    if (items.length == 0 ) {
      items.push(<Option value={'None'}>{'None'}</Option>)
    }

    return (
      <Columns>
        <Column>

         
            <Select
              onChange={this.onProcessSelected}
              value={selected_process}
            >
              {items}
            </Select>


        </Column>
        <Column>

        </Column>
      </Columns>
    )
  }



  renderProcess() {
   
    const status_msg = this.state.status_msg
    const config_topic = status_msg.config_topic

    const { userRestricted} = this.props.ros
    const ignore_restrictions = (this.props.ignore_restrictions !== undefined) ? this.props.ignore_restrictions : false
    

    
    const controls_restricted = userRestricted.indexOf('SYSTEM-PROCESS-CONTROL') !== -1 && (ignore_restrictions === false)

    const show_process = (this.props.show_process !== undefined) ? this.props.show_process: status_msg.show_process


    const has_controls = status_msg.has_controls
    var allways_show_controls = (this.props.allways_show_controls !== undefined) ? (this.props.allways_show_controls  && has_controls): false
    var show_controls = (allways_show_controls === true) ? true : this.state.show_controls
    if (status_msg.show_controls === false || controls_restricted === true || has_controls === false) {
      allways_show_controls = false
      show_controls = false
    }
    else {
      show_controls = (show_controls === true || allways_show_controls === true)
    }

    const has_results = status_msg.has_results
    const show_results = (this.props.show_results !== undefined) ? (this.props.show_results  && has_results): has_results
    

    const { sendBoolMsg, sendTriggerMsg } = this.props.ros
    const namespace = status_msg.namespace
    // Normalized rather than read raw: a ProcessStatus from a node built
    // against a different nepi_interfaces can arrive missing these, and an
    // undefined here would otherwise reach AsyncToggle as its checked prop.
    const enabled = (status_msg.enabled === true)
    const show_enable = (this.props.show_enable !== undefined) ? this.props.show_enable: status_msg.show_enable
    const running = (status_msg.running === true)
    const process_ready = (status_msg.process_ready === true)
    const msg_str = (status_msg.msg_str !== undefined && status_msg.msg_str !== null) ? status_msg.msg_str : ''


      return (
        <React.Fragment>

              { ( show_process === true ) ?

                <Columns>
                <Column>
                    <Label title={"Select Process"}>
                      {this.renderProcessSelector()}
                    </Label>
                </Column>
                <Column>
                      <ButtonMenu>
                      <Button onClick={() => sendTriggerMsg(namespace + '/reload_process')}>{"RELOAD"}</Button>
                    </ButtonMenu>
                </Column>
              </Columns>
                : null}


             {(show_enable === true) ?
              <Columns>
                <Column>
                    <Label title={"Enable"}>
                      <AsyncToggle
                        disabled={process_ready === false}
                        checked={enabled === true}
                        onClick={() => sendBoolMsg(namespace + "/set_enable", !enabled)}>
                      </AsyncToggle>
                    </Label>
                </Column>
                <Column>
                    <Label title={"Running"}>
                      <BooleanIndicator value={running === true} />
                    </Label>
                </Column>
              </Columns>
              : null }

              {/* {(show_enable === true && msg_str !== '' && msg_str !== undefined) ?
                <pre style={{ height: "24px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {msg_str}
                </pre>
              : null } */}


              { ( show_results === true ) ?
              <Nepi_IF_Data
                make_section={false}
                title={null}
                allways_show_data={true}
                namespace={ status_msg.namespace}
                status_msg={status_msg.results}
                />
                : null}

              {/* The process enable. This is the control that starts and stops
                  the process itself; everything below it is display state.
                  Enabled is what the operator asked for, Running is what the
                  node reports back, and they are shown separately so an enable
                  the node could not honour is visible rather than silent. */}
 

              <Columns>
                <Column>


                    {(allways_show_controls === false && has_controls === true) ?
                    <Label title="Show Controls">
                        {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                        <Toggle
                          checked={show_controls===true}
                          onClick={() => onChangeSwitchStateValue.bind(this)("show_controls",show_controls)}>
                        </Toggle>
                    </Label>
                    : null }

                  </Column>
                  <Column>



                  </Column>
                </Columns>




      { ( show_controls === true ) ?
      <NepiIFControls
        make_section={false}
        title={null}
        allways_show_controls={true}
        namespace={ status_msg.namespace}
        status_msg={status_msg.controls}
        />
        : null}

      { ( config_topic !== '' ) ?
        <NepiIFConfig
          namespace={config_topic}
          title={"Nepi_IF_Config"}
        />
        : null}
            
        </React.Fragment>
      )

  }




  render() {
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true

    const status_msg = this.state.status_msg
    const { userRestricted} = this.props.ros
    const ignore_restrictions = (this.props.ignore_restrictions !== undefined) ? this.props.ignore_restrictions : false
    const process_view_restricted = userRestricted.indexOf('SYSTEM-PROCESS-VIEW') !== -1 && (ignore_restrictions === false)
    if (status_msg == null || process_view_restricted === true){
      return (
        <Columns>
        <Column>
       
        </Column>
        </Columns>
      )


    }
    else if (make_section === false){

      return (

          <React.Fragment>

               {this.renderProcess()}


          </React.Fragment>
      )
    }
    else {
      return (

          <Section title={(this.props.title !== undefined) ? this.props.title : "Process"}>

              {this.renderProcess()}


        </Section>
     )
   }

  }

}
export default Nepi_IF_ConnectProcess
