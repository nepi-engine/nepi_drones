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
//import EnableAdjustment from "./EnableAdjustment"
import Toggle from "react-toggle"
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Input from "./Input"
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"




import { onChangeSwitchStateValue, convertStrToStrList} from "./Utilities"
import {Queue} from "./Utilities"



@inject("ros")
@observer

// Component that contains RBX Controls
class NepiRBXMessages extends Component {
  constructor(props) {
    super(props)

    // these states track the values through  Status messages
    this.state = {


      show_messages: false,

      current_lat: null,
      current_long: null,
      current_altitude: null,
      current_heading: null,
      current_yaw: null,
      process_current: null,
      process_last: null,
      ready: null,
      battery: null,
      errors_current: null,
      errors_prev: null,
      cmd_success: null,
      manual_ready: null,
      autonomous_ready: null,
      last_cmd_str: null,
      last_error_message: null,
      msg_queue_size: 50,
      status_msg: null,

      has_battery_feedback: null,

      MessagesStatusListener : null,
      MessagesStatusMsgListener : null,
      MessagesStatusStrListener: null,
      navposeListener: null,

      status_str_list: null
    }

    this.msg_queue = new Queue()

    this.updateMessagesStatusListener = this.updateMessagesStatusListener.bind(this)
    this.updateMessagesStatusMsgListener = this.updateMessagesStatusMsgListener.bind(this)


    this.MessagesStatusListener = this.MessagesStatusListener.bind(this)
    this.MessagesStatusMsgListener = this.MessagesStatusMsgListener.bind(this)
    this.MessagesStatusStrListener = this.MessagesStatusStrListener.bind(this)
    this.updateMessagesStatusStrListener = this.updateMessagesStatusStrListener.bind(this)
    this.updateNavposeListener = this.updateNavposeListener.bind(this)
    this.navposeListener = this.navposeListener.bind(this)
    this.convertMsgListToStr = this.convertMsgListToStr.bind(this)
    this.convertStrListToJoinedStr = this.convertStrListToJoinedStr.bind(this)
    

    this.onEnterMessagesQueueVar = this.onEnterMessagesQueueVar.bind(this)
    this.onUpdateMessagesInputBoxValue = this.onUpdateMessagesInputBoxValue.bind(this)

  }



  // Callback for handling ROS DeviceRBXStatus messages.
  // Pose values (current_*) are not part of DeviceRBXStatus; they come from the
  // NavPose topic via navposeListener below.
  MessagesStatusListener(message) {
    this.setState({
      process_current: message.process_current ,
      process_last: message.process_last ,
      ready: message.ready ,
      battery: message.battery ,

      errors_current: [message.errors_current.x_m ,message.errors_current.y_m, message.errors_current.z_m, message.errors_current.heading_deg, message.errors_current.roll_deg, message.errors_current.pitch_deg, message.errors_current.yaw_deg],
      errors_prev: [message.errors_prev.x_m ,message.errors_prev.y_m, message.errors_prev.z_m, message.errors_prev.heading_deg, message.errors_prev.roll_deg, message.errors_prev.pitch_deg, message.errors_prev.yaw_deg],
      cmd_success: message.cmd_success ,
      manual_ready: message.manual_control_mode_ready ,
      autonomous_ready: message.autonomous_control_mode_ready ,
      last_cmd_str: message.last_cmd_string ,
      last_error_message: message.last_error_message
    })


  }

  // Callback for handling NavPose messages. The system/info panel pose values
  // come from the device's NavPose topic. Robots with no WGS84 reference
  // (this.props.is_local_frame) report position as native ENU x_m/y_m/z_m
  // instead of latitude/longitude/altitude_m -- current_lat/long/altitude
  // are reused to carry whichever pair is meaningful for this device, and
  // render() below relabels them accordingly.
  navposeListener(message) {
    this.setState({
      current_lat: this.props.is_local_frame ? message.x_m : message.latitude ,
      current_long: this.props.is_local_frame ? message.y_m : message.longitude ,
      current_altitude: this.props.is_local_frame ? message.z_m : message.altitude_m ,
      current_heading: message.heading_deg ,
      current_yaw: message.yaw_deg
    })
  }

  MessagesStatusMsgListener(message) {
    const msg_str = message.data + "\n"
    this.msg_queue.pushItem(msg_str)
    var q_len = this.msg_queue.getLength()
    while (q_len > this.state.msg_queue_size){
      q_len = this.msg_queue.getLength()
      this.msg_queue.pullItem()
    }
    const msg_list = this.msg_queue.getItemsReversed()
    const msg_str_join = msg_list.join("")
    this.setState({status_msg : msg_str_join})
  }

  MessagesStatusStrListener(message) {
    const status_str = message.data
    const status_str_list = convertStrToStrList(status_str)
    this.setState({status_str_list: status_str_list})
  }
  

  // Function for configuring and subscribing to Status
  updateMessagesStatusListener() {
    const Namespace = this.props.rbxNamespace
    if (this.state.MessagesStatusListener ) {
      this.state.MessagesStatusListener.unsubscribe()
    }
    var listener = this.props.ros.setupStatusListener(
          Namespace + "/status",
          "nepi_interfaces/DeviceRBXStatus",
          this.MessagesStatusListener
        )
    this.setState({ MessagesStatusListener : listener})
      }


    // Function for configuring and subscribing to the device NavPose topic
    updateNavposeListener() {
      const navposeTopic = this.props.rbxNamespace.split('/rbx')[0] + "/npx/navpose"
    if (this.state.navposeListener ) {
      this.state.navposeListener.unsubscribe()
    }
    var navposeListener = this.props.ros.setupStatusListener(
          navposeTopic,
          "nepi_interfaces/NavPose",
          this.navposeListener
        )
    this.setState({ navposeListener : navposeListener})
     }


    updateMessagesStatusMsgListener() {
      const Namespace = this.props.rbxNamespace
    if (this.state.MessagesStatusMsgListener ) {
      this.state.MessagesStatusMsgListener.unsubscribe()
    }
    var msglistener = this.props.ros.setupStringListener(
          Namespace + "/status_msg",
          this.MessagesStatusMsgListener
        )
    this.setState({ MessagesStatusMsgListener : msglistener})
     }
    
     updateMessagesStatusStrListener() {
      const Namespace = this.props.rbxNamespace
    if (this.state.MessagesStatusStrListener ) {
      this.state.MessagesStatusStrListener.unsubscribe()
    }
    var statusListener = this.props.ros.setupStringListener(
          Namespace + "/status_str",
          this.MessagesStatusStrListener
        )
    this.setState({ MessagesStatusStrListener : statusListener})
     }
  

  // Lifecycle method called on mount. Sets up listeners for the initially
  // selected device.
  componentDidMount() {
    const { rbxNamespace } = this.props
    if (rbxNamespace && rbxNamespace.indexOf('null') === -1) {
      this.updateMessagesStatusMsgListener()
      this.updateMessagesStatusListener()
      this.updateMessagesStatusStrListener()
      this.updateNavposeListener()
      this.msg_queue = new Queue()
    }
  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { rbxNamespace } = this.props
    if (prevProps.rbxNamespace !== rbxNamespace && rbxNamespace !== null) {
      if (rbxNamespace.indexOf('null') === -1){
        this.updateMessagesStatusMsgListener()
        this.updateMessagesStatusListener()
        this.updateMessagesStatusStrListener()
        this.updateNavposeListener()
        this.msg_queue = new Queue()
        this.render()
      }
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.MessagesStatusListener ) {
      this.state.MessagesStatusListener.unsubscribe()
    }
    if (this.state.MessagesStatusMsgListener ) {
      this.state.MessagesStatusMsgListener.unsubscribe()
    }
    if (this.state.MessagesStatusStrListener ) {
      this.state.MessagesStatusStrListener.unsubscribe()
    }
    if (this.state.navposeListener ) {
      this.state.navposeListener.unsubscribe()
    }
  }


  onUpdateMessagesInputBoxValue(event,stateVarStr) {
    var key = stateVarStr
    var value = event.target.value
    var obj  = {}
    obj[key] = value
    this.setState(obj)
    document.getElementById(event.target.id).style.color = Styles.vars.colors.red
    this.render()
  }

  onEnterMessagesQueueVar(event) {
    if(event.key === 'Enter'){
      var value = parseInt(event.target.value, 10)
      if (!isNaN(value)){
        if (value <10){
          value=10
        }
        else if (value > 100){
          value=100
        }
        this.setState({msg_queue_size: value})
      }
      document.getElementById(event.target.id).style.color = Styles.vars.colors.black
    }
  }

  convertMsgListToStr(){
    const msg_list = this.msg_queue.getItems()
    const msg_str = msg_list.join("")
    return msg_str
  }

  convertStrListToJoinedStr(str_list) {
    var mod_str_list = []
    for (var i = 0; i < str_list.length; ++i) {
      mod_str_list.push(str_list[i]+"\n")
    }
    const joined_str = mod_str_list.join("")
    return joined_str

  }

  
  render() {
    return (
      <Section title={"System Informaion"}>

            <Columns>
            <Column>

            <Label title={"Current Battery"}>
              <Input 
              disabled value={this.state.battery}
                id="current_battery"
              />
            </Label>

            </Column>
            <Column>

            </Column>
            </Columns>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

            <label style={{fontWeight: 'bold'}}>
              {"Nav Pose"}
            </label>

            <Columns>
            <Column>

            <Label title={this.props.is_local_frame ? "X (m)" : "Latitude"}>
              <Input
                disabled value={this.state.current_lat}
                id="InitLatitude"
              />
            </Label>

            <Label title={this.props.is_local_frame ? "Y (m)" : "Longitude"}>
              <Input
                disabled value={this.state.current_long}
                id="InitLongitude"
              />
            </Label>

            <Label title={this.props.is_local_frame ? "Z (m)" : "Altitude (m)"}>
              <Input
                disabled value={this.state.current_altitude}
                id="InitAltitude"
              />
            </Label>

            </Column>
            <Column>

              <Label title={"Yaw (deg)"}>
                <Input
                  disabled value={this.state.current_yaw}
                  id="InitYaw"
                />
              </Label>

              <div hidden={!this.props.has_set_home}>
                <Label title={""}></Label>
                <ButtonMenu>
                  <Button onClick={() => this.props.ros.sendTriggerMsg(this.props.rbxNamespace + "/set_home_current")}>{"Set Home"}</Button>
                </ButtonMenu>
              </div>

              </Column>
              </Columns>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

            <Columns>
            <Column>
            
            <Label title="Show System Messages">
                    <Toggle
                      checked={this.state.show_messages===true}
                      onClick={() => onChangeSwitchStateValue.bind(this)("show_messages",this.state.show_messages)}>
                    </Toggle>
              </Label>

            </Column>
            <Column>
            </Column>
            </Columns>


          <div hidden={!this.state.show_messages}>

            <Label title={"Last Command"} >
          </Label>
          <pre style={{ height: "25px", overflowY: "auto" }}>
            {this.state.last_cmd_str}
          </pre>

          <Label title={"Last Error"} >
          </Label>
          <pre style={{ height: "25px", overflowY: "auto" }}>
            {this.state.last_error_message}
          </pre>


            <div align={"left"} textAlign={"left"}> 
        <label style={{fontWeight: 'bold'}}>
          {"RBX Status"}
        </label>
          <pre style={{ height: "600px", overflowY: "auto" }}>
            {this.state.status_str_list ? this.convertStrListToJoinedStr(this.state.status_str_list) : ""}
          </pre>
          </div>


              </div>
      </Section>
    )
  }

}
export default NepiRBXMessages
