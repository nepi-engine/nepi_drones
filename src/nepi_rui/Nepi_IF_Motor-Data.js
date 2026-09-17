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
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"
import { Column, Columns } from "./Columns"
import { round } from "./Utilities"

@inject("ros")
@observer

// Read-only motor device data component. Subscribes to the device's MotorsStatus
// on the namespace prop (namespace/motor_status) and renders telemetry only. No
// command publishers, no editable inputs. The companion Nepi_IF_Motor-Controls
// component owns every command widget for the same device.
//
// Motor devices have no dedicated store listener setup method and no
// capabilities dictionary, so the subscription is made through the generic
// setupStatusListener.
//
// One row per motor, each with a divider and the bold motor_name header. The
// pre-split Nepi_IF_ConnectMotor placed a motor's read-only cells and its
// command cells side by side in adjacent columns of a single row; with data and
// controls in separate components those become two stacked blocks, each with its
// own per-motor rows.
class NepiIFMotorData extends Component {
  constructor(props) {
    super(props)

    // these states track the values through MotorsStatus messages
    this.state = {

      namespace: 'None',
      status_msg: null,

      statusListener: null,

    }

    this.renderMotorStatus = this.renderMotorStatus.bind(this)
    this.renderMotorRow = this.renderMotorRow.bind(this)
    this.renderData = this.renderData.bind(this)

    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

  }

  // Callback for handling ROS MotorsStatus messages. Nothing in this component
  // is editable, so the incoming status is tracked directly.
  statusListener(message) {
    this.setState({ status_msg: message })
  }

  // Function for configuring and subscribing to the device's motor status topic
  // (namespace/motor_status), message type MotorsStatus.
  updateStatusListener() {
    const { namespace } = this.props
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({ status_msg: null, statusListener: null})
    }
    if (namespace != null && namespace !== 'None'){
      const statusNamespace = namespace + '/motor_status'
      var statusListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_interfaces/MotorsStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener})
    }
    this.setState({ namespace: namespace})

  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { namespace } = this.props
    if (namespace !== this.state.namespace){
      if (namespace !== null) {
        this.updateStatusListener()
      }
    }
  }

  componentDidMount() {
    this.updateStatusListener()
    }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to MotorsStatus message
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
  }


  // Read-only telemetry fields for a single motor, backed by one MotorStatus
  // element of the MotorsStatus array. No divider or motor-name header (the row
  // that hosts this supplies those); no command publishers here.
  //
  // The rotatable-wheel (steering) rows render only for a motor reporting
  // has_rotation true, and the rotation limit rows only where that rotation is
  // limited (has_continuous_rotation false). A non-steering motor's rows are
  // unchanged.
  renderMotorStatus(motor) {
    const motor_enable = motor.motor_enable
    const motor_dir = motor.motor_dir
    const motor_max_speed = round(motor.motor_max_speed + .001, 2)
    const motor_speed_ratio = round(motor.motor_speed_ratio + .001, 2)
    const motor_speed = round(motor.motor_speed + .001, 2)
    const motor_position = round(motor.motor_position + .001, 2)

    const has_rotation = (motor.has_rotation === true)
    const has_continuous_rotation = (motor.has_continuous_rotation === true)
    const rotation_min_limit = round(motor.rotation_min_limit + .001, 2)
    const rotation_max_limit = round(motor.rotation_max_limit + .001, 2)
    const rotation_max_speed = round(motor.rotation_max_speed + .001, 2)
    const rotation_speed_ratio = round(motor.rotation_speed_ratio + .001, 2)
    const rotation = round(motor.rotation + .001, 2)

    return (
      <React.Fragment>

        <Label title={"Enabled"}>
          <BooleanIndicator value={motor_enable} />
        </Label>

        <Label title={"Direction"}>
          <Input disabled value={motor_dir} />
        </Label>

        <Label title={"Max Speed (dps)"}>
          <Input disabled value={motor_max_speed} />
        </Label>

        <Label title={"Speed Ratio (0-1)"}>
          <Input disabled value={motor_speed_ratio} />
        </Label>

        <Label title={"Speed (dps)"}>
          <Input disabled value={motor_speed} />
        </Label>

        <Label title={"Position (deg)"}>
          <Input disabled value={motor_position} />
        </Label>

        <div hidden={(has_rotation === false)}>

          <Label title={"Continuous Rotation"}>
            <BooleanIndicator value={has_continuous_rotation} />
          </Label>

          <div hidden={(has_continuous_rotation === true)}>

            <Label title={"Rotation Min Limit (deg)"}>
              <Input disabled value={rotation_min_limit} />
            </Label>

            <Label title={"Rotation Max Limit (deg)"}>
              <Input disabled value={rotation_max_limit} />
            </Label>

          </div>

          <Label title={"Rotation Max Speed (dps)"}>
            <Input disabled value={rotation_max_speed} />
          </Label>

          <Label title={"Rotation Speed Ratio (0-1)"}>
            <Input disabled value={rotation_speed_ratio} />
          </Label>

          <Label title={"Rotation (deg)"}>
            <Input disabled value={rotation} />
          </Label>

        </div>

      </React.Fragment>
    )
  }

  // One row per motor: a divider, the bold motor_name header, then that motor's
  // read-only status.
  renderMotorRow(motor, index) {
    const motor_name = (motor.motor_name !== undefined && motor.motor_name !== '') ? motor.motor_name : ('motor_' + index)

    return (
      <React.Fragment key={motor_name + '_data_row'}>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={motor_name} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>

        <Columns>
          <Column>
            {this.renderMotorStatus(motor)}
          </Column>
        </Columns>

      </React.Fragment>
    )
  }

  // Read-only device telemetry, backed by MotorsStatus. Renders the device
  // header (device name / motor count) and one row per motor.
  renderData() {
    const status_msg = this.state.status_msg
    if (status_msg == null || status_msg.motors == null) {
      return (
        <Columns>
          <Column>

          </Column>
        </Columns>
      )
    }

    const device_name = status_msg.device_name
    const motors = status_msg.motors

    return (
      <React.Fragment>

        <Label title={"Device Name"}>
          <Input disabled value={device_name} />
        </Label>

        <Label title={"Motor Count"}>
          <Input disabled value={motors.length} />
        </Label>

        { motors.map((motor, index) => this.renderMotorRow(motor, index)) }

      </React.Fragment>
    )
  }


  render() {
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true
    const status_msg = this.state.status_msg
    if (status_msg == null){
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

                    {this.renderData()}

          </React.Fragment>
      )
    }
    else {
      return (

          <Section title={(this.props.title !== undefined) ? this.props.title : null}>

              {this.renderData()}

        </Section>
     )
    }
  }

}
export default NepiIFMotorData
