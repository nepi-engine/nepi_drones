/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
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
import Select, { Option } from "./Select"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"
import { Columns, Column } from "./Columns"
import { round } from "./Utilities"

import NepiIFSimControls from "./Nepi_IF_Sim-Controls"

@inject("ros")
@observer

// Reusable component for a simulated device hosted by a sim connector node.
// Owns the two selectors, the status and info display, and composition of the
// controls child. Modeled on the connect-app IF components: this component holds
// the one status subscription its own widgets need and passes the device
// namespace down as a prop, and the controls child owns its own subscriptions.
//
// Both selectors are populated entirely from live reported lists on the device's
// status message, so neither this component nor the page above it knows the name
// of any simulator, world, or robot model. An empty list renders an empty
// selector rather than blocking, and the selectors are never gated on something
// already being selected -- gating them would deadlock the page, since with no
// selector nothing could be chosen.
class NepiIFSim extends Component {
  constructor(props) {
    super(props)

    this.state = {

      // Sim device namespace (<app>/sim), from the namespace prop
      namespace: null,

      // SimStatus from that namespace -- the only status this component
      // subscribes to. The controls child owns its own.
      status_msg: null,
      statusListener: null,

    }

    this.getSimNamespace = this.getSimNamespace.bind(this)

    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

    this.onSimulatorSelected = this.onSimulatorSelected.bind(this)
    this.onRobotConfigSelected = this.onRobotConfigSelected.bind(this)

    this.renderSimulatorSelector = this.renderSimulatorSelector.bind(this)
    this.renderRobotConfigSelector = this.renderRobotConfigSelector.bind(this)
    this.renderData = this.renderData.bind(this)
  }

  // Resolve the sim device namespace from the namespace prop
  getSimNamespace() {
    return (this.props.namespace !== undefined) ? this.props.namespace : null
  }

  componentDidMount() {
    this.updateStatusListener()
  }

  // Lifecycle method called when the component updates.
  // Re-point the status listener when the namespace prop changes.
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getSimNamespace()
    if (namespace !== this.state.namespace) {
      this.updateStatusListener()
    }
  }

  // Lifecycle method called just before the component unmounts.
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    this.setState({ statusListener: null })
  }

  // Function for configuring and subscribing to the sim namespace status topic
  // (<app>/sim/status), message type SimStatus.
  updateStatusListener() {
    const namespace = this.getSimNamespace()
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null, status_msg: null })
    }
    if (namespace != null && namespace !== 'None') {
      var statusListener = this.props.ros.setupStatusListener(
        namespace + '/status',
        "nepi_app_sim_connector/SimStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ namespace: namespace })
  }

  // Callback for SimStatus messages.
  statusListener(message) {
    this.setState({ status_msg: message })
  }

  // Handler for the simulator Select. Publishes a std_msgs/String to the sim
  // namespace select_simulator topic.
  onSimulatorSelected(event) {
    const namespace = this.getSimNamespace()
    if (namespace != null && namespace !== 'None') {
      this.props.ros.sendStringMsg(namespace + '/select_simulator', event.target.value)
    }
  }

  // Handler for the robot config Select. Publishes a std_msgs/String to the sim
  // namespace select_robot_config topic. Selecting a config selects a kind of
  // robot, so the device re-derives and republishes its capability report --
  // which is why the controls child re-queries capabilities when the reported
  // selected config changes.
  onRobotConfigSelected(event) {
    const namespace = this.getSimNamespace()
    if (namespace != null && namespace !== 'None') {
      this.props.ros.sendStringMsg(namespace + '/select_robot_config', event.target.value)
    }
  }

  // Simulator selector, backed by the status message's reported lists. Populated
  // from available_simulators / available_simulator_names, both of which the
  // device fills by scanning live ROS state for devices that declare themselves
  // simulators. An empty list means no simulator is running right now; the
  // selector still renders, showing only None.
  renderSimulatorSelector() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }

    const available = (status_msg.available_simulators !== undefined)
      ? status_msg.available_simulators : []
    const names = (status_msg.available_simulator_names !== undefined)
      ? status_msg.available_simulator_names : []
    const selected = (status_msg.selected_simulator !== undefined && status_msg.selected_simulator !== '')
      ? status_msg.selected_simulator : 'None'

    var items = []
    items.push(<Option key={'None'} value={'None'}>{'None'}</Option>)
    for (var i = 0; i < available.length; i++) {
      const display = (names[i] !== undefined && names[i] !== '') ? names[i] : available[i]
      items.push(<Option key={available[i]} value={available[i]}>{display}</Option>)
    }

    return (
      <Label title={"Simulator"}>
        <Select
          onChange={this.onSimulatorSelected}
          value={selected}
        >
          {items}
        </Select>
      </Label>
    )
  }

  // Robot config selector, backed by the status message's reported list of named
  // robot configs. Selecting one tells the simulator which kind of robot is
  // wanted.
  renderRobotConfigSelector() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }

    const available = (status_msg.available_robot_configs !== undefined)
      ? status_msg.available_robot_configs : []
    const selected = (status_msg.selected_robot_config !== undefined
                      && status_msg.selected_robot_config !== '')
      ? status_msg.selected_robot_config : 'None'

    var items = []
    if (available.length === 0) {
      items.push(<Option key={'None'} value={'None'}>{'None'}</Option>)
    }
    for (var i = 0; i < available.length; i++) {
      items.push(<Option key={available[i]} value={available[i]}>{available[i]}</Option>)
    }

    return (
      <Label title={"Robot Config"}>
        <Select
          onChange={this.onRobotConfigSelected}
          value={selected}
        >
          {items}
        </Select>
      </Label>
    )
  }

  // Read-only status and info display, backed by SimStatus. bridge_connected and
  // telemetry_age_sec are shown as two separate readings on purpose: a connected
  // bridge whose telemetry has gone stale (a paused simulator, a hung callback)
  // is a different condition from a disconnected one, and only showing both makes
  // that distinguishable.
  renderData() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }

    const sensor_topics = (status_msg.available_sensor_topics !== undefined)
      ? status_msg.available_sensor_topics : []

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={"Device Name"}>
          <Input disabled value={status_msg.device_name} />
        </Label>

        <Label title={"Bridge Connected"}>
          <BooleanIndicator value={status_msg.bridge_connected} />
        </Label>

        <Label title={"Telemetry Age (s)"}>
          <Input disabled value={round(status_msg.telemetry_age_sec + .001, 2)} />
        </Label>

        <Label title={"Ready"}>
          <BooleanIndicator value={status_msg.ready} />
        </Label>

        <Label title={"Current Process"}>
          <Input disabled value={status_msg.process_current} />
        </Label>

        <Label title={"Last Process"}>
          <Input disabled value={status_msg.process_last} />
        </Label>

        <Label title={"Last Cmd Success"}>
          <BooleanIndicator value={status_msg.cmd_success} />
        </Label>

        <Label title={"Sensor Topics"}>
          <Input disabled value={String(sensor_topics.length)} />
        </Label>

        <Label title={"Last Error"}>
          <Input disabled value={status_msg.last_error_message} />
        </Label>

      </React.Fragment>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const title = (this.props.title !== undefined) ? this.props.title : "Sim Connector"
    const namespace = this.getSimNamespace()
    const status_msg = this.state.status_msg

    // No status yet: render nothing, matching the connect-app IF components'
    // not-ready branch.
    if (status_msg == null) {
      return (
        <Columns>
          <Column>

          </Column>
        </Columns>
      )
    }

    // Section visibility resolves prop-overrides-default, the same defaulting
    // the connect-app IF components use for their show_* props.
    const show_selectors = (this.props.show_selectors !== undefined) ? this.props.show_selectors : true
    const show_data = (this.props.show_data !== undefined) ? this.props.show_data : true
    const show_controls = (this.props.show_controls !== undefined) ? this.props.show_controls : true

    const content = (
      <React.Fragment>

        {(show_selectors === true) ?
          <React.Fragment>
            {this.renderSimulatorSelector()}
            {this.renderRobotConfigSelector()}
          </React.Fragment>
        : null}

        {(show_data === true) ?
          this.renderData()
        : null}

        {(show_controls === true) ?
          <NepiIFSimControls
            namespace={namespace}
            make_section={false}
          />
        : null}

      </React.Fragment>
    )

    if (make_section === false) {
      return (
        <React.Fragment>
          {content}
        </React.Fragment>
      )
    }
    else {
      return (
        <Section title={title}>
          {content}
        </Section>
      )
    }
  }

}

export default NepiIFSim
