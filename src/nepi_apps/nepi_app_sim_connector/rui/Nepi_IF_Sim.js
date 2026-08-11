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
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"
import { Columns, Column } from "./Columns"
import { round } from "./Utilities"

import NepiIFSimControls from "./Nepi_IF_Sim-Controls"
import NepiIFSimLauncher from "./Nepi_IF_SimLauncher"

// Downloadable reference for the "Upload Robot Config" option below --
// every field a robot_configs entry understands (see
// sim_connector_app_params.yaml's own robot_configs for the checked-in
// equivalents), commented so an operator can see what each one does without
// cross-referencing the app source. Kept as one flat mapping, matching what
// uploadRobotConfigCb on the device actually expects: the raw fields, not a
// name-wrapped entry -- display_name inside the file IS the name.
const SAMPLE_ROBOT_CONFIG_YAML =
`# Sample NEPI Sim Connector robot config.
#
# Edit the values below to match your own robot, then use "Upload Robot
# Config" here to try it out -- it appears in the Robot Config selector as
# whatever display_name you set below, and is applied immediately.
#
# Every field is optional; anything you omit is treated as capability-off
# (false / 0 / empty list), same as an incomplete entry in
# sim_connector_app_params.yaml.

# Shown in the Robot Config selector dropdown.
display_name: "Custom Test Rover"

# Free-text, informational only -- not shown in the selector.
description: "Example 2-wheel differential-drive rover, uploaded for testing."

# wheel_count is informational; motor_count sizes the motor-ratio controls.
wheel_count: 2
motor_count: 2

# Each has_* flag shows or hides the matching control in the RUI.
has_goto_position: true
has_goto_pose: false
has_goto_location: false
has_go_home: true
has_set_home: true
has_go_stop: true

# Actions run on setup / on a "go" trigger. RESET + RETURN_HOME is what every
# checked-in config uses; go_actions is empty on every current entry.
setup_actions:
  - RESET
  - RETURN_HOME
go_actions: []

# If true, the RUI offers a camera view mode selector populated from
# available_camera_view_modes.
has_camera_view_control: true
available_camera_view_modes:
  - SCENE_CAMERA
  - ROBOT_CAMERA

# Environment controls (e.g. lighting/weather toggles the sim bridge exposes).
has_environment_controls: true
`

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

    // Hidden <input type="file"> target for the Upload Robot Config button
    // -- a ref rather than state, since the input element itself is never
    // rendered differently; only clicked programmatically.
    this.uploadInputRef = React.createRef()

    this.getSimNamespace = this.getSimNamespace.bind(this)

    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

    this.onRobotConfigSelected = this.onRobotConfigSelected.bind(this)
    this.onUploadConfigClicked = this.onUploadConfigClicked.bind(this)
    this.onUploadConfigFileChange = this.onUploadConfigFileChange.bind(this)
    this.onDownloadSampleConfigClicked = this.onDownloadSampleConfigClicked.bind(this)

    this.renderRobotConfigSelector = this.renderRobotConfigSelector.bind(this)
    this.renderRobotConfigUpload = this.renderRobotConfigUpload.bind(this)
    this.renderFieldPair = this.renderFieldPair.bind(this)
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

  // Opens the hidden file picker below -- kept as a separate, visible Button
  // rather than styling the raw <input type="file"> itself, matching every
  // other control in this component.
  onUploadConfigClicked() {
    if (this.uploadInputRef.current != null) {
      this.uploadInputRef.current.click()
    }
  }

  // Reads the picked file as text and publishes it whole to
  // sim/upload_robot_config (std_msgs/String) -- the device parses it as
  // YAML, validates it, and applies it immediately on success (see
  // uploadRobotConfigCb). Any parse/validation failure is reported the same
  // way an unrecognized robot config selection already is: a pub_warn from
  // the device, not a round trip back through this component.
  onUploadConfigFileChange(event) {
    const file = (event.target.files && event.target.files.length > 0)
      ? event.target.files[0] : null
    event.target.value = ''  // allow re-picking the same file next time
    const namespace = this.getSimNamespace()
    if (file == null || namespace == null || namespace === 'None') {
      return
    }
    const reader = new FileReader()
    reader.onload = () => {
      this.props.ros.sendStringMsg(namespace + '/upload_robot_config', String(reader.result))
    }
    reader.readAsText(file)
  }

  // Client-side only -- SAMPLE_ROBOT_CONFIG_YAML is a static reference file,
  // not device state, so there is nothing to fetch over ROS for this.
  onDownloadSampleConfigClicked() {
    const blob = new Blob([SAMPLE_ROBOT_CONFIG_YAML], { type: 'text/yaml' })
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a')
    link.href = url
    link.download = 'sample_robot_config.yaml'
    document.body.appendChild(link)
    link.click()
    document.body.removeChild(link)
    URL.revokeObjectURL(url)
  }

  // Robot config selector, backed by the status message's reported list of named
  // robot configs. Selecting one tells the simulator which kind of robot is
  // wanted. Option value is always the raw config key (what select_robot_config
  // actually takes, and what a bridge script matches against on the wire);
  // available_robot_config_names is only ever used for the label shown, the
  // same reported-list-plus-names shape the simulator selector above uses --
  // so this can show "Quadcopter" without any wire-protocol string changing.
  renderRobotConfigSelector() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }

    const available = (status_msg.available_robot_configs !== undefined)
      ? status_msg.available_robot_configs : []
    const names = (status_msg.available_robot_config_names !== undefined)
      ? status_msg.available_robot_config_names : []
    const selected = (status_msg.selected_robot_config !== undefined
                      && status_msg.selected_robot_config !== '')
      ? status_msg.selected_robot_config : 'None'

    var items = []
    if (available.length === 0) {
      items.push(<Option key={'None'} value={'None'}>{'None'}</Option>)
    }
    for (var i = 0; i < available.length; i++) {
      const display = (names[i] !== undefined && names[i] !== '') ? names[i] : available[i]
      items.push(<Option key={available[i]} value={available[i]}>{display}</Option>)
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

  // Upload-your-own-robot option, offered right alongside the Robot Config
  // selector above rather than buried somewhere else -- uploading one both
  // adds it to that selector (as whatever display_name it declares) and
  // selects it immediately, so this and the selector are really one choice,
  // not two features. Download Sample gives a concrete, correctly-shaped
  // starting point instead of requiring a reference to the schema
  // documented anywhere else.
  renderRobotConfigUpload() {
    return (
      <React.Fragment>
        <input
          type="file"
          accept=".yaml,.yml,text/yaml"
          ref={this.uploadInputRef}
          style={{ display: 'none' }}
          onChange={this.onUploadConfigFileChange}
        />
        <ButtonMenu>
          <Button onClick={this.onUploadConfigClicked}>{"Upload Robot Config"}</Button>
          <Button onClick={this.onDownloadSampleConfigClicked}>{"Download Sample Config"}</Button>
        </ButtonMenu>
      </React.Fragment>
    )
  }

  // Puts two Label fields side by side instead of each taking a full-width
  // row on its own -- most of these values are a word, a number, or a single
  // indicator square, so stacking them one per row (the default Label
  // layout) leaves most of a wide panel's width empty. Label already splits
  // its own container 50/50 between title and value, so two Labels each
  // given ~49% of a shared flex row keeps that same title:value ratio
  // within each half rather than skewing it.
  renderFieldPair(fieldA, fieldB) {
    return (
      <div style={{ display: "flex" }}>
        <div style={{ width: "49%" }}>{fieldA}</div>
        <div style={{ width: "2%" }} />
        <div style={{ width: "49%" }}>{fieldB}</div>
      </div>
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

        {this.renderFieldPair(
          <Label title={"Device Name"}>
            <Input disabled value={status_msg.device_name} />
          </Label>,
          <Label title={"Bridge Connected"}>
            <BooleanIndicator value={status_msg.bridge_connected} />
          </Label>
        )}

        {this.renderFieldPair(
          <Label title={"Telemetry Age (s)"}>
            <Input disabled value={round(status_msg.telemetry_age_sec + .001, 2)} />
          </Label>,
          <Label title={"Ready"}>
            <BooleanIndicator value={status_msg.ready} />
          </Label>
        )}

        {this.renderFieldPair(
          <Label title={"Current Process"}>
            <Input disabled value={status_msg.process_current} />
          </Label>,
          <Label title={"Last Process"}>
            <Input disabled value={status_msg.process_last} />
          </Label>
        )}

        {this.renderFieldPair(
          <Label title={"Last Cmd Success"}>
            <BooleanIndicator value={status_msg.cmd_success} />
          </Label>,
          <Label title={"Sensor Topics"}>
            <Input disabled value={String(sensor_topics.length)} />
          </Label>
        )}

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
            {/* NepiIFSimLauncher's own target selector below IS the
                Simulator selector for this whole panel -- it already lists
                every real option (Gazebo/Webots/Stage/PyBullet/WPILib).
                This component used to render a SECOND "Simulator" selector
                here, backed by available_simulators/select_simulator (live
                discovery of OTHER NEPI devices that declare themselves
                simulators -- see getAvailableSimulators/simDiscoveryCb in
                sim_connector_app_node.py, a deliberately different axis
                from the SSH launch-target list). That mechanism stays
                intact server-side for a possible future use, but nothing
                on this deployment ever populates it, so showing it here
                just produced two same-titled, mostly-empty-vs-real
                "Simulator" fields stacked on top of each other -- removed
                rather than merged, since the two lists mean genuinely
                different things (already-connected device vs.
                launch-this-on-the-VM) and forcing them into one dropdown
                would misrepresent both. */}
            {this.renderRobotConfigSelector()}
            {this.renderRobotConfigUpload()}
            {/* Deploy/Kill/Install for the additive simulator auto-launch
                capability (see docs/SIMULATOR_AUTO_LAUNCH_PLAN.md) -- lives
                directly under Robot Config rather than as its own titled
                section, since choosing a model above and clicking Deploy
                here is one flow, not two. make_section={false} is
                NepiIFSimLauncher's own default now, but named explicitly
                here since this IS the one place it's meant to render. */}
            <NepiIFSimLauncher
              namespace={namespace}
              make_section={false}
            />
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
