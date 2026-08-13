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
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import { Column, Columns } from "./Columns"
import { SliderAdjustment } from "./AdjustmentWidgets"
import { setElementStyleModified, clearElementStyleModified } from "./Utilities"

import NepiIFImageViewer from "./Nepi_IF_ImageViewer"

@inject("ros")
@observer

// Command component for a simulated device. Subscribes to the device's SimStatus
// on the namespace prop, and calls the capabilities query once the namespace
// resolves so it knows which controls exist.
//
// Every control here is rendered purely from a capability flag. Nothing is
// hardcoded to a particular simulator, robot, or world. When every flag is false
// and every reported list is empty, this component renders no controls at all --
// that is the intended behavior for a device that declares no capabilities, not a
// bug to work around.
//
// The capabilities query is re-issued whenever the reported selected robot config
// changes, because selecting a robot config selects a kind of robot, and a
// robot's kind is exactly what the capability flags describe.
class NepiIFSimControls extends Component {
  constructor(props) {
    super(props)

    this.state = {

      namespace: 'None',
      status_msg: null,
      statusListener: null,

      // The RBX namespace last resolved from status_msg.selected_simulator --
      // see the rig-offset block below for why this is tracked separately from
      // status_msg itself.
      rbx_namespace: '',

      // SimInfo is latched and is the authoritative source for the two
      // selections that are not per-tick telemetry: which image topic is
      // streaming and which camera view mode is set. Reading them here rather
      // than off the cached capabilities snapshot means they cannot go stale
      // between capability queries.
      info_msg: null,
      infoListener: null,
      selected_view_mode: '',

      // Cached capabilities response. The capability flags are decided by the
      // device, not by this component.
      capabilities: null,
      // Which selected_robot_config the cached capabilities were fetched for, so
      // a config change triggers exactly one re-query.
      capabilities_for_config: null,

      // Locally tracked motor slider values (percent), so dragging is instant
      // rather than waiting on a status round trip.
      motor_slider_values: [],
      motor_slider_text: [],

      // Edit buffers for the goto fields.
      gotoPositionX: '',
      gotoPositionY: '',
      gotoPositionZ: '',
      gotoPositionYaw: '',
      gotoPoseRoll: '',
      gotoPosePitch: '',
      gotoPoseYaw: '',
      gotoLocationLat: '',
      gotoLocationLong: '',
      gotoLocationAlt: '',
      gotoLocationYaw: '',

      // Selections held locally so a dropdown reflects the click immediately.
      selected_environment_option: 'None',

      // Camera rig positioning (robot-view/scene-view offset, and the
      // FIRST_PERSON/THIRD_PERSON toggle) is NOT part of the SimCapabilitiesQuery/
      // device_if_sim.py contract above -- it is published straight to whichever
      // RBX driver SimStatus.selected_simulator currently names, over that
      // driver's own generic Settings mechanism (camera_offset_x/y/z,
      // scene_offset_x/y/z, camera_view_mode -- see rbx_sim_node.py's
      // CAMERA_SETTING_NAMES). Two reasons this bypasses the capabilities-driven
      // controls above instead of going through a new device_if_sim.py-owned
      // capability:
      //   1. This is the ONLY delivery path to the actually-deployed simulator
      //      tonight. device_if_sim.py's OWN existing "Camera View Mode" selector
      //      (see has_view_control below) talks to sim_connector_bridge_gazebo.py
      //      over this app's port-9030 listener -- a separate, newer bridge
      //      script that generic_rover's currently-deployed launch_command does
      //      not start. Going through it here would render controls that publish
      //      into the void.
      //   2. Rig offset was previously rendered generically on NepiDeviceRBX.js
      //      (any RBX driver's Settings show up there) and has been moved here on
      //      request -- "the rbx viewer shouldn't know it's a simulation ... all
      //      sim specific commands should be set [in the sim connector]". Moving
      //      the UI does not change which mechanism it drives; it is still a
      //      Setting update on the RBX driver, same as it always was.
      rbxSettingsListener: null,
      rbxSettingsNamesList: [],
      rbxSettingsValuesDict: {},
      camera_offset_x: '',
      camera_offset_y: '',
      camera_offset_z: '',
      scene_offset_x: '',
      scene_offset_y: '',
      scene_offset_z: '',

    }

    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateInfoListener = this.updateInfoListener.bind(this)
    this.infoListener = this.infoListener.bind(this)
    this.queryCapabilities = this.queryCapabilities.bind(this)
    this.updateRbxSettingsListener = this.updateRbxSettingsListener.bind(this)
    this.rbxSettingsListener = this.rbxSettingsListener.bind(this)
    this.onEnterSetCameraOffset = this.onEnterSetCameraOffset.bind(this)

    this.onUpdateInput = this.onUpdateInput.bind(this)
    this.publishMotorRatio = this.publishMotorRatio.bind(this)
    this.turnOffAllMotors = this.turnOffAllMotors.bind(this)

    this.renderMotorControls = this.renderMotorControls.bind(this)
    this.renderGotoControls = this.renderGotoControls.bind(this)
    this.renderHomeControls = this.renderHomeControls.bind(this)
    this.renderCameraControls = this.renderCameraControls.bind(this)
    this.renderCameraOffsetControls = this.renderCameraOffsetControls.bind(this)
    this.renderCameraRigControls = this.renderCameraRigControls.bind(this)
    this.renderEnvironmentControls = this.renderEnvironmentControls.bind(this)
    this.renderControls = this.renderControls.bind(this)
  }

  // Callback for handling SimStatus messages.
  statusListener(message) {
    this.setState({ status_msg: message })
  }

  // Function for configuring and subscribing to SimStatus on the namespace prop.
  updateStatusListener() {
    const { namespace } = this.props
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({ status_msg: null, statusListener: null })
    }
    if (namespace != null && namespace !== 'None') {
      var statusListener = this.props.ros.setupStatusListener(
        namespace + '/status',
        "nepi_app_sim_connector/SimStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ namespace: namespace, capabilities: null, capabilities_for_config: null })
  }

  // Callback for handling SimInfo messages.
  infoListener(message) {
    this.setState({ info_msg: message })
    if (message.camera_view_mode !== this.state.selected_view_mode) {
      this.setState({ selected_view_mode: message.camera_view_mode })
    }
  }

  // Function for configuring and subscribing to SimInfo on the namespace prop.
  updateInfoListener() {
    const { namespace } = this.props
    if (this.state.infoListener != null) {
      this.state.infoListener.unsubscribe()
      this.setState({ info_msg: null, infoListener: null })
    }
    if (namespace != null && namespace !== 'None') {
      var infoListener = this.props.ros.setupStatusListener(
        namespace + '/info',
        "nepi_app_sim_connector/SimInfo",
        this.infoListener
      )
      this.setState({ infoListener: infoListener })
    }
  }

  // Fetch the cached capability report. Which controls exist is the device's
  // decision, served from a query rather than inferred from status fields.
  queryCapabilities(forConfig) {
    const namespace = this.props.namespace
    if (namespace == null || namespace === 'None') {
      return
    }
    this.setState({ capabilities_for_config: forConfig })
    // callService returns a Promise, or null when the service is not (yet) on
    // the graph. A null is not an error worth surfacing: the next config change
    // or namespace resolve re-queries.
    const request = this.props.ros.callService({
      name: namespace + '/capabilities_query',
      messageType: "nepi_app_sim_connector/SimCapabilitiesQuery"
    })
    if (request == null) {
      this.setState({ capabilities_for_config: null })
      return
    }
    request.then((response) => {
      if (response != null) {
        this.setState({ capabilities: response })
      }
    })
  }

  componentDidMount() {
    this.updateStatusListener()
    this.updateInfoListener()
  }

  // Lifecycle method called when the component updates. Re-points both listeners
  // when the namespace prop changes, and re-queries capabilities when the
  // namespace resolves or the selected robot config changes.
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { namespace } = this.props
    if (namespace !== this.state.namespace) {
      if (namespace !== null) {
        this.updateStatusListener()
        this.updateInfoListener()
      }
      return
    }

    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return
    }
    const config = status_msg.selected_robot_config
    if (config !== this.state.capabilities_for_config) {
      this.queryCapabilities(config)
    }

    // Re-point the RBX Settings listener whenever which simulator is selected
    // changes -- selected_simulator IS the RBX namespace (see
    // getAvailableSimulators in sim_connector_app_node.py: "the device namespace
    // is the status topic minus its trailing '/status'").
    const selected_simulator = (status_msg.selected_simulator !== undefined) ? status_msg.selected_simulator : ''
    if (selected_simulator !== prevState.rbx_namespace) {
      this.setState({ rbx_namespace: selected_simulator })
      this.updateRbxSettingsListener(selected_simulator)
    }

    // Seed the motor sliders once the motor count becomes known, and resize them
    // when a robot config change changes it.
    const motors = (status_msg.current_motor_control_settings !== undefined)
      ? status_msg.current_motor_control_settings : []
    if (motors.length !== this.state.motor_slider_values.length) {
      const seeded = motors.map((motor) => Math.round(motor.speed_ratio * 100))
      this.setState({
        motor_slider_values: seeded,
        motor_slider_text: seeded.map((v) => String(v))
      })
    }
  }

  // Lifecycle method called just before the component unmounts.
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    if (this.state.infoListener) {
      this.state.infoListener.unsubscribe()
    }
    if (this.state.rbxSettingsListener) {
      this.state.rbxSettingsListener.unsubscribe()
    }
  }

  // Callback for handling nepi_interfaces/SettingsStatus messages from the RBX
  // driver named by selected_simulator. Tracks the setting NAMES it actually
  // registers, so the rig-offset/view-mode controls gate on whether the
  // underlying Setting exists, the same way NepiDeviceRBX.js used to before this
  // control moved here.
  rbxSettingsListener(message) {
    const settings = (message.settings_list !== undefined) ? message.settings_list : []
    var namesList = []
    var valuesDict = {}
    for (let ind = 0; ind < settings.length; ind++) {
      namesList.push(settings[ind].name_str)
      valuesDict[settings[ind].name_str] = settings[ind].value_str
    }
    this.setState({ rbxSettingsNamesList: namesList, rbxSettingsValuesDict: valuesDict })

    // Seed/resync each offset edit buffer only when the DEVICE's own value
    // changed (or on first sight), never on every status tick -- otherwise a
    // 1Hz status message overwrites whatever is being typed.
    const offsetNames = ["camera_offset_x", "camera_offset_y", "camera_offset_z",
                        "scene_offset_x", "scene_offset_y", "scene_offset_z"]
    var updates = {}
    for (let i = 0; i < offsetNames.length; i++) {
      const name = offsetNames[i]
      const deviceVal = valuesDict[name]
      if (deviceVal !== undefined && deviceVal !== this.state.rbxSettingsValuesDict[name]) {
        updates[name] = deviceVal
      }
    }
    if (Object.keys(updates).length > 0) {
      this.setState(updates)
    }
  }

  // Function for configuring and subscribing to the selected RBX device's
  // settings/status. rbxNamespace is passed explicitly (rather than read back
  // off state) so the caller's own just-computed value is what gets subscribed,
  // avoiding a stale read the same render cycle it changed.
  updateRbxSettingsListener(rbxNamespace) {
    if (this.state.rbxSettingsListener) {
      this.state.rbxSettingsListener.unsubscribe()
      this.setState({ rbxSettingsListener: null, rbxSettingsNamesList: [] })
    }
    if (rbxNamespace !== null && rbxNamespace !== '' && rbxNamespace !== 'None') {
      var listener = this.props.ros.setupSettingsStatusListener(
        rbxNamespace + "/settings/status",
        this.rbxSettingsListener
      )
      this.setState({ rbxSettingsListener: listener })
    }
  }

  // Enter-to-apply for a camera offset input. Publishes the single Setting
  // being edited (Float) directly to the RBX driver named by selected_simulator
  // -- see the constructor comment on rbxSettingsListener for why this bypasses
  // the capabilities-driven controls elsewhere in this file.
  onEnterSetCameraOffset(event, settingName) {
    if (event.key === 'Enter') {
      const value = parseFloat(event.target.value)
      if (!isNaN(value)) {
        const { updateSetting } = this.props.ros
        updateSetting(this.state.rbx_namespace + "/settings", settingName, "Float", String(value))
      }
      const el = document.getElementById(event.target.id)
      if (el) {
        clearElementStyleModified(el)
      }
    }
  }

  // Editable-input change handler: mark the box modified while dirty and buffer
  // the typed value, per the RUI editable-input convention. Never sends on a
  // keystroke -- the commit happens on Enter.
  onUpdateInput(e, stateKey) {
    const el = document.getElementById(e.target.id)
    if (el) {
      setElementStyleModified(el)
    }
    this.setState({ [stateKey]: e.target.value })
  }

  // Publishes one motor's ratio (0-1) to set_motor_control.
  publishMotorRatio(motor_ind, percent) {
    const { sendMotorControlMsg } = this.props.ros
    const namespace = this.props.namespace
    if (namespace == null || namespace === 'None') {
      return
    }
    const values = this.state.motor_slider_values.slice()
    values[motor_ind] = percent
    this.setState({ motor_slider_values: values, motor_slider_text: values.map((v) => String(v)) })
    sendMotorControlMsg(namespace + "/set_motor_control", motor_ind, percent / 100.0)
  }

  turnOffAllMotors() {
    const count = this.state.motor_slider_values.length
    for (var i = 0; i < count; i++) {
      this.publishMotorRatio(i, 0)
    }
  }

  // Per-motor and per-wheel sliders. Rendered from has_manual_controls plus the
  // reported motor or wheel count -- a wheeled robot's outputs are labelled
  // Wheel, a robot with motors and no wheels Motor, which is the whole point of
  // reporting has_wheels separately from has_motors.
  renderMotorControls() {
    const caps = this.state.capabilities
    const status_msg = this.state.status_msg
    if (caps == null || status_msg == null || caps.has_manual_controls !== true) {
      return null
    }

    const motors = (status_msg.current_motor_control_settings !== undefined)
      ? status_msg.current_motor_control_settings : []
    if (motors.length === 0) {
      return null
    }

    const namespace = this.props.namespace
    const output_label = (caps.has_wheels === true) ? "Wheel" : "Motor"
    const manual_ready = status_msg.manual_control_mode_ready

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={"Manual Control Ready"}>
          <Input disabled value={String(manual_ready)} />
        </Label>

        {motors.map((motor, i) => (
          <div key={motor.motor_ind}>
            <SliderAdjustment
              disabled={manual_ready !== true}
              title={output_label + " " + (motor.motor_ind + 1)}
              topic={namespace + "/set_motor_control"}
              msgType={"nepi_interfaces/MotorControl"}
              adjustment={(this.state.motor_slider_values[i] !== undefined)
                ? this.state.motor_slider_values[i] : 0}
              onSliderChangeOverride={(value) => this.publishMotorRatio(motor.motor_ind, value)}
              min={0}
              max={100}
              unit={"%"}
            />
            <Label title={"Set " + output_label + " " + (motor.motor_ind + 1) + " %"}>
              <Input
                id={"SimMotorPercentInput" + motor.motor_ind}
                disabled={manual_ready !== true}
                value={(this.state.motor_slider_text[i] !== undefined)
                  ? this.state.motor_slider_text[i] : "0"}
                onChange={(e) => {
                  const text = this.state.motor_slider_text.slice()
                  text[i] = e.target.value
                  const el = document.getElementById("SimMotorPercentInput" + motor.motor_ind)
                  if (el) {
                    setElementStyleModified(el)
                  }
                  this.setState({ motor_slider_text: text })
                }}
                onKeyDown={(e) => {
                  if (e.key === 'Enter') {
                    const el = document.getElementById("SimMotorPercentInput" + motor.motor_ind)
                    if (el) {
                      clearElementStyleModified(el)
                    }
                    const percent = parseFloat(e.target.value)
                    if (!isNaN(percent)) {
                      this.publishMotorRatio(motor.motor_ind, Math.max(0, Math.min(100, percent)))
                    }
                  }
                }}
                style={{ width: "4em" }}
              />
            </Label>
          </div>
        ))}

        <ButtonMenu>
          <Button onClick={this.turnOffAllMotors}>{"All " + output_label + "s Off"}</Button>
        </ButtonMenu>

      </React.Fragment>
    )
  }

  // Goto input fields, one block per goto capability the device reports.
  renderGotoControls() {
    const caps = this.state.capabilities
    const status_msg = this.state.status_msg
    if (caps == null || status_msg == null) {
      return null
    }
    if (caps.has_goto_position !== true && caps.has_goto_pose !== true
        && caps.has_goto_location !== true) {
      return null
    }

    const namespace = this.props.namespace
    const { sendFloatGotoPositionMsg, sendFloatGotoPoseMsg, sendFloatGotoLocationMsg } = this.props.ros
    const autonomous_ready = status_msg.autonomous_control_mode_ready

    const clearDirty = (ids) => {
      for (var i = 0; i < ids.length; i++) {
        const el = document.getElementById(ids[i])
        if (el) {
          clearElementStyleModified(el)
        }
      }
    }

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={"Autonomous Control Ready"}>
          <Input disabled value={String(autonomous_ready)} />
        </Label>

        {(caps.has_goto_position === true) ?
          <React.Fragment>

            <Label title={"Goto Position X (m)"}>
              <Input
                id={"SimGotoPositionX"}
                value={this.state.gotoPositionX}
                onChange={(e) => this.onUpdateInput(e, "gotoPositionX")}
              />
            </Label>

            <Label title={"Goto Position Y (m)"}>
              <Input
                id={"SimGotoPositionY"}
                value={this.state.gotoPositionY}
                onChange={(e) => this.onUpdateInput(e, "gotoPositionY")}
              />
            </Label>

            {/* z is only offered when the device reports it as a supported axis */}
            {(caps.control_support !== undefined && caps.control_support.z === true) ?
              <Label title={"Goto Position Z (m)"}>
                <Input
                  id={"SimGotoPositionZ"}
                  value={this.state.gotoPositionZ}
                  onChange={(e) => this.onUpdateInput(e, "gotoPositionZ")}
                />
              </Label>
            : null}

            <Label title={"Goto Position Yaw (deg)"}>
              <Input
                id={"SimGotoPositionYaw"}
                value={this.state.gotoPositionYaw}
                onChange={(e) => this.onUpdateInput(e, "gotoPositionYaw")}
              />
            </Label>

            <ButtonMenu>
              <Button
                onClick={() => {
                  sendFloatGotoPositionMsg(
                    namespace + "/goto_position",
                    this.state.gotoPositionX === '' ? '0' : this.state.gotoPositionX,
                    this.state.gotoPositionY === '' ? '0' : this.state.gotoPositionY,
                    this.state.gotoPositionZ === '' ? '0' : this.state.gotoPositionZ,
                    this.state.gotoPositionYaw === '' ? '0' : this.state.gotoPositionYaw)
                  clearDirty(["SimGotoPositionX", "SimGotoPositionY",
                              "SimGotoPositionZ", "SimGotoPositionYaw"])
                }}
              >{"Send Goto Position"}</Button>
            </ButtonMenu>

          </React.Fragment>
        : null}

        {(caps.has_goto_pose === true) ?
          <React.Fragment>

            <Label title={"Goto Pose Roll (deg)"}>
              <Input
                id={"SimGotoPoseRoll"}
                value={this.state.gotoPoseRoll}
                onChange={(e) => this.onUpdateInput(e, "gotoPoseRoll")}
              />
            </Label>

            <Label title={"Goto Pose Pitch (deg)"}>
              <Input
                id={"SimGotoPosePitch"}
                value={this.state.gotoPosePitch}
                onChange={(e) => this.onUpdateInput(e, "gotoPosePitch")}
              />
            </Label>

            <Label title={"Goto Pose Yaw (deg)"}>
              <Input
                id={"SimGotoPoseYaw"}
                value={this.state.gotoPoseYaw}
                onChange={(e) => this.onUpdateInput(e, "gotoPoseYaw")}
              />
            </Label>

            <ButtonMenu>
              <Button
                onClick={() => {
                  sendFloatGotoPoseMsg(
                    namespace + "/goto_pose",
                    this.state.gotoPoseRoll === '' ? '-999' : this.state.gotoPoseRoll,
                    this.state.gotoPosePitch === '' ? '-999' : this.state.gotoPosePitch,
                    this.state.gotoPoseYaw === '' ? '-999' : this.state.gotoPoseYaw)
                  clearDirty(["SimGotoPoseRoll", "SimGotoPosePitch", "SimGotoPoseYaw"])
                }}
              >{"Send Goto Pose"}</Button>
            </ButtonMenu>

          </React.Fragment>
        : null}

        {(caps.has_goto_location === true) ?
          <React.Fragment>

            <Label title={"Goto Location Lat"}>
              <Input
                id={"SimGotoLocationLat"}
                value={this.state.gotoLocationLat}
                onChange={(e) => this.onUpdateInput(e, "gotoLocationLat")}
              />
            </Label>

            <Label title={"Goto Location Long"}>
              <Input
                id={"SimGotoLocationLong"}
                value={this.state.gotoLocationLong}
                onChange={(e) => this.onUpdateInput(e, "gotoLocationLong")}
              />
            </Label>

            <Label title={"Goto Location Alt (m)"}>
              <Input
                id={"SimGotoLocationAlt"}
                value={this.state.gotoLocationAlt}
                onChange={(e) => this.onUpdateInput(e, "gotoLocationAlt")}
              />
            </Label>

            <Label title={"Goto Location Yaw (deg)"}>
              <Input
                id={"SimGotoLocationYaw"}
                value={this.state.gotoLocationYaw}
                onChange={(e) => this.onUpdateInput(e, "gotoLocationYaw")}
              />
            </Label>

            <ButtonMenu>
              <Button
                onClick={() => {
                  sendFloatGotoLocationMsg(
                    namespace + "/goto_location",
                    this.state.gotoLocationLat === '' ? '-999' : this.state.gotoLocationLat,
                    this.state.gotoLocationLong === '' ? '-999' : this.state.gotoLocationLong,
                    this.state.gotoLocationAlt === '' ? '-999' : this.state.gotoLocationAlt,
                    this.state.gotoLocationYaw === '' ? '-999' : this.state.gotoLocationYaw)
                  clearDirty(["SimGotoLocationLat", "SimGotoLocationLong",
                              "SimGotoLocationAlt", "SimGotoLocationYaw"])
                }}
              >{"Send Goto Location"}</Button>
            </ButtonMenu>

          </React.Fragment>
        : null}

      </React.Fragment>
    )
  }

  // Home, stop, and enumerated setup/go action buttons. Every one is gated on
  // its own capability flag or on its reported option list being non-empty.
  renderHomeControls() {
    const caps = this.state.capabilities
    if (caps == null) {
      return null
    }

    const setup_actions = (caps.setup_action_options !== undefined) ? caps.setup_action_options : []
    const go_actions = (caps.go_action_options !== undefined) ? caps.go_action_options : []

    const has_any = (caps.has_go_home === true || caps.has_set_home === true
                     || caps.has_go_stop === true || setup_actions.length > 0
                     || go_actions.length > 0)
    if (has_any === false) {
      return null
    }

    const namespace = this.props.namespace
    const { sendTriggerMsg, sendIntMsg } = this.props.ros

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <ButtonMenu>
          {(caps.has_go_home === true) ?
            <Button onClick={() => sendTriggerMsg(namespace + "/go_home")}>{"Go Home"}</Button>
          : null}
          {(caps.has_go_stop === true) ?
            <Button onClick={() => sendTriggerMsg(namespace + "/go_stop")}>{"Stop"}</Button>
          : null}
          {(caps.has_set_home === true) ?
            <Button onClick={() => sendTriggerMsg(namespace + "/set_home_current")}>{"Set Home Here"}</Button>
          : null}
        </ButtonMenu>

        {(setup_actions.length > 0) ?
          <ButtonMenu>
            {setup_actions.map((action, i) => (
              <Button
                key={action}
                onClick={() => sendIntMsg(namespace + "/setup_action", String(i))}
              >{action}</Button>
            ))}
          </ButtonMenu>
        : null}

        {(go_actions.length > 0) ?
          <ButtonMenu>
            {go_actions.map((action, i) => (
              <Button
                key={action}
                onClick={() => sendIntMsg(namespace + "/go_action", String(i))}
              >{action}</Button>
            ))}
          </ButtonMenu>
        : null}

      </React.Fragment>
    )
  }

  // Camera selector, live image pane, and camera view-mode selector. All three
  // are driven by reported lists: available_image_topics (itself the image-typed
  // subset of the device's one typed sensor-topic list) and
  // available_camera_view_modes.
  renderCameraControls() {
    const caps = this.state.capabilities
    const status_msg = this.state.status_msg
    if (caps == null || status_msg == null) {
      return null
    }

    const image_topics = (caps.available_image_topics !== undefined) ? caps.available_image_topics : []
    const view_modes = (caps.available_camera_view_modes !== undefined)
      ? caps.available_camera_view_modes : []
    const has_camera = (caps.has_camera === true && image_topics.length > 0)
    const has_view_control = (caps.has_camera_view_control === true && view_modes.length > 0)

    if (has_camera === false && has_view_control === false) {
      return null
    }

    const namespace = this.props.namespace
    const { sendStringMsg } = this.props.ros
    // The latched info message is authoritative for which topic is streaming;
    // the capabilities snapshot is only the fallback before info arrives.
    const info_msg = this.state.info_msg
    const active_image_topic = (info_msg != null && info_msg.active_image_topic !== undefined)
      ? info_msg.active_image_topic
      : ((caps.active_image_topic !== undefined) ? caps.active_image_topic : '')

    var image_items = []
    image_items.push(<Option key={'None'} value={'None'}>{'None'}</Option>)
    for (var i = 0; i < image_topics.length; i++) {
      image_items.push(<Option key={image_topics[i]} value={image_topics[i]}>{image_topics[i]}</Option>)
    }

    var mode_items = []
    for (var m = 0; m < view_modes.length; m++) {
      mode_items.push(<Option key={view_modes[m]} value={view_modes[m]}>{view_modes[m]}</Option>)
    }

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        {(has_camera === true) ?
          <React.Fragment>

            <Label title={"Camera"}>
              <Select
                onChange={(e) => sendStringMsg(namespace + "/set_active_image_topic", e.target.value)}
                value={(active_image_topic !== '') ? active_image_topic : 'None'}
              >
                {image_items}
              </Select>
            </Label>

            {(active_image_topic !== '' && active_image_topic !== 'None') ?
              <NepiIFImageViewer
                id={"simControlsImageViewer"}
                image_topic={active_image_topic}
                title={"Camera"}
              />
            : null}

          </React.Fragment>
        : null}

        {(has_view_control === true) ?
          <Label title={"Camera View Mode"}>
            <Select
              onChange={(e) => {
                this.setState({ selected_view_mode: e.target.value })
                sendStringMsg(namespace + "/set_camera_view_mode", e.target.value)
              }}
              value={this.state.selected_view_mode}
            >
              {mode_items}
            </Select>
          </Label>
        : null}

      </React.Fragment>
    )
  }

  // Robot-view/scene-view rig positioning. Unlike everything else in this
  // file, gated on the RBX driver's own reported Setting names
  // (rbxSettingsNamesList, from selected_simulator's settings/status), not on
  // the SimCapabilitiesQuery response above -- see the constructor comment on
  // rbxSettingsListener for why. namePrefix/titlePrefix let this same block
  // render either offset triple: "camera_offset_*" (robot view) or
  // "scene_offset_*" (scene view, only on rbx_sim_node.py -- the rover has two
  // independently-positioned rigid cameras; rbx_ardupilot_node.py's single
  // aimable rig only ever defines camera_offset_*).
  renderCameraOffsetControls(namePrefix, titlePrefix) {
    const offsets = [
      { name: namePrefix + "_x", title: titlePrefix + " Offset X (m)" },
      { name: namePrefix + "_y", title: titlePrefix + " Offset Y (m)" },
      { name: namePrefix + "_z", title: titlePrefix + " Offset Z (m)" },
    ]
    return (
      <React.Fragment>
        {offsets.map((offset) => (
          <Label key={offset.name} title={offset.title}>
            <Input
              id={"SimCam_" + offset.name}
              value={this.state[offset.name]}
              onChange={(e) => {
                const el = document.getElementById("SimCam_" + offset.name)
                if (el) {
                  setElementStyleModified(el)
                }
                this.setState({ [offset.name]: e.target.value })
              }}
              onKeyDown={(e) => this.onEnterSetCameraOffset(e, offset.name)}
            />
          </Label>
        ))}
      </React.Fragment>
    )
  }

  // Robot-view/scene-view toggle plus both offset triples, all gated on the RBX
  // driver's own reported Settings -- see renderCameraOffsetControls above.
  renderCameraRigControls() {
    const rbx_ns = this.state.rbx_namespace
    if (rbx_ns === null || rbx_ns === '' || rbx_ns === 'None') {
      return null
    }
    const settings = this.state.rbxSettingsNamesList
    const has_view_toggle = settings.includes("camera_view_mode")
    const has_camera_offsets = settings.includes("camera_offset_x")
    const has_scene_offsets = settings.includes("scene_offset_x")
    if (has_view_toggle === false && has_camera_offsets === false && has_scene_offsets === false) {
      return null
    }

    const { updateSetting } = this.props.ros

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={"Camera Rig"} labelStyle={{ fontWeight: 'bold' }}/>

        {(has_view_toggle === true) ?
          <ButtonMenu>
            <Button onClick={() => updateSetting(rbx_ns + "/settings", "camera_view_mode", "Discrete", "FIRST_PERSON")}>{"Robot View"}</Button>
            <Button onClick={() => updateSetting(rbx_ns + "/settings", "camera_view_mode", "Discrete", "THIRD_PERSON")}>{"Scene View"}</Button>
          </ButtonMenu>
        : null}

        {(has_camera_offsets === true) ? this.renderCameraOffsetControls("camera_offset", "Robot View Camera") : null}
        {(has_scene_offsets === true) ? this.renderCameraOffsetControls("scene_offset", "Scene View Camera") : null}

      </React.Fragment>
    )
  }

  // Environment toggles, one per reported environment option. The reported list
  // is what makes this generalize past any one hardcoded option.
  renderEnvironmentControls() {
    const caps = this.state.capabilities
    if (caps == null || caps.has_environment_controls !== true) {
      return null
    }
    const options = (caps.available_environment_options !== undefined)
      ? caps.available_environment_options : []
    if (options.length === 0) {
      return null
    }

    const namespace = this.props.namespace
    const { sendStringMsg } = this.props.ros

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={"Environment"} labelStyle={{ fontWeight: 'bold' }}/>

        <ButtonMenu>
          {options.map((option) => (
            <Button
              key={option}
              onClick={() => sendStringMsg(namespace + "/set_environment_option", option)}
            >{option}</Button>
          ))}
        </ButtonMenu>

      </React.Fragment>
    )
  }

  renderControls() {
    const namespace = this.props.namespace
    if (namespace == null || namespace === 'None') {
      return (
        <Columns>
          <Column>

          </Column>
        </Columns>
      )
    }

    return (
      <React.Fragment>

        {this.renderMotorControls()}
        {this.renderGotoControls()}
        {this.renderHomeControls()}
        {this.renderCameraControls()}
        {this.renderCameraRigControls()}
        {this.renderEnvironmentControls()}

      </React.Fragment>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return (
        <Columns>
          <Column>

          </Column>
        </Columns>
      )
    }
    else if (make_section === false) {
      return (
        <React.Fragment>

          {this.renderControls()}

        </React.Fragment>
      )
    }
    else {
      return (
        <Section title={(this.props.title !== undefined) ? this.props.title : null}>

          {this.renderControls()}

        </Section>
      )
    }
  }

}
export default NepiIFSimControls
