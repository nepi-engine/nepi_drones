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
import Toggle from "react-toggle"
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

      // Which of the always-live color/depth mirror topics each half of
      // renderCommonImageViewer's preview currently shows -- independent
      // per column, so the operator can e.g. watch robot color while
      // watching scene depth at the same time.
      robot_view_mode: 'robot_color',
      scene_view_mode: 'scene_color',

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

      // Per-option on/off state, held locally since SimStatus doesn't report
      // which environment options are currently active server-side. Resets to
      // "all off" on page reload -- a known limitation, not a synced source of
      // truth. See docs/SIM_CONNECTOR_REMAINING_WORK.md if this needs fixing.
      environment_option_enabled: {},

      // "Show Settings" button state -- lets the operator preview the full
      // configuration surface (capability toggles, camera offsets, movement
      // limits, environment) before any simulator is deployed. Found live
      // (2026-08-18): every render* method below hid its controls entirely
      // whenever rbx_namespace was empty, which is correct for anything that
      // genuinely can't exist without a live device (the image-source
      // candidate list, the live camera preview) but wrong for the Settings
      // every RBX sim driver declares unconditionally (autonomous_movement_
      // enabled/camera_controls_enabled are declared
      // identically in rbx_sim_node.py, rbx_ardupilot_node.py,
      // rbx_webots_node.py, and rbx_webots_quadcopter_node.py -- confirmed by
      // direct inspection, not assumed). Toggling this on renders those
      // universal controls with factory-default-style placeholder values;
      // toggling a control while no device is live just no-ops harmlessly
      // (updateSetting has nothing live to reach), same as this app's own
      // documented "no-ops every command" pattern elsewhere. Once a real
      // simulator connects, rbxSettingsNamesList/rbxSettingsValuesDict take
      // over and the same controls become genuinely live.
      //
      // Auto-flips to true on the not-live -> live transition (see
      // componentDidUpdate) so the pre-existing default (settings visible as
      // soon as a simulator connects) is unchanged -- but unlike before
      // (2026-08-19), it now genuinely gates visibility even while live too,
      // so the Hide Settings button actually hides something instead of
      // disappearing the moment there's something worth hiding.
      show_settings: false,

      // Live control of the RBX driver's own Settings (camera_offset_x/y/z,
      // scene_offset_x/y/z, camera_view_mode) used to be rendered HERE, bypassing
      // the capabilities-driven controls above and publishing straight to
      // whichever RBX driver SimStatus.selected_simulator names. Per request,
      // that control moved BACK to the generic RBX device panel
      // (NepiDeviceRBX.js) -- "the rbx driver should pretty much have every
      // feature AVAILABLE no matter the robot... the right things should show
      // up and disappear from the rbx panel." This app's role for those
      // Settings is not to duplicate their live control, only to configure
      // (elsewhere in this file) whether the corresponding CONTROL SURFACE is
      // exposed at all for the current robot config -- e.g. autonomous
      // movement below.
      //
      // The RBX-namespace resolution machinery stays: rbxSettingsNamesList is
      // still how this component knows whether the connected RBX driver
      // defines autonomous_movement_enabled at all
      // (an older driver without this feature keeps working exactly as before,
      // simply without a Sim Connector control for it), and
      // updateRbxSettingsListener/rbxSettingsListener publish/read those
      // Settings the same way camera_offset_x always did.
      rbxSettingsListener: null,
      rbxSettingsNamesList: [],
      rbxSettingsValuesDict: {},

      // Edit buffers for the camera/scene offset triples and the movement
      // limit Settings, same pattern as NepiDeviceRBX.js's own offsetNames
      // buffers -- resynced from the device's own value in
      // rbxSettingsListener below only when it actually changes, never on
      // every status tick, so an in-progress edit is never clobbered.
      camera_offset_x: '', camera_offset_y: '', camera_offset_z: '',
      scene_offset_x: '', scene_offset_y: '', scene_offset_z: '',

      // Environment dropdown display value -- "Flat Ground"/"Obstacle
      // Course" labels for FLAT_GROUND/OBSTACLE_COURSE, matching
      // NepiDeviceRBX.js's own onSelectEnvironment convention exactly.
      selected_environment_setting: 'Flat Ground',

      // Set to the operator's own dropdown pick whenever it was made while
      // !isRbxLive() (renderEnvironmentSetting's onChange can't send
      // updateSetting yet -- no driver namespace exists to send it to), and
      // cleared once it has actually been sent. Without this, a pre-launch
      // pick was silently dropped: the freshly-launched device comes up with
      // whatever "environment" its own Settings persistence carried over
      // from a PREVIOUS session (independent of anything the RUI showed),
      // rbxSettingsListener below then resyncs the dropdown to match that
      // stale value once live, and since the dropdown already read the
      // operator's intended value the whole time, reselecting the same
      // option fires no onChange -- confirmed live (2026-08-28) that only
      // toggling away and back (two genuine onChange events) actually sent
      // anything. Tracking the pending pick here lets rbxSettingsListener
      // send it the moment the device goes live, instead of the operator
      // having to notice and manually force a change.
      environment_setting_pending: null,

    }

    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateInfoListener = this.updateInfoListener.bind(this)
    this.infoListener = this.infoListener.bind(this)
    this.queryCapabilities = this.queryCapabilities.bind(this)
    this.updateRbxSettingsListener = this.updateRbxSettingsListener.bind(this)
    this.rbxSettingsListener = this.rbxSettingsListener.bind(this)

    this.onUpdateInput = this.onUpdateInput.bind(this)
    this.publishMotorRatio = this.publishMotorRatio.bind(this)
    this.turnOffAllMotors = this.turnOffAllMotors.bind(this)

    this.renderMotorControls = this.renderMotorControls.bind(this)
    this.renderGotoControls = this.renderGotoControls.bind(this)
    this.renderHomeControls = this.renderHomeControls.bind(this)
    this.renderCameraControls = this.renderCameraControls.bind(this)
    this.renderRobotCapabilityControls = this.renderRobotCapabilityControls.bind(this)
    this.renderCommonImageViewer = this.renderCommonImageViewer.bind(this)
    this.renderImageSourceCuration = this.renderImageSourceCuration.bind(this)
    this.renderEnvironmentControls = this.renderEnvironmentControls.bind(this)
    this.toggleEnvironmentOption = this.toggleEnvironmentOption.bind(this)
    this.isRbxLive = this.isRbxLive.bind(this)
    this.toggleShowSettings = this.toggleShowSettings.bind(this)

    this.onEnterSetRbxFloatSetting = this.onEnterSetRbxFloatSetting.bind(this)
    this.renderCameraOffsetControls = this.renderCameraOffsetControls.bind(this)
    this.renderEnvironmentSetting = this.renderEnvironmentSetting.bind(this)
    this.renderDepthMapToggle = this.renderDepthMapToggle.bind(this)

    this.renderLiveControls = this.renderLiveControls.bind(this)
    this.renderConfigControls = this.renderConfigControls.bind(this)
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
      const was_live = (prevState.rbx_namespace !== null && prevState.rbx_namespace !== ''
        && prevState.rbx_namespace !== 'None')
      const updates = { rbx_namespace: selected_simulator }
      // Auto-open on the not-live -> live transition, matching the
      // longstanding default (settings always visible once live) that
      // existed before the Show/Hide button carried any real effect while
      // live -- see the button's own comment for why it now does.
      if (!was_live && selected_simulator !== null && selected_simulator !== ''
          && selected_simulator !== 'None') {
        updates.show_settings = true
      }
      this.setState(updates)
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
  // driver named by selected_simulator. Tracks both the setting NAMES it
  // actually registers (so renderRobotCapabilityControls can tell an older
  // driver without this feature from one reporting it FALSE) and their current
  // values (so the checkboxes reflect the device's real state, including a
  // change made from the RBX panel itself -- this is a two-way Setting, not a
  // one-shot config write).
  rbxSettingsListener(message) {
    const settings = (message.settings_list !== undefined) ? message.settings_list : []
    var namesList = []
    var valuesDict = {}
    for (let ind = 0; ind < settings.length; ind++) {
      namesList.push(settings[ind].name_str)
      valuesDict[settings[ind].name_str] = settings[ind].value_str
    }
    this.setState({ rbxSettingsNamesList: namesList, rbxSettingsValuesDict: valuesDict })

    // Seed/resync each edit buffer only when the DEVICE's own value changed
    // (or on first sight), never on every status tick -- otherwise a status
    // message overwrites whatever is being typed. Same pattern as
    // NepiDeviceRBX.js's own offsetNames sync.
    const floatSettingNames = ["camera_offset_x", "camera_offset_y", "camera_offset_z",
                               "scene_offset_x", "scene_offset_y", "scene_offset_z"]
    var updates = {}
    for (let i = 0; i < floatSettingNames.length; i++) {
      const name = floatSettingNames[i]
      const deviceVal = valuesDict[name]
      if (deviceVal !== undefined && deviceVal !== this.state.rbxSettingsValuesDict[name]) {
        updates[name] = deviceVal
      }
    }
    if (valuesDict["environment"] !== undefined
        && valuesDict["environment"] !== this.state.rbxSettingsValuesDict["environment"]) {
      if (this.state.environment_setting_pending !== null) {
        // The operator picked this before the device existed to send it to
        // (renderEnvironmentSetting's onChange couldn't reach updateSetting
        // yet). Now that a real SettingsStatus has arrived, the driver
        // namespace exists -- send the pending pick for real instead of
        // letting this status resync the dropdown back to whatever
        // "environment" this fresh device happened to carry over from a
        // previous session. Clearing the pending flag here, not in the
        // onChange handler, is what makes this a one-shot catch-up rather
        // than resending on every subsequent status tick.
        const pending = this.state.environment_setting_pending
        updates.environment_setting_pending = null
        this.props.ros.updateSetting(this.state.rbx_namespace + "/settings", "environment", "Discrete",
          pending === "Obstacle Course" ? "OBSTACLE_COURSE" : "FLAT_GROUND")
      } else {
        updates.selected_environment_setting =
          (valuesDict["environment"] === "OBSTACLE_COURSE") ? "Obstacle Course" : "Flat Ground"
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

  // Robot-config CAPABILITY configuration -- "customize the capabilities that
  // are open" for the currently selected simulator. Distinct from
  // renderGotoControls/renderCameraControls above (which CONTROL the robot
  // live): these toggles decide WHETHER a control surface the robot type
  // structurally supports is exposed at all, on both this app's own panel
  // (has_goto_position etc, unaffected by these toggles) and the generic RBX
  // device panel (which gates on autonomous_movement_enabled the same way it
  // already gates on camera_offset_x -- see NepiDeviceRBX-Controls.js). Gated
  // on rbxSettingsNamesList, not on
  // SimCapabilitiesQuery: this is a property of the connected RBX DRIVER
  // (rbx_sim_node.py's CAPABILITY_SETTING_NAMES), same delivery path as the
  // camera Settings always used, so an older driver without this feature keeps
  // working exactly as before -- simply without a control for it here.
  renderRobotCapabilityControls() {
    const live = this.isRbxLive()
    if (!this.state.show_settings) {
      return null
    }
    const settings = this.state.rbxSettingsNamesList
    const values = this.state.rbxSettingsValuesDict
    // Preview mode (not live, Show Settings clicked): these three toggles are
    // declared identically by every RBX sim driver's CAPABILITY_SETTING_NAMES
    // (confirmed by direct inspection of rbx_sim_node.py, rbx_ardupilot_node.py,
    // rbx_webots_node.py, rbx_webots_quadcopter_node.py), so showing them
    // unconditionally before any simulator connects is accurate, not a guess.
    // Image-source curation stays live-only -- there is no candidate topic
    // list to curate before a real camera exists.
    const has_autonomous_toggle = live ? settings.includes("autonomous_movement_enabled") : true
    const has_camera_toggle = live ? settings.includes("camera_controls_enabled") : true
    const has_image_curation = live && settings.includes("enabled_image_sources")
    if (has_autonomous_toggle === false
        && has_camera_toggle === false && has_image_curation === false) {
      return null
    }

    const { updateSetting } = this.props.ros
    const setToggle = (name, checked) => {
      if (!live) {
        return
      }
      updateSetting(this.state.rbx_namespace + "/settings", name, "Discrete", checked ? "TRUE" : "FALSE")
    }

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={"Robot Capabilities"} labelStyle={{ fontWeight: 'bold' }}/>

        <Columns>
          <Column>
            {(has_autonomous_toggle === true) ?
              <Label title={"Automated Movement"}>
                <Toggle
                  checked={values["autonomous_movement_enabled"] !== "FALSE"}
                  onClick={() => setToggle("autonomous_movement_enabled", values["autonomous_movement_enabled"] === "FALSE")}
                />
              </Label>
            : null}

            {(has_camera_toggle === true) ?
              <Label title={"Camera Controls"}>
                <Toggle
                  checked={values["camera_controls_enabled"] !== "FALSE"}
                  onClick={() => setToggle("camera_controls_enabled", values["camera_controls_enabled"] === "FALSE")}
                />
              </Label>
            : null}
          </Column>
        </Columns>

        {this.renderImageSourceCuration()}

      </React.Fragment>
    )
  }

  // Image Viewer -- "choose what image sources are good and what aren't" for
  // THIS sim instance. One checkbox per candidate image topic, membership
  // toggling that topic in the comma-separated enabled_image_sources Setting
  // NepiDeviceRBX.js's createImageOptions filters the robot's own Image
  // Source dropdown by (see that method's own comment for the other half of
  // this feature: an allowlisted topic now shows up there even from outside
  // this robot's own namespace, which is what makes offering a real physical
  // camera here actually useful).
  //
  // Candidates come from this.props.ros.imageTopics -- the RUI-wide live
  // sensor_msgs/Image topic list (Store.js updateImageTopics), NOT from
  // this.state.capabilities.available_image_topics. Found live (2026-08-18)
  // as the reason this section never appeared at all for the Gazebo rover or
  // ArduPilot driver: that capability field is only ever populated by the
  // newer shared "generic connector" bridge protocol (Webots/PyBullet/WPILib,
  // see sim_connector_app_node.py's getAvailableSensorTopics), which RBX_SIM
  // and RBX_ARDUPILOT's own separate heartbeat+bridge mechanism never feeds
  // -- so available_image_topics was structurally always empty for them,
  // silently hiding this whole feature rather than a capability genuinely
  // being absent. The live system-wide list is accurate for every driver.
  renderImageSourceCuration() {
    const live = this.isRbxLive()
    if (!live) {
      // Genuinely needs live detection -- there is no candidate topic list
      // to curate before a real camera exists to publish one.
      return null
    }
    const rbx_ns = this.state.rbx_namespace
    if (!this.state.rbxSettingsNamesList.includes("enabled_image_sources")) {
      return null
    }
    const image_topics = this.props.ros.imageTopics || []
    // Scoped (2026-08-26, widened 2026-08-27) to exclude only OTHER apps'
    // own generated/derived images, not to a namespace allowlist -- an
    // earlier version of this filter required a topic to start with this
    // robot's own namespace or the sim connector app's own mirror
    // namespace, which correctly dropped unrelated app output
    // (app_ai_targeting/targeting_image, app_file_pub_img/color_image) but
    // ALSO dropped a genuinely relevant physical camera connected to this
    // same device (nexigo_02/idx/color_image) that isn't part of either
    // namespace -- reported live: "the sim app shows the right sim
    // cameras, but its missing the nexigo physical camera". A real device's
    // own topics (anything NOT published by another "app_*" node) are
    // always fair candidates for "what does this robot's camera look
    // like", the same way NepiDeviceRBX.js's own createImageOptions
    // already treats them -- only another app's derived/processed output
    // is the kind of noise this curation exists to hide (see that
    // "trillion camera images" report above, which was about app outputs,
    // not physical cameras). base_namespace is this device's root (one
    // level above the sim connector app's own namespace); own_app_segment
    // is this app's own node name, so its mirror topics
    // (robot_color/scene_color/etc, which live one level under
    // base_namespace) still pass even though "app_sim_connector" itself
    // starts with "app_".
    const appNamespace = this.props.namespace.split('/sim')[0]
    const base_namespace = appNamespace.substring(0, appNamespace.lastIndexOf('/'))
    const own_app_segment = appNamespace.substring(appNamespace.lastIndexOf('/') + 1)
    const candidates = image_topics.filter((topic) => {
      if (topic.startsWith(base_namespace + '/') === false) {
        return false
      }
      const owner_segment = topic.substring(base_namespace.length + 1).split('/')[0]
      const is_other_app_output = owner_segment.startsWith('app_') && owner_segment !== own_app_segment
      if (is_other_app_output) {
        return false
      }
      // Exclude every raw "color_2d_image" topic (this app's own dead
      // bridge-relay bare topic, AND every RBX driver's raw per-robot
      // color_2d_image/robot_view/scene_view topics) -- those are internal
      // sources the generic mirror topics below already relay from, for
      // whichever robot/simulator is actually selected, so surfacing them
      // separately here is redundant no matter which robot type is running
      // (works for any robot, not just a hardcoded name). The two mirror
      // topics themselves (renderCommonImageViewer's appNamespace +
      // "/robot_view"/"/scene_view") don't contain "color_2d_image" in their
      // name at all, so they pass through this filter untouched.
      //
      // Also exclude any bare "<namespace>/image" topic -- device_if_rbx.py's
      // own ImageIF republishes whatever image_source is currently selected
      // under every RBX driver's own namespace this exact way (confirmed via
      // ardupilot_sitl/image showing up as a 4th candidate alongside the two
      // mirrors and the physical camera). Same redundancy as color_2d_image:
      // the universal mirrors already cover "whatever this robot's camera
      // is" for any robot type, so this generic per-driver echo doesn't need
      // its own curation entry either. endsWith, not includes, so a genuine
      // future topic that merely contains "/image" elsewhere in its name
      // isn't accidentally caught.
      // Also exclude any "*_depth_map" topic -- robot_depth_map/scene_depth_map
      // (and any driver's own per-robot equivalent) carry raw 32FC1-meters
      // float data for downstream processing, not a human-viewable image (see
      // rbx_sim_node.py's own publisher comment: "for later processing rather
      // than viewing"). enabled_image_sources is purely a human-facing
      // "which camera looks good" allowlist for NepiDeviceRBX.js's
      // Image_Source dropdown -- it has no processing-pipeline consumer -- so
      // offering a raw depth map here just adds an option that renders as
      // useless noise. The underlying topic keeps publishing untouched for
      // any real subscriber; this only trims what this one picker offers.
      // Reported live: "the robot depth map and scene depth map seem kind of
      // redundant here -- theres no reason for both" (2026-08-26) -- they
      // aren't redundant as DATA (colorized-for-viewing vs. raw-for-processing
      // serve different consumers), just both wrongly offered in a
      // viewing-only picker.
      return topic.includes('color_2d_image') === false && topic.includes('zed_node') === false
        && topic.endsWith('/image') === false && topic.endsWith('_depth_map') === false
    })
    if (candidates.length === 0) {
      return null
    }

    const { updateSetting } = this.props.ros
    const currentRaw = this.state.rbxSettingsValuesDict["enabled_image_sources"]
    const current = (currentRaw !== undefined) ? String(currentRaw) : ''
    // Empty Setting means "unrestricted" (every candidate implicitly
    // allowed) -- shown here as every checkbox already checked, matching
    // what the RBX panel actually does with an empty value, rather than
    // showing everything unchecked and implying nothing is enabled.
    const enabled = (current.trim() === '')
      ? candidates.slice()
      : current.split(',').map((s) => s.trim()).filter((s) => s !== '')

    const toggleTopic = (topic, checked) => {
      const next = checked
        ? enabled.concat(topic).filter((t, i, arr) => arr.indexOf(t) === i)
        : enabled.filter((t) => t !== topic)
      updateSetting(rbx_ns + "/settings", "enabled_image_sources", "String", next.join(','))
    }

    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <Label title={"Image Viewer"} labelStyle={{ fontWeight: 'bold' }}/>
        {candidates.map((topic) => (
          <Label key={topic} title={topic}>
            <Toggle
              checked={enabled.includes(topic)}
              onClick={() => toggleTopic(topic, !enabled.includes(topic))}
            />
          </Label>
        ))}
      </React.Fragment>
    )
  }

  // Enter-to-apply for any plain editable Float RBX Setting (camera/scene
  // offsets, movement limits) -- one generic handler rather than one per
  // Setting, since they all share the exact same publish shape. Ported from
  // NepiDeviceRBX.js's onEnterSetCameraOffset, generalized to any setting
  // name/state key pair (they're the same string for every Setting this
  // component edits, so a single argument suffices).
  onEnterSetRbxFloatSetting(event, settingName) {
    if (event.key === 'Enter') {
      const value = parseFloat(event.target.value)
      // Preview mode (no live device yet): nothing to send it to. Still
      // clears the modified-style below so the input doesn't look stuck.
      if (!isNaN(value) && this.isRbxLive()) {
        const { updateSetting } = this.props.ros
        updateSetting(this.state.rbx_namespace + "/settings", settingName, "Float", String(value))
      }
      const el = document.getElementById(event.target.id)
      if (el) {
        clearElementStyleModified(el)
      }
    }
  }


  // Editable X/Y/Z Float inputs for a camera_offset_*/scene_offset_* triple.
  // This is now the ONLY place these Settings are editable -- removed from
  // NepiDeviceRBX.js per request: camera positioning is a sim-only concept
  // (nothing to "offset" on a real camera the same way), so it belongs
  // exclusively here rather than duplicated on the generic RBX device panel.
  // Gated on that triple's own presence plus camera_controls_enabled. In
  // preview mode (not live, Show Settings clicked) both triples are assumed
  // present -- every current Gazebo-based driver (rbx_sim_node.py,
  // rbx_ardupilot_node.py) declares both; the Webots drivers only declare
  // camera_offset (one real camera, no scene view) and correctly narrow
  // down to just that once a live connection reports the real list.
  renderCameraOffsetControls(namePrefix, titlePrefix) {
    const live = this.isRbxLive()
    if (!this.state.show_settings) {
      return null
    }
    const settings = this.state.rbxSettingsNamesList
    const values = this.state.rbxSettingsValuesDict
    const camera_controls_enabled = !live || !settings.includes("camera_controls_enabled")
      || values["camera_controls_enabled"] !== "FALSE"
    if ((live && !settings.includes(namePrefix + "_x")) || !camera_controls_enabled) {
      return null
    }

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
              id={"SimRbx_" + offset.name}
              value={this.state[offset.name]}
              onChange={(event) => {
                const el = document.getElementById("SimRbx_" + offset.name)
                if (el) {
                  setElementStyleModified(el)
                }
                var obj = {}
                obj[offset.name] = event.target.value
                this.setState(obj)
              }}
              onKeyDown={(event) => this.onEnterSetRbxFloatSetting(event, offset.name)}
            />
          </Label>
        ))}
      </React.Fragment>
    )
  }

  // "Depth Map" toggle for the RBX driver's own depth_map_enabled Setting --
  // colorizes both camera views (close = blue, far = red) in place of plain
  // color. Same gating as renderCameraOffsetControls (live presence check,
  // camera_controls_enabled) since it's a sibling camera Setting, not a
  // capability toggle -- see rbx_sim_node.py's depth_map_enabled comment.
  renderDepthMapToggle() {
    const live = this.isRbxLive()
    if (!this.state.show_settings) {
      return null
    }
    const settings = this.state.rbxSettingsNamesList
    const values = this.state.rbxSettingsValuesDict
    const camera_controls_enabled = !live || !settings.includes("camera_controls_enabled")
      || values["camera_controls_enabled"] !== "FALSE"
    if ((live && !settings.includes("depth_map_enabled")) || !camera_controls_enabled) {
      return null
    }

    const { updateSetting } = this.props.ros
    return (
      <Label title={"Depth Map"}>
        <Toggle
          checked={values["depth_map_enabled"] === "TRUE"}
          onClick={() => {
            if (!live) {
              return
            }
            updateSetting(this.state.rbx_namespace + "/settings", "depth_map_enabled", "Discrete",
              values["depth_map_enabled"] === "TRUE" ? "FALSE" : "TRUE")
          }}
        />
      </Label>
    )
  }

  // "Flat Ground"/"Obstacle Course" dropdown for the RBX driver's own
  // "environment" Setting -- distinct from renderEnvironmentControls below
  // (a different, currently-dead-for-every-deployable-target mechanism, see
  // docs/SIM_CONNECTOR_CONFIG_CONTROLS_PLAN.md). Ported from
  // NepiDeviceRBX.js's onSelectEnvironment, same hardcoded label convention.
  renderEnvironmentSetting() {
    const live = this.isRbxLive()
    if (!this.state.show_settings) {
      return null
    }
    if (live && !this.state.rbxSettingsNamesList.includes("environment")) {
      return null
    }
    const { updateSetting } = this.props.ros
    return (
      <Label title={"Environment"}>
        <Select
          onChange={(event) => {
            const value = event.target.value
            if (!live) {
              // No driver namespace to send updateSetting to yet -- remember
              // the pick as pending so rbxSettingsListener can send it the
              // moment this device goes live, instead of it being lost.
              this.setState({ selected_environment_setting: value, environment_setting_pending: value })
              return
            }
            this.setState({ selected_environment_setting: value, environment_setting_pending: null })
            updateSetting(this.state.rbx_namespace + "/settings", "environment", "Discrete",
              value === "Obstacle Course" ? "OBSTACLE_COURSE" : "FLAT_GROUND")
          }}
          value={this.state.selected_environment_setting}
        >
          <Option value="Flat Ground">{"Flat Ground"}</Option>
          <Option value="Obstacle Course">{"Obstacle Course"}</Option>
        </Select>
      </Label>
    )
  }

  // Live preview of the currently selected_simulator's own robot/scene
  // camera feeds -- requested live (2026-08-18) as "another camera viewer
  // panel in the sim connector, just like how it is in the robot scene."
  // Points at sim_connector_app_node.py's own six-topic mirror set (always
  // the currently selected robot's real feeds, re-pointed automatically as
  // selected_simulator changes -- see that node's own comment) rather than
  // duplicating a topic-selector here; the only per-column choice is color
  // vs. depth (renamed from the old, now-dead robot_view/scene_view names
  // 2026-08-20, when those became two of six always-live topics instead of
  // one toggled feed -- found live: the stale names left this panel
  // permanently subscribed to publisher-less topics). this.props.namespace
  // is ".../app_sim_connector/sim" (the SimDeviceIF sub-namespace); the
  // mirror topics are siblings of it under the app's own bare node
  // namespace. scene_view naturally shows nothing for a robot with no scene
  // camera (the Webots drivers) -- NepiIFImageViewer already renders a
  // blank/waiting state for a topic with no publisher, an honest reflection
  // of "this robot has no scene camera," not a bug.
  renderCommonImageViewer() {
    const live = this.isRbxLive()
    if (!live) {
      return null
    }
    const appNamespace = this.props.namespace.split('/sim')[0]
    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={"Camera Viewer"} labelStyle={{ fontWeight: 'bold' }}/>

        <Columns>
          <Column>
            <Label title={"Robot View"}>
              <Select
                onChange={(event) => this.setState({ robot_view_mode: event.target.value })}
                value={this.state.robot_view_mode}
              >
                <Option value={"robot_color"}>{"Color"}</Option>
                <Option value={"robot_depth"}>{"Depth"}</Option>
              </Select>
            </Label>
            <NepiIFImageViewer
              id={"simConnectorRobotViewViewer"}
              image_topic={appNamespace + "/" + this.state.robot_view_mode}
              title={""}
            />
          </Column>
          <Column>
            <Label title={"Scene View"}>
              <Select
                onChange={(event) => this.setState({ scene_view_mode: event.target.value })}
                value={this.state.scene_view_mode}
              >
                <Option value={"scene_color"}>{"Color"}</Option>
                <Option value={"scene_depth"}>{"Depth"}</Option>
              </Select>
            </Label>
            <NepiIFImageViewer
              id={"simConnectorSceneViewViewer"}
              image_topic={appNamespace + "/" + this.state.scene_view_mode}
              title={""}
            />
          </Column>
        </Columns>

      </React.Fragment>
    )
  }

  // Sends a real (option, enabled) pair, JSON-encoded onto the existing
  // std_msgs/String topic -- see device_if_sim.py's setEnvironmentOptionCb for
  // the matching decode side. Flips local toggle state optimistically; there is
  // no status field reporting real server-side on/off state to reconcile against.
  toggleEnvironmentOption(option) {
    const namespace = this.props.namespace
    const { sendStringMsg } = this.props.ros
    const current = this.state.environment_option_enabled[option] === true
    const next = !current
    sendStringMsg(namespace + "/set_environment_option",
      JSON.stringify({ option: option, enabled: next }))
    this.setState({
      environment_option_enabled: {
        ...this.state.environment_option_enabled,
        [option]: next
      }
    })
  }

  // Whether a real RBX driver is currently connected and reporting its own
  // Settings -- the one signal every render* method below actually needs.
  // Kept as a single named helper rather than repeating the same null/''/
  // 'None' check inline everywhere (six call sites before this change).
  isRbxLive() {
    const rbx_ns = this.state.rbx_namespace
    return !(rbx_ns === null || rbx_ns === '' || rbx_ns === 'None')
  }

  toggleShowSettings() {
    this.setState({ show_settings: !this.state.show_settings })
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

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={"Environment"} labelStyle={{ fontWeight: 'bold' }}/>

        <ButtonMenu>
          {options.map((option) => {
            const isOn = this.state.environment_option_enabled[option] === true
            return (
              <Button
                key={option}
                style={isOn ? { fontWeight: 'bold', textDecoration: 'underline' } : {}}
                onClick={() => this.toggleEnvironmentOption(option)}
              >{option + (isOn ? " (on)" : " (off)")}</Button>
            )
          })}
        </ButtonMenu>

      </React.Fragment>
    )
  }

  // Live control: directly commands the robot right now (motor ratios, goto
  // setpoints, home/stop/setup actions, the live camera preview). This is
  // the surface `show_live_controls={false}` hides -- "the sim connector's
  // job is standing up the right sim/robot config, not direct control...
  // that lives in Devices -> Robots" (see NepiAppSimConnector.js).
  renderLiveControls() {
    return (
      <React.Fragment>
        {this.renderMotorControls()}
        {this.renderGotoControls()}
        {this.renderHomeControls()}
        {this.renderCameraControls()}
      </React.Fragment>
    )
  }

  // Configuration: decides WHICH control surfaces show up at all, here and
  // in Devices -> Robots, for the currently connected RBX driver -- this is
  // this app's actual point, and always renders regardless of
  // show_live_controls. See docs/SIM_CONNECTOR_CONFIG_CONTROLS_PLAN.md.
  renderConfigControls() {
    return (
      <React.Fragment>
        {this.renderCommonImageViewer()}
        {this.renderRobotCapabilityControls()}
        <Columns>
          <Column>
            {this.renderCameraOffsetControls("camera_offset", "Robot View Camera")}
          </Column>
          <Column>
            {this.renderCameraOffsetControls("scene_offset", "Scene View Camera")}
          </Column>
        </Columns>
        <Columns>
          <Column>
            {this.renderDepthMapToggle()}
          </Column>
          <Column>
            {this.renderEnvironmentSetting()}
          </Column>
        </Columns>
        {this.renderEnvironmentControls()}
      </React.Fragment>
    )
  }

  renderControls() {
    const namespace = this.props.namespace
    const show_live_controls = (this.props.show_live_controls !== undefined)
      ? this.props.show_live_controls : true
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

        {(show_live_controls === true) ? this.renderLiveControls() : null}

        {/* Always offered, live or not -- show_settings auto-opens on the
            not-live -> live transition (see componentDidUpdate) so the
            default behavior is unchanged (settings visible as soon as a
            simulator connects), but the operator can still collapse this
            whole section afterward if the robot/scene image views and
            controls are taking up space they want back. Previously hidden
            entirely once live, on the assumption there was "nothing left to
            reveal" -- true for visibility, but that also meant no way to
            HIDE it again once a sim was running (found live 2026-08-19). */}
        <Label title={"Sim Control Settings"}>
          <Button onClick={this.toggleShowSettings}>
            {(this.state.show_settings === true) ? "Hide Settings" : "Show Settings"}
          </Button>
        </Label>

        {this.renderConfigControls()}

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
