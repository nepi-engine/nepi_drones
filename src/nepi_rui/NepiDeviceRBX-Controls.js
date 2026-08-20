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
import Button, { ButtonMenu } from "./Button"
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Input from "./Input"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"
import { SliderAdjustment } from "./AdjustmentWidgets"


import { onEnterSetStateFloatValue, createMenuListFromStrList, onUpdateSetStateValue, onDropdownSelectedSetState, setElementStyleModified, clearElementStyleModified } from "./Utilities"
import { buildTeleopKeyMap, loadTeleopBindings } from "./TeleopKeymap"

// Motors above this ratio are considered high speed for the props-off warning
const MOTOR_WARNING_RATIO = 0.5

// Teleop key bindings -- each entry is a [linear_x, linear_y, linear_z,
// angular_z] RATIO contribution (summed across every currently-held mapped
// key, then clamped to [-1,1] per axis, so e.g. holding two keys together
// drives forward while turning). Forward/back and turn-left/turn-right work
// identically for a rover or a drone; strafe and altitude are axes only a
// drone has any meaning for -- rbx_sim_node.py's own setTeleopVelocity
// ignores linear_y/linear_z entirely for the rover.
//
// Rebindable from the Sim Connector's keybind editor (TeleopKeymap.js is the
// shared source of truth for both sides) -- this is no longer a fixed
// constant, it's rebuilt fresh every time teleop key capture starts so an
// edit made before switching a robot's control type to Teleop takes effect
// without a page reload.
//
// How often a still-held key re-sends its command (ms). Comfortably inside
// rbx_sim_node.py's TELEOP_CMD_TIMEOUT_SEC (750ms) so a single dropped
// WebSocket frame is invisible in normal operation.
const TELEOP_SEND_INTERVAL_MS = 200

// True if two arrays have the same length and elements, used to detect when
// locally-tracked slider values actually changed (vs. an unrelated re-render)
// so the editable percent text boxes only get resynced when they should be.
function sameNumberArray(a, b) {
  if (a === b) {
    return true
  }
  if (!a || !b || a.length !== b.length) {
    return false
  }
  for (var i = 0; i < a.length; i++) {
    if (a[i] !== b[i]) {
      return false
    }
  }
  return true
}

@inject("ros")
@observer

// Component that contains RBX Process Controls
class NepiDeviceControls extends Component {
  constructor(props) {
    super(props)

    // these states track the values through Status messages
    this.state = {
      show_process_controls: true,

      // current_* pose values are sourced from the device NavPose topic, not
      // from DeviceRBXStatus (which no longer carries them).
      current_lat: null,
      current_long: null,
      current_altitude: null,
      current_heading: null,
      current_roll: null,
      current_pitch: null,
      current_yaw: null,
      process_current: null,
      process_last: null,
      ready: null,
      battery: null,
      errors_current_x: null,
      errors_current_y: null,
      errors_current_z: null,
      errors_current_heading_deg: null,
      errors_current_roll_deg: null,
      errors_current_pitch_deg: null,
      errors_current_yaw_deg: null,
      errors_prev: [0, 0, 0, 0, 0, 0, 0],
      cmd_success: null,
      manual_ready: null,
      teleop_ready: null,
      autonomous_ready: null,
      last_cmd_str: null,
      last_error_message: null,

      rbx_capabilities: null,
      has_battery_feedback: null,
      has_manual_controls: null,
      has_autonomous_controls: null,
      has_set_home: null,
      has_go_home: null,
      has_go_stop: null,
      has_goto_pose: null,
      has_goto_position: null,
      has_goto_location: null,
      has_fake_gps: null,

      action_options: null,
      data_products: null,

      controls_type_list: null,
      controls_type_menu: null,
      selected_control_type: null,

      controls_auto_list: null,
      controls_auto_menu: null,
      selected_auto_control: null,
      actions_list: null,
      actions_menu: null,
      selected_go_action: null,
      selected_go_action_index: 0,

      controlsStatusListener: null,
      navposeListener: null,

      // Teleop (keyboard-driven) movement. teleop_active_keys is a plain
      // {key: true} map (not a Set -- setState/React works with plain
      // objects, and a Set here would need extra plumbing for no benefit)
      // of every currently-held mapped key, recomputed into a single
      // (linear_x, linear_y, linear_z, angular_z) vector and re-sent on an
      // interval WHILE any mapped key is held (teleop_send_timer) -- this is
      // the client side of the same "keep re-asserting, never trust a single
      // packet" reasoning rbx_sim_node.py's own comments give for its
      // TELEOP_CMD_TIMEOUT_SEC self-healing window; a single dropped
      // WebSocket frame must not leave the robot moving forever.
      teleop_active_keys: {},
      teleop_send_timer: null,
      // Whether this component currently owns the window keydown/keyup
      // listeners -- only true while "Teleop" is the selected control type,
      // so keyboard input elsewhere on the page (typing in an unrelated text
      // box) is never hijacked as a drive command.
      teleop_capture_active: false,
      // key -> ratio vector, rebuilt from current bindings (possibly rebound
      // via the Sim Connector's keybind editor) each time capture starts --
      // see startTeleopKeyCapture and TeleopKeymap.js.
      teleop_key_map: buildTeleopKeyMap(),

      roll_deg: 0,
      pitch_deg: 0,
      yaw_deg_pose: 0,

      x_meters: 0,
      y_meters: 0,
      z_meters: 0,
      yaw_deg_position: 0,

      location_lat: 0,
      location_long: 0,
      altitude_meters: 0,
      yaw_deg_location: 0,

      // Manual motor control. motor_control_settings mirrors the status
      // message's current_motor_control_settings (MotorControl[]) -- used only
      // to learn the motor count and each motor's index, and to seed initial
      // values once. The actual slider positions are tracked locally in
      // motor_slider_values/main_slider_value so dragging is instantly
      // responsive instead of waiting on a round trip through the status topic.
      motor_control_settings: [],
      motor_slider_values: [],
      main_slider_value: 0,

      // Editable-text mirrors of the sliders above, so a percent can be typed
      // directly instead of dragged. Kept in sync with the slider values in
      // componentDidUpdate, following the same seed/track pattern as the
      // sliders themselves; onChange/onKeyDown manage local edits (see the
      // Editable Input Box Pattern in nepi_rui/CLAUDE.md).
      main_slider_text: "0",
      motor_slider_text: [],

      // motor_test_timeout_s setting, surfaced here (in addition to the
      // generic Settings panel) since it directly controls the motor
      // sliders' behavior.
      settingsStatusListener: null,
      motor_test_timeout_value: "",
      motor_test_timeout_input: "",

    }


    this.updateControlsStatusListener = this.updateControlsStatusListener.bind(this)
    this.controlsStatusListener = this.controlsStatusListener.bind(this)
    this.updateNavposeListener = this.updateNavposeListener.bind(this)
    this.navposeListener = this.navposeListener.bind(this)

    this.onDropdownSelectedAction = this.onDropdownSelectedAction.bind(this)
    this.sendGoActionIndex = this.sendGoActionIndex.bind(this)
    this.setLocationToCurrent = this.setLocationToCurrent.bind(this)
    this.doNothing = this.doNothing.bind(this)

    this.motorsInSync = this.motorsInSync.bind(this)
    this.motorsAboveWarningRatio = this.motorsAboveWarningRatio.bind(this)
    this.sendMainSliderRatio = this.sendMainSliderRatio.bind(this)
    this.sendMotorRatio = this.sendMotorRatio.bind(this)
    this.syncMotorsToMain = this.syncMotorsToMain.bind(this)
    this.turnOffAllMotors = this.turnOffAllMotors.bind(this)

    this.onMainPercentInputChange = this.onMainPercentInputChange.bind(this)
    this.onMainPercentInputSubmit = this.onMainPercentInputSubmit.bind(this)
    this.onMotorPercentInputChange = this.onMotorPercentInputChange.bind(this)
    this.onMotorPercentInputSubmit = this.onMotorPercentInputSubmit.bind(this)

    this.updateSettingsStatusListener = this.updateSettingsStatusListener.bind(this)
    this.settingsStatusListener = this.settingsStatusListener.bind(this)
    this.onMotorTestTimeoutInputChange = this.onMotorTestTimeoutInputChange.bind(this)
    this.onMotorTestTimeoutInputSubmit = this.onMotorTestTimeoutInputSubmit.bind(this)

    this.handleTeleopKeyDown = this.handleTeleopKeyDown.bind(this)
    this.handleTeleopKeyUp = this.handleTeleopKeyUp.bind(this)
    this.sendTeleopVector = this.sendTeleopVector.bind(this)
    this.startTeleopKeyCapture = this.startTeleopKeyCapture.bind(this)
    this.stopTeleopKeyCapture = this.stopTeleopKeyCapture.bind(this)
    this.renderTeleopControls = this.renderTeleopControls.bind(this)
  }

  // No-op helper used by guarded command buttons when the robot is not ready.
  doNothing() { }

  // Sums every currently-held mapped key's contribution into one
  // [linear_x, linear_y, linear_z, angular_z] vector (clamped per axis to
  // [-1,1], so opposite keys held together cancel rather than exceeding the
  // ratio range) and sends it. Called on every keydown/keyup (so a release or
  // a new key press takes effect immediately) AND on the resend timer while
  // any mapped key stays held.
  sendTeleopVector(activeKeys) {
    const { sendTeleopVelocityMsg } = this.props.ros
    const namespace = this.props.rbxNamespace
    if (namespace == null || namespace === 'None') {
      // Held-key UI state (teleop_active_keys, driving the "W (held)" label
      // in renderTeleopControls) is purely local React state -- it renders
      // correctly regardless of whether rbxNamespace actually resolved, so
      // this silent return used to look identical to "broken" from the RUI:
      // holding a key showed as held with zero visible effect and no way to
      // tell "no device selected in this page's own dropdown" apart from an
      // actual bug. Surfaced the same way last_error_message already is.
      if (this.state.last_error_message !== "No robot selected -- pick one above before using Teleop") {
        this.setState({ last_error_message: "No robot selected -- pick one above before using Teleop" })
      }
      return
    }
    var vec = [0, 0, 0, 0]
    Object.keys(activeKeys).forEach((key) => {
      const contribution = this.state.teleop_key_map[key]
      if (contribution) {
        for (var i = 0; i < 4; i++) {
          vec[i] += contribution[i]
        }
      }
    })
    vec = vec.map((v) => Math.max(-1, Math.min(1, v)))
    sendTeleopVelocityMsg(namespace, vec[0], vec[1], vec[2], vec[3])
  }

  handleTeleopKeyDown(event) {
    const key = event.key.toLowerCase()
    if (!(key in this.state.teleop_key_map)) {
      return
    }
    // Stop the browser's own scrolling on the arrow keys/space -- this panel
    // owns them while Teleop is selected.
    event.preventDefault()
    if (this.state.teleop_active_keys[key]) {
      return // Already held (key-repeat) -- nothing new to send or start.
    }
    const keys = Object.assign({}, this.state.teleop_active_keys, { [key]: true })
    this.setState({ teleop_active_keys: keys })
    this.sendTeleopVector(keys)
    if (!this.state.teleop_send_timer) {
      const timer = setInterval(() => this.sendTeleopVector(this.state.teleop_active_keys), TELEOP_SEND_INTERVAL_MS)
      this.setState({ teleop_send_timer: timer })
    }
  }

  handleTeleopKeyUp(event) {
    const key = event.key.toLowerCase()
    if (!(key in this.state.teleop_key_map)) {
      return
    }
    const keys = Object.assign({}, this.state.teleop_active_keys)
    delete keys[key]
    this.setState({ teleop_active_keys: keys })
    this.sendTeleopVector(keys)
    if (Object.keys(keys).length === 0 && this.state.teleop_send_timer) {
      clearInterval(this.state.teleop_send_timer)
      this.setState({ teleop_send_timer: null })
    }
  }

  startTeleopKeyCapture() {
    if (this.state.teleop_capture_active) {
      return
    }
    // Refresh from current bindings every time capture starts -- picks up
    // any rebind made via the Sim Connector's keybind editor since the last
    // time Teleop was selected, without needing a page reload.
    window.addEventListener('keydown', this.handleTeleopKeyDown)
    window.addEventListener('keyup', this.handleTeleopKeyUp)
    this.setState({ teleop_capture_active: true, teleop_key_map: buildTeleopKeyMap() })
  }

  stopTeleopKeyCapture() {
    if (!this.state.teleop_capture_active) {
      return
    }
    window.removeEventListener('keydown', this.handleTeleopKeyDown)
    window.removeEventListener('keyup', this.handleTeleopKeyUp)
    if (this.state.teleop_send_timer) {
      clearInterval(this.state.teleop_send_timer)
    }
    // Explicit stop -- whatever was held when the panel closed must not keep
    // driving after keyboard capture ends.
    this.sendTeleopVector({})
    this.setState({ teleop_capture_active: false, teleop_active_keys: {}, teleop_send_timer: null })
  }

  // Live indicator of which mapped keys are currently held, plus the key
  // legend -- shown only while "Teleop" is the selected control type (the
  // same block that gates the window listeners themselves in
  // componentDidUpdate).
  renderTeleopControls() {
    if (this.state.selected_control_type !== "Teleop") {
      return null
    }
    const held = this.state.teleop_active_keys
    // Bindings, not just this.state.teleop_key_map's keys -- reading them
    // fresh (rather than reversing teleop_key_map back into an action name)
    // keeps this legend correct even if a future rebind hasn't triggered a
    // capture restart yet. Cheap: a few localStorage reads on a legend that
    // only renders while Teleop is selected.
    const bindings = loadTeleopBindings()
    const heldLabel = (key) => (held[key] ? key.toUpperCase() + " (held)" : key.toUpperCase())
    // Driven by teleop_control_mode_ready (DeviceRBXStatus.msg), the same
    // driver-supplied teleopControlsReadyFunction every command already
    // gets checked against -- NOT a hardcoded "armed + GUIDED + airborne"
    // assumption here, since a rover's own readiness criteria (bridge
    // connected, teleop_movement_enabled) look nothing like a drone's. The
    // driver decides what "ready" means for whatever it's actually
    // talking to; this just displays that decision live, continuously,
    // rather than only ever surfacing a rejection reason after the fact
    // once a held key already silently did nothing (last_error_message
    // below still covers that, for a driver that hasn't been rebuilt with
    // this status field yet).
    const teleop_ready = this.state.teleop_ready === true
    return (
      <React.Fragment>
        <Label title={"Teleop Ready"}>
          <BooleanIndicator value={teleop_ready} />
        </Label>
        <Label title={"Teleop: click this page, then use the keys below"}/>
        <Label title={heldLabel(bindings.forward) + " forward, " + heldLabel(bindings.backward) + " back, "
                     + heldLabel(bindings.turn_right) + " turn right, " + heldLabel(bindings.turn_left) + " turn left"}/>
        <Label title={"Drone only -- " + heldLabel(bindings.strafe_left) + "/" + heldLabel(bindings.strafe_right)
                     + " strafe left/right, " + heldLabel(bindings.altitude_up) + "/" + heldLabel(bindings.altitude_down)
                     + " altitude up/down"}/>
        <Label title={"Keybinds can be changed in the Sim Connector app"}/>
        {/* Held keys send commands regardless of whether the vehicle is
            actually in a state that can act on them (e.g. a drone must be
            armed, in GUIDED mode, and airborne) -- the driver rejects a
            not-ready command silently over the wire, so without this the
            RUI shows "W (held)" forever with no visible effect and no way
            to tell "rejected" apart from "broken". last_error_message is
            already tracked in state (controlsStatusListener) but was never
            rendered anywhere before this. */}
        {(this.state.last_error_message) ?
          <Label title={"Last error: " + this.state.last_error_message}/>
        : null}
      </React.Fragment>
    )
  }


  // Callback for handling ROS DeviceRBXStatus messages
  controlsStatusListener(message) {
    const { rbxDevices } = this.props.ros
    this.setState({
      process_current: message.process_current,
      process_last: message.process_last,
      ready: message.ready,
      battery: message.battery,
      errors_current_x: [message.errors_current.x_m],
      errors_current_y: [message.errors_current.y_m],
      errors_current_z: [message.errors_current.z_m],
      errors_current_heading_deg: [message.errors_current.heading_deg],
      errors_current_roll_deg: [message.errors_current.roll_deg],
      errors_current_pitch_deg: [message.errors_current.pitch_deg],
      errors_current_yaw_deg: [message.errors_current.yaw_deg],
      errors_prev: [message.errors_prev.x_m, message.errors_prev.y_m, message.errors_prev.z_m, message.errors_prev.heading_deg, message.errors_prev.roll_deg, message.errors_prev.pitch_deg, message.errors_prev.yaw_deg],
      cmd_success: message.cmd_success,
      manual_ready: message.manual_control_mode_ready,
      teleop_ready: message.teleop_control_mode_ready,
      autonomous_ready: message.autonomous_control_mode_ready,
      last_cmd_str: message.last_cmd_string,
      last_error_message: message.last_error_message,
      motor_control_settings: message.current_motor_control_settings ? message.current_motor_control_settings : []
    })
    // Was gated on `this.state.rbx_capabilities === null` -- fetched once and
    // never resynced, so a capability toggled later from the Sim Connector app
    // (has_autonomous_controls for "automated movement", etc.) never reached
    // this panel. NepiDeviceRBX.js's capabilitiesPollTimer keeps
    // rbxDevices[namespace] itself fresh; this component now just needs to
    // stop ignoring updates after the first. Everything below is a pure
    // re-derivation from `capabilities`, safe to re-run on every status tick.
    {
      const capabilities = rbxDevices[this.props.rbxNamespace]
      if (capabilities) {
        const actions = (capabilities.go_action_options !== undefined && capabilities.go_action_options !== null) ? capabilities.go_action_options : []
        const actions_menu_options = createMenuListFromStrList(actions, false, [], [], [])

        this.setState({
          rbx_capabilities: capabilities,
          has_battery_feedback: capabilities.has_battery_feedback,
          has_manual_controls: capabilities.has_manual_controls,
          has_autonomous_controls: capabilities.has_autonomous_controls,
          has_set_home: capabilities.has_set_home,
          has_go_home: capabilities.has_go_home,
          has_go_stop: capabilities.has_go_stop,
          has_goto_pose: capabilities.has_goto_pose,
          has_goto_position: capabilities.has_goto_position,
          has_goto_location: capabilities.has_goto_location,
          has_fake_gps: capabilities.has_fake_gps,
          action_options: capabilities.go_action_options,
          actions_list: actions,
          actions_menu: actions_menu_options,
          data_products: capabilities.data_products,
        })

        // A robot TYPE supporting autonomous movement (has_autonomous_controls)
        // is a different question from whether THIS deployment wants it
        // exposed right now -- e.g. a Sim Connector robot config with
        // "automated movement" unchecked. That's a Setting
        // (autonomous_movement_enabled), same mechanism as camera_offset_x,
        // not a capability -- a real robot that never defines this Setting at
        // all keeps behaving exactly as before (settingEnabled treats an
        // absent Setting as enabled, never as a reason to hide something a
        // capability already says exists).
        const settingsNamesList = (this.props.settingsNamesList !== undefined) ? this.props.settingsNamesList : []
        const settingsValuesDict = (this.props.settingsValuesDict !== undefined) ? this.props.settingsValuesDict : {}
        const settingEnabled = (name) => {
          if (!settingsNamesList.includes(name)) {
            return true
          }
          return settingsValuesDict[name] !== "FALSE"
        }
        const autonomous_movement_enabled = settingEnabled("autonomous_movement_enabled")
        // Teleop has no capabilities-query flag at all (deliberately -- see
        // device_if_rbx.py's setTeleopVelocityFunction comment): its Setting's
        // mere presence is both "does this driver support it" and, via
        // settingEnabled, "is it turned on for this deployment".
        const has_teleop_controls = settingsNamesList.includes("teleop_movement_enabled")
                                    && settingEnabled("teleop_movement_enabled")

        var controls_type_list = ["None"]
        if (capabilities.has_manual_controls) {
          controls_type_list.push("Manual")
        }
        if (capabilities.has_autonomous_controls && autonomous_movement_enabled) {
          controls_type_list.push("Autonomous")
        }
        if (has_teleop_controls) {
          controls_type_list.push("Teleop")
        }
        const controls_type_menu = createMenuListFromStrList(controls_type_list, false, [], [], [])

        var controls_auto_list = ["None"]
        if (actions.length > 0) {
          controls_auto_list.push("Action")
        }
        if (capabilities.has_goto_pose && autonomous_movement_enabled) {
          controls_auto_list.push("Pose")
        }
        if (capabilities.has_goto_position && autonomous_movement_enabled) {
          controls_auto_list.push("Position")
        }
        if (capabilities.has_goto_location && autonomous_movement_enabled) {
          controls_auto_list.push("Location")
        }
        const controls_auto_menu = createMenuListFromStrList(controls_auto_list, false, [], [], [])

        this.setState({
          controls_type_list: controls_type_list,
          controls_type_menu: controls_type_menu,
          controls_auto_list: controls_auto_list,
          controls_auto_menu: controls_auto_menu
        })

        // A capability can now disappear out from under an already-selected
        // dropdown value (e.g. "automated movement" toggled off from the Sim
        // Connector while "Autonomous" is the current control type) -- reset
        // to a valid selection rather than leaving a stale one whose controls
        // no longer make sense to show.
        if (this.state.selected_control_type !== null
            && !controls_type_list.includes(this.state.selected_control_type)) {
          this.setState({ selected_control_type: "None" })
        }
        if (this.state.selected_auto_control !== null
            && !controls_auto_list.includes(this.state.selected_auto_control)) {
          this.setState({ selected_auto_control: "None" })
        }
      }
    }
  }

  // Callback for handling NavPose messages. The current-position values used by
  // the "Set to Current" goto helper come from the device's NavPose topic.
  navposeListener(message) {
    this.setState({
      current_lat: message.latitude,
      current_long: message.longitude,
      current_altitude: message.altitude_m,
      current_heading: message.heading_deg,
      current_roll: message.roll_deg,
      current_pitch: message.pitch_deg,
      current_yaw: message.yaw_deg
    })
  }

  // Function for configuring and subscribing to DeviceRBXStatus
  updateControlsStatusListener() {
    const namespace = this.props.rbxNamespace
    if (this.state.controlsStatusListener) {
      this.state.controlsStatusListener.unsubscribe()
    }
    var listener = this.props.ros.setupRBXStatusListener(
      namespace,
      this.controlsStatusListener
    )
    this.setState({ controlsStatusListener: listener })
  }

  // Function for configuring and subscribing to the device NavPose topic
  updateNavposeListener() {
    const navposeTopic = this.props.rbxNamespace.split('/rbx')[0] + "/npx/navpose"
    if (this.state.navposeListener) {
      this.state.navposeListener.unsubscribe()
    }
    var navposeListener = this.props.ros.setupStatusListener(
      navposeTopic,
      "nepi_interfaces/NavPose",
      this.navposeListener
    )
    this.setState({ navposeListener: navposeListener })
  }

  // Function for configuring and subscribing to the device's Settings status
  // (used here only to surface motor_test_timeout_s next to the motor
  // sliders it governs -- the full settings editor still lives in the
  // generic Settings panel on the main RBX device page).
  updateSettingsStatusListener() {
    const settingsNamespace = this.props.rbxNamespace + "/settings"
    if (this.state.settingsStatusListener) {
      this.state.settingsStatusListener.unsubscribe()
    }
    var settingsStatusListener = this.props.ros.setupSettingsStatusListener(
      settingsNamespace + "/status",
      this.settingsStatusListener
    )
    this.setState({ settingsStatusListener: settingsStatusListener })
  }

  // Callback for handling ROS SettingsStatus messages. Only syncs the
  // editable input when the confirmed backend value actually changes, so it
  // doesn't stomp on text the user is actively typing.
  settingsStatusListener(message) {
    const settings = message.settings_list ? message.settings_list : []
    var timeout_setting = null
    for (var i = 0; i < settings.length; i++) {
      if (settings[i].name_str === "motor_test_timeout_s") {
        timeout_setting = settings[i]
      }
    }
    if (timeout_setting !== null && timeout_setting.value_str !== this.state.motor_test_timeout_value) {
      this.setState({
        motor_test_timeout_value: timeout_setting.value_str,
        motor_test_timeout_input: timeout_setting.value_str
      })
    }
  }

  // Lifecycle method called on mount. Subscribes for the initially
  // selected device.
  componentDidMount() {
    const { rbxNamespace } = this.props
    if (rbxNamespace && rbxNamespace.indexOf('null') === -1) {
      this.updateControlsStatusListener()
      this.updateNavposeListener()
      this.updateSettingsStatusListener()
    }
  }

  // Lifecycle method called when component updates.
  // Used to track changes in the selected device.
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { rbxNamespace } = this.props
    if (prevProps.rbxNamespace !== rbxNamespace && rbxNamespace !== null) {
      if (rbxNamespace.indexOf('null') === -1) {
        this.updateControlsStatusListener()
        this.updateNavposeListener()
        this.updateSettingsStatusListener()
      }
    }
    // Seed the local slider values from the status message exactly once
    // (whenever the motor count first becomes known, or changes) -- after
    // that, the sliders are locally authoritative so dragging is smooth and
    // doesn't fight the slower async status echo on every tick.
    if (this.state.motor_control_settings.length > 0 &&
      this.state.motor_control_settings.length !== this.state.motor_slider_values.length) {
      const seeded_values = this.state.motor_control_settings.map((motor) => Math.round(motor.speed_ratio * 100))
      const all_equal = seeded_values.every((v) => v === seeded_values[0])
      this.setState({
        motor_slider_values: seeded_values,
        main_slider_value: all_equal ? seeded_values[0] : this.state.main_slider_value
      })
    }
    // Keep the typeable percent text boxes in sync whenever the underlying
    // slider values actually change (dragging, sync, turn-off, or a typed
    // value being submitted elsewhere) -- but never touch them otherwise, so
    // this doesn't fight a value the user is actively typing.
    if (prevState.main_slider_value !== this.state.main_slider_value) {
      this.setState({ main_slider_text: String(this.state.main_slider_value) })
    }
    if (!sameNumberArray(prevState.motor_slider_values, this.state.motor_slider_values)) {
      this.setState({ motor_slider_text: this.state.motor_slider_values.map((v) => String(v)) })
    }
    // Keyboard capture follows the selected control type, not the other way
    // around: leaving "Teleop" (switching to Manual/Autonomous/None, OR the
    // capability disappearing out from under the current selection -- see the
    // has_teleop_controls-driven reset above) must always release the window
    // keydown/keyup listeners, the same way navigating away from any other
    // page-level keyboard shortcut would.
    if (prevState.selected_control_type !== this.state.selected_control_type) {
      if (this.state.selected_control_type === "Teleop") {
        this.startTeleopKeyCapture()
      } else {
        this.stopTeleopKeyCapture()
      }
    }
  }

  // Lifecycle method called just before the component unmounts.
  // Used to unsubscribe from Status messages
  componentWillUnmount() {
    // Release the window keyboard listeners and stop the robot -- navigating
    // away from this page must not leave a teleop command latched.
    this.stopTeleopKeyCapture()
    if (this.state.controlsStatusListener) {
      this.state.controlsStatusListener.unsubscribe()
    }
    if (this.state.navposeListener) {
      this.state.navposeListener.unsubscribe()
    }
    if (this.state.settingsStatusListener) {
      this.state.settingsStatusListener.unsubscribe()
    }
  }


  sendGoActionIndex() {
    const { sendIntMsg } = this.props.ros
    const namespace = this.props.rbxNamespace + "/go_action"
    if (this.state.selected_go_action_index !== null) {
      sendIntMsg(namespace, this.state.selected_go_action_index)
    }
  }

  onDropdownSelectedAction(event) {
    this.setState({
      selected_go_action: event.target.value,
      selected_go_action_index: event.target.selectedIndex
    })
  }

  setLocationToCurrent(event) {
    this.setState({
      location_lat: this.state.current_lat,
      location_long: this.state.current_long,
      altitude_meters: this.state.current_altitude,
      yaw_deg_location: this.state.current_yaw,
    })
  }

  // True when every motor's (locally tracked) slider value agrees (or there
  // are 0-1 motors) -- the main slider only has one meaningful shared value
  // to show and drive when this is true. Checked against the local slider
  // values, not the async status echo, so this updates instantly while
  // dragging instead of lagging a status round trip behind.
  motorsInSync() {
    const values = this.state.motor_slider_values
    if (!values || values.length <= 1) {
      return true
    }
    const first_value = values[0]
    return values.every((v) => v === first_value)
  }

  // True only when every motor's slider is currently set above the warning
  // threshold -- drives the high-speed / props-off warning.
  motorsAboveWarningRatio() {
    const values = this.state.motor_slider_values
    if (!values || values.length === 0) {
      return false
    }
    return values.every((v) => (v / 100) > MOTOR_WARNING_RATIO)
  }

  // Publishes one motor's ratio (0-1) to set_motor_control.
  publishMotorRatio(motor_ind, ratio) {
    const { sendMotorControlMsg } = this.props.ros
    sendMotorControlMsg(this.props.rbxNamespace + "/set_motor_control", motor_ind, ratio)
  }

  // Main slider onChange while motors agree with each other -- moves every
  // motor together in real time. Updates local state immediately (instant,
  // smooth visual feedback) and publishes the command in the same step.
  sendMainSliderRatio(new_percent) {
    const updated_values = this.state.motor_slider_values.map(() => new_percent)
    this.setState({ main_slider_value: new_percent, motor_slider_values: updated_values })
    const ratio = new_percent / 100
    this.state.motor_control_settings.forEach((motor) => this.publishMotorRatio(motor.motor_ind, ratio))
  }

  // Main slider onChange while motors disagree -- the slider stays draggable
  // (just greyed out) so the user can stage a target value, but doesn't touch
  // any motor until Sync is pressed.
  moveMainSliderPreview(new_percent) {
    this.setState({ main_slider_value: new_percent })
  }

  // Routes a main-slider move (drag or typed) to the right behavior above,
  // depending on whether the motors currently agree.
  setMainSliderTarget(new_percent) {
    const clamped = Math.max(-100, Math.min(100, new_percent))
    if (this.motorsInSync()) {
      this.sendMainSliderRatio(clamped)
    } else {
      this.moveMainSliderPreview(clamped)
    }
  }

  // Individual motor slider onChange. Same immediate-local-update pattern.
  sendMotorRatio(motor_ind, new_percent) {
    const updated_values = this.state.motor_slider_values.slice()
    updated_values[motor_ind] = new_percent
    this.setState({ motor_slider_values: updated_values })
    this.publishMotorRatio(motor_ind, new_percent / 100)
  }

  // Sync button: force every motor to match the main slider's current value.
  syncMotorsToMain() {
    const updated_values = this.state.motor_slider_values.map(() => this.state.main_slider_value)
    this.setState({ motor_slider_values: updated_values })
    const ratio = this.state.main_slider_value / 100
    this.state.motor_control_settings.forEach((motor) => this.publishMotorRatio(motor.motor_ind, ratio))
  }

  // Turn Off button: zero every motor immediately.
  turnOffAllMotors() {
    const updated_values = this.state.motor_slider_values.map(() => 0)
    this.setState({ main_slider_value: 0, motor_slider_values: updated_values })
    this.state.motor_control_settings.forEach((motor) => this.publishMotorRatio(motor.motor_ind, 0))
  }

  // Typed-percent handlers for the main slider -- onChange just tracks the
  // draft text (marked dirty via setElementStyleModified in the JSX), Enter
  // commits it through the same path a drag would use.
  onMainPercentInputChange(text) {
    this.setState({ main_slider_text: text })
  }

  onMainPercentInputSubmit(text) {
    const value = parseFloat(text)
    if (!isNaN(value)) {
      this.setMainSliderTarget(value)
    }
  }

  // Typed-percent handlers for an individual motor slider.
  onMotorPercentInputChange(i, text) {
    const updated_text = this.state.motor_slider_text.slice()
    updated_text[i] = text
    this.setState({ motor_slider_text: updated_text })
  }

  onMotorPercentInputSubmit(motor_ind, text) {
    const value = parseFloat(text)
    if (!isNaN(value)) {
      this.sendMotorRatio(motor_ind, Math.max(-100, Math.min(100, value)))
    }
  }

  // Typed-value handlers for the motor_test_timeout_s setting.
  onMotorTestTimeoutInputChange(text) {
    this.setState({ motor_test_timeout_input: text })
  }

  onMotorTestTimeoutInputSubmit(text) {
    const { updateSetting } = this.props.ros
    const value = parseFloat(text)
    if (!isNaN(value)) {
      updateSetting(this.props.rbxNamespace + "/settings", "motor_test_timeout_s", "Float", String(value))
    }
  }


  render() {
    const { sendTriggerMsg, sendFloatGotoPoseMsg, sendFloatGotoPositionMsg, sendFloatGotoLocationMsg } = this.props.ros
    const NoneOption = <Option>None</Option>
    const namespace = this.props.rbxNamespace
    // Ground rovers (e.g. RBX_SIM) report no roll/pitch control support --
    // hide the roll/pitch pose controls and error readouts for them without
    // affecting robots (e.g. drones) that do support roll/pitch. Defaults to
    // shown while capabilities haven't loaded yet, matching this file's
    // existing null-safe capability-gating pattern elsewhere.
    const control_support = (this.state.rbx_capabilities !== null) ? this.state.rbx_capabilities.control_support : null
    const has_roll = (control_support !== null && control_support !== undefined) ? (control_support.roll === true) : true
    const has_pitch = (control_support !== null && control_support !== undefined) ? (control_support.pitch === true) : true
    return (
      <Section title={"Process Controls"}>

        <div hidden={!this.state.show_process_controls}>

          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />


          <Columns>
            <Column>

              <Label title={"System Ready"}>
                <BooleanIndicator value={(this.state.ready !== null) ? this.state.ready : false} />
              </Label>

              <Label title={"Select Control Type"}>
                <Select
                  id="selected_control_type"
                  onChange={(event) => onDropdownSelectedSetState.bind(this)(event, "selected_control_type")}
                  value={this.state.selected_control_type}
                >
                  {this.state.controls_type_list ? this.state.controls_type_menu : NoneOption}
                </Select>
              </Label>


            </Column>
            <Column>

              <Label title={"Current Process"}>
                <Input
                  disabled value={this.state.process_current}
                  id="current_process"
                />
              </Label>

              <Label title={"Last Process"}>
                <Input
                  disabled value={this.state.process_last}
                  id="last_process"
                />
              </Label>

              <Label title={"Last Process Success"}>
                <BooleanIndicator value={(this.state.cmd_success !== null) ? this.state.cmd_success : false} />
              </Label>


            </Column>
          </Columns>

          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />


          <div hidden={(this.state.selected_control_type !== "Autonomous")}>
            <Columns>
              <Column>


                <div hidden={(this.state.selected_control_type !== "Autonomous")}>
                  <Label title={"Autonomous Ready"}>
                    <BooleanIndicator value={(this.state.autonomous_ready !== null) ? this.state.autonomous_ready : false} />
                  </Label>
                </div>

              </Column>
              <Column>

                {/* Was unconditional -- rendered whenever Autonomous mode was
                    selected, with no check on has_go_stop, unlike every
                    sibling button here (Go Home checks has_go_home). A robot
                    reporting has_go_stop: false still got a Stop button that
                    published go_stop into the void.
                    Visibility only, deliberately no click-time readiness gate
                    (unlike Go Home): device_if_rbx.py's own goStopCb is
                    explicitly documented as ungated -- "unlike
                    gotoPoseCb/gotoPositionCb/gotoLocationCb, this callback has
                    no 'another command is active' ready-check" -- because Stop
                    must interrupt an in-progress command, which is exactly
                    when status.ready is False. Gating the click here would
                    make Stop unresponsive at the one moment it exists to be
                    used. */}
                <div hidden={(!this.state.has_go_stop)}>
                  <ButtonMenu>
                    <Button onClick={() => sendTriggerMsg(namespace + "/go_stop")}>{"stop"}</Button>
                  </ButtonMenu>
                </div>

              </Column>
              <Column>

                <div hidden={(!this.state.has_go_home)}>
                  <ButtonMenu>
                    <Button onClick={() => this.state.autonomous_ready ?
                      sendTriggerMsg(namespace + "/go_home") :
                      this.doNothing()
                    }>{"Go Home"}</Button>
                  </ButtonMenu>
                </div>

              </Column>
            </Columns>
          </div>

          <div hidden={(this.state.selected_control_type !== "Teleop")}>
            <Columns>
              <Column>
                {this.renderTeleopControls()}
              </Column>
            </Columns>
          </div>

          <div hidden={(this.state.selected_control_type !== "Manual")}>
            <Columns>
              <Column>

                <Label title={"Manual Ready"}>
                  <BooleanIndicator value={(this.state.manual_ready !== null) ? this.state.manual_ready : false} />
                </Label>

                <div hidden={!this.motorsAboveWarningRatio()}>
                  <label style={{ color: Styles.vars.colors.red, fontWeight: 'bold' }}>
                    {"Warning: all motors above 50% speed -- props should be removed"}
                  </label>
                </div>

              </Column>
              <Column>

                <ButtonMenu>
                  <Button onClick={() => this.turnOffAllMotors()}>{"Turn Off All Motors"}</Button>
                </ButtonMenu>

              </Column>
            </Columns>

            <Label title={"Motor Test Timeout (s)"}>
              <Input
                id={"MotorTestTimeoutInput"}
                value={this.state.motor_test_timeout_input}
                onChange={(event) => {
                  setElementStyleModified(document.getElementById("MotorTestTimeoutInput"))
                  this.onMotorTestTimeoutInputChange(event.target.value)
                }}
                onKeyDown={(event) => {
                  if (event.key === 'Enter') {
                    clearElementStyleModified(document.getElementById("MotorTestTimeoutInput"))
                    this.onMotorTestTimeoutInputSubmit(event.target.value)
                  }
                }}
                style={{ width: "5em" }}
              />
            </Label>

            <Label title={""}></Label>

            <Columns>
              <Column>

                <div style={!this.motorsInSync() ? { opacity: 0.5 } : {}}>
                  <SliderAdjustment
                    disabled={this.state.manual_ready !== true}
                    title={"All Motors"}
                    tooltip={"Sets every motor's speed together. Greys out and stops applying changes when motors disagree -- drag to stage a target, then hit Sync to snap every motor to it."}
                    topic={namespace + "/set_motor_control"}
                    msgType={"nepi_interfaces/MotorControl"}
                    adjustment={this.state.main_slider_value}
                    onSliderChangeOverride={(value) => this.setMainSliderTarget(value)}
                    min={-100}
                    max={100}
                    unit={"%"}
                  />
                </div>

                <Label title={"Set All Motors %"}>
                  <Input
                    id={"MainMotorPercentInput"}
                    disabled={this.state.manual_ready !== true}
                    value={this.state.main_slider_text}
                    onChange={(event) => {
                      setElementStyleModified(document.getElementById("MainMotorPercentInput"))
                      this.onMainPercentInputChange(event.target.value)
                    }}
                    onKeyDown={(event) => {
                      if (event.key === 'Enter') {
                        clearElementStyleModified(document.getElementById("MainMotorPercentInput"))
                        this.onMainPercentInputSubmit(event.target.value)
                      }
                    }}
                    style={{ width: "4em" }}
                  />
                </Label>

                <div hidden={this.motorsInSync()}>
                  <ButtonMenu>
                    <Button onClick={() => this.syncMotorsToMain()}>{"Sync All Motors"}</Button>
                  </ButtonMenu>
                </div>

              </Column>
            </Columns>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

            <Columns>
              <Column>

                {this.state.motor_control_settings.map((motor, i) => (
                  <div key={motor.motor_ind}>
                    <SliderAdjustment
                      disabled={this.state.manual_ready !== true}
                      title={"Motor " + (motor.motor_ind + 1)}
                      topic={namespace + "/set_motor_control"}
                      msgType={"nepi_interfaces/MotorControl"}
                      adjustment={(this.state.motor_slider_values[i] !== undefined) ? this.state.motor_slider_values[i] : 0}
                      onSliderChangeOverride={(value) => this.sendMotorRatio(motor.motor_ind, value)}
                      min={-100}
                      max={100}
                      unit={"%"}
                    />
                    <Label title={"Set Motor " + (motor.motor_ind + 1) + " %"}>
                      <Input
                        id={"MotorPercentInput" + motor.motor_ind}
                        disabled={this.state.manual_ready !== true}
                        value={(this.state.motor_slider_text[i] !== undefined) ? this.state.motor_slider_text[i] : "0"}
                        onChange={(event) => {
                          setElementStyleModified(document.getElementById("MotorPercentInput" + motor.motor_ind))
                          this.onMotorPercentInputChange(i, event.target.value)
                        }}
                        onKeyDown={(event) => {
                          if (event.key === 'Enter') {
                            clearElementStyleModified(document.getElementById("MotorPercentInput" + motor.motor_ind))
                            this.onMotorPercentInputSubmit(motor.motor_ind, event.target.value)
                          }
                        }}
                        style={{ width: "4em" }}
                      />
                    </Label>
                  </div>
                ))}

              </Column>
            </Columns>

          </div>

          <Label title={""}></Label>

          <div hidden={(this.state.selected_control_type !== "Autonomous")}>
            <Columns>
              <Column>

                <Label title={"Select Goto Type"}>
                  <Select
                    id="selected_auto_control"
                    onChange={(event) => onDropdownSelectedSetState.bind(this)(event, "selected_auto_control")}
                    value={this.state.selected_auto_control}
                  >
                    {this.state.controls_auto_list ? this.state.controls_auto_menu : NoneOption}
                  </Select>
                </Label>


                <div hidden={(this.state.selected_auto_control !== "Pose")}>

                  <label style={{ fontWeight: 'bold' }}>
                    {"GoTo Pose (Body)"}
                  </label>


                  <div hidden={!has_roll}>
                    <Label title={"Roll Deg"}>
                      <Input
                        value={this.state.roll_deg}
                        id="roll_deg"
                        onChange={(event) => onUpdateSetStateValue.bind(this)(event, "roll_deg")}
                        onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "roll_deg")}
                        style={{ width: "80%" }}
                      />
                    </Label>
                  </div>

                  <div hidden={!has_pitch}>
                    <Label title={"Pitch Deg"}>
                      <Input
                        value={this.state.pitch_deg}
                        id="pitch_deg"
                        onChange={(event) => onUpdateSetStateValue.bind(this)(event, "pitch_deg")}
                        onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "pitch_deg")}
                        style={{ width: "80%" }}
                      />
                    </Label>
                  </div>

                  <Label title={"Yaw Deg"}>
                    <Input
                      value={this.state.yaw_deg_pose}
                      id="yaw_deg_pose"
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event, "yaw_deg_pose")}
                      onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "yaw_deg_pose")}
                      style={{ width: "80%" }}
                    />
                  </Label>

                  <ButtonMenu>
                    <Button onClick={() => this.state.autonomous_ready ?
                      sendFloatGotoPoseMsg(namespace + "/goto_pose", this.state.roll_deg, this.state.pitch_deg, this.state.yaw_deg_pose) :
                      this.doNothing()
                    }>{"Send"}</Button>
                  </ButtonMenu>
                </div>

                <div hidden={(this.state.selected_auto_control !== "Position")}>
                  <label style={{ fontWeight: 'bold' }}>
                    {"GoTo Position (Body)"}
                  </label>


                  <Label title={"Forward -> X (m)"}>
                    <Input
                      value={this.state.x_meters}
                      id="x_meters"
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event, "x_meters")}
                      onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "x_meters")}
                      style={{ width: "80%" }}
                    />
                  </Label>

                  <Label title={"Left -> Y (m)"}>
                    <Input
                      value={this.state.y_meters}
                      id="y_meters"
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event, "y_meters")}
                      onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "y_meters")}
                      style={{ width: "80%" }}
                    />
                  </Label>

                  <Label title={"Up -> Z (m)"}>
                    <Input
                      value={this.state.z_meters}
                      id="z_meters"
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event, "z_meters")}
                      onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "z_meters")}
                      style={{ width: "80%" }}
                    />
                  </Label>

                  <Label title={"Yaw Deg"}>
                    <Input
                      value={this.state.yaw_deg_position}
                      id="yaw_deg_position"
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event, "yaw_deg_position")}
                      onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "yaw_deg_position")}
                      style={{ width: "80%" }}
                    />
                  </Label>

                  <ButtonMenu>
                    <Button onClick={() => this.state.autonomous_ready ?
                      sendFloatGotoPositionMsg(namespace + "/goto_position", this.state.x_meters, this.state.y_meters, this.state.z_meters, this.state.yaw_deg_position) :
                      this.doNothing()
                    }>{"Send"}</Button>
                  </ButtonMenu>

                </div>

                <div hidden={(this.state.selected_auto_control !== "Location")}>
                  <label style={{ fontWeight: 'bold' }}>
                    {"GoTo Location (Geo WSG84)"}
                  </label>

                  <ButtonMenu>
                    <Button onClick={() => this.setLocationToCurrent()}>{"Set to Current"}</Button>
                  </ButtonMenu>


                  <Label title={"Latitude"}>
                    <Input
                      value={this.state.location_lat}
                      id="location_lat"
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event, "location_lat")}
                      onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "location_lat")}
                      style={{ width: "80%" }}
                    />
                  </Label>


                  <Label title={"Longitude"}>
                    <Input
                      value={this.state.location_long}
                      id="location_long"
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event, "location_long")}
                      onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "location_long")}
                      style={{ width: "80%" }}
                    />
                  </Label>

                  <Label title={"Altitude (m)"}>
                    <Input
                      value={this.state.altitude_meters}
                      id="altitude_meters"
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event, "altitude_meters")}
                      onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "altitude_meters")}
                      style={{ width: "80%" }}
                    />
                  </Label>

                  <Label title={"Yaw Deg"}>
                    <Input
                      value={this.state.yaw_deg_location}
                      id="yaw_deg_location"
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event, "yaw_deg_location")}
                      onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event, "yaw_deg_location")}
                      style={{ width: "80%" }}
                    />
                  </Label>



                  <ButtonMenu>
                    <Button onClick={() => this.state.autonomous_ready ?
                      sendFloatGotoLocationMsg(namespace + "/goto_location", this.state.location_lat, this.state.location_long, this.state.altitude_meters, this.state.yaw_deg_location) :
                      this.doNothing()
                    }>{"Send"}</Button>
                  </ButtonMenu>

                </div>


                <div hidden={(this.state.selected_auto_control !== "Action")}>
                  <Label title={""}></Label>
                  <Label title={""}></Label>

                  <Label title={"Select Action"}>
                    <Select
                      id="selected_go_action"
                      onChange={(event) => onDropdownSelectedSetState.bind(this)(event, "selected_go_action")}
                      value={this.state.selected_go_action}
                    >
                      {this.state.actions_list ? this.state.actions_menu : NoneOption}
                    </Select>
                  </Label>

                  <ButtonMenu>
                    <Button onClick={() => this.state.autonomous_ready ?
                      this.sendGoActionIndex() :
                      this.doNothing()
                    }>{"Send Action"}</Button>
                  </ButtonMenu>
                </div>

              </Column>
              <Column>

                <label>
                  {"Current Errors"}
                </label>

                <Label title={"x (m)"}>
                  <Input
                    disabled value={this.state.errors_current_x}
                    id="x_error"
                  />

                </Label>

                <Label title={"y (m)"}>
                  <Input
                    disabled value={this.state.errors_current_y}
                    id="y_error"
                  />

                </Label>

                <Label title={"z (m)"}>
                  <Input
                    disabled value={this.state.errors_current_z}
                    id="z_error"
                  />

                </Label>

                <div hidden={!has_roll}>
                  <Label title={"Roll"}>
                    <Input
                      disabled value={this.state.errors_current_roll_deg}
                      id="roll_error"
                    />

                  </Label>
                </div>

                <div hidden={!has_pitch}>
                  <Label title={"Pitch"}>
                    <Input
                      disabled value={this.state.errors_current_pitch_deg}
                      id="pitch_error"
                    />

                  </Label>
                </div>

                <Label title={"Yaw"}>
                  <Input
                    disabled value={this.state.errors_current_yaw_deg}
                    id="yaw_error"
                  />

                </Label>

                <Label title={"Heading"}>
                  <Input
                    disabled value={this.state.errors_current_heading_deg}
                    id="heading_error"
                  />

                </Label>

              </Column>
            </Columns>


            <Label title={"Last Command"} ></Label>


            <Input
              disabled value={this.state.last_cmd_str}
              id="last_command"
            />



          </div>



        </div>

      </Section>
    )
  }

}
export default NepiDeviceControls
