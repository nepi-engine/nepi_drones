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
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import Toggle from "react-toggle"
import Input from "./Input"
import Styles from "./Styles"

import NepiDeviceControls from "./NepiDeviceRBX-Controls"
import NepiDeviceMessages from "./NepiDeviceRBX-Info"

import NepiDeviceInfo from "./Nepi_IF_DeviceInfo"
import ImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFSettings from "./Nepi_IF_Settings"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFConfig from "./Nepi_IF_Config"

import { createShortValuesFromNamespaces, createMenuListFromStrList,
  onDropdownSelectedSendIndex, onUpdateSetStateValue,
  setElementStyleModified, clearElementStyleModified } from "./Utilities"


@inject("ros")
@observer

// RBX Device page
class NepiDeviceRBX extends Component {
  constructor(props) {
    super(props)

    this.state = {

      show_controls: true,
      show_settings: true,
      show_save_data: true,

      rbx_capabilities: null,

      device_name: null,
      serial_num: null,
      hw_version: null,
      sw_version: null,
      standby: null,
      state_index: null,
      mode_index: null,
      error_bound_m: 0,
      error_bound_deg: 0,
      error_stabilize_s: 0,
      cmd_timeout: null,
      image_source: null,
      image_status_overlay: null,
      fake_gps_enabled: null,
      states_list: null,
      states_menu: null,
      modes_list: null,
      modes_menu: null,
      image_topic: null,

      actions_list: null,
      actions_menu: null,
      selected_setup_action: null,
      selected_setup_action_index: 0,
      selected_environment: "Flat Ground",

      currentRBXNamespace: null,
      currentRBXNamespaceText: "No device selected",

      rbxInfoListener: null,

      // Which Settings this device's driver actually registers -- e.g.
      // camera_view_mode, camera_offset_x/y/z (ArduPilot's chase-cam
      // feature) are driver-specific, not present on every RBX driver, so
      // controls built on top of them are gated on this list rather than
      // an unrelated capability flag that would silently no-op for drivers
      // that don't define them.
      rbxSettingsListener: null,
      settingsNamesList: [],
      settingsValuesDict: {},

      // Local edit buffers for the camera offset inputs -- kept separate
      // from settingsValuesDict so an in-progress edit isn't clobbered by
      // the next settings/status message mid-typing. Synced from the device
      // whenever the device's own reported value changes.
      camera_offset_x: "",
      camera_offset_y: "",
      camera_offset_z: ""
    }

    this.updateInfoListener = this.updateInfoListener.bind(this)
    this.infoListener = this.infoListener.bind(this)
    this.updateRBXSettingsListener = this.updateRBXSettingsListener.bind(this)
    this.rbxSettingsListener = this.rbxSettingsListener.bind(this)
    this.onEnterSetCameraOffset = this.onEnterSetCameraOffset.bind(this)
    this.renderCameraOffsetControls = this.renderCameraOffsetControls.bind(this)

    this.onTopicRBXSelected = this.onTopicRBXSelected.bind(this)
    this.clearTopicRBXSelection = this.clearTopicRBXSelection.bind(this)
    this.createTopicOptions = this.createTopicOptions.bind(this)
    this.createImageOptions = this.createImageOptions.bind(this)
    this.onEnterSetInputErrorBoundValue = this.onEnterSetInputErrorBoundValue.bind(this)
    this.sendErrorBounds = this.sendErrorBounds.bind(this)
    this.onDropdownSelectedAction = this.onDropdownSelectedAction.bind(this)
    this.sendSetupActionIndex = this.sendSetupActionIndex.bind(this)
    this.renderImageViewer = this.renderImageViewer.bind(this)
  }


  // Callback for handling ROS DeviceRBXInfo messages.
  // Pose values are not part of DeviceRBXInfo; the System Information panel
  // sources them from the NavPose topic.
  infoListener(message) {
    const { rbxDevices } = this.props.ros
    this.setState({
      device_name: message.device_name,
      serial_num: message.serial_num,
      hw_version: message.hw_version,
      sw_version: message.sw_version,
      standby: message.standby,
      state_index: message.state,
      mode_index: message.mode,
      error_bound_m: message.error_bounds.max_distance_error_m,
      error_bound_deg: message.error_bounds.max_rotation_error_deg,
      error_stabilize_s: message.error_bounds.min_stabilize_time_s,
      cmd_timeout: message.cmd_timeout,
      image_source: message.image_source,
      image_status_overlay: message.image_status_overlay
    })
    if (this.state.rbx_capabilities === null) {
      const capabilities = rbxDevices[this.state.currentRBXNamespace]
      if (capabilities) {
        const states = capabilities.state_options
        const states_menu_options = createMenuListFromStrList(states, false, [], [], [])
        const modes = capabilities.mode_options
        const modes_menu_options = createMenuListFromStrList(modes, false, [], [], [])
        const actions = capabilities.setup_action_options
        const actions_menu_options = createMenuListFromStrList(actions, false, [], [], [])

        this.setState({
          rbx_capabilities: capabilities,
          states_list: states,
          states_menu: states_menu_options,
          modes_list: modes,
          modes_menu: modes_menu_options,
          actions_list: actions,
          actions_menu: actions_menu_options,
        })
      }
    }
  }


  // Function for configuring and subscribing to the device /info topic
  updateInfoListener() {
    const deviceNamespace = this.state.currentRBXNamespace

    if (this.state.rbxInfoListener) {
      this.state.rbxInfoListener.unsubscribe()
      this.setState({
        rbx_capabilities: null,
        states_list: null,
        modes_list: null,
        actions_list: null
      })
    }
    if (deviceNamespace !== null && deviceNamespace.indexOf('null') === -1) {
      var listener = this.props.ros.setupStatusListener(
        deviceNamespace + "/info",
        "nepi_interfaces/DeviceRBXInfo",
        this.infoListener
      )
      this.setState({ rbxInfoListener: listener })
    }
  }


  // Lifecycle method called when component updates.
  // Used to track changes in the selected device.
  componentDidUpdate(prevProps, prevState) {
    const currentRBXNamespace = this.state.currentRBXNamespace
    if (prevState.currentRBXNamespace !== currentRBXNamespace && currentRBXNamespace !== null) {
      if (currentRBXNamespace.indexOf('null') === -1) {
        this.setState({ image_topic: currentRBXNamespace.split('/rbx')[0] + "/image" })
        this.updateInfoListener()
        this.updateRBXSettingsListener()
      }
    }
  }


  // Lifecycle method called just before the component unmounts.
  // Used to unsubscribe from the /info topic.
  componentWillUnmount() {
    if (this.state.rbxInfoListener) {
      this.state.rbxInfoListener.unsubscribe()
    }
    if (this.state.rbxSettingsListener) {
      this.state.rbxSettingsListener.unsubscribe()
    }
  }


  // Callback for handling nepi_interfaces/SettingsStatus messages -- tracks
  // just the setting NAMES this device's driver actually registers, so
  // driver-specific controls (camera POV toggle, camera offset) can gate
  // their own visibility on whether the underlying Setting exists at all,
  // the same way has_manual_controls/has_fake_gps already gate on a real
  // capability rather than assuming every RBX driver looks alike.
  rbxSettingsListener(message) {
    const settings = (message.settings_list !== undefined) ? message.settings_list : []
    var namesList = []
    var valuesDict = {}
    for (let ind = 0; ind < settings.length; ind++) {
      namesList.push(settings[ind].name_str)
      valuesDict[settings[ind].name_str] = settings[ind].value_str
    }
    this.setState({ settingsNamesList: namesList, settingsValuesDict: valuesDict })

    // Seed/resync each offset edit buffer only when the DEVICE's own value
    // changed (or on first sight), never on every status tick -- otherwise
    // a 1Hz status message overwrites whatever is being typed.
    const offsetNames = ["camera_offset_x", "camera_offset_y", "camera_offset_z"]
    var updates = {}
    for (let i = 0; i < offsetNames.length; i++) {
      const name = offsetNames[i]
      const deviceVal = valuesDict[name]
      if (deviceVal !== undefined && deviceVal !== this.state.settingsValuesDict[name]) {
        updates[name] = deviceVal
      }
    }
    if (Object.keys(updates).length > 0) {
      this.setState(updates)
    }
  }


  // Enter-to-apply for a camera offset input. Publishes the single Setting
  // being edited (Float, matching rbx_ardupilot_node.py's own CAP_SETTINGS
  // type for camera_offset_x/y/z) via the same updateSetting path the POV
  // toggle buttons use -- camera_rig_controller_ardupilot.py picks the new
  // offset up on its next aim cycle, for whichever view mode is active.
  onEnterSetCameraOffset(event, settingName) {
    if (event.key === 'Enter') {
      const value = parseFloat(event.target.value)
      if (!isNaN(value)) {
        const { updateSetting } = this.props.ros
        const namespace = this.state.currentRBXNamespace + "/settings"
        updateSetting(namespace, settingName, "Float", String(value))
      }
      const el = document.getElementById(event.target.id)
      if (el) {
        clearElementStyleModified(el)
      }
    }
  }


  // Function for configuring and subscribing to this device's settings/status
  updateRBXSettingsListener() {
    const deviceNamespace = this.state.currentRBXNamespace

    if (this.state.rbxSettingsListener) {
      this.state.rbxSettingsListener.unsubscribe()
      this.setState({ rbxSettingsListener: null, settingsNamesList: [] })
    }
    if (deviceNamespace !== null && deviceNamespace.indexOf('null') === -1) {
      var listener = this.props.ros.setupSettingsStatusListener(
        deviceNamespace + "/settings/status",
        this.rbxSettingsListener
      )
      this.setState({ rbxSettingsListener: listener })
    }
  }


  // Function for creating topic options for Select input
  createTopicOptions(topics) {
    var items = []
    items.push(<Option>{"None"}</Option>)
    var device_name = ""
    for (var i = 0; i < topics.length; i++) {
      device_name = topics[i].split('/rbx')[0].split('/').pop()
      items.push(<Option value={topics[i]}>{device_name}</Option>)
    }
    // Check that our current selection hasn't disappeared as an available option
    const { currentRBXNamespace } = this.state
    if ((currentRBXNamespace != null) && (!topics.includes(currentRBXNamespace))) {
      this.clearTopicRBXSelection()
    }
    return items
  }

  createImageOptions(RBXDeviceNamespace) {
    var items = []
    items.push(<Option>{"None"}</Option>)

    const image_topics = this.props.ros.imageTopics
    var img_topics = []

    // Scope this list to THIS robot's own cameras.
    //
    // props.ros.imageTopics is the RUI-wide list of every sensor_msgs/Image
    // topic on the system (Store.js updateImageTopics), shared by every image
    // selector in the app. Offering all of it here listed things that have
    // nothing to do with the selected robot -- on this device: another app's
    // feed (app_sim_connector/color_2d_image) and a physical USB camera
    // (nexigo_02/idx/color_image) -- and createShortValuesFromNamespaces
    // renders only the last two path segments, so unrelated topics could even
    // display under identical-looking labels. For a robot panel the useful
    // answer is "this robot's camera".
    //
    // RBXDeviceNamespace ends in "/rbx"; the device's own image topics are
    // siblings of it under the plain node namespace (RBXRobotIF's
    // self.namespace is a CHILD of self.node_namespace -- see
    // device_if_rbx.py), so match on the node namespace, not on
    // RBXDeviceNamespace itself.
    //
    // ownImageTopic is RBXRobotIF's own republished output, fed BY whatever
    // is selected here -- selecting your own output as your own input is
    // never a real choice, so it stays excluded even though it lives in the
    // right namespace.
    const nodeNamespace = RBXDeviceNamespace.split('/rbx')[0]
    const ownImageTopic = nodeNamespace + "/image"

    for (var i = 0; i < image_topics.length; i++) {
      const topic = image_topics[i]
      if (topic === ownImageTopic || topic.includes('zed_node') === true) {
        continue
      }
      if (topic.startsWith(nodeNamespace + "/") === true) {
        img_topics.push(topic)
      }
    }

    // Fall back to the unscoped list rather than offering nothing but "None":
    // a robot driver that publishes no camera of its own would otherwise have
    // no selectable source at all, which is strictly worse than a longer
    // list. Robots that DO publish their own camera (the ArduPilot driver's
    // color_2d_image, the sim rover's) never reach this.
    if (img_topics.length === 0) {
      for (var j = 0; j < image_topics.length; j++) {
        const other = image_topics[j]
        if (other === ownImageTopic || other.includes('zed_node') === true) {
          continue
        }
        img_topics.push(other)
      }
    }

    const img_topics_short = createShortValuesFromNamespaces(img_topics)
    for (i = 0; i < img_topics.length; i++) {
      items.push(<Option value={img_topics[i]}>{img_topics_short[i]}</Option>)
    }
    return items
  }

  clearTopicRBXSelection() {
    if (this.state.rbxInfoListener) {
      this.state.rbxInfoListener.unsubscribe()
    }
    this.setState({
      currentRBXNamespace: null,
      currentRBXNamespaceText: "No device selected",
      image_topic: null,
      rbx_capabilities: null,
      states_list: null,
      modes_list: null,
      actions_list: null,
      rbxInfoListener: null
    })
  }

  // Handler for RBX device topic selection
  onTopicRBXSelected(event) {
    var rbx = event.nativeEvent.target.selectedIndex
    var text = event.nativeEvent.target[rbx].text
    var value = event.target.value

    // Handle the "None" option -- always index 0
    if (rbx === 0) {
      this.clearTopicRBXSelection()
      return
    }

    this.setState({
      currentRBXNamespace: value,
      currentRBXNamespaceText: text,
    })
  }


  onEnterSetInputErrorBoundValue(event, stateVarStr) {
    if (event.key === 'Enter') {
      const value = parseFloat(event.target.value)
      if (!isNaN(value)) {
        var obj = {}
        obj[stateVarStr] = value
        this.setState(obj)
      }
      this.sendErrorBounds()
      document.getElementById(event.target.id).style.color = Styles.vars.colors.black
    }
  }

  sendErrorBounds() {
    const { sendErrorBoundsMsg } = this.props.ros
    const max_m = this.state.error_bound_m
    const max_d = this.state.error_bound_deg
    const min_stab = this.state.error_stabilize_s
    const namespace = this.state.currentRBXNamespace + "/set_goto_error_bounds"
    sendErrorBoundsMsg(namespace, max_m, max_d, min_stab)
  }

  sendSetupActionIndex() {
    const { sendIntMsg } = this.props.ros
    const namespace = this.state.currentRBXNamespace + "/setup_action"
    if (this.state.selected_setup_action_index !== null) {
      sendIntMsg(namespace, this.state.selected_setup_action_index)
    }
  }

  onDropdownSelectedAction(event) {
    this.setState({
      selected_setup_action: event.target.value,
      selected_setup_action_index: event.target.selectedIndex
    })
  }

  // Environment dropdown ("Flat Ground" / "Obstacle Course") -- its own
  // separate control from the generic "Setup Actions" dropdown above (which
  // only lists RESET_SIM/RETURN_HOME). Backed by rbx_sim_node.py's
  // "environment" Setting (not a setup action -- that was redundant with
  // this dropdown once this existed), applying immediately on selection like
  // the Set Mode/Set State dropdowns rather than needing its own Send button.
  // "Flat Ground" is the label for FLAT_GROUND (the default basic-room world
  // sim_rover_gazebo always launches into).
  onSelectEnvironment(event) {
    const { updateSetting } = this.props.ros
    const value = event.target.value
    this.setState({ selected_environment: value })
    updateSetting(this.state.currentRBXNamespace + "/settings", "environment", "Discrete",
      value === "Obstacle Course" ? "OBSTACLE_COURSE" : "FLAT_GROUND")
  }


  renderDeviceSelection() {
    const { rbxDevices, sendStringMsg, sendBoolMsg } = this.props.ros
    const NoneOption = <Option>None</Option>
    const deviceSelected = (this.state.currentRBXNamespace != null)
    const has_fake_gps = (this.state.rbx_capabilities !== null) ? (this.state.rbx_capabilities.has_fake_gps === true) : false
    const namespace = this.state.currentRBXNamespace
    return (
      <React.Fragment>
        <Section title={"Robot Selection and Configuration"}>
          <Columns>
            <Column>

              <Label title={"Device"}>
                <Select
                  onChange={this.onTopicRBXSelected}
                  value={namespace}
                >
                  {this.createTopicOptions(Object.keys(rbxDevices))}
                </Select>
              </Label>

            </Column>
            <Column>
            </Column>
          </Columns>


          <div align={"left"} textAlign={"left"} hidden={!deviceSelected}>


            <Columns>
              <Column>
                <div hidden={(has_fake_gps === false)}>
                  <Label title="Enable Fake GPS">
                    <Toggle
                      checked={this.state.fake_gps_enabled === true}
                      onClick={() => sendBoolMsg(namespace + "/enable_fake_gps", this.state.fake_gps_enabled === false)}>
                    </Toggle>
                  </Label>
                </div>

                <Label title={"Image_Source"}>
                  <Select
                    id="image_source"
                    onChange={(event) => sendStringMsg(namespace + "/set_image_topic", event.target.value)}
                    value={this.state.image_source}
                  >
                    {namespace
                      ? this.createImageOptions(namespace)
                      : NoneOption}
                  </Select>
                </Label>

              </Column>
              <Column>

                <Label title="Image Status Overlay">
                  <Toggle
                    checked={this.state.image_status_overlay === true}
                    onClick={() => sendBoolMsg(namespace + "/enable_image_overlay", this.state.image_status_overlay === false)}>
                  </Toggle>
                </Label>

                <Label title="">
                </Label>

              </Column>
            </Columns>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

            <label style={{ fontWeight: 'bold' }}>
              {"GoTo Error Bounds"}
            </label>

            <Columns>
              <Column>

                <Label title={"Max (m)"}>
                  <Input
                    value={this.state.error_bound_m}
                    id="error_m"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event, "error_bound_m")}
                    onKeyDown={(event) => this.onEnterSetInputErrorBoundValue(event, "error_bound_m")}
                    style={{ width: "80%" }}
                  />
                </Label>

              </Column>
              <Column>

                <Label title={"Max deg"}>
                  <Input
                    value={this.state.error_bound_deg}
                    id="error_deg"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event, "error_bound_deg")}
                    onKeyDown={(event) => this.onEnterSetInputErrorBoundValue(event, "error_bound_deg")}
                    style={{ width: "80%" }}
                  />
                </Label>

              </Column>
              <Column>

                <Label title={"Stablize Time (s)"}>
                  <Input
                    value={this.state.error_stabilize_s}
                    id="error_stablize"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event, "error_stabilize_s")}
                    onKeyDown={(event) => this.onEnterSetInputErrorBoundValue(event, "error_stabilize_s")}
                    style={{ width: "80%" }}
                  />
                </Label>

              </Column>
            </Columns>

          </div>
        </Section>

      </React.Fragment>
    )
  }

  renderSetupControls() {
    const NoneOption = <Option>None</Option>
    const current_state = (this.state.rbx_capabilities !== null && this.state.states_list !== null) ? this.state.states_list[this.state.state_index] : "None"
    const current_mode = (this.state.rbx_capabilities !== null && this.state.modes_list !== null) ? this.state.modes_list[this.state.mode_index] : "None"
    const namespace = this.state.currentRBXNamespace
    // modes_list/states_list are set directly from capabilities' string[]
    // options -- an empty (but loaded) [] is truthy in JS, so without this
    // length check these dropdowns rendered blank/empty for robots that
    // legitimately have no modes/states (e.g. RBX_SIM's rover) instead of
    // just not showing at all.
    const has_modes = (this.state.modes_list !== null && this.state.modes_list.length > 0)
    const has_states = (this.state.states_list !== null && this.state.states_list.length > 0)
    // The Environment dropdown below is backed by rbx_sim_node.py's
    // "environment" Setting, not a capability with its own dedicated flag --
    // reuses the same has_goto_location-based rover heuristic as
    // has_camera_pov_toggle elsewhere in this file (a robot with no WGS84
    // goto is the fixed-obstacle-course sim rover, not a drone).
    const has_obstacle_course = (this.state.rbx_capabilities !== null) ? (this.state.rbx_capabilities.has_goto_location !== true) : false
    return (
      <React.Fragment>
        <Section title={"Setup Controls"}>

          <Columns>
            <Column>

              <div hidden={!has_modes}>
                <Label title={"Set Mode"}>
                  <Select
                    id="device_mode"
                    onChange={(event) => onDropdownSelectedSendIndex.bind(this)(event, namespace + "/set_mode")}
                    value={current_mode}
                  >
                    {this.state.modes_list ? this.state.modes_menu : NoneOption}
                  </Select>
                </Label>
              </div>

              <div hidden={!has_states}>
                <Label title={"Set State"}>
                  <Select
                    id="device_state"
                    onChange={(event) => onDropdownSelectedSendIndex.bind(this)(event, namespace + "/set_state")}
                    value={current_state}
                  >
                    {this.state.states_list ? this.state.states_menu : NoneOption}
                  </Select>
                </Label>
              </div>

            </Column>
            <Column>

              <Label title={"Setup Actions"}>
                <Select
                  id="action_select"
                  onChange={(event) => this.onDropdownSelectedAction(event)}
                  value={this.state.selected_setup_action}
                >
                  {this.state.actions_list ? this.state.actions_menu : NoneOption}
                </Select>
              </Label>

              <ButtonMenu>
                <Button onClick={() => this.sendSetupActionIndex()}>{"Send Action"}</Button>
              </ButtonMenu>

            </Column>
          </Columns>

          <div hidden={!has_obstacle_course}>
            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
            <Label title={"Environment"}>
              <Select
                id="environment_select"
                onChange={(event) => this.onSelectEnvironment(event)}
                value={this.state.selected_environment}
              >
                <Option value="Flat Ground">{"Flat Ground"}</Option>
                <Option value="Obstacle Course">{"Obstacle Course"}</Option>
              </Select>
            </Label>
          </div>

        </Section>

      </React.Fragment>
    )
  }

  // Camera offset X/Y/Z, shown only for drivers that actually define these
  // Settings (ArduPilot's camera-rig chase cam). Same offset triple applies
  // to both view modes -- switching FIRST_PERSON/THIRD_PERSON only changes
  // how camera_rig_controller_ardupilot.py aims the camera, not which
  // offset it reads (see that driver's own CAMERA_SETTING_NAMES comment),
  // so this renders as one shared block under the POV buttons rather than a
  // per-mode set.
  renderCameraOffsetControls() {
    const offsets = [
      { name: "camera_offset_x", title: "Camera Offset X (m)" },
      { name: "camera_offset_y", title: "Camera Offset Y (m)" },
      { name: "camera_offset_z", title: "Camera Offset Z (m)" },
    ]
    return (
      <React.Fragment>
        {offsets.map((offset) => (
          <Label key={offset.name} title={offset.title}>
            <Input
              id={"rbx_" + offset.name}
              value={this.state[offset.name]}
              onChange={(event) => {
                const el = document.getElementById("rbx_" + offset.name)
                if (el) {
                  setElementStyleModified(el)
                }
                var obj = {}
                obj[offset.name] = event.target.value
                this.setState(obj)
              }}
              onKeyDown={(event) => this.onEnterSetCameraOffset(event, offset.name)}
            />
          </Label>
        ))}
      </React.Fragment>
    )
  }

  renderImageViewer() {
    const { updateSetting } = this.props.ros
    const namespace = this.state.currentRBXNamespace
    // The camera_view_mode Setting (FIRST_PERSON/THIRD_PERSON) only exists on
    // drivers that actually define it -- gate on the real Settings list
    // (settingsNamesList, from this device's own settings/status) rather
    // than a capability flag that would silently no-op for a driver
    // without it. Previously reused has_goto_location on the assumption
    // that only the goto_location-less rover sim has a chase cam to
    // switch, but rbx_ardupilot_node.py (a real goto_location drone) also
    // defines camera_view_mode for its own camera-rig chase-cam feature --
    // that assumption hid this toggle for exactly the driver it was built
    // for.
    const has_camera_pov_toggle = this.state.settingsNamesList.includes("camera_view_mode")
    const has_camera_offsets = this.state.settingsNamesList.includes("camera_offset_x")
    return (
      <React.Fragment>
        <Columns>
          <Column equalWidth={false}>
            {/* This page already renders a full <NepiIFSaveData> panel below
                (the device-side snapshot/save-data pipeline) -- ImageViewer
                itself also embeds two more copies of that same panel
                internally (show_save_controls, default true), and separately
                its own client-side "Snapshot" (a local PNG download,
                unrelated to device data recording) duplicated the Snapshot
                button with an identical label. Both disabled here only;
                other ImageViewer consumers without their own SaveData panel
                keep them.

                streamingImageQuality/streamingImageRate override
                ImageViewer's defaults (JPEG quality 95, up to 20fps) for the
                live MJPEG stream web_video_server serves straight to the
                browser -- on the NEPI device's Raspberry Pi hardware, that
                per-viewer real-time encode (not the VM-side relay, and not
                this source resolution) was the actual bottleneck behind the
                "laggy" complaint. The RUI's own quality-selector control for
                this is dead/commented-out code (Nepi_IF_ImageViewer.js), so
                overriding the defaults here is the only way to actually
                change it. */}
            <ImageViewer
              image_topic={this.state.image_topic}
              title={""}
              hideQualitySelector={false}
              show_topic_selector={false}
              show_browser_save_button={false}
              show_save_controls={false}
              streamingImageQuality={50}
              streamingImageRate={10}
            />
            <div hidden={!has_camera_pov_toggle}>
              <ButtonMenu>
                <Button onClick={() => updateSetting(namespace + "/settings", "camera_view_mode", "Discrete", "FIRST_PERSON")}>{"Robot View"}</Button>
                <Button onClick={() => updateSetting(namespace + "/settings", "camera_view_mode", "Discrete", "THIRD_PERSON")}>{"Scene View"}</Button>
              </ButtonMenu>
            </div>
            <div hidden={!has_camera_offsets}>
              {this.renderCameraOffsetControls()}
            </div>
          </Column>
        </Columns>
      </React.Fragment>
    )
  }


  render() {
    const deviceSelected = (this.state.currentRBXNamespace != null)
    const namespace = this.state.currentRBXNamespace
    return (
      <Columns>
        <Column>

          {(deviceSelected === true) ?
            <NepiDeviceInfo
              deviceNamespace={namespace}
              status_topic={"/info"}
              status_msg_type={"nepi_interfaces/DeviceRBXInfo"}
              name_update_topic={"/update_device_name"}
              name_reset_topic={"/reset_device_name"}
              title={"Device Info"}
            />
            : null}

          {this.renderImageViewer()}

          {(deviceSelected === true) ?
            <NepiIFSaveData
              saveNamespace={namespace}
              make_section={true}
              title={"Save Data"}
            />
            : null}

          {(deviceSelected === true) ?
            <NepiDeviceMessages
              rbxNamespace={namespace}
              is_local_frame={(this.state.rbx_capabilities !== null) ? (this.state.rbx_capabilities.has_goto_location !== true) : false}
              has_set_home={(this.state.rbx_capabilities !== null) ? (this.state.rbx_capabilities.has_set_home === true) : false}
              title={"System Information"}
            />
            : null}

        </Column>
        <Column>
          {this.renderDeviceSelection()}

          {(deviceSelected === true) ?
            this.renderSetupControls()
            : null}

          {(deviceSelected === true) ?
            <NepiIFConfig
              namespace={namespace}
              title={"Save Config"}
              show_save_all={true}
            />
            : null}

          {(deviceSelected === true) ?
            <NepiDeviceControls
              rbxNamespace={namespace}
              title={"Process Controls"}
            />
            : null}

          {(deviceSelected === true) ?
            <NepiIFSettings
              settingsNamespace={namespace + '/settings'}
              allways_show_settings={true}
              title={"Device Settings"}
            />
            : null}

        </Column>
      </Columns>
    )
  }
}

export default NepiDeviceRBX
