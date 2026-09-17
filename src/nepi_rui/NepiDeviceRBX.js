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
import AsyncToggle from "./AsyncToggle"
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
  onDropdownSelectedSendIndex, onUpdateSetStateValue } from "./Utilities"


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
      home_lat: null,
      home_long: null,
      home_alt: null,
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

      currentRBXNamespace: null,
      currentRBXNamespaceText: "No device selected",

      rbxInfoListener: null,

      // Requested live 2026-09-16: "theres a bunch of random image sources
      // that appear... only things actually inputting real image topics
      // should show up." rbx_ardupilot_node.py/rbx_sim_node.py both
      // deliberately advertise a fixed set of camera relay topics
      // unconditionally (idle placeholders on hardware with no camera --
      // see that driver's own comment), and enabled_image_sources is the
      // existing, purpose-built Setting for curating which candidates
      // should actually be offered (Nepi_IF_Sim-Controls.js's own
      // renderImageSourceCuration writes it) -- createImageOptions() below
      // just never read it back. Comma-separated topic list, or empty for
      // "unrestricted" (no curation saved yet).
      rbxSettingsListener: null,
      enabled_image_sources: "",

      // Requested live 2026-09-16: "if linked in sim connector, when i go to
      // the ardupilot sitl in devices -> robots, there should be some
      // indicator next to the device saying linked with .. for whichever
      // its linked with." robot_link/status (nepi_app_sim_connector's own
      // Robot Link feature, see Nepi_IF_RobotLink.js) is a JSON-encoded
      // std_msgs/String, not a typed message -- same reasoning as that
      // file's own docstring. There's exactly one sim connector app
      // instance, at <this device's own root namespace>/app_sim_connector,
      // so this derives that root from currentRBXNamespace rather than
      // hardcoding "/nepi/device1" (which wouldn't survive a different
      // ROOTNAME/DEVICE_ID). Absence of the app (not installed/running) is
      // silently fine -- setupStatusListener on a topic with no publisher
      // just never calls back, same as every other optional listener here.
      robotLinkListener: null,
      robot_link_status: null
    }

    this.updateInfoListener = this.updateInfoListener.bind(this)
    this.infoListener = this.infoListener.bind(this)
    this.updateRbxSettingsListener = this.updateRbxSettingsListener.bind(this)
    this.rbxSettingsListenerCb = this.rbxSettingsListenerCb.bind(this)
    this.updateRobotLinkListener = this.updateRobotLinkListener.bind(this)
    this.robotLinkListenerCb = this.robotLinkListenerCb.bind(this)

    this.setDeviceSelection = this.setDeviceSelection.bind(this)
    this.clearDeviceSelection = this.clearDeviceSelection.bind(this)
    this.reconcileDeviceSelection = this.reconcileDeviceSelection.bind(this)
    this.selectDeviceNamespace = this.selectDeviceNamespace.bind(this)

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
      image_status_overlay: message.image_status_overlay,
      home_lat: message.home_lat,
      home_long: message.home_long,
      home_alt: message.home_alt
    })
    if (this.state.rbx_capabilities === null) {
      const capabilities = rbxDevices[this.state.currentRBXNamespace]
      if (capabilities) {
        const states = capabilities.state_options
        const states_menu_options = createMenuListFromStrList(states, false, [], [], [])
        const modes = capabilities.mode_options
        const modes_menu_options = createMenuListFromStrList(modes, false, [], [], [])
        const actions = capabilities.setup_action_options
        // RESET_SIM filtered out here (requested live 2026-09-14): it now
        // has its own dedicated button in the Sim Connector app's Robot
        // Capabilities section (Nepi_IF_Sim-Controls.js's
        // renderRobotCapabilityControls) instead of sitting in this
        // generic Setup Actions dropdown. Every other setup action (e.g.
        // RETURN_HOME, TAKEOFF/LAUNCH/LAND on the flight drivers) stays.
        const actions_menu_options = createMenuListFromStrList(actions, false, ["RESET_SIM"], [], [])

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


  // Reads just the one Setting createImageOptions() needs (enabled_image_sources),
  // off the same nepi_interfaces/SettingsStatus every device's Nepi_IF_Settings
  // panel already subscribes to -- a second, independent subscription here
  // rather than threading the value down from NepiIFSettings, since this
  // component builds the dropdown before NepiIFSettings mounts in the render
  // tree below it and has no other current channel for a device's live Setting
  // values. Mirrors Nepi_IF_Sim-Controls.js's own updateRbxSettingsListener/
  // rbxSettingsListener (see that file for why controls_name_list/
  // controls_msg_list, and why value lives in ctrl.value, a string[]).
  rbxSettingsListenerCb(message) {
    const names = (message.controls_name_list !== undefined) ? message.controls_name_list : []
    const msgs = (message.controls_msg_list !== undefined) ? message.controls_msg_list : []
    const ind = names.indexOf("enabled_image_sources")
    const ctrl = (ind !== -1) ? msgs[ind] : null
    const value = (ctrl != null && ctrl.value !== undefined && ctrl.value.length > 0) ? ctrl.value[0] : ""
    this.setState({ enabled_image_sources: value })
  }

  updateRbxSettingsListener(rbxNamespace) {
    if (this.state.rbxSettingsListener) {
      this.state.rbxSettingsListener.unsubscribe()
      this.setState({ rbxSettingsListener: null, enabled_image_sources: "" })
    }
    if (rbxNamespace !== null && rbxNamespace !== '' && rbxNamespace !== 'None') {
      var listener = this.props.ros.setupSettingsStatusListener(
        rbxNamespace + "/settings/status",
        this.rbxSettingsListenerCb
      )
      this.setState({ rbxSettingsListener: listener })
    }
  }

  updateRobotLinkListener(rbxNamespace) {
    if (this.state.robotLinkListener) {
      this.state.robotLinkListener.unsubscribe()
      this.setState({ robotLinkListener: null, robot_link_status: null })
    }
    if (rbxNamespace === null || rbxNamespace === '' || rbxNamespace === 'None') {
      return
    }
    const deviceBaseNamespace = rbxNamespace.split('/rbx')[0]
    const rootNamespace = deviceBaseNamespace.substring(0, deviceBaseNamespace.lastIndexOf('/'))
    const listener = this.props.ros.setupStatusListener(
      rootNamespace + "/app_sim_connector/sim/robot_link/status",
      "std_msgs/String",
      this.robotLinkListenerCb
    )
    this.setState({ robotLinkListener: listener })
  }

  robotLinkListenerCb(message) {
    try {
      this.setState({ robot_link_status: JSON.parse(message.data) })
    } catch (e) {
      // Malformed/empty payload -- keep the last good status.
    }
  }

  // Renders "Linked with <other device>" next to the device selector when
  // Robot Link is actively enabled and this device is one end of it.
  renderLinkIndicator() {
    const status = this.state.robot_link_status
    const namespace = this.state.currentRBXNamespace
    if (status == null || namespace == null || status.enabled !== true) {
      return null
    }
    var otherNamespace = null
    if (status.sim_namespace === namespace) {
      otherNamespace = status.physical_namespace
    } else if (status.physical_namespace === namespace) {
      otherNamespace = status.sim_namespace
    }
    if (!otherNamespace) {
      return null
    }
    const otherName = otherNamespace.split('/rbx')[0].split('/').pop()
    return (
      <Label title={""}>
        <p style={{ color: Styles.vars.colors.green }}>{"Linked with " + otherName}</p>
      </Label>
    )
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
    // Reconcile first: this is the safe phase for the setState calls that used
    // to run during render out of createTopicOptions.
    this.reconcileDeviceSelection(Object.keys(this.props.ros.rbxDevices))

    const currentRBXNamespace = this.state.currentRBXNamespace
    if (prevState.currentRBXNamespace !== currentRBXNamespace && currentRBXNamespace !== null) {
      if (currentRBXNamespace.indexOf('null') === -1) {
        this.setState({ image_topic: currentRBXNamespace.split('/rbx')[0] + "/image" })
        this.updateInfoListener()
        this.updateRbxSettingsListener(currentRBXNamespace)
        this.updateRobotLinkListener(currentRBXNamespace)
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
    if (this.state.robotLinkListener) {
      this.state.robotLinkListener.unsubscribe()
    }
  }


  // Function for creating topic options for Select input
  createTopicOptions(topics) {
    // const namespace = this.state.currentRBXNamespace
    var items = []
    items.push(<Option>{"None"}</Option>)
    var device_name = ""
    for (var i = 0; i < topics.length; i++) {
      device_name = topics[i].split('/rbx')[0].split('/').pop()
      items.push(<Option value={topics[i]}>{device_name}</Option>)
    }
    // Selection reconciliation deliberately does NOT happen here. This runs
    // during render, and both branches call setState -- see
    // reconcileDeviceSelection(), called from componentDidUpdate instead.
    return items
  }

  // Keep the current selection consistent with what is actually on the system:
  // drop a selection whose device has disappeared, and adopt the first device
  // when nothing is selected yet.
  //
  // Runs from componentDidUpdate, not from render. It also selects by NAMESPACE
  // rather than by calling setDeviceSelection: that method is a DOM event
  // handler and reads event.nativeEvent.target, so handing it a namespace
  // string threw "Cannot read properties of undefined (reading 'target')" and
  // took the whole page down as soon as any RBX device existed.
  reconcileDeviceSelection(topics) {
    const namespace = this.state.currentRBXNamespace
    const selected = (namespace != null && namespace !== 'None')

    if (selected && topics.includes(namespace) === false) {
      this.clearDeviceSelection()
      return
    }
    if (selected === false && topics.length > 0) {
      this.selectDeviceNamespace(topics[0])
    }
  }

  // The selection itself, independent of how it was chosen. Both the Select
  // handler and the auto-adopt path above go through here.
  selectDeviceNamespace(namespace) {
    if (namespace == null || namespace === 'None') {
      this.clearDeviceSelection()
      return
    }
    this.setState({
      currentRBXNamespace: namespace,
      currentRBXNamespaceText: namespace.split('/rbx')[0].split('/').pop(),
    })
  }

  createImageOptions(RBXDeviceNamespace) {
    var items = []
    items.push(<Option>{"None"}</Option>)

    // RBXDeviceNamespace is currentRBXNamespace, which ends in "/rbx" (e.g.
    // ".../ardupilot_ttyUSB0/rbx"), but a device's own camera relay topics
    // live directly under its base namespace (".../ardupilot_ttyUSB0/
    // color_2d_image/..."), not under "/rbx" -- so the startsWith check
    // below never actually matched anything and a device's own (idle, on
    // hardware with no camera) relay topics showed up as candidates in its
    // own Image Source dropdown. Strip the "/rbx" suffix first so
    // self-exclusion actually works.
    const deviceBaseNamespace = RBXDeviceNamespace.split('/rbx')[0]

    // enabled_image_sources (see rbxSettingsListenerCb above) is the
    // existing curation Setting: when the operator has saved a non-empty
    // allowlist via the Sim Connector's Robot Capabilities panel, only
    // those topics should appear here at all -- matching what
    // Nepi_IF_Sim-Controls.js's own comment already documented as the
    // intended contract. Empty means "nothing curated yet", so fall back
    // to the full (self-excluded) candidate list rather than showing none.
    const enabledRaw = this.state.enabled_image_sources || ""
    const enabledList = enabledRaw.split(',').map((s) => s.trim()).filter((s) => s !== "")

    const image_topics = this.props.ros.imageTopics
    var img_topics = []

    for (var i = 0; i < image_topics.length; i++) {
      const topic = image_topics[i]
      if (topic.includes('zed_node') === true) {
        continue
      }
      // Explicit curation (enabled_image_sources) always wins, for whichever
      // device it was saved on -- an operator who deliberately added a
      // topic to the allowlist gets it, even if it would otherwise be
      // filtered below.
      if (enabledList.includes(topic)) {
        img_topics.push(topic)
        continue
      }
      if (enabledList.length > 0) {
        // A non-empty allowlist is exhaustive -- everything not explicitly
        // in it is excluded, matching Nepi_IF_Sim-Controls.js's own
        // renderImageSourceCuration contract.
        continue
      }
      // No curation saved for THIS device -- generalize self-exclusion to
      // every RBX driver's own relay-pattern topics (not just this
      // device's own), the same exclusion renderImageSourceCuration
      // already applies when building candidates: "color_2d_image" and any
      // bare ".../image" are always an internal per-device echo (idle
      // placeholders on hardware with no camera, or redundant with the sim
      // connector's own mirror topics for a sim device) -- never a
      // meaningful "image source" for ANY device's own picker, own or
      // another's. Requested live 2026-09-16: "make sure this happens for
      // every robot device" -- this was previously only self-excluded,
      // requiring per-device manual curation (as done for ardupilot_ttyUSB0)
      // to hide every OTHER device's own relay topics too.
      if (topic.startsWith(deviceBaseNamespace) === true) {
        continue
      }
      if (topic.includes('color_2d_image') === true || topic.endsWith('/image') === true) {
        continue
      }
      img_topics.push(topic)
    }

    const img_topics_short = createShortValuesFromNamespaces(img_topics)
    for (i = 0; i < img_topics.length; i++) {
      items.push(<Option value={img_topics[i]}>{img_topics_short[i]}</Option>)
    }
    return items
  }

  clearDeviceSelection() {
    if (this.state.rbxInfoListener) {
      this.state.rbxInfoListener.unsubscribe()
    }
    if (this.state.rbxSettingsListener) {
      this.state.rbxSettingsListener.unsubscribe()
    }
    if (this.state.robotLinkListener) {
      this.state.robotLinkListener.unsubscribe()
    }
    this.setState({
      currentRBXNamespace: null,
      currentRBXNamespaceText: "No device selected",
      image_topic: null,
      rbx_capabilities: null,
      states_list: null,
      modes_list: null,
      actions_list: null,
      rbxInfoListener: null,
      rbxSettingsListener: null,
      enabled_image_sources: "",
      robotLinkListener: null,
      robot_link_status: null
    })
  }

  // Handler for RBX device topic selection. Event-shaped, so it only ever
  // receives a real DOM event from the Select below; everything else selects
  // through selectDeviceNamespace().
  setDeviceSelection(event) {
    var rbx = event.nativeEvent.target.selectedIndex

    // Handle the "None" option -- always index 0
    if (rbx === 0) {
      this.clearDeviceSelection()
      return
    }

    this.setState({
      currentRBXNamespace: event.target.value,
      currentRBXNamespaceText: event.nativeEvent.target[rbx].text,
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
    // Looked up by VALUE against the full, unfiltered actions_list -- NOT
    // event.target.selectedIndex (DOM position), which no longer matches
    // the driver's own action index now that RESET_SIM is filtered out of
    // this dropdown (see actions_menu_options' own comment). A driver-side
    // action's real index can differ from its position here even before
    // this filter (RESET_SIM sits at index 0 for some drivers, a later
    // index for others -- see each rbx_*_node.py's own RBX_SETUP_ACTIONS
    // list), so position-based dispatch was never safe to begin with; this
    // makes it correct regardless of what else this dropdown ever excludes.
    const actions = this.state.actions_list || []
    this.setState({
      selected_setup_action: event.target.value,
      selected_setup_action_index: actions.indexOf(event.target.value)
    })
  }


  renderDeviceSelection() {
    const { rbxDevices, sendStringMsg, sendBoolMsg, sendGeoPointMsg } = this.props.ros
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
                  onChange={this.setDeviceSelection}
                  value={namespace}
                >
                  {this.createTopicOptions(Object.keys(rbxDevices))}
                </Select>
              </Label>

              {this.renderLinkIndicator()}

            </Column>
            <Column>
            </Column>
          </Columns>


          <div align={"left"} textAlign={"left"} hidden={!deviceSelected}>


            <Columns>
              <Column>
                <div hidden={(has_fake_gps === false)}>
                  <Label title="Enable Fake GPS">
                    <AsyncToggle
                      checked={this.state.fake_gps_enabled === true}
                      onClick={() => sendBoolMsg(namespace + "/enable_fake_gps", this.state.fake_gps_enabled === false)}>
                    </AsyncToggle>
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
                  <AsyncToggle
                    checked={this.state.image_status_overlay === true}
                    onClick={() => sendBoolMsg(namespace + "/enable_image_overlay", this.state.image_status_overlay === false)}>
                  </AsyncToggle>
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

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
            <label style={{ fontWeight: 'bold' }}>
              {"Home Location"}
            </label>

            <Columns>
              <Column>

                <Label title={"latitude"}>
                  <Input
                    value={this.state.home_lat}
                    id="home_lat"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event, "home_lat")}
                    style={{ width: "80%" }}
                  />
                </Label>

              </Column>
              <Column>

                <Label title={"longitude"}>
                  <Input
                    value={this.state.home_long}
                    id="home_long"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event, "home_long")}
                    style={{ width: "80%" }}
                  />
                </Label>

              </Column>
              <Column>

                <Label title={"altitude"}>
                  <Input
                    value={this.state.home_alt}
                    id="home_alt"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event, "home_alt")}
                    style={{ width: "80%" }}
                  />
                </Label>

                <ButtonMenu>
                  <Button onClick={() => sendGeoPointMsg(namespace + "/set_home", this.state.home_lat, this.state.home_long, this.state.home_alt)}>{"Set Home"}</Button>
                </ButtonMenu>

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
    return (
      <React.Fragment>
        <Section title={"Setup Controls"}>

          <Columns>
            <Column>

              <Label title={"Set Mode"}>
                <Select
                  id="device_mode"
                  onChange={(event) => onDropdownSelectedSendIndex.bind(this)(event, namespace + "/set_mode")}
                  value={current_mode}
                >
                  {this.state.modes_list ? this.state.modes_menu : NoneOption}
                </Select>
              </Label>


              <Label title={"Set State"}>
                <Select
                  id="device_state"
                  onChange={(event) => onDropdownSelectedSendIndex.bind(this)(event, namespace + "/set_state")}
                  value={current_state}
                >
                  {this.state.states_list ? this.state.states_menu : NoneOption}
                </Select>
              </Label>

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

        </Section>

      </React.Fragment>
    )
  }

  renderImageViewer() {
    return (
      <React.Fragment>
        <Columns>
          <Column equalWidth={false}>
            <ImageViewer
              image_topic={this.state.image_topic}
              title={""}
              hideQualitySelector={false}
              show_topic_selector={false}
            />
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
              allways_show_controls={true}
              title={"Device Settings"}
            />
            : null}

        </Column>
      </Columns>
    )
  }
}

export default NepiDeviceRBX
