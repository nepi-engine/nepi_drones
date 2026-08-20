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
      capabilitiesPollTimer: null,

      // Which Settings this device's driver actually registers -- driver-
      // specific, not present on every RBX driver, so controls built on top
      // of them are gated on this list rather than an unrelated capability
      // flag that would silently no-op for drivers that don't define them.
      rbxSettingsListener: null,
      settingsNamesList: [],
      settingsValuesDict: {}
    }

    this.updateInfoListener = this.updateInfoListener.bind(this)
    this.infoListener = this.infoListener.bind(this)
    this.updateRBXSettingsListener = this.updateRBXSettingsListener.bind(this)
    this.rbxSettingsListener = this.rbxSettingsListener.bind(this)

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
    // Was gated on `this.state.rbx_capabilities === null` -- fetched once per
    // device selection and never again. That hid the whole point of the Sim
    // Connector capability-configuration UI: toggling "automated movement" (or
    // any other has_* flag) there mutates the driver's live caps_report object
    // in place (device_if_rbx.py's capabilities_query_callback returns it by
    // reference, not a fresh snapshot), so the NEXT service call already
    // reflects it -- but nothing was making that next call happen, and even if
    // it had, this guard would have thrown the answer away. Now re-synced on
    // every /info tick (paired with the capabilitiesPollTimer in
    // updateInfoListener below, which is what actually re-issues the query).
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
    if (this.state.capabilitiesPollTimer) {
      clearInterval(this.state.capabilitiesPollTimer)
      this.setState({ capabilitiesPollTimer: null })
    }
    if (deviceNamespace !== null && deviceNamespace.indexOf('null') === -1) {
      var listener = this.props.ros.setupStatusListener(
        deviceNamespace + "/info",
        "nepi_interfaces/DeviceRBXInfo",
        this.infoListener
      )
      this.setState({ rbxInfoListener: listener })

      // Re-issue the capabilities_query service call periodically so a
      // capability toggled from the Sim Connector app (has_goto_position for
      // "automated movement", has_camera_view_control, etc.) actually reaches
      // this panel without requiring the user to reselect the device or reload
      // the page. Store.js's own rbxDevices cache (callRBXCapabilitiesQueryService)
      // otherwise only refreshes on ROS-graph topology change -- a capability
      // flip on an already-connected device is invisible to it. 3s: fast
      // enough to feel live, far below anything that would visibly load the
      // rosbridge connection.
      const pollTimer = setInterval(() => {
        this.props.ros.callRBXCapabilitiesQueryService(deviceNamespace)
      }, 3000)
      this.setState({ capabilitiesPollTimer: pollTimer })
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
    if (this.state.capabilitiesPollTimer) {
      clearInterval(this.state.capabilitiesPollTimer)
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
    var device_names = []
    for (var i = 0; i < topics.length; i++) {
      device_name = topics[i].split('/rbx')[0].split('/').pop()
      device_names.push(device_name)
      items.push(<Option value={topics[i]}>{device_name}</Option>)
    }
    // Check that our current selection hasn't disappeared as an available option
    const { currentRBXNamespace } = this.state
    if ((currentRBXNamespace != null) && (!topics.includes(currentRBXNamespace))) {
      this.clearTopicRBXSelection()
    } else if (currentRBXNamespace == null && topics.length === 1) {
      // Auto-select the sole discovered device, mirroring
      // sim_connector_app_node.py's simDiscoveryCb (auto-select when nothing
      // is selected and exactly one candidate exists). Without this,
      // currentRBXNamespace stays null on every fresh mount -- including
      // every page reload -- and the entire Process Controls panel
      // (Teleop included) stays unrendered until this dropdown is manually
      // re-picked, an easy step to miss after a refresh.
      this.setState({
        currentRBXNamespace: topics[0],
        currentRBXNamespaceText: device_names[0],
      })
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

    // Sim Connector's own "choose what image sources are good and what
    // aren't" curation -- enabled_image_sources is a comma-separated
    // allowlist Setting (see rbx_sim_node.py's CAPABILITY_SETTING_NAMES).
    // Empty (a driver that doesn't define it, or hasn't set it, or every
    // candidate is simply left at its default-enabled state) means
    // unrestricted -- every discovered topic is offered, matching
    // Nepi_IF_Sim-Controls.js's own renderImageSourceCuration, which shows
    // every candidate as already checked for this exact state. Non-empty
    // means the operator has deliberately narrowed the list to specific
    // topics -- curationRestricted gates that path so a curated-down-to-zero
    // list (e.g. every allowed topic happens to be temporarily unpublished)
    // doesn't silently fall through to "show everything unfiltered",
    // defeating the curation the operator explicitly set up exactly when it
    // looks like it worked.
    //
    // When active, the allowlist can ADD topics from OUTSIDE this robot's
    // own namespace, not just narrow the namespace-scoped list -- found live
    // (2026-08-18): a physical camera genuinely connected to this device
    // (e.g. nexigo_02/idx/color_image) could never be offered as an image
    // source for a simulated robot, even after the operator explicitly
    // allowlisted it via the Sim Connector's own image-source curation
    // control, because this scoping ran BEFORE the allowlist and only ever
    // narrowed within it. The allowlist is the operator's own deliberate,
    // per-instance choice (surfaced by the Sim Connector precisely so a real
    // camera can stand in for a simulated one) -- it should be honored
    // wherever the topic actually lives, not silently re-scoped back to
    // "this robot's own" after the fact.
    const enabledSourcesRaw = this.state.settingsValuesDict["enabled_image_sources"]
    const curationRestricted = (enabledSourcesRaw !== undefined && String(enabledSourcesRaw).trim() !== '')
    if (curationRestricted) {
      const allowlist = String(enabledSourcesRaw).split(',').map((s) => s.trim()).filter((s) => s !== '')
      const namespaceScoped = img_topics.filter((topic) => allowlist.includes(topic))
      const allowlistedElsewhere = image_topics.filter((topic) =>
        allowlist.includes(topic) && topic !== ownImageTopic && !namespaceScoped.includes(topic))
      img_topics = namespaceScoped.concat(allowlistedElsewhere)
    } else {
      // Empty enabled_image_sources means unrestricted -- Nepi_IF_Sim-
      // Controls.js's own renderImageSourceCuration shows EVERY candidate as
      // already checked/enabled for exactly this state, so this dropdown
      // needs to actually match that: every other discovered topic (not just
      // a "no topics of our own" fallback) gets offered too. Previously this
      // only ran when img_topics was completely empty, so a robot that
      // already publishes its own camera (any Gazebo-based driver) could
      // never also offer a physical camera the operator left at its
      // enabled-by-default state -- found live (2026-08-19): nexigo_02/idx/
      // color_image stayed missing from this dropdown even though the
      // curation checklist showed it enabled, because sim_rover1 already had
      // its own raw topics and the old fallback condition never triggered.
      //
      // Same color_2d_image/bare-"/image" exclusion as renderImageSourceCuration's
      // own candidate filter, not just ownImageTopic/zed_node -- without it,
      // OTHER devices' internal relay-source echoes (e.g. a second robot's
      // own color_2d_image/robot_view, or app_sim_connector's own bare
      // color_2d_image topic) got added here as extra, redundant options
      // alongside the real mirrors below (found live 2026-08-19).
      for (var j = 0; j < image_topics.length; j++) {
        const other = image_topics[j]
        if (other === ownImageTopic || other.includes('zed_node') === true
            || other.includes('color_2d_image') === true || other.endsWith('/image') === true) {
          continue
        }
        if (!img_topics.includes(other)) {
          img_topics.push(other)
        }
      }
    }

    // app_sim_connector's robot_view/scene_view mirrors exist specifically so
    // this panel doesn't need to know a simulated robot's own raw topic names
    // (sim_rover1/..., a quadcopter's own namespace, etc.) -- see
    // sim_connector_app_node.py's commonViewImageCb, which republishes
    // whichever robot is currently active under these two fixed names. They
    // live under this device's root, one level above nodeNamespace, not
    // under nodeNamespace itself, so the scoping loop above never finds
    // them, and they were previously reachable only via the Sim Connector's
    // own separate enabled_image_sources curation step.
    //
    // When live (this IS a simulator-backed device), the mirrors REPLACE only
    // the robot's own literal color_2d_image/robot_view + .../scene_view
    // duplicates -- NOT the whole list. An earlier version of this replaced
    // img_topics wholesale, which also silently dropped every curation-
    // allowlisted topic from OUTSIDE the robot's own namespace (found live
    // 2026-08-19: nexigo_02/idx/color_image, explicitly enabled via the Sim
    // Connector's own curation checklist, disappeared from this dropdown the
    // moment a simulator was active) -- exactly the case the allowlist logic
    // above was written to support. Filtered against the live image_topics
    // list so a physical (non-simulated) device never gets offered mirrors
    // that don't actually exist, in which case nothing here changes.
    const deviceRoot = nodeNamespace.split('/').slice(0, -1).join('/')
    const simMirrorTopics = [
      deviceRoot + "/app_sim_connector/robot_view",
      deviceRoot + "/app_sim_connector/scene_view"
    ].filter((topic) => image_topics.includes(topic))
    if (simMirrorTopics.length > 0) {
      const ownDuplicateTopics = [
        nodeNamespace + "/color_2d_image/robot_view",
        nodeNamespace + "/color_2d_image/scene_view"
      ]
      // Exclude simMirrorTopics themselves too, not just ownDuplicateTopics --
      // the unrestricted branch above already adds every other discovered
      // topic (including these same two mirrors) when nothing is curated, so
      // without this they were being prepended a second time (found live
      // 2026-08-19: robot_view/scene_view each appeared twice in the
      // dropdown).
      img_topics = simMirrorTopics.concat(
        img_topics.filter((topic) => !ownDuplicateTopics.includes(topic) && !simMirrorTopics.includes(topic)))
    }

    // If the currently-active source (this.state.image_source, driven by the
    // device's own status report -- see statusListener) just fell out of the
    // computed list -- e.g. the operator unchecked it in the Sim Connector's
    // curation checklist -- proactively tell the device to go back to "None"
    // rather than leaving it silently relaying from a topic this dropdown no
    // longer even offers. Found live (2026-08-19): unchecking nexigo_02 in
    // the curation list made it disappear from this dropdown, but the
    // device's OWN image_source param was untouched (curation is a pure RUI/
    // Settings-list concept; the topic itself is still publishing), so it
    // kept right on relaying nexigo's feed -- with the Select unable to
    // match its own bound value against any remaining <Option>, LOOKING like
    // "None" was selected while the backend silently disagreed.
    const activeSource = this.state.image_source
    if (activeSource && activeSource !== 'None' && !img_topics.includes(activeSource)) {
      const { sendStringMsg } = this.props.ros
      sendStringMsg(RBXDeviceNamespace + "/set_image_topic", "None")
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

                {/* Depth Map deliberately NOT here -- tried putting it on
                    this generic panel (2026-08-19) reasoning it was device-
                    agnostic, but it isn't: the colorized feed comes from
                    camera_rig_controller.py reading Gazebo's own depth
                    sensor plugin, entirely independent of whatever this
                    Image_Source dropdown has selected. Toggling it while a
                    real camera (e.g. nexigo_02) is the selected source has
                    no effect on that camera at all -- confirmed live, this
                    is Sim-only for real, not just Sim-first. Stays in
                    Nepi_IF_Sim-Controls.js exclusively, same reasoning that
                    already keeps camera_offset_x/y/z out of this panel. */}
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

        </Section>

      </React.Fragment>
    )
  }

  renderImageViewer() {
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
              // Passed through so autonomous_movement_enabled/
              // teleop_movement_enabled (Settings a Sim Connector robot-config
              // toggle writes to) can hide the corresponding dropdown option
              // entirely rather than leaving a control visible-but-inert --
              // capabilities alone only say WHETHER a robot type supports
              // something, not whether the current deployment wants it
              // exposed.
              settingsNamesList={this.state.settingsNamesList}
              settingsValuesDict={this.state.settingsValuesDict}
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
