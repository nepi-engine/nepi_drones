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
import { round, setElementStyleModified, clearElementStyleModified } from "./Utilities"
import yaml from "js-yaml"

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

// Curated physical-dimension fields -- one entry per generate_model_sdf.py
// independent parameter (see that script's own ROVER_DEFAULT_DIMENSIONS/
// OBSTACLE_COURSE_DEFAULT_DIMENSIONS for the derivations these feed). Default
// values here match the script's own defaults, shown until a real device
// response arrives (sim/robot_dimensions_yaml / sim/environment_dimensions_yaml).
const ROBOT_DIMENSION_FIELDS = [
  { name: "wheel_radius_m", title: "Wheel Radius (m)", default: 0.1 },
  { name: "wheel_width_m", title: "Wheel Width (m)", default: 0.05 },
  { name: "track_width_m", title: "Track Width (m)", default: 0.34 },
  { name: "wheelbase_m", title: "Wheelbase (m)", default: 0.3 },
  { name: "chassis_length_m", title: "Chassis Length (m)", default: 0.4 },
  { name: "chassis_width_m", title: "Chassis Width (m)", default: 0.3 },
  { name: "chassis_height_m", title: "Chassis Height (m)", default: 0.1 },
]

const ENVIRONMENT_DIMENSION_FIELDS = [
  { name: "course_start_x_m", title: "Course Start X (m)", default: 2.0 },
  { name: "corridor_width_m", title: "Corridor Width (m)", default: 6.0 },
  { name: "wall_length_m", title: "Wall Length (m)", default: 22.0 },
  { name: "wall_thickness_m", title: "Wall Thickness (m)", default: 0.2 },
  { name: "wall_height_m", title: "Wall Height (m)", default: 1.0 },
  { name: "baffle_a_x_m", title: "Baffle A Position X (m)", default: 8.0 },
  { name: "baffle_b_x_m", title: "Baffle B Position X (m)", default: 14.0 },
  { name: "baffle_gap_m", title: "Baffle Drive-Through Gap (m)", default: 0.4 },
  { name: "baffle_thickness_m", title: "Baffle Thickness (m)", default: 0.2 },
  { name: "ramp_start_x_m", title: "Ramp Start X (m)", default: 18.0 },
  { name: "ramp_rise_m", title: "Ramp Rise (m)", default: 0.35 },
  { name: "ramp_angle_deg", title: "Ramp Angle (deg)", default: 9.97 },
  { name: "ramp_plateau_length_m", title: "Ramp Plateau Length (m)", default: 1.0 },
]

function defaultDimensionFields(fieldDefs) {
  var fields = {}
  fieldDefs.forEach((f) => { fields[f.name] = f.default })
  return fields
}

// Coerces one dimension field to a finite number for the diagrams below,
// falling back to that field's own factory default -- fields can hold a
// string mid-edit (including a momentarily invalid one like "" or "-"
// while typing a negative number), so this never lets a diagram draw NaN
// geometry.
function numericDimensionField(fields, fieldDefs, name) {
  const num = Number(fields[name])
  if (isFinite(num)) {
    return num
  }
  const def = fieldDefs.find((f) => f.name === name)
  return def ? def.default : 0
}

const DIAGRAM_BG = "#1c1e21"

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

      // Robot config viewer/download -- see renderRobotConfigViewer.
      // robot_config_yaml is the LAST config text the device reported on
      // sim/robot_config_yaml (latched, so it also survives a page reload
      // once at least one view has ever been requested); viewing_config_name
      // is which config that text is FOR, tracked separately so a click on
      // one "View" button while a different config's text is still displayed
      // doesn't read as already showing the new one before the round trip
      // completes.
      robot_config_yaml: '',
      viewing_config_name: 'None',
      robotConfigYamlListener: null,
      // Collapsed by default -- one "View Robot Configs" button reveals the
      // per-config buttons + text area, instead of that whole block always
      // taking up space on the page.
      show_robot_config_viewer: false,

      // Lifted up from NepiIFSimLauncher: two separate instances are mounted
      // below (selector up top, deploy controls at the bottom -- see their
      // `only` props), and this dropdown selection has to be shared between
      // them or the deploy instance can never see what was picked in the
      // selector instance. Owned here and passed down as a controlled prop
      // to both, rather than living in either instance's own local state.
      selected_launch_target: 'None',

      // Robot config selection, tracked locally (optimistic) rather than
      // read purely from status_msg.selected_robot_config -- found live
      // (2026-08-18): selecting "Quadcopter" then immediately clicking
      // Deploy on a freshly-loaded page could race the backend's own
      // processing of the select_robot_config message, so Deploy read a
      // stale (still-default, or still-rover) selected_robot_config and
      // launched the rover instead of the quadcopter override -- only
      // fixed itself after a kill + redeploy gave the round trip time to
      // land. Deploy now explicitly re-sends this value immediately before
      // launching (see NepiIFSimLauncher's onDeployClicked/onNewSimClicked),
      // so the backend's own selection is always fresh by the time it reads
      // it, regardless of any earlier message's timing. null means "no
      // local override yet, fall back to status_msg".
      selected_robot_config_local: null,

      // FOV data -- read once (mount time) from two plain, latched
      // std_msgs/Float32 topics published directly by sim_connector_app_node.py
      // (sim/camera_horizontal_fov_deg, sim/camera_vertical_fov_deg). Not part
      // of SimStatus/status_msg -- these never change at runtime, so a
      // dedicated latched topic pair avoids extending the .msg (which would
      // force a catkin message-regeneration rebuild) for two static numbers.
      camera_horizontal_fov_deg: null,
      camera_vertical_fov_deg: null,
      cameraHorizontalFovListener: null,
      cameraVerticalFovListener: null,

      // Physical-dimension editing (robot chassis/wheel + environment
      // corridor/ramp geometry) -- curated fields, hydrated from the
      // device's stored dimensions.yaml on mount (sim/get_robot_dimensions
      // / sim/get_environment_dimensions request -> sim/*_dimensions_yaml
      // latched reply), defaulting to generate_model_sdf.py's own factory
      // values until that reply arrives. dirty mirrors sim/*_dimensions_dirty
      // -- true means the device has a pending edit not yet applied to the
      // VM's own model.sdf (only happens right before the next Launch).
      robot_dimensions_fields: defaultDimensionFields(ROBOT_DIMENSION_FIELDS),
      environment_dimensions_fields: defaultDimensionFields(ENVIRONMENT_DIMENSION_FIELDS),
      robot_dimensions_dirty: false,
      environment_dimensions_dirty: false,

      // Snapshot the diagram (renderRobotDimensionsDiagram/
      // renderEnvironmentDimensionsDiagram below) actually draws from --
      // deliberately NOT the same object as *_dimensions_fields above, which
      // updates on every keystroke while editing. Requested live (2026-08-31)
      // as a preview that "updates every time they click Save Dimensions",
      // not one that redraws mid-edit before a value is even a real number
      // (a field can be a lone "-" or "" while typing). Set in two places:
      // applyDimensionsYaml (so the diagram shows the device's actual current
      // dimensions immediately on load, before any local edit) and
      // onSaveDimensionsClicked (so it then tracks exactly what "Save
      // Dimensions" last committed).
      robot_dimensions_preview_fields: defaultDimensionFields(ROBOT_DIMENSION_FIELDS),
      environment_dimensions_preview_fields: defaultDimensionFields(ENVIRONMENT_DIMENSION_FIELDS),
      robotDimensionsYamlListener: null,
      environmentDimensionsYamlListener: null,
      robotDimensionsDirtyListener: null,
      environmentDimensionsDirtyListener: null,

    }

    // Hidden <input type="file"> target for the Upload Robot Config button
    // -- a ref rather than state, since the input element itself is never
    // rendered differently; only clicked programmatically.
    this.uploadInputRef = React.createRef()
    // Hidden <input type="file"> targets for the raw-SDF-upload escape
    // hatch -- one per role, same reasoning as uploadInputRef above.
    this.uploadRobotSdfInputRef = React.createRef()
    this.uploadEnvironmentSdfInputRef = React.createRef()

    this.getSimNamespace = this.getSimNamespace.bind(this)

    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateCameraFovListeners = this.updateCameraFovListeners.bind(this)

    this.onRobotConfigSelected = this.onRobotConfigSelected.bind(this)
    this.onUploadConfigClicked = this.onUploadConfigClicked.bind(this)
    this.onUploadConfigFileChange = this.onUploadConfigFileChange.bind(this)
    this.onDownloadSampleConfigClicked = this.onDownloadSampleConfigClicked.bind(this)
    this.updateRobotConfigYamlListener = this.updateRobotConfigYamlListener.bind(this)
    this.robotConfigYamlListener = this.robotConfigYamlListener.bind(this)
    this.onViewConfigClicked = this.onViewConfigClicked.bind(this)
    this.onDownloadConfigClicked = this.onDownloadConfigClicked.bind(this)
    this.onLaunchTargetSelected = this.onLaunchTargetSelected.bind(this)

    this.renderRobotConfigSelector = this.renderRobotConfigSelector.bind(this)
    this.renderRobotConfigSettings = this.renderRobotConfigSettings.bind(this)
    this.renderFieldPair = this.renderFieldPair.bind(this)
    this.renderData = this.renderData.bind(this)

    this.updateDimensionsListeners = this.updateDimensionsListeners.bind(this)
    this.onSaveDimensionsClicked = this.onSaveDimensionsClicked.bind(this)
    this.onDownloadDimensionsClicked = this.onDownloadDimensionsClicked.bind(this)
    this.onUploadModelSdfClicked = this.onUploadModelSdfClicked.bind(this)
    this.onUploadModelSdfFileChange = this.onUploadModelSdfFileChange.bind(this)
    this.renderDimensionFields = this.renderDimensionFields.bind(this)
    this.renderDimensionsEditor = this.renderDimensionsEditor.bind(this)
    this.renderRobotDimensionsDiagram = this.renderRobotDimensionsDiagram.bind(this)
    this.renderEnvironmentDimensionsDiagram = this.renderEnvironmentDimensionsDiagram.bind(this)
    this.renderDimensionsDiagramSafe = this.renderDimensionsDiagramSafe.bind(this)
  }

  // Resolve the sim device namespace from the namespace prop
  getSimNamespace() {
    return (this.props.namespace !== undefined) ? this.props.namespace : null
  }

  componentDidMount() {
    this.updateStatusListener()
    this.updateRobotConfigYamlListener()
    this.updateCameraFovListeners()
    this.updateDimensionsListeners()
  }

  // Lifecycle method called when the component updates.
  // Re-point the status listener when the namespace prop changes.
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getSimNamespace()
    if (namespace !== this.state.namespace) {
      this.updateStatusListener()
      this.updateRobotConfigYamlListener()
      this.updateCameraFovListeners()
      this.updateDimensionsListeners()
    }

    // Once the server catches up and reports the SAME config we optimistically
    // set locally, drop the local override and track the server again -- this
    // is what lets a later server-side change (e.g. resolve_robot_config
    // applying an override during a launch) become visible instead of staying
    // masked by a stale local value forever.
    const status_msg = this.state.status_msg
    if (this.state.selected_robot_config_local !== null && status_msg != null
        && status_msg.selected_robot_config === this.state.selected_robot_config_local) {
      this.setState({ selected_robot_config_local: null })
    }
  }

  // Lifecycle method called just before the component unmounts.
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    if (this.state.robotConfigYamlListener) {
      this.state.robotConfigYamlListener.unsubscribe()
    }
    if (this.state.cameraHorizontalFovListener) {
      this.state.cameraHorizontalFovListener.unsubscribe()
    }
    if (this.state.cameraVerticalFovListener) {
      this.state.cameraVerticalFovListener.unsubscribe()
    }
    if (this.state.robotDimensionsYamlListener) {
      this.state.robotDimensionsYamlListener.unsubscribe()
    }
    if (this.state.environmentDimensionsYamlListener) {
      this.state.environmentDimensionsYamlListener.unsubscribe()
    }
    if (this.state.robotDimensionsDirtyListener) {
      this.state.robotDimensionsDirtyListener.unsubscribe()
    }
    if (this.state.environmentDimensionsDirtyListener) {
      this.state.environmentDimensionsDirtyListener.unsubscribe()
    }
    this.setState({ statusListener: null, robotConfigYamlListener: null,
                    cameraHorizontalFovListener: null, cameraVerticalFovListener: null,
                    robotDimensionsYamlListener: null, environmentDimensionsYamlListener: null,
                    robotDimensionsDirtyListener: null, environmentDimensionsDirtyListener: null })
  }

  // Function for configuring and subscribing to the two static FOV topics --
  // see camera_horizontal_fov_deg's own comment in the constructor for why
  // these are separate latched Float32 topics rather than part of SimStatus.
  updateCameraFovListeners() {
    const namespace = this.getSimNamespace()
    if (this.state.cameraHorizontalFovListener != null) {
      this.state.cameraHorizontalFovListener.unsubscribe()
    }
    if (this.state.cameraVerticalFovListener != null) {
      this.state.cameraVerticalFovListener.unsubscribe()
    }
    if (namespace == null || namespace === 'None') {
      this.setState({ cameraHorizontalFovListener: null, cameraVerticalFovListener: null,
                      camera_horizontal_fov_deg: null, camera_vertical_fov_deg: null })
      return
    }
    const hListener = this.props.ros.setupStatusListener(
      namespace + '/camera_horizontal_fov_deg',
      "std_msgs/Float32",
      (message) => this.setState({ camera_horizontal_fov_deg: message.data })
    )
    const vListener = this.props.ros.setupStatusListener(
      namespace + '/camera_vertical_fov_deg',
      "std_msgs/Float32",
      (message) => this.setState({ camera_vertical_fov_deg: message.data })
    )
    this.setState({ cameraHorizontalFovListener: hListener, cameraVerticalFovListener: vListener })
  }

  // Physical-dimension editing: subscribes to both roles' *_dimensions_yaml
  // (latched reply to a get_*_dimensions request) and *_dimensions_dirty
  // topics, then immediately requests the current values -- unlike the FOV
  // topics above, these aren't published unprompted at startup, only in
  // response to a request, so a freshly (re)mounted component needs to ask
  // before it has anything real to show.
  updateDimensionsListeners() {
    const namespace = this.getSimNamespace()
    ;[this.state.robotDimensionsYamlListener, this.state.environmentDimensionsYamlListener,
      this.state.robotDimensionsDirtyListener, this.state.environmentDimensionsDirtyListener]
      .forEach((listener) => { if (listener != null) { listener.unsubscribe() } })
    if (namespace == null || namespace === 'None') {
      this.setState({ robotDimensionsYamlListener: null, environmentDimensionsYamlListener: null,
                      robotDimensionsDirtyListener: null, environmentDimensionsDirtyListener: null })
      return
    }
    const robotYamlListener = this.props.ros.setupStatusListener(
      namespace + '/robot_dimensions_yaml', "std_msgs/String",
      (message) => this.applyDimensionsYaml('robot', message.data)
    )
    const environmentYamlListener = this.props.ros.setupStatusListener(
      namespace + '/environment_dimensions_yaml', "std_msgs/String",
      (message) => this.applyDimensionsYaml('environment', message.data)
    )
    const robotDirtyListener = this.props.ros.setupStatusListener(
      namespace + '/robot_dimensions_dirty', "std_msgs/Bool",
      (message) => this.setState({ robot_dimensions_dirty: message.data })
    )
    const environmentDirtyListener = this.props.ros.setupStatusListener(
      namespace + '/environment_dimensions_dirty', "std_msgs/Bool",
      (message) => this.setState({ environment_dimensions_dirty: message.data })
    )
    this.setState({ robotDimensionsYamlListener: robotYamlListener,
                    environmentDimensionsYamlListener: environmentYamlListener,
                    robotDimensionsDirtyListener: robotDirtyListener,
                    environmentDimensionsDirtyListener: environmentDirtyListener })
    this.props.ros.sendTriggerMsg(namespace + '/get_robot_dimensions')
    this.props.ros.sendTriggerMsg(namespace + '/get_environment_dimensions')
  }

  // Parses a *_dimensions_yaml reply and merges it over the current field
  // defaults -- "merges over defaults" rather than "replaces wholesale" so
  // a field the device hasn't stored yet (a fresh install, or a field added
  // to ROBOT_DIMENSION_FIELDS after some devices already have a stored
  // dimensions.yaml) still shows its sensible default instead of blank/NaN.
  // Silently ignores anything that isn't a real YAML mapping -- the "no
  // stored dimensions yet" placeholder text isn't valid YAML on purpose, so
  // this simply keeps the JS-side defaults in that case rather than erroring.
  applyDimensionsYaml(role, yamlText) {
    var parsed = null
    try {
      parsed = yaml.load(yamlText)
    } catch (e) {
      parsed = null
    }
    if (parsed === null || typeof parsed !== 'object') {
      return
    }
    const fieldsKey = role + '_dimensions_fields'
    const previewKey = role + '_dimensions_preview_fields'
    this.setState((prevState) => ({
      [fieldsKey]: { ...prevState[fieldsKey], ...parsed },
      [previewKey]: { ...prevState[previewKey], ...parsed },
    }))
  }

  // Function for configuring and subscribing to sim/robot_config_yaml --
  // the device's answer to whichever config name was last sent to
  // sim/get_robot_config (see onViewConfigClicked).
  updateRobotConfigYamlListener() {
    const namespace = this.getSimNamespace()
    if (this.state.robotConfigYamlListener != null) {
      this.state.robotConfigYamlListener.unsubscribe()
      this.setState({ robotConfigYamlListener: null, robot_config_yaml: '', viewing_config_name: 'None' })
    }
    if (namespace != null && namespace !== 'None') {
      var listener = this.props.ros.setupStatusListener(
        namespace + '/robot_config_yaml',
        "std_msgs/String",
        this.robotConfigYamlListener
      )
      this.setState({ robotConfigYamlListener: listener })
    }
  }

  // Callback for sim/robot_config_yaml messages.
  robotConfigYamlListener(message) {
    this.setState({ robot_config_yaml: message.data })
  }

  // Requests a named config's YAML text -- either a checked-in preset
  // (available_robot_configs) or the special sample/uploaded ones aren't
  // reachable this way, since only the device's own self.robot_configs dict
  // (real, currently-loaded configs) answers get_robot_config.
  onViewConfigClicked(configName) {
    const namespace = this.getSimNamespace()
    if (namespace == null || namespace === 'None') {
      return
    }
    this.setState({ viewing_config_name: configName })
    this.props.ros.sendStringMsg(namespace + '/get_robot_config', configName)
  }

  // Shared setter passed down to both NepiIFSimLauncher instances -- see
  // selected_launch_target in this.state for why this lives here instead of
  // in NepiIFSimLauncher's own state.
  onLaunchTargetSelected(value) {
    this.setState({ selected_launch_target: value })
  }

  // Downloads whatever YAML text is currently displayed -- same Blob pattern
  // as onDownloadSampleConfigClicked, just device-reported content instead of
  // a static client-side string.
  onDownloadConfigClicked() {
    const text = this.state.robot_config_yaml
    const name = this.state.viewing_config_name
    if (text === '' || name === 'None') {
      return
    }
    const blob = new Blob([text], { type: 'text/yaml' })
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a')
    link.href = url
    link.download = name + '.yaml'
    document.body.appendChild(link)
    link.click()
    document.body.removeChild(link)
    URL.revokeObjectURL(url)
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
  // The robot config currently DISPLAYED -- local override if one is
  // pending, else whatever the server last reported. See
  // selected_robot_config_local's own comment; used both by the selector's
  // own display and as the value NepiIFSimLauncher's Deploy re-sends
  // immediately before launching.
  getSelectedRobotConfig() {
    if (this.state.selected_robot_config_local !== null) {
      return this.state.selected_robot_config_local
    }
    const status_msg = this.state.status_msg
    return (status_msg != null && status_msg.selected_robot_config !== undefined
            && status_msg.selected_robot_config !== '')
      ? status_msg.selected_robot_config : 'None'
  }

  onRobotConfigSelected(event) {
    const value = event.target.value
    // Optimistic local update FIRST -- see selected_robot_config_local's own
    // comment for why the dropdown no longer waits on a status round trip
    // to reflect a selection, and why Deploy needs this value immediately
    // available rather than only in flight to the backend.
    this.setState({ selected_robot_config_local: value })
    const namespace = this.getSimNamespace()
    if (namespace != null && namespace !== 'None') {
      this.props.ros.sendStringMsg(namespace + '/select_robot_config', value)
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

  // Sends the CURRENT complete fields object for one role, not just
  // whichever single field was last edited -- matches the same "all current
  // values together, always" convention the VM's own camera-settings push
  // uses (sendCameraSettings): the device overwrites its stored
  // dimensions.yaml wholesale with whatever it receives, so a partial send
  // would silently reset every other field to its generator default.
  onSaveDimensionsClicked(role) {
    const namespace = this.getSimNamespace()
    if (namespace == null || namespace === 'None') {
      return
    }
    const fields = this.state[role + '_dimensions_fields']
    const yamlText = yaml.dump(fields)
    this.props.ros.sendStringMsg(namespace + '/set_' + role + '_dimensions', yamlText)
    // Snapshot for the diagram -- see robot_dimensions_preview_fields'
    // own comment for why this is a separate copy from the live-editing
    // fields above.
    this.setState({ [role + '_dimensions_preview_fields']: { ...fields } })
  }

  // Client-side only, downloads the CURRENTLY EDITED fields (not a fresh
  // device round-trip) as YAML -- a convenience snapshot/backup of the
  // curated values, not the actual rendered model.sdf (that only exists on
  // the VM; fetching it would need a second SSH round trip this feature
  // doesn't add). The raw-SDF escape hatch below is for uploading a hand-
  // authored model.sdf, not for downloading the live one.
  onDownloadDimensionsClicked(role) {
    const fields = this.state[role + '_dimensions_fields']
    const yamlText = yaml.dump(fields)
    const blob = new Blob([yamlText], { type: 'text/yaml' })
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a')
    link.href = url
    link.download = role + '_dimensions.yaml'
    document.body.appendChild(link)
    link.click()
    document.body.removeChild(link)
    URL.revokeObjectURL(url)
  }

  onUploadModelSdfClicked(role) {
    const ref = (role === 'robot') ? this.uploadRobotSdfInputRef : this.uploadEnvironmentSdfInputRef
    if (ref.current != null) {
      ref.current.click()
    }
  }

  // Raw-SDF-upload escape hatch -- publishes the picked file's whole text to
  // sim/upload_robot_model_sdf or sim/upload_environment_model_sdf. No
  // client-side XML validation (this app has no SDF parser); a bad upload
  // surfaces the same way a bad hand-edit would on the next Launch, not as
  // an error here.
  onUploadModelSdfFileChange(role, event) {
    const file = (event.target.files && event.target.files.length > 0)
      ? event.target.files[0] : null
    event.target.value = ''
    const namespace = this.getSimNamespace()
    if (file == null || namespace == null || namespace === 'None') {
      return
    }
    const reader = new FileReader()
    reader.onload = () => {
      this.props.ros.sendStringMsg(namespace + '/upload_' + role + '_model_sdf', String(reader.result))
    }
    reader.readAsText(file)
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
    const selected = this.getSelectedRobotConfig()

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

  // Robot Config Settings -- collapsed by default (same show/hide toggle
  // pattern as before, just relabeled: "View Robot Configs" undersold what
  // this section does once upload/download live here too, not just viewing).
  // Combines what used to be two separate always-visible render methods:
  //   - Upload-your-own-robot + Download Sample Config (previously rendered
  //     unconditionally right next to the Robot Config selector -- moved in
  //     here since they're config-management actions, not part of the
  //     pick-and-deploy flow Deploy now sits right under).
  //   - Per-config "View" button (one per available_robot_configs entry --
  //     "each one that I have preset right now... downloadable too, and some
  //     viewer where they can see each config") plus the shared display/
  //     download area below. Deliberately keyed on available_robot_configs,
  //     not a hardcoded drone/rover pair: whatever this deployment's
  //     sim_connector_app_params.yaml actually offers gets a View button, so
  //     a future third preset needs no RUI change to be viewable.
  // Wrapped in Section (bordered box + title) once expanded so it reads as
  // its own distinct panel -- previously just a bare top border on the same
  // black background as everything else, easy to miss entirely.
  renderRobotConfigSettings() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }
    const available = (status_msg.available_robot_configs !== undefined)
      ? status_msg.available_robot_configs : []
    const names = (status_msg.available_robot_config_names !== undefined)
      ? status_msg.available_robot_config_names : []

    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <ButtonMenu>
          <Button onClick={() => this.setState({ show_robot_config_viewer: !this.state.show_robot_config_viewer })}>
            {(this.state.show_robot_config_viewer ? "Hide" : "Show") + " Config Settings"}
          </Button>
        </ButtonMenu>
        {(this.state.show_robot_config_viewer === true) ?
          <Section title={"Config Settings"}>
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
            {(available.length > 0) ?
              <React.Fragment>
                <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
                <ButtonMenu>
                  {available.map((configName, i) => (
                    <Button key={configName} onClick={() => this.onViewConfigClicked(configName)}>
                      {(names[i] !== undefined && names[i] !== '') ? names[i] : configName}
                    </Button>
                  ))}
                </ButtonMenu>
                {(this.state.robot_config_yaml !== '') ?
                  <React.Fragment>
                    {/* Was rows=16/width=100% -- filled most of the page for a
                        handful-of-fields config. A fixed, modest box with its own
                        scrollbar keeps this from dominating the panel regardless
                        of how long any one config's YAML gets. */}
                    <textarea
                      readOnly
                      value={this.state.robot_config_yaml}
                      rows={8}
                      style={{ width: "60%", maxWidth: "40em", fontFamily: "monospace",
                              whiteSpace: "pre", overflow: "auto", display: "block" }}
                    />
                    <ButtonMenu>
                      <Button onClick={this.onDownloadConfigClicked}>{"Download " + this.state.viewing_config_name + ".yaml"}</Button>
                    </ButtonMenu>
                  </React.Fragment>
                : null}
              </React.Fragment>
            : null}
            {this.renderDimensionsEditor('robot', 'Robot Dimensions', ROBOT_DIMENSION_FIELDS,
                                          this.uploadRobotSdfInputRef)}
            {this.renderDimensionsEditor('environment', 'Environment Dimensions', ENVIRONMENT_DIMENSION_FIELDS,
                                          this.uploadEnvironmentSdfInputRef)}
          </Section>
        : null}
      </React.Fragment>
    )
  }

  // One editable Input per curated field, two per row via renderFieldPair --
  // same editable-input pattern as the camera offset controls
  // (Nepi_IF_Sim-Controls.js's renderCameraOffsetControls): id for
  // setElementStyleModified targeting, onChange updates local state, Enter
  // saves (see onSaveDimensionsClicked's own comment for why Enter sends
  // the WHOLE fields object, not just the one edited field).
  renderDimensionFields(role, fieldDefs) {
    const fields = this.state[role + '_dimensions_fields']
    var rows = []
    for (var i = 0; i < fieldDefs.length; i += 2) {
      const a = fieldDefs[i]
      const b = (i + 1 < fieldDefs.length) ? fieldDefs[i + 1] : null
      const renderOne = (f) => (
        <Label key={f.name} title={f.title}>
          <Input
            id={"SimDim_" + role + "_" + f.name}
            value={fields[f.name]}
            onChange={(event) => {
              const el = document.getElementById("SimDim_" + role + "_" + f.name)
              if (el) {
                setElementStyleModified(el)
              }
              // Read event.target.value into a plain variable BEFORE the
              // setState updater below, not inside it -- React's synthetic
              // event is pooled and its fields (including .target) get
              // nulled out once this handler returns, and the updater
              // function form (needed here to spread the PREVIOUS nested
              // fields object rather than replace it) runs on React's own
              // schedule, not necessarily before that happens. Reading
              // event.target.value lazily inside the updater crashed with
              // "Cannot read properties of null (reading 'value')" on
              // every keystroke -- reported live (2026-08-31) as the whole
              // RUI going black, since nothing here has an error boundary.
              const value = event.target.value
              const fieldsKey = role + '_dimensions_fields'
              this.setState((prevState) => ({
                [fieldsKey]: { ...prevState[fieldsKey], [f.name]: value }
              }))
            }}
            onKeyDown={(event) => {
              if (event.key !== 'Enter') {
                return
              }
              const el = document.getElementById("SimDim_" + role + "_" + f.name)
              if (el) {
                clearElementStyleModified(el)
              }
              this.onSaveDimensionsClicked(role)
            }}
          />
        </Label>
      )
      rows.push(
        <React.Fragment key={a.name}>
          {(b != null) ? this.renderFieldPair(renderOne(a), renderOne(b)) : renderOne(a)}
        </React.Fragment>
      )
    }
    return rows
  }

  // Top-down schematic of the rover from robot_dimensions_preview_fields
  // (see that state field's own comment for why it's a separate snapshot,
  // not the live-editing fields). Wheel positions match
  // generate_model_sdf.py's own WHEEL_POSITIONS exactly (front is +x,
  // left is +y) rather than guessing at the layout independently, so this
  // stays true to what actually gets built on the VM. Chassis height has
  // no top-down representation -- called out in the caption text instead.
  renderRobotDimensionsDiagram(fields) {
    // Clamped to non-negative here, at the one place every dimension is
    // read, rather than at each derived width/height below -- a negative
    // physical dimension (typed by accident, or while testing) is
    // nonsensical regardless of which shape it would have fed, and letting
    // one through would make scale negative, which makes every SVG
    // width/height in this diagram negative too.
    const get = (name) => Math.max(0, numericDimensionField(fields, ROBOT_DIMENSION_FIELDS, name))
    const wheelRadius = get('wheel_radius_m')
    const wheelWidth = get('wheel_width_m')
    const trackWidth = get('track_width_m')
    const wheelbase = get('wheelbase_m')
    const chassisLength = get('chassis_length_m')
    const chassisWidth = get('chassis_width_m')
    const chassisHeight = get('chassis_height_m')

    const boundW = Math.max(chassisLength, wheelbase + 2 * wheelRadius, 0.05)
    const boundH = Math.max(chassisWidth, trackWidth + wheelWidth, 0.05)
    const viewW = 280
    const viewH = 170
    const pad = 30
    const scale = Math.min((viewW - 2 * pad) / boundW, (viewH - 2 * pad) / boundH)
    const cx = viewW / 2
    const cy = viewH / 2

    // sy=+1 (left, REP103 convention) drawn upward -- smaller SVG y --
    // same up-is-positive mapping renderEnvironmentDimensionsDiagram uses,
    // so the two panels read consistently if seen side by side.
    const wheelPositions = [
      { key: 'FL', sx: 1, sy: 1 }, { key: 'FR', sx: 1, sy: -1 },
      { key: 'RL', sx: -1, sy: 1 }, { key: 'RR', sx: -1, sy: -1 },
    ]
    const wheelBoxW = 2 * wheelRadius * scale
    const wheelBoxH = wheelWidth * scale

    return (
      <React.Fragment>
        <svg viewBox={`0 0 ${viewW} ${viewH}`} width="100%" height={viewH}
             style={{ background: DIAGRAM_BG, borderRadius: 4 }}>
          <rect x={cx - (chassisLength * scale) / 2} y={cy - (chassisWidth * scale) / 2}
                width={chassisLength * scale} height={chassisWidth * scale}
                fill="none" stroke={Styles.vars.colors.blue} strokeWidth="2" />
          {wheelPositions.map((p) => (
            <rect key={p.key}
                  x={cx + p.sx * (wheelbase / 2) * scale - wheelBoxW / 2}
                  y={cy - p.sy * (trackWidth / 2) * scale - wheelBoxH / 2}
                  width={wheelBoxW} height={wheelBoxH}
                  fill={Styles.vars.colors.grey1} />
          ))}
          <polygon fill={Styles.vars.colors.orange} points={
            (cx + (chassisLength / 2) * scale + 7) + "," + cy + " " +
            (cx + (chassisLength / 2) * scale - 3) + "," + (cy - 6) + " " +
            (cx + (chassisLength / 2) * scale - 3) + "," + (cy + 6)
          } />
        </svg>
        <div style={{ fontSize: 11, color: Styles.vars.colors.grey1, marginTop: Styles.vars.spacing.xs }}>
          {"Wheelbase " + wheelbase.toFixed(2) + "m · Track " + trackWidth.toFixed(2) +
           "m · Chassis " + chassisLength.toFixed(2) + "×" + chassisWidth.toFixed(2) +
           "×" + chassisHeight.toFixed(2) + "m (L×W×H)"}
        </div>
      </React.Fragment>
    )
  }

  // Top-down schematic of the obstacle course from
  // environment_dimensions_preview_fields. Baffle and ramp placement match
  // generate_model_sdf.py's buildObstacleCourseSdf exactly: each baffle
  // attaches to one side wall and reaches to within baffle_gap_m of the
  // centerline (so a robot driving straight down the middle clears both by
  // that margin), and the ramp's horizontal run is derived from
  // rise/tan(angle) the same way that function derives it -- this is
  // computed independently in JS rather than asking the device for it,
  // so the diagram matches what "Save Dimensions" is ABOUT to build, not
  // what was last actually loaded there. The ramp only has a top-down
  // footprint here (a shaded zone); its rise isn't a top-down-representable
  // quantity, so it's called out in the caption text instead.
  renderEnvironmentDimensionsDiagram(fields) {
    // Clamped to non-negative here, at the one place every dimension is
    // read, rather than at each derived width/height below -- see
    // renderRobotDimensionsDiagram's own comment for why (same reasoning,
    // negative scale from one negative input making every shape's
    // width/height negative too).
    const get = (name) => Math.max(0, numericDimensionField(fields, ENVIRONMENT_DIMENSION_FIELDS, name))
    const courseStartX = get('course_start_x_m')
    const corridorWidth = get('corridor_width_m')
    const wallLength = get('wall_length_m')
    const wallThickness = get('wall_thickness_m')
    const baffleAX = get('baffle_a_x_m')
    const baffleBX = get('baffle_b_x_m')
    const baffleGap = get('baffle_gap_m')
    const baffleThickness = get('baffle_thickness_m')
    const rampStartX = get('ramp_start_x_m')
    const rampRise = get('ramp_rise_m')
    // Also capped below 90 -- tan() approaches infinity as the angle
    // approaches a vertical ramp, which would make `run` (and everything
    // derived from it) blow up rather than just draw a very steep ramp.
    const rampAngleDeg = Math.min(get('ramp_angle_deg'), 89.9)

    const plateauLength = get('ramp_plateau_length_m')

    const halfCorridor = corridorWidth / 2
    const angleRad = (rampAngleDeg * Math.PI) / 180
    const run = angleRad > 0 ? rampRise / Math.tan(angleRad) : 0
    const rampEndX = rampStartX + 2 * run + plateauLength

    const boundW = Math.max(wallLength, rampEndX, 0.5)
    const boundH = corridorWidth + 2 * wallThickness
    const viewW = 560
    const viewH = 190
    const padX = 20
    const padY = 24
    const scale = Math.min((viewW - 2 * padX) / boundW, (viewH - 2 * padY) / boundH)
    const midY = viewH / 2
    const toX = (worldX) => padX + worldX * scale
    // world +y (toward the wall baffle_a attaches to) drawn upward, same
    // up-is-positive convention as renderRobotDimensionsDiagram.
    const toY = (worldY) => midY - worldY * scale

    const baffleReach = Math.max(halfCorridor - baffleGap, 0)

    return (
      <React.Fragment>
        <svg viewBox={`0 0 ${viewW} ${viewH}`} width="100%" height={viewH}
             style={{ background: DIAGRAM_BG, borderRadius: 4 }}>
          <rect x={toX(0)} y={toY(halfCorridor)} width={wallLength * scale} height={corridorWidth * scale}
                fill="#26292d" />
          {(rampEndX > rampStartX) ?
            <rect x={toX(rampStartX)} y={toY(halfCorridor)} width={(rampEndX - rampStartX) * scale}
                  height={corridorWidth * scale} fill={Styles.vars.colors.blue} opacity="0.18" />
          : null}
          <rect x={toX(0)} y={toY(halfCorridor + wallThickness)} width={wallLength * scale}
                height={wallThickness * scale} fill={Styles.vars.colors.grey1} />
          <rect x={toX(0)} y={toY(-halfCorridor)} width={wallLength * scale}
                height={wallThickness * scale} fill={Styles.vars.colors.grey1} />
          <rect x={toX(baffleAX - baffleThickness / 2)} y={toY(halfCorridor)}
                width={baffleThickness * scale} height={baffleReach * scale}
                fill={Styles.vars.colors.orange} />
          <rect x={toX(baffleBX - baffleThickness / 2)} y={toY(-baffleGap)}
                width={baffleThickness * scale} height={baffleReach * scale}
                fill={Styles.vars.colors.orange} />
          <circle cx={toX(courseStartX)} cy={toY(0)} r="4" fill={Styles.vars.colors.green} />
          <text x={toX(courseStartX)} y={toY(halfCorridor + wallThickness) - 4} textAnchor="middle"
                fill={Styles.vars.colors.green} fontSize="9">{"START"}</text>
          {(rampEndX > rampStartX) ?
            <text x={toX((rampStartX + rampEndX) / 2)} y={toY(0) + 3} textAnchor="middle"
                  fill={Styles.vars.colors.blue} fontSize="10">{"RAMP"}</text>
          : null}
        </svg>
        <div style={{ fontSize: 11, color: Styles.vars.colors.grey1, marginTop: Styles.vars.spacing.xs }}>
          {"Corridor " + corridorWidth.toFixed(2) + "m × " + wallLength.toFixed(2) +
           "m · Ramp rises " + rampRise.toFixed(2) + "m over " + run.toFixed(2) +
           "m at " + rampAngleDeg.toFixed(1) + "°, then a " + plateauLength.toFixed(2) +
           "m plateau"}
        </div>
      </React.Fragment>
    )
  }

  // Full sub-section for one role: curated fields (renderDimensionFields
  // above), the schematic preview (renderRobotDimensionsDiagram/
  // renderEnvironmentDimensionsDiagram), a "changes apply on next Launch"
  // note gated on the *_dirty status (Gazebo only reads model.sdf at spawn
  // time, never live), and the raw-SDF-upload/download escape hatch for
  // geometry the curated fields don't cover.
  // Wraps whichever diagram renderer in a try/catch -- a bad or
  // still-in-progress numeric value should degrade to a plain message, never
  // take the WHOLE page down. There is no error boundary anywhere in this
  // app (or, as far as this pass found, in nepi_rui generally), so an
  // uncaught exception during render unmounts all of React, not just this
  // one panel -- reported live (2026-08-31) as "the whole thing goes black"
  // right after this preview feature shipped.
  renderDimensionsDiagramSafe(role, previewFields) {
    try {
      return (role === 'robot') ? this.renderRobotDimensionsDiagram(previewFields)
                                 : this.renderEnvironmentDimensionsDiagram(previewFields)
    } catch (e) {
      return (
        <div style={{ fontSize: 11, color: Styles.vars.colors.red, marginTop: Styles.vars.spacing.xs }}>
          {"Preview unavailable for the current values (" + e.message + ")"}
        </div>
      )
    }
  }

  renderDimensionsEditor(role, title, fieldDefs, uploadInputRef) {
    const dirty = this.state[role + '_dimensions_dirty']
    const previewFields = this.state[role + '_dimensions_preview_fields']
    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <Section title={title}>
          {this.renderDimensionFields(role, fieldDefs)}
          {this.renderDimensionsDiagramSafe(role, previewFields)}
          {(dirty === true) ?
            <div style={{
              fontStyle: "italic",
              color: Styles.vars.colors.grey1,
              marginTop: Styles.vars.spacing.small,
              marginBottom: Styles.vars.spacing.small,
            }}>
              {"Edited -- applies on the next Launch"}
            </div>
          : null}
          <input
            type="file"
            accept=".sdf,.xml,text/xml"
            ref={uploadInputRef}
            style={{ display: 'none' }}
            onChange={(event) => this.onUploadModelSdfFileChange(role, event)}
          />
          <ButtonMenu>
            <Button onClick={() => this.onSaveDimensionsClicked(role)}>{"Save Dimensions"}</Button>
            <Button onClick={() => this.onDownloadDimensionsClicked(role)}>{"Download Dimensions (YAML)"}</Button>
            <Button onClick={() => this.onUploadModelSdfClicked(role)}>{"Upload Raw model.sdf"}</Button>
          </ButtonMenu>
        </Section>
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

        {this.renderFieldPair(
          <Label title={"Camera Horizontal FOV (deg)"}>
            <Input disabled value={(this.state.camera_horizontal_fov_deg != null)
              ? round(this.state.camera_horizontal_fov_deg, 1) : ""} />
          </Label>,
          <Label title={"Camera Vertical FOV (deg)"}>
            <Input disabled value={(this.state.camera_vertical_fov_deg != null)
              ? round(this.state.camera_vertical_fov_deg, 1) : ""} />
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
            {/* NepiIFSimLauncher's own target selector IS the Simulator
                selector for this whole panel -- it already lists every real
                option (Gazebo/Webots/PyBullet/WPILib). Placed FIRST,
                alone: choosing which simulator to deploy is the first
                decision in the flow (simulator -> robot config -> deploy),
                so only the selector half of NepiIFSimLauncher renders here
                (only="selector") -- its Deploy/Kill/Install controls are
                intentionally moved below Robot Config + the capability
                controls (see NepiIFSimLauncher only="deploy" near the
                bottom), so Deploy isn't the first/most prominent thing on
                the page ahead of picking a real robot config. This
                component used to render a SECOND "Simulator" selector here,
                backed by available_simulators/select_simulator (live
                discovery of OTHER NEPI devices that declare themselves
                simulators -- see getAvailableSimulators/simDiscoveryCb in
                sim_connector_app_node.py, a deliberately different axis from
                this SSH launch-target list). That mechanism stays intact
                server-side for a possible future use, but nothing on this
                deployment ever populates it, so showing it here just
                produced two same-titled, mostly-empty-vs-real "Simulator"
                fields stacked on top of each other -- removed rather than
                merged, since the two lists mean genuinely different things
                (already-connected device vs. launch-this-on-the-VM) and
                forcing them into one dropdown would misrepresent both. */}
            <NepiIFSimLauncher
              namespace={namespace}
              make_section={false}
              only={"selector"}
              selected_target={this.state.selected_launch_target}
              onTargetSelected={this.onLaunchTargetSelected}
            />
            {this.renderRobotConfigSelector()}

            {/* Deploy/Kill/Install controls -- right after picking WHAT to
                run (simulator) and WHICH robot config, before Robot Config
                Settings and the capability-configuration controls below.
                Those only shape what a robot exposes once running (or, for
                Robot Config Settings, manage config presets), not the
                pick-and-go deploy decision -- keeping Deploy right under the
                two things that actually decide what gets deployed. */}
            <NepiIFSimLauncher
              namespace={namespace}
              make_section={false}
              only={"deploy"}
              selected_target={this.state.selected_launch_target}
              onTargetSelected={this.onLaunchTargetSelected}
              selected_robot_config={this.getSelectedRobotConfig()}
            />

            {this.renderRobotConfigSettings()}
          </React.Fragment>
        : null}

        {(show_data === true) ?
          this.renderData()
        : null}

        {/* Always mounted, even when show_controls is false: NepiIFSimControls
            renders two logically separate groups internally -- live control
            (motor sliders, goto SEND buttons, home/stop actions, the live
            camera viewer) gated on show_live_controls, and configuration
            (capability toggles, image-source curation, camera view/offset
            settings, environment, movement limits) which always renders
            regardless. Configuring what shows up in Devices -> Robots is
            this app's actual purpose -- hiding it along with live control
            was an unintended side effect of the two being bundled into one
            gated component. See docs/SIM_CONNECTOR_CONFIG_CONTROLS_PLAN.md. */}
        <NepiIFSimControls
          namespace={namespace}
          make_section={false}
          show_live_controls={show_controls}
        />

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
