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

// Mirrors sim_connector_app_node.py's own PROTECTED_DIMENSION_CONFIG_NAMES --
// the built-in, undeletable config names per role (see that constant's own
// comment for the full design). No name here is ever "Default" -- each
// built-in is named for what it actually is.
const PROTECTED_DIMENSION_CONFIG_NAMES = {
  robot: ['4-Wheel Rover'],
  environment: ['Flat', 'Obstacle Course', 'Aerial Obstacle Course', 'Custom Obstacles'],
}
// Mirrors sim_connector_app_node.py's own custom_obstacles model value --
// the one environment model with no fixed curated field set (see
// CUSTOM_OBSTACLE_TYPES/renderCustomObstaclesEditor below).
const CUSTOM_OBSTACLES_MODEL = 'custom_obstacles'
// Mirrors sim_connector_app_node.py's own FALLBACK_DIMENSION_CONFIG_NAME --
// what each role's selector/config list starts on before any real status
// has arrived.
const FALLBACK_DIMENSION_CONFIG_NAME = {
  robot: '4-Wheel Rover',
  environment: 'Obstacle Course',
}
// Mirrors sim_connector_app_node.py's own ENVIRONMENT_MODEL_NONE -- the
// Flat built-in's reserved model value, meaning "no model to generate,
// push, or preview," not a missing/unset field.
const ENVIRONMENT_MODEL_NONE = 'none'

function isProtectedDimensionConfig(role, name) {
  return (PROTECTED_DIMENSION_CONFIG_NAMES[role] || []).indexOf(name) !== -1
}

// Curated physical-dimension fields -- one entry per generate_model_sdf.py
// independent parameter (see that script's own ROVER_DEFAULT_DIMENSIONS/
// OBSTACLE_COURSE_DEFAULT_DIMENSIONS/AERIAL_OBSTACLE_COURSE_DEFAULT_DIMENSIONS
// for the derivations these feed). Default values here match the script's
// own defaults, shown until a real device response arrives
// (sim/robot_dimensions_yaml / sim/environment_dimensions_yaml).
const ROBOT_DIMENSION_FIELDS = [
  { name: "wheel_radius_m", title: "Wheel Radius (m)", default: 0.1 },
  { name: "wheel_width_m", title: "Wheel Width (m)", default: 0.05 },
  { name: "track_width_m", title: "Track Width (m)", default: 0.34 },
  { name: "wheelbase_m", title: "Wheelbase (m)", default: 0.3 },
  { name: "chassis_length_m", title: "Chassis Length (m)", default: 0.4 },
  { name: "chassis_width_m", title: "Chassis Width (m)", default: 0.3 },
  { name: "chassis_height_m", title: "Chassis Height (m)", default: 0.1 },
  // altUnit renders a second, companion Input right next to the main one,
  // showing/editing the SAME underlying field (weight_kg) converted to a
  // different unit -- both inputs always agree since there is only ever one
  // stored value (kg); the lbs box is purely a display/edit convenience.
  // Requested live: "add a weight parameter... should work in lbs and kgs."
  // Also drives base_link's mass AND its recomputed-from-mass inertia in
  // generate_model_sdf.py's buildRoverSdf -- see that function's own
  // comment for why inertia can't be left hardcoded once mass is editable.
  { name: "weight_kg", title: "Weight (kg)", default: 5.0,
    altUnit: { title: "Weight (lbs)", toAlt: (kg) => kg * 2.20462, fromAlt: (lbs) => lbs / 2.20462 } },
  { name: "camera_horizontal_fov_deg", title: "Camera Horizontal FOV (deg)", default: 80.0 },
]

const OBSTACLE_COURSE_DIMENSION_FIELDS = [
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

const AERIAL_OBSTACLE_COURSE_DIMENSION_FIELDS = [
  { name: "course_start_x_m", title: "Course Start X (m)", default: 3.0 },
  { name: "gate_count", title: "Gate Count", default: 4 },
  { name: "gate_spacing_m", title: "Gate Spacing (m)", default: 6.0 },
  { name: "gate_opening_width_m", title: "Gate Opening Width (m)", default: 2.0 },
  { name: "gate_opening_height_m", title: "Gate Opening Height (m)", default: 2.0 },
  { name: "gate_frame_thickness_m", title: "Gate Frame Thickness (m)", default: 0.15 },
  { name: "gate_base_height_m", title: "Gate Base Height (m)", default: 2.0 },
  { name: "gate_height_step_m", title: "Gate Height Step (m)", default: 1.0 },
]

// Custom Obstacles has no fixed field list -- it's an operator-built LIST
// of these, edited via renderCustomObstaclesEditor instead of
// renderDimensionFields. Each type's own independent params mirror
// generate_model_sdf.py's matching _obstacle*Link function exactly. x/y/
// yaw_deg are common to every type that supports rotation (circle has no
// yaw -- it's rotationally symmetric).
const CUSTOM_OBSTACLE_TYPES = ['wall', 'circle', 'triangle']
const CUSTOM_OBSTACLE_TYPE_DEFAULTS = {
  wall: { type: 'wall', x: 2.0, y: 0.0, yaw_deg: 0.0, length_m: 2.0, thickness_m: 0.2, height_m: 1.0 },
  circle: { type: 'circle', x: 2.0, y: 0.0, radius_m: 0.5, height_m: 1.0 },
  triangle: { type: 'triangle', x: 2.0, y: 0.0, yaw_deg: 0.0, base_m: 1.0, depth_m: 1.0, height_m: 1.0 },
}
const CUSTOM_OBSTACLE_TYPE_FIELD_NAMES = {
  wall: ['x', 'y', 'yaw_deg', 'length_m', 'thickness_m', 'height_m'],
  circle: ['x', 'y', 'radius_m', 'height_m'],
  triangle: ['x', 'y', 'yaw_deg', 'base_m', 'depth_m', 'height_m'],
}

// Rotates a LOCAL (obstacle-frame) offset by yaw_deg and translates it to
// the obstacle's world position -- shared by every rotated obstacle type's
// own vertex/handle placement (a wall's resize corner, a triangle's three
// vertices), so the math lives in exactly one place instead of being
// re-derived per shape.
function localToWorldPoint(originX, originY, yawDeg, localX, localY) {
  const rad = (yawDeg * Math.PI) / 180
  return {
    x: originX + localX * Math.cos(rad) - localY * Math.sin(rad),
    y: originY + localX * Math.sin(rad) + localY * Math.cos(rad),
  }
}

// Which curated field set applies depends on which model the CURRENTLY
// SELECTED environment config targets (see sim_connector_app_node.py's own
// ENVIRONMENT_MODEL_FIELD_KEY comment) -- 'environment' is the only role
// where this varies; 'robot' always means generic_rover. custom_obstacles
// maps to an empty field list here since it's edited through
// renderCustomObstaclesEditor instead, never renderDimensionFields.
const ENVIRONMENT_DIMENSION_FIELDS_BY_MODEL = {
  obstacle_course: OBSTACLE_COURSE_DIMENSION_FIELDS,
  aerial_obstacle_course: AERIAL_OBSTACLE_COURSE_DIMENSION_FIELDS,
  [CUSTOM_OBSTACLES_MODEL]: [],
  [ENVIRONMENT_MODEL_NONE]: [],
}

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

    // Lets onSelectDimensionConfig reach NepiIFSimControls's own
    // setEnvironmentSetting directly -- see that method's own comment for
    // why the two components need to stay in sync (this dropdown edits
    // geometry fields only; NepiIFSimControls's "Environment" dropdown is
    // the one that actually spawns/despawns obstacles in Gazebo).
    this.simControlsRef = React.createRef()

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
      // Same collapsed-by-default pattern, environment's own equivalent
      // panel (see renderEnvironmentConfigSettings).
      show_environment_config_viewer: false,

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
      environment_dimensions_fields: defaultDimensionFields(OBSTACLE_COURSE_DIMENSION_FIELDS),
      robot_dimensions_dirty: false,
      environment_dimensions_dirty: false,
      // Which model the currently-selected ENVIRONMENT config targets (see
      // ENVIRONMENT_DIMENSION_FIELDS_BY_MODEL) -- read from the device's own
      // sim/environment_dimensions_selected_model (latched), updated
      // whenever the selected config changes. 'robot' has no equivalent --
      // it always means generic_rover.
      environment_dimensions_model: 'obstacle_course',
      environmentDimensionsModelListener: null,

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
      environment_dimensions_preview_fields: defaultDimensionFields(OBSTACLE_COURSE_DIMENSION_FIELDS),
      robotDimensionsYamlListener: null,
      environmentDimensionsYamlListener: null,
      robotDimensionsDirtyListener: null,
      environmentDimensionsDirtyListener: null,

      // Named dimensions configs -- see sim_connector_app_node.py's own
      // "Named dimensions configs" comment for the full design. Names/
      // selected come from two new latched topics per role; save_as_name is
      // this component's own pending-input state for the "Save As New..."
      // control, cleared once a save actually goes out.
      robot_dimensions_config_names: [FALLBACK_DIMENSION_CONFIG_NAME.robot],
      environment_dimensions_config_names: ['Flat', 'Obstacle Course', 'Aerial Obstacle Course', 'Custom Obstacles'],
      robot_dimensions_selected_config: FALLBACK_DIMENSION_CONFIG_NAME.robot,
      environment_dimensions_selected_config: FALLBACK_DIMENSION_CONFIG_NAME.environment,
      // Merged robot button row's own optimistic selection (see
      // renderRobotConfigAndDimensionsButtons) -- tracks which NAME was last
      // clicked there so Delete Selected Config targets exactly that name
      // without waiting on the async selected-config echo from either axis.
      robot_merged_selected_name: null,
      // Raw YAML text of whichever config was selected/saved most
      // recently -- see applyDimensionsYaml's own comment. Shown read-only
      // by renderRobotConfigAndDimensionsButtons (robot) /
      // renderEnvironmentConfigSelector (environment), the same "click a
      // config, see its YAML" pattern both axes share.
      robot_dimensions_config_yaml_text: '',
      environment_dimensions_config_yaml_text: '',
      robot_dimensions_save_as_name: '',
      environment_dimensions_save_as_name: '',
      robotDimensionsConfigNamesListener: null,
      environmentDimensionsConfigNamesListener: null,
      robotDimensionsSelectedConfigListener: null,
      environmentDimensionsSelectedConfigListener: null,

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
    this.renderEnvironmentConfigSelector = this.renderEnvironmentConfigSelector.bind(this)
    this.renderRobotConfigSettings = this.renderRobotConfigSettings.bind(this)
    this.renderEnvironmentConfigSettings = this.renderEnvironmentConfigSettings.bind(this)
    this.renderFieldPair = this.renderFieldPair.bind(this)
    this.renderData = this.renderData.bind(this)

    this.updateDimensionsListeners = this.updateDimensionsListeners.bind(this)
    this.applyDimensionConfigNames = this.applyDimensionConfigNames.bind(this)
    this.onSaveDimensionsClicked = this.onSaveDimensionsClicked.bind(this)
    this.onSelectDimensionConfig = this.onSelectDimensionConfig.bind(this)
    this.saveDimensionsAsNamed = this.saveDimensionsAsNamed.bind(this)
    this.onSaveDimensionConfigAsClicked = this.onSaveDimensionConfigAsClicked.bind(this)
    this.onDeleteDimensionConfigClicked = this.onDeleteDimensionConfigClicked.bind(this)
    this.onDeleteMergedRobotConfigClicked = this.onDeleteMergedRobotConfigClicked.bind(this)
    this.renderRobotConfigAndDimensionsButtons = this.renderRobotConfigAndDimensionsButtons.bind(this)
    this.renderAerialObstacleCourseDiagram = this.renderAerialObstacleCourseDiagram.bind(this)
    this.onDownloadDimensionsClicked = this.onDownloadDimensionsClicked.bind(this)
    this.onUploadModelSdfClicked = this.onUploadModelSdfClicked.bind(this)
    this.onUploadModelSdfFileChange = this.onUploadModelSdfFileChange.bind(this)
    this.renderDragHandle = this.renderDragHandle.bind(this)
    this.startDimensionDrag = this.startDimensionDrag.bind(this)
    this.getCustomObstacles = this.getCustomObstacles.bind(this)
    this.setCustomObstacles = this.setCustomObstacles.bind(this)
    this.onAddObstacleClicked = this.onAddObstacleClicked.bind(this)
    this.onDeleteObstacleClicked = this.onDeleteObstacleClicked.bind(this)
    this.onObstacleFieldInputChange = this.onObstacleFieldInputChange.bind(this)
    this.startObstacleDrag = this.startObstacleDrag.bind(this)
    this.renderObstacleFieldRow = this.renderObstacleFieldRow.bind(this)
    this.renderCustomObstacleShape = this.renderCustomObstacleShape.bind(this)
    this.renderCustomObstaclesDiagram = this.renderCustomObstaclesDiagram.bind(this)
    this.renderCustomObstaclesEditor = this.renderCustomObstaclesEditor.bind(this)
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
    ;[this.state.robotDimensionsConfigNamesListener, this.state.environmentDimensionsConfigNamesListener,
      this.state.robotDimensionsSelectedConfigListener, this.state.environmentDimensionsSelectedConfigListener,
      this.state.environmentDimensionsModelListener]
      .forEach((listener) => { if (listener != null) { listener.unsubscribe() } })
    this.setState({ statusListener: null, robotConfigYamlListener: null,
                    cameraHorizontalFovListener: null, cameraVerticalFovListener: null,
                    robotDimensionsYamlListener: null, environmentDimensionsYamlListener: null,
                    robotDimensionsDirtyListener: null, environmentDimensionsDirtyListener: null,
                    robotDimensionsConfigNamesListener: null, environmentDimensionsConfigNamesListener: null,
                    robotDimensionsSelectedConfigListener: null, environmentDimensionsSelectedConfigListener: null,
                    environmentDimensionsModelListener: null })
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
      this.state.robotDimensionsDirtyListener, this.state.environmentDimensionsDirtyListener,
      this.state.robotDimensionsConfigNamesListener, this.state.environmentDimensionsConfigNamesListener,
      this.state.robotDimensionsSelectedConfigListener, this.state.environmentDimensionsSelectedConfigListener,
      this.state.environmentDimensionsModelListener]
      .forEach((listener) => { if (listener != null) { listener.unsubscribe() } })
    if (namespace == null || namespace === 'None') {
      this.setState({ robotDimensionsYamlListener: null, environmentDimensionsYamlListener: null,
                      robotDimensionsDirtyListener: null, environmentDimensionsDirtyListener: null,
                      robotDimensionsConfigNamesListener: null, environmentDimensionsConfigNamesListener: null,
                      robotDimensionsSelectedConfigListener: null, environmentDimensionsSelectedConfigListener: null,
                      environmentDimensionsModelListener: null })
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
    // Named-config lists/selection -- both latched and published unprompted
    // at startup by the backend (see sim_connector_app_node.py's own
    // constructor comment), unlike the yaml/dirty topics above, so there is
    // nothing to explicitly request here.
    const robotConfigNamesListener = this.props.ros.setupStatusListener(
      namespace + '/robot_dimensions_config_names', "std_msgs/String",
      (message) => this.applyDimensionConfigNames('robot', message.data)
    )
    const environmentConfigNamesListener = this.props.ros.setupStatusListener(
      namespace + '/environment_dimensions_config_names', "std_msgs/String",
      (message) => this.applyDimensionConfigNames('environment', message.data)
    )
    const robotSelectedConfigListener = this.props.ros.setupStatusListener(
      namespace + '/robot_dimensions_selected_config', "std_msgs/String",
      (message) => this.setState({ robot_dimensions_selected_config: message.data })
    )
    const environmentSelectedConfigListener = this.props.ros.setupStatusListener(
      namespace + '/environment_dimensions_selected_config', "std_msgs/String",
      (message) => this.setState({ environment_dimensions_selected_config: message.data })
    )
    // Which model the currently-selected environment config targets --
    // drives which curated field set (ENVIRONMENT_DIMENSION_FIELDS_BY_MODEL)
    // is shown/edited. Tops up any of that model's fields not yet present
    // in state with their JS-side defaults (preserving whatever real values
    // are already there) so a freshly-relevant field never shows blank/NaN
    // before the config's own *_dimensions_yaml reply (which this app
    // publishes right after this one, see publishSelectedDimensionConfig)
    // arrives with the real values.
    const environmentModelListener = this.props.ros.setupStatusListener(
      namespace + '/environment_dimensions_selected_model', "std_msgs/String",
      (message) => {
        const model = message.data
        const fieldDefs = ENVIRONMENT_DIMENSION_FIELDS_BY_MODEL[model] || []
        this.setState((prevState) => ({
          environment_dimensions_model: model,
          environment_dimensions_fields: { ...defaultDimensionFields(fieldDefs), ...prevState.environment_dimensions_fields },
          environment_dimensions_preview_fields: { ...defaultDimensionFields(fieldDefs), ...prevState.environment_dimensions_preview_fields },
        }))
      }
    )
    this.setState({ robotDimensionsYamlListener: robotYamlListener,
                    environmentDimensionsYamlListener: environmentYamlListener,
                    robotDimensionsDirtyListener: robotDirtyListener,
                    environmentDimensionsDirtyListener: environmentDirtyListener,
                    robotDimensionsConfigNamesListener: robotConfigNamesListener,
                    environmentDimensionsConfigNamesListener: environmentConfigNamesListener,
                    robotDimensionsSelectedConfigListener: robotSelectedConfigListener,
                    environmentDimensionsSelectedConfigListener: environmentSelectedConfigListener,
                    environmentDimensionsModelListener: environmentModelListener })
    this.props.ros.sendTriggerMsg(namespace + '/get_robot_dimensions')
    this.props.ros.sendTriggerMsg(namespace + '/get_environment_dimensions')
  }

  // Parses a *_dimensions_config_names reply (a JSON array of name strings,
  // see sim_connector_app_node.py's publishAvailableDimensionConfigs) into
  // this component's own array state for the dropdown below. Malformed/
  // non-array payloads are ignored, same defensive convention
  // applyDimensionsYaml already uses for its own topic.
  applyDimensionConfigNames(role, jsonText) {
    var names = null
    try {
      names = JSON.parse(jsonText)
    } catch (e) {
      names = null
    }
    if (!Array.isArray(names)) {
      return
    }
    this.setState({ [role + '_dimensions_config_names']: names })
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
    // Also captured raw, unparsed, into *_dimensions_config_yaml_text --
    // renderRobotConfigAndDimensionsButtons/renderEnvironmentConfigSelector's
    // own read-only viewers show exactly this text for whichever named
    // config is currently selected, the same "click a config, see its YAML"
    // behavior the capability robot-config buttons already have
    // (getRobotConfigCb/robot_config_yaml). This topic
    // is what select_<role>_dimensions_config's own backend handler
    // (applyDimensionConfigByName) re-publishes to, so selecting a named
    // config populates this viewer with no separate request/topic needed.
    this.setState({ [role + '_dimensions_config_yaml_text']: yamlText })
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

  // Switches to a saved dimensions config -- selectDimensionConfigCb on the
  // device side loads it, makes it the active one (same effect Save
  // Dimensions already has), and echoes it back over the existing
  // *_dimensions_yaml topic, so applyDimensionsYaml refreshes the editable
  // fields, preview diagram, AND the relevant YAML viewer (see
  // applyDimensionsYaml's own comment) without this handler touching any of
  // them directly. Takes the
  // name directly (not an event) -- driven by a button click, the same
  // shape onViewConfigClicked already uses for capability robot configs.
  onSelectDimensionConfig(role, name) {
    const namespace = this.getSimNamespace()
    if (namespace == null || namespace === 'None' || !name) {
      return
    }
    this.props.ros.sendStringMsg(namespace + '/select_' + role + '_dimensions_config', name)
    // Drives NepiIFSimControls's own "environment" Setting (the control
    // that actually spawns/despawns obstacles in Gazebo) so this selector
    // is the ONE place environment changes take visual effect -- requested
    // live (2026-09-04): "environment should be live changeable from the
    // top selector too" (the separate dropdown this used to also require
    // picking is gone now, see NepiIFSimControls's own setEnvironmentSetting
    // comment). Sends every environment dimensions-config name now, not
    // just "Flat"/"Obstacle Course" -- setEnvironmentSetting translates the
    // name and the backend's own Setting validation harmlessly ignores a
    // guess that doesn't match any real currently-scanned model, so this
    // is safe even for a name with nothing to actually spawn.
    if (role === 'environment' && this.simControlsRef.current) {
      this.simControlsRef.current.setEnvironmentSetting(name)
    }
  }

  // Saves the CURRENTLY EDITED fields (not the last-loaded config) under a
  // new or existing name, and makes it the active one -- see
  // saveDimensionConfigCb's own comment for why "save" always also means
  // "use". An empty name pops an alert instead of quietly no-oping (or
  // disabling the button, which gave no feedback at all about WHY nothing
  // happened) -- requested live (2026-08-31). Saving over a built-in name is
  // rejected the same way (deleteDimensionConfigCb-side protection has an
  // exact save-side counterpart, saveDimensionConfigCb).
  // Shared by the "Save As New Config" button below and by the deploy-time
  // unsaved-edits prompt (see NepiIFSimLauncher's confirmUnsavedDimensionsOrPrompt,
  // wired via the onSaveUnsavedDimensionsAs prop) -- both mean the exact
  // same thing, "save the CURRENTLY EDITED fields under this name and make
  // it the active one" (see saveDimensionConfigCb's own comment for why
  // "save" always also means "use").
  saveDimensionsAsNamed(role, name) {
    const namespace = this.getSimNamespace()
    if (name === '') {
      window.alert('Please name the config before saving.')
      return
    }
    if (isProtectedDimensionConfig(role, name)) {
      window.alert('"' + name + '" is a built-in config and can\'t be saved over. Choose a different name.')
      return
    }
    if (namespace == null || namespace === 'None') {
      return
    }
    const yamlText = yaml.dump(this.state[role + '_dimensions_fields'])
    this.props.ros.sendStringMsg(namespace + '/save_' + role + '_dimensions_config',
      JSON.stringify({ name: name, yaml: yamlText }))
  }

  // An empty name pops an alert instead of quietly no-oping (or disabling
  // the button, which gave no feedback at all about WHY nothing happened)
  // -- requested live (2026-08-31).
  onSaveDimensionConfigAsClicked(role) {
    const name = this.state[role + '_dimensions_save_as_name'].trim()
    this.saveDimensionsAsNamed(role, name)
    this.setState({ [role + '_dimensions_save_as_name']: '' })
  }

  // Deletes whichever dimensions config is currently selected. A built-in
  // (PROTECTED_DIMENSION_CONFIG_NAMES) pops an alert rather than silently
  // no-oping or just staying disabled with no explanation -- requested live
  // (2026-08-31); deleteDimensionConfigCb enforces the same rule
  // independently on the device side regardless.
  onDeleteDimensionConfigClicked(role) {
    const namespace = this.getSimNamespace()
    const selected = this.state[role + '_dimensions_selected_config']
    if (namespace == null || namespace === 'None' || !selected) {
      return
    }
    if (isProtectedDimensionConfig(role, selected)) {
      window.alert('"' + selected + '" is a built-in config and can\'t be deleted.')
      return
    }
    this.props.ros.sendStringMsg(namespace + '/delete_' + role + '_dimensions_config', selected)
  }

  // Delete counterpart for the merged robot button row
  // (renderRobotConfigAndDimensionsButtons) -- targets whichever NAME was
  // last clicked there (robot_merged_selected_name), set synchronously on
  // click, rather than robot_dimensions_selected_config, which only updates
  // once the device echoes a select back and would otherwise race a click
  // that selects then immediately deletes. "Quadcopter" has no dimensions
  // counterpart to delete (see ROBOT_DIMENSIONS_MODEL's own comment in
  // sim_connector_app_node.py) -- popping the same built-in alert for it
  // keeps that consistent with 4-Wheel Rover's own protection instead of
  // silently deleting an unrelated, currently-selected dimensions config.
  onDeleteMergedRobotConfigClicked() {
    const namespace = this.getSimNamespace()
    const name = this.state.robot_merged_selected_name
    if (namespace == null || namespace === 'None' || !name) {
      return
    }
    if (isProtectedDimensionConfig('robot', name) || name === 'Quadcopter') {
      window.alert('"' + name + '" is a built-in config and can\'t be deleted.')
      return
    }
    if (this.state.robot_dimensions_config_names.indexOf(name) === -1) {
      window.alert('"' + name + '" has no saved dimensions entry to delete.')
      return
    }
    this.props.ros.sendStringMsg(namespace + '/delete_robot_dimensions_config', name)
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

  // Quick-access counterpart to renderRobotConfigSelector above -- same
  // Label-left/Select-right shape, right underneath it, for picking WHICH
  // saved environment dimensions config is active (Flat/Obstacle Course/
  // Aerial Obstacle Course/Custom Obstacles/anything saved). Deliberately
  // not gated on launcher_state: picking a different one still only takes
  // effect at the next Launch (Gazebo has no way to hot-reload a course
  // layout into a running world), but there is no reason to block the
  // SELECTION itself while a sim happens to be running.
  // Plain picker only, same as renderRobotConfigSelector above -- viewing
  // the YAML, deleting a config, and editing dimension fields all live down
  // in renderEnvironmentConfigSettings's own collapsed panel instead
  // (mirrors Robot Config Settings exactly). Reported live (2026-09-03):
  // an earlier version of this method rendered the YAML viewer and a
  // Delete button directly here, "just out there" instead of tucked into a
  // panel like Robot Config's own management controls -- moved.
  // selected === '' means the active values no longer match ANY saved
  // config (an in-progress, unsaved edit -- see setDimensionsCb's own
  // comment on the device) -- shown as its own explicit placeholder option
  // rather than letting the browser fall back to silently highlighting
  // whichever option happens to be first, which would misleadingly look
  // like that config is what's actually active.
  renderEnvironmentConfigSelector() {
    const names = this.state.environment_dimensions_config_names
    const selected = this.state.environment_dimensions_selected_config
    return (
      <Label title={"Environment Config"}>
        <Select
          onChange={(event) => this.onSelectDimensionConfig('environment', event.target.value)}
          value={selected}
        >
          {(selected === '') ? <Option key={''} value={''}>{'(Unsaved Edits)'}</Option> : null}
          {names.map((name) => <Option key={name} value={name}>{name}</Option>)}
        </Select>
      </Label>
    )
  }

  // Config-settings-panel counterpart of the plain selector above -- same
  // "Show/Hide Config Settings" collapsed-panel pattern renderRobotConfigSettings
  // already uses, so Environment gets its own equally-scoped panel instead
  // of sharing one with Robot's own management controls (which is what an
  // earlier version of this file did -- reported live (2026-09-03):
  // "the environment config and dimensions stuff should only be in its
  // panel similar to the robot config one -- not just out there").
  // Houses: the config picker's own YAML viewer + Delete button (moved out
  // of the always-visible selector above), the dimension fields
  // editor/diagram (or the custom obstacles editor, for that one model),
  // and a Reset button that returns to the built-in "Obstacle Course"
  // config -- requested live: "there should be reset buttons that moves it
  // back to the default rover/obstacle course values" (see
  // renderRobotConfigSettings's own matching Reset button for the robot
  // side, and FALLBACK_DIMENSION_CONFIG_NAME for where these two built-in
  // names come from).
  renderEnvironmentConfigSettings() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }
    const selected = this.state.environment_dimensions_selected_config
    const yamlText = this.state.environment_dimensions_config_yaml_text
    const environmentFieldDefs = ENVIRONMENT_DIMENSION_FIELDS_BY_MODEL[this.state.environment_dimensions_model] || []
    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <ButtonMenu>
          <Button onClick={() => this.setState({ show_environment_config_viewer: !this.state.show_environment_config_viewer })}>
            {(this.state.show_environment_config_viewer ? "Hide" : "Show") + " Environment Config Settings"}
          </Button>
        </ButtonMenu>
        {(this.state.show_environment_config_viewer === true) ?
          <Section title={"Environment Config Settings"}>
            {/* Dimensions editor first, config viewer (YAML + Delete) below
                it -- same reordering as Robot Config Settings above, and
                same reasoning. "Reset to Obstacle Course" removed from
                here entirely -- it now lives as a "Reset to Default"
                button inside the dimensions editor itself
                (renderDimensionsEditor/renderCustomObstaclesEditor), not
                as a config-viewer action. */}
            {(this.state.environment_dimensions_model === CUSTOM_OBSTACLES_MODEL) ?
              this.renderCustomObstaclesEditor()
            :
              this.renderDimensionsEditor('environment', 'Environment Dimensions', environmentFieldDefs,
                                            this.uploadEnvironmentSdfInputRef)
            }
            <ButtonMenu>
              <Button disabled={!selected} onClick={() => this.onDeleteDimensionConfigClicked('environment')}>
                {"Delete Selected Config"}
              </Button>
            </ButtonMenu>
            {(yamlText !== '') ?
              <textarea
                readOnly
                value={yamlText}
                rows={6}
                style={{ width: "60%", maxWidth: "40em", fontFamily: "monospace",
                        whiteSpace: "pre", overflow: "auto", display: "block",
                        backgroundColor: DIAGRAM_BG, color: Styles.vars.colors.grey0 }}
              />
            : null}
          </Section>
        : null}
      </React.Fragment>
    )
  }


  // Merged button row for BOTH axes of "robot config" -- the capability
  // profile (Quadcopter/4-Wheel Rover/any custom saved profile,
  // available_robot_configs) and the chassis/wheel dimensions config
  // (robot_dimensions_config_names). Requested live (2026-08-31): these were
  // two separate, visually near-identical button rows for what reads as the
  // same two robot kinds -- merged into one, keyed by DISPLAY NAME (the two
  // axes deliberately share display names for their built-ins, see
  // ROBOT_BUILTIN_DIMENSION_CONFIG_NAME's own comment in
  // sim_connector_app_node.py), so a name present in both appears once, and
  // clicking it drives whichever axis (or both) actually has an entry under
  // that name. "4-Wheel Rover" drives both (views its capability YAML AND
  // switches to its dimensions config, updating both viewers below);
  // "Quadcopter" only has a capability counterpart (its airframe is a
  // vendored third-party model with no dimensions.yaml of its own) so only
  // that fires. Delete stays scoped to the dimensions axis (see
  // onDeleteMergedRobotConfigClicked) -- a saved CAPABILITY config has had
  // no delete affordance in this RUI since its own per-config Delete button
  // was removed here (2026-08-31); nothing here reintroduces it.
  renderRobotConfigAndDimensionsButtons() {
    const status_msg = this.state.status_msg
    const capabilityKeys = (status_msg != null && status_msg.available_robot_configs !== undefined)
      ? status_msg.available_robot_configs : []
    const capabilityNames = (status_msg != null && status_msg.available_robot_config_names !== undefined)
      ? status_msg.available_robot_config_names : []
    var capabilityByName = {}
    capabilityKeys.forEach((key, i) => {
      const display = (capabilityNames[i] !== undefined && capabilityNames[i] !== '') ? capabilityNames[i] : key
      capabilityByName[display] = key
    })
    const dimensionNames = this.state.robot_dimensions_config_names
    const selectedCapabilityKey = this.getSelectedRobotConfig()
    const selectedDimensionsName = this.state.robot_dimensions_selected_config

    var names = []
    var seen = {}
    ;['Quadcopter', FALLBACK_DIMENSION_CONFIG_NAME.robot].forEach((n) => {
      if ((capabilityByName[n] !== undefined || dimensionNames.indexOf(n) !== -1) && !seen[n]) {
        names.push(n); seen[n] = true
      }
    })
    Object.keys(capabilityByName).concat(dimensionNames)
      .filter((n) => !seen[n])
      .sort((a, b) => a.localeCompare(b))
      .forEach((n) => { if (!seen[n]) { names.push(n); seen[n] = true } })

    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <Label title={"Robot Configs"} labelStyle={{ fontWeight: 'bold' }} />
        <ButtonMenu>
          {names.map((name) => {
            const isSelected = (capabilityByName[name] !== undefined && capabilityByName[name] === selectedCapabilityKey) ||
                                (name === selectedDimensionsName)
            return (
              <Button
                key={name}
                style={isSelected ? { backgroundColor: Styles.vars.colors.blue } : undefined}
                onClick={() => {
                  this.setState({ robot_merged_selected_name: name })
                  if (capabilityByName[name] !== undefined) {
                    this.onViewConfigClicked(capabilityByName[name])
                  }
                  if (dimensionNames.indexOf(name) !== -1) {
                    this.onSelectDimensionConfig('robot', name)
                  }
                }}
              >
                {name}
              </Button>
            )
          })}
          <Button onClick={this.onDeleteMergedRobotConfigClicked}>
            {"Delete Selected Config"}
          </Button>
        </ButtonMenu>
        {(this.state.robot_config_yaml !== '') ?
          <React.Fragment>
            <textarea
              readOnly
              value={this.state.robot_config_yaml}
              rows={8}
              style={{ width: "60%", maxWidth: "40em", fontFamily: "monospace",
                      whiteSpace: "pre", overflow: "auto", display: "block",
                      backgroundColor: DIAGRAM_BG, color: Styles.vars.colors.grey0 }}
            />
            <ButtonMenu>
              <Button onClick={this.onDownloadConfigClicked}>{"Download " + this.state.viewing_config_name + ".yaml"}</Button>
            </ButtonMenu>
          </React.Fragment>
        : null}
        {(this.state.robot_dimensions_config_yaml_text !== '') ?
          <textarea
            readOnly
            value={this.state.robot_dimensions_config_yaml_text}
            rows={6}
            style={{ width: "60%", maxWidth: "40em", fontFamily: "monospace",
                    whiteSpace: "pre", overflow: "auto", display: "block",
                    backgroundColor: DIAGRAM_BG, color: Styles.vars.colors.grey0 }}
          />
        : null}
      </React.Fragment>
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
  // Scoped to ROBOT only -- environment's own dimensions editor used to
  // live in here too, sharing this one panel; split out into its own
  // renderEnvironmentConfigSettings instead (reported live, 2026-09-03:
  // "the environment config and dimensions stuff should only be in its
  // panel similar to the robot config one"). Renamed "Config Settings" ->
  // "Robot Config Settings" at the same time, for that same symmetry.
  // Reset button added alongside Delete -- requested live: "there should be
  // reset buttons that moves it back to the default rover/obstacle course
  // values" (see FALLBACK_DIMENSION_CONFIG_NAME for where "4-Wheel Rover"
  // comes from, and renderEnvironmentConfigSettings for its counterpart).
  renderRobotConfigSettings() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }

    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <ButtonMenu>
          <Button onClick={() => this.setState({ show_robot_config_viewer: !this.state.show_robot_config_viewer })}>
            {(this.state.show_robot_config_viewer ? "Hide" : "Show") + " Robot Config Settings"}
          </Button>
        </ButtonMenu>
        {(this.state.show_robot_config_viewer === true) ?
          <Section title={"Robot Config Settings"}>
            {/* Dimensions editor first, config viewer/upload/download below
                it -- requested live (2026-09-04): "in the robot and
                environment config settings, the first thing in both should
                be the respective dimensions editor, and then the config
                viewers under that." Reset now lives in the dimensions
                editor itself (renderDimensionsEditor's own Reset button),
                not here -- "the reset config should be reset dimensions
                instead there in the robot and environment dimensions
                editing area." */}
            {this.renderDimensionsEditor('robot', 'Robot Dimensions', ROBOT_DIMENSION_FIELDS,
                                          this.uploadRobotSdfInputRef)}
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
            {this.renderRobotConfigAndDimensionsButtons()}
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
  // Small square drag handle drawn at (x, y) in SVG viewBox units -- shared
  // visual for every draggable/resizable control across the dimension
  // diagrams. title is a hover tooltip; onPointerDown starts the actual
  // drag gesture (see startDimensionDrag).
  renderDragHandle(x, y, cursor, title, onPointerDown) {
    return (
      <rect x={x - 4} y={y - 4} width={8} height={8}
            fill={Styles.vars.colors.green} stroke={DIAGRAM_BG} strokeWidth="1"
            style={{ cursor: cursor }} onPointerDown={onPointerDown}>
        <title>{title}</title>
      </rect>
    )
  }

  // Starts a pointer-drag gesture on a diagram handle -- shared by every
  // draggable/resizable handle across the dimension diagrams (robot wheel/
  // chassis corners here; environment obstacle handles reuse this same
  // method). axisX/axisY each describe one field this handle drives:
  // {field, sign, multiplier, min}. sign flips which screen direction
  // increases the value (a mirrored handle -- e.g. a wheel drawn at +x --
  // drags "further from center is positive"); multiplier converts a
  // HALF-extent screen delta into the FULL-extent field value (2 for
  // anything symmetric about the origin/center, like wheelbase_m or a
  // width; 1 for a size that already IS the half-extent shown, like
  // wheel_radius_m, which IS the radius). Either axis may be omitted for a
  // handle that only drives one field.
  //
  // Updates both the live-editing fields (so the number boxes track the
  // drag) and the preview fields (so the diagram redraws every frame) --
  // unlike a text edit, a drag gesture never produces an invalid
  // intermediate value, so there's no reason to wait for a commit before
  // the picture moves (see robot_dimensions_preview_fields' own comment for
  // why text edits DO wait). Commits (pushes to the device) on release, the
  // same as hitting Enter in a field -- see onSaveDimensionsClicked.
  //
  // viewBoxWidth/svgScale together convert a screen-pixel delta into world
  // meters, accounting for the SVG being drawn at a CSS width that may
  // differ from its viewBox (width="100%" is responsive) -- read from the
  // actual rendered element's bounding box at drag start via
  // getBoundingClientRect, never assumed to be 1:1 with the viewBox.
  startDimensionDrag(role, fieldDefs, axisX, axisY, viewBoxWidth, svgScale, event) {
    event.preventDefault()
    event.stopPropagation()
    const svgEl = event.currentTarget.ownerSVGElement
    const rect = (svgEl != null) ? svgEl.getBoundingClientRect() : null
    const pixelsPerViewBoxUnit = (rect != null && rect.width > 0) ? (rect.width / viewBoxWidth) : 1
    const worldPerPixel = 1 / (pixelsPerViewBoxUnit * svgScale)
    const startClientX = event.clientX
    const startClientY = event.clientY
    const fields = this.state[role + '_dimensions_fields']
    const startX = (axisX != null) ? numericDimensionField(fields, fieldDefs, axisX.field) : 0
    const startY = (axisY != null) ? numericDimensionField(fields, fieldDefs, axisY.field) : 0

    const onMove = (moveEvent) => {
      const dxWorld = (moveEvent.clientX - startClientX) * worldPerPixel
      const dyWorld = (moveEvent.clientY - startClientY) * worldPerPixel
      var updates = {}
      if (axisX != null) {
        const minX = (axisX.min !== undefined) ? axisX.min : 0.01
        updates[axisX.field] = Math.max(minX, startX + axisX.sign * dxWorld * axisX.multiplier)
      }
      if (axisY != null) {
        const minY = (axisY.min !== undefined) ? axisY.min : 0.01
        updates[axisY.field] = Math.max(minY, startY + axisY.sign * dyWorld * axisY.multiplier)
      }
      const fieldsKey = role + '_dimensions_fields'
      const previewKey = role + '_dimensions_preview_fields'
      this.setState((prevState) => ({
        [fieldsKey]: { ...prevState[fieldsKey], ...updates },
        [previewKey]: { ...prevState[previewKey], ...updates },
      }))
    }
    const onUp = () => {
      window.removeEventListener('pointermove', onMove)
      window.removeEventListener('pointerup', onUp)
      this.onSaveDimensionsClicked(role)
    }
    window.addEventListener('pointermove', onMove)
    window.addEventListener('pointerup', onUp)
  }

  //**********************
  // Custom Obstacles -- the one environment model with no fixed curated
  // field set (see CUSTOM_OBSTACLE_TYPES' own comment). Every action below
  // writes straight into environment_dimensions_fields.obstacles the same
  // way any other dimensions edit writes a flat field, then commits through
  // the existing onSaveDimensionsClicked('environment') -- no new backend
  // topic or message shape was needed for any of this.

  getCustomObstacles() {
    const obstacles = this.state.environment_dimensions_fields.obstacles
    return Array.isArray(obstacles) ? obstacles : []
  }

  // Replaces the whole obstacle list and immediately commits it (push to
  // the device, which pushes to the VM) -- used by add/delete/drag, all of
  // which are discrete, already-complete actions (a click, or a drag
  // release), unlike a text field mid-keystroke, so there's no reason to
  // wait for a separate save step the way renderDimensionFields' Enter-to-
  // save does.
  setCustomObstacles(obstacles) {
    this.setState((prevState) => ({
      environment_dimensions_fields: { ...prevState.environment_dimensions_fields, obstacles: obstacles },
    }), () => this.onSaveDimensionsClicked('environment'))
  }

  // New obstacles land staggered along +x (not stacked on top of each
  // other at a fixed spot) so a freshly-added one is immediately visible
  // and draggable rather than hidden under whatever was added just before it.
  onAddObstacleClicked(type) {
    const template = CUSTOM_OBSTACLE_TYPE_DEFAULTS[type]
    if (template == null) {
      return
    }
    const obstacles = this.getCustomObstacles()
    const next = { ...template, x: 2.0 + obstacles.length * 1.5 }
    this.setCustomObstacles(obstacles.concat([next]))
  }

  onDeleteObstacleClicked(index) {
    this.setCustomObstacles(this.getCustomObstacles().filter((_o, i) => i !== index))
  }

  // Live-editing only (no auto-commit per keystroke) -- Enter commits, same
  // convention renderDimensionFields' own numeric inputs already use.
  onObstacleFieldInputChange(index, field, value) {
    const obstacles = this.getCustomObstacles().map((o, i) => (i === index) ? { ...o, [field]: value } : o)
    this.setState((prevState) => ({
      environment_dimensions_fields: { ...prevState.environment_dimensions_fields, obstacles: obstacles },
    }))
  }

  // Array-aware counterpart to startDimensionDrag, for one CUSTOM
  // OBSTACLE's field(s) inside the obstacles list rather than a role's own
  // flat top-level fields. axisA/axisB each describe one field this handle
  // drives: {field, angleDeg, sign, multiplier, min}. angleDeg is the WORLD
  // direction (0 = +x, 90 = +y) this handle's drag is projected onto -- a
  // rotated shape's own length/thickness (or base/depth) axes pass
  // yaw_deg/yaw_deg+90 here instead of a fixed 0/90, so resizing a rotated
  // wall by its end handle still lengthens it along the wall, not along the
  // world axes; a plain move handle (repositioning the whole obstacle)
  // passes 0/90 for x/y. sign/multiplier/min work exactly like
  // startDimensionDrag's own. Commits on release, same as every other
  // dimensions edit.
  startObstacleDrag(index, axisA, axisB, viewBoxWidth, svgScale, event) {
    event.preventDefault()
    event.stopPropagation()
    const svgEl = event.currentTarget.ownerSVGElement
    const rect = (svgEl != null) ? svgEl.getBoundingClientRect() : null
    const pixelsPerViewBoxUnit = (rect != null && rect.width > 0) ? (rect.width / viewBoxWidth) : 1
    const worldPerPixel = 1 / (pixelsPerViewBoxUnit * svgScale)
    const startClientX = event.clientX
    const startClientY = event.clientY
    const obstacle = this.getCustomObstacles()[index] || {}
    const startA = (axisA != null) ? (Number(obstacle[axisA.field]) || 0) : 0
    const startB = (axisB != null) ? (Number(obstacle[axisB.field]) || 0) : 0

    const project = (worldDX, worldDY, angleDeg) => {
      const rad = (angleDeg * Math.PI) / 180
      return worldDX * Math.cos(rad) + worldDY * Math.sin(rad)
    }

    const onMove = (moveEvent) => {
      const svgDX = (moveEvent.clientX - startClientX) * worldPerPixel
      const svgDY = (moveEvent.clientY - startClientY) * worldPerPixel
      const worldDX = svgDX
      const worldDY = -svgDY // SVG y grows downward; world +y is drawn upward
      var updates = {}
      if (axisA != null) {
        const delta = project(worldDX, worldDY, axisA.angleDeg)
        const min = (axisA.min !== undefined) ? axisA.min : -1000
        updates[axisA.field] = Math.max(min, startA + axisA.sign * delta * axisA.multiplier)
      }
      if (axisB != null) {
        const delta = project(worldDX, worldDY, axisB.angleDeg)
        const min = (axisB.min !== undefined) ? axisB.min : -1000
        updates[axisB.field] = Math.max(min, startB + axisB.sign * delta * axisB.multiplier)
      }
      this.setState((prevState) => {
        const current = Array.isArray(prevState.environment_dimensions_fields.obstacles)
          ? prevState.environment_dimensions_fields.obstacles : []
        const next = current.map((o, i) => (i === index) ? { ...o, ...updates } : o)
        return {
          environment_dimensions_fields: { ...prevState.environment_dimensions_fields, obstacles: next },
          environment_dimensions_preview_fields: { ...prevState.environment_dimensions_preview_fields, obstacles: next },
        }
      })
    }
    const onUp = () => {
      window.removeEventListener('pointermove', onMove)
      window.removeEventListener('pointerup', onUp)
      this.onSaveDimensionsClicked('environment')
    }
    window.addEventListener('pointermove', onMove)
    window.addEventListener('pointerup', onUp)
  }

  // One row of precise numeric inputs per obstacle, alongside the diagram's
  // own drag handles -- typing is more precise than dragging for an exact
  // value, dragging is faster for rough placement; both write to the same
  // fields. Enter commits, same convention as every other dimensions field.
  renderObstacleFieldRow(o, index) {
    const fieldNames = CUSTOM_OBSTACLE_TYPE_FIELD_NAMES[o.type] || []
    return (
      <div key={index} style={{ display: 'flex', flexWrap: 'wrap', alignItems: 'flex-end',
                                 borderTop: "1px solid " + Styles.vars.colors.grey2,
                                 paddingTop: Styles.vars.spacing.xs, marginTop: Styles.vars.spacing.xs }}>
        <div style={{ width: "90px", fontWeight: 'bold', textTransform: 'capitalize' }}>{o.type}</div>
        {fieldNames.map((fieldName) => (
          <div key={fieldName} style={{ width: "110px", marginRight: Styles.vars.spacing.xs }}>
            <Label title={fieldName}>
              <Input
                id={"Obstacle_" + index + "_" + fieldName}
                value={o[fieldName]}
                onChange={(event) => {
                  const value = event.target.value
                  this.onObstacleFieldInputChange(index, fieldName, value)
                }}
                onKeyDown={(event) => { if (event.key === 'Enter') { this.onSaveDimensionsClicked('environment') } }}
              />
            </Label>
          </div>
        ))}
        <Button onClick={() => this.onDeleteObstacleClicked(index)}>{"Delete"}</Button>
      </div>
    )
  }

  // Draws one obstacle's shape (plus its move/resize handles and a small
  // delete marker) at its current position/size/rotation -- box for a
  // wall, circle for a circle, polygon for a triangle, matching
  // generate_model_sdf.py's own _obstacle*Link geometry exactly (a wall's
  // pose is centered on it with a yaw rotation; a triangle's vertices are
  // (0, depth/2), (0, -depth/2), (base, 0) in its own local frame, per
  // _obstacleTriangleLink). Dragging the shape itself repositions it (x/y);
  // the small green corner handle resizes it -- see startObstacleDrag's own
  // comment for how a rotated shape's resize axes are derived from yaw_deg.
  renderCustomObstacleShape(o, index, toX, toY, scale, viewW) {
    const type = o.type
    const x = Number(o.x) || 0
    const y = Number(o.y) || 0
    const yawDeg = Number(o.yaw_deg) || 0
    const screenX = toX(x)
    const screenY = toY(y)
    const moveAxisX = { field: 'x', angleDeg: 0, sign: 1, multiplier: 1, min: -1000 }
    const moveAxisY = { field: 'y', angleDeg: 90, sign: 1, multiplier: 1, min: -1000 }

    var shape = null
    var resizeHandle = null
    if (type === 'wall') {
      const length = Math.max(Number(o.length_m) || 1, 0.01)
      const thickness = Math.max(Number(o.thickness_m) || 0.2, 0.01)
      shape = (
        <rect x={screenX - (length * scale) / 2} y={screenY - (thickness * scale) / 2}
              width={length * scale} height={thickness * scale}
              fill={Styles.vars.colors.orange}
              transform={"rotate(" + (-yawDeg) + " " + screenX + " " + screenY + ")"}
              style={{ cursor: 'move' }}
              onPointerDown={(e) => this.startObstacleDrag(index, moveAxisX, moveAxisY, viewW, scale, e)}>
          <title>{"Drag to move this wall"}</title>
        </rect>
      )
      const corner = localToWorldPoint(x, y, yawDeg, length / 2, thickness / 2)
      resizeHandle = this.renderDragHandle(toX(corner.x), toY(corner.y), 'nesw-resize',
        'Drag to resize wall length/thickness',
        (e) => this.startObstacleDrag(index,
          { field: 'length_m', angleDeg: yawDeg, sign: 1, multiplier: 2, min: 0.05 },
          { field: 'thickness_m', angleDeg: yawDeg + 90, sign: 1, multiplier: 2, min: 0.02 },
          viewW, scale, e))
    } else if (type === 'circle') {
      const radius = Math.max(Number(o.radius_m) || 0.5, 0.01)
      shape = (
        <circle cx={screenX} cy={screenY} r={radius * scale} fill={Styles.vars.colors.red}
                style={{ cursor: 'move' }}
                onPointerDown={(e) => this.startObstacleDrag(index, moveAxisX, moveAxisY, viewW, scale, e)}>
          <title>{"Drag to move this circle"}</title>
        </circle>
      )
      resizeHandle = this.renderDragHandle(screenX + radius * scale, screenY, 'ew-resize',
        'Drag to resize circle radius',
        (e) => this.startObstacleDrag(index,
          { field: 'radius_m', angleDeg: 0, sign: 1, multiplier: 1, min: 0.05 }, null, viewW, scale, e))
    } else if (type === 'triangle') {
      const base = Math.max(Number(o.base_m) || 1, 0.01)
      const depth = Math.max(Number(o.depth_m) || 1, 0.01)
      const p1 = localToWorldPoint(x, y, yawDeg, 0, depth / 2)
      const p2 = localToWorldPoint(x, y, yawDeg, 0, -depth / 2)
      const p3 = localToWorldPoint(x, y, yawDeg, base, 0)
      shape = (
        <polygon points={
          toX(p1.x) + "," + toY(p1.y) + " " + toX(p2.x) + "," + toY(p2.y) + " " + toX(p3.x) + "," + toY(p3.y)
        } fill={Styles.vars.colors.blue} style={{ cursor: 'move' }}
          onPointerDown={(e) => this.startObstacleDrag(index, moveAxisX, moveAxisY, viewW, scale, e)}>
          <title>{"Drag to move this triangle"}</title>
        </polygon>
      )
      resizeHandle = this.renderDragHandle(toX(p3.x), toY(p3.y), 'nesw-resize',
        'Drag to resize triangle base/depth',
        (e) => this.startObstacleDrag(index,
          { field: 'base_m', angleDeg: yawDeg, sign: 1, multiplier: 1, min: 0.05 },
          { field: 'depth_m', angleDeg: yawDeg + 90, sign: 1, multiplier: 2, min: 0.05 },
          viewW, scale, e))
    } else {
      return null
    }

    return (
      <g key={index}>
        {shape}
        {resizeHandle}
        <circle cx={screenX} cy={screenY - 12} r={6} fill={Styles.vars.colors.red}
                style={{ cursor: 'pointer' }} onClick={() => this.onDeleteObstacleClicked(index)}>
          <title>{"Delete this obstacle"}</title>
        </circle>
        <text x={screenX} y={screenY - 8.5} textAnchor="middle" fontSize="9" fill={Styles.vars.colors.white}
              style={{ pointerEvents: 'none' }}>{"x"}</text>
      </g>
    )
  }

  // Top-down schematic (same x-right/y-up convention as every other
  // diagram here) auto-framed to whatever obstacles currently exist --
  // unlike the fixed-template diagrams, there's no fixed course geometry to
  // size the view to, so the bounding box is derived from the obstacles
  // list itself, with a sane minimum so an empty or near-empty list still
  // renders a usable, not-degenerate canvas to add the first obstacle into.
  renderCustomObstaclesDiagram(fields) {
    const obstacles = Array.isArray(fields.obstacles) ? fields.obstacles : []
    var minX = 0, maxX = 6, minY = -3, maxY = 3
    obstacles.forEach((o) => {
      const x = Number(o.x) || 0
      const y = Number(o.y) || 0
      const extent = Math.max(Number(o.length_m) || 0, Number(o.base_m) || 0,
        (Number(o.radius_m) || 0) * 2, 1) / 2 + 1
      minX = Math.min(minX, x - extent)
      maxX = Math.max(maxX, x + extent)
      minY = Math.min(minY, y - extent)
      maxY = Math.max(maxY, y + extent)
    })
    const boundW = Math.max(maxX - minX, 1)
    const boundH = Math.max(maxY - minY, 1)
    const viewW = 560
    const viewH = 220
    const padX = 20
    const padY = 20
    const scale = Math.min((viewW - 2 * padX) / boundW, (viewH - 2 * padY) / boundH)
    const toX = (worldX) => padX + (worldX - minX) * scale
    const toY = (worldY) => (viewH - padY) - (worldY - minY) * scale

    return (
      <React.Fragment>
        <svg viewBox={`0 0 ${viewW} ${viewH}`} width="100%" height={viewH}
             style={{ background: DIAGRAM_BG, borderRadius: 4 }}>
          {(obstacles.length === 0) ?
            <text x={viewW / 2} y={viewH / 2} textAnchor="middle" fontSize="12" fill={Styles.vars.colors.grey1}>
              {"No obstacles yet -- use the buttons below to add one"}
            </text>
          : null}
          {obstacles.map((o, i) => this.renderCustomObstacleShape(o, i, toX, toY, scale, viewW))}
        </svg>
        <div style={{ fontSize: 11, color: Styles.vars.colors.grey1, marginTop: Styles.vars.spacing.xs }}>
          {obstacles.length + " obstacle" + (obstacles.length === 1 ? "" : "s") +
           " -- drag a shape to move it, its corner handle to resize, or the red dot to delete it"}
        </div>
      </React.Fragment>
    )
  }

  // Environment editor for the 'Custom Obstacles' built-in (and any config
  // saved from it) -- a dynamic, operator-built LIST of obstacles instead
  // of one fixed template's curated numeric fields. See
  // sim_container/scripts/generate_model_sdf.py's buildCustomObstaclesSdf
  // for how this list becomes model.sdf.
  renderCustomObstaclesEditor() {
    const dirty = this.state.environment_dimensions_dirty
    const previewFields = this.state.environment_dimensions_preview_fields
    const obstacles = this.getCustomObstacles()
    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <Section title={"Environment Dimensions -- Custom Obstacles"}>
          {this.renderCustomObstaclesDiagram(previewFields)}
          {(dirty === true) ?
            <div style={{ fontStyle: "italic", color: Styles.vars.colors.grey1,
                          marginTop: Styles.vars.spacing.small, marginBottom: Styles.vars.spacing.small }}>
              {"Edited -- applies on the next Launch"}
            </div>
          : null}
          <ButtonMenu>
            {CUSTOM_OBSTACLE_TYPES.map((type) => (
              <Button key={type} onClick={() => this.onAddObstacleClicked(type)}>
                {"Add " + type.charAt(0).toUpperCase() + type.slice(1)}
              </Button>
            ))}
            {/* Same "Reset to Default" this role's other dimensions editor
                (renderDimensionsEditor) has -- this custom-obstacles editor
                is environment's OWN dimensions editor for that one model,
                just a dynamic obstacle list instead of fixed numeric
                fields, so it gets the same button rather than leaving this
                one editor without a reset. */}
            <Button onClick={() => this.onSelectDimensionConfig('environment', FALLBACK_DIMENSION_CONFIG_NAME.environment)}>
              {"Reset to " + FALLBACK_DIMENSION_CONFIG_NAME.environment}
            </Button>
          </ButtonMenu>
          {obstacles.map((o, i) => this.renderObstacleFieldRow(o, i))}
          <Label title={"Name New Config"}>
            <Input
              id={"DimensionsSaveAsName_environment"}
              value={this.state.environment_dimensions_save_as_name}
              onChange={(event) => this.setState({ environment_dimensions_save_as_name: event.target.value })}
              onKeyDown={(event) => { if (event.key === 'Enter') { this.onSaveDimensionConfigAsClicked('environment') } }}
            />
          </Label>
          <ButtonMenu>
            <Button onClick={() => this.onSaveDimensionConfigAsClicked('environment')}>
              {"Save As New Config"}
            </Button>
          </ButtonMenu>
        </Section>
      </React.Fragment>
    )
  }

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
      // Companion unit-converted Input (see weight_kg's own altUnit
      // comment) -- edits the SAME fields[f.name] (always stored in the
      // field's own base unit), just converted on the way in/out, so the
      // two inputs can never disagree.
      const renderOnePair = (f) => (f.altUnit == null) ? renderOne(f) : (
        <React.Fragment key={f.name}>
          {renderOne(f)}
          <Label title={f.altUnit.title}>
            <Input
              id={"SimDim_" + role + "_" + f.name + "_alt"}
              value={(fields[f.name] !== '' && !isNaN(parseFloat(fields[f.name])))
                ? round(f.altUnit.toAlt(parseFloat(fields[f.name])), 3) : fields[f.name]}
              onChange={(event) => {
                const el = document.getElementById("SimDim_" + role + "_" + f.name + "_alt")
                if (el) {
                  setElementStyleModified(el)
                }
                const altValue = parseFloat(event.target.value)
                if (isNaN(altValue)) {
                  return
                }
                const fieldsKey = role + '_dimensions_fields'
                this.setState((prevState) => ({
                  [fieldsKey]: { ...prevState[fieldsKey], [f.name]: String(f.altUnit.fromAlt(altValue)) }
                }))
              }}
              onKeyDown={(event) => {
                if (event.key !== 'Enter') {
                  return
                }
                const el = document.getElementById("SimDim_" + role + "_" + f.name + "_alt")
                if (el) {
                  clearElementStyleModified(el)
                }
                this.onSaveDimensionsClicked(role)
              }}
            />
          </Label>
        </React.Fragment>
      )
      rows.push(
        <React.Fragment key={a.name}>
          {(b != null) ? this.renderFieldPair(renderOnePair(a), renderOnePair(b)) : renderOnePair(a)}
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

    // FL wheel's own screen position -- the one wheel with live drag
    // handles (see the handles below); the other three are drawn from the
    // same wheelbase_m/track_width_m fields via wheelPositions' mirroring,
    // so dragging FL moves all four together automatically.
    const flX = cx + (wheelbase / 2) * scale
    const flY = cy - (trackWidth / 2) * scale

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
          {/* Drag the chassis corner to resize chassis_length_m/
              chassis_width_m together; drag the FL wheel itself to
              reposition (drives wheelbase_m/track_width_m, mirrored to all
              four wheels); drag the small handle on the FL wheel's outer
              corner to resize wheel_radius_m/wheel_width_m. */}
          {this.renderDragHandle(
            cx + (chassisLength * scale) / 2, cy + (chassisWidth * scale) / 2,
            'nwse-resize', 'Drag to resize chassis length/width',
            (e) => this.startDimensionDrag('robot', ROBOT_DIMENSION_FIELDS,
              { field: 'chassis_length_m', sign: 1, multiplier: 2 },
              { field: 'chassis_width_m', sign: 1, multiplier: 2 },
              viewW, scale, e)
          )}
          <rect x={flX - wheelBoxW / 2} y={flY - wheelBoxH / 2} width={wheelBoxW} height={wheelBoxH}
                fill="transparent" style={{ cursor: 'move' }}
                onPointerDown={(e) => this.startDimensionDrag('robot', ROBOT_DIMENSION_FIELDS,
                  { field: 'wheelbase_m', sign: 1, multiplier: 2 },
                  { field: 'track_width_m', sign: -1, multiplier: 2 },
                  viewW, scale, e)}>
            <title>{"Drag to move this wheel (wheelbase/track width)"}</title>
          </rect>
          {this.renderDragHandle(
            flX + wheelBoxW / 2, flY + wheelBoxH / 2,
            'nesw-resize', 'Drag to resize wheel radius/width',
            (e) => this.startDimensionDrag('robot', ROBOT_DIMENSION_FIELDS,
              { field: 'wheel_radius_m', sign: 1, multiplier: 1 },
              { field: 'wheel_width_m', sign: 1, multiplier: 2 },
              viewW, scale, e)
          )}
        </svg>
        <div style={{ fontSize: 11, color: Styles.vars.colors.grey1, marginTop: Styles.vars.spacing.xs }}>
          {"Wheelbase " + wheelbase.toFixed(2) + "m · Track " + trackWidth.toFixed(2) +
           "m · Chassis " + chassisLength.toFixed(2) + "×" + chassisWidth.toFixed(2) +
           "×" + chassisHeight.toFixed(2) + "m (L×W×H) · drag the shapes above to edit"}
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
    const get = (name) => Math.max(0, numericDimensionField(fields, OBSTACLE_COURSE_DIMENSION_FIELDS, name))
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

  // Side-view (x-z plane, looking along the corridor's y axis) schematic of
  // the aerial obstacle course from environment_dimensions_preview_fields --
  // gate spacing and the height each one climbs to are the two things that
  // actually vary meaningfully in this projection; opening width isn't
  // representable side-on (a gate viewed edge-on is just its frame
  // thickness), so it's called out in the caption text instead, the same
  // convention renderRobotDimensionsDiagram/renderEnvironmentDimensionsDiagram
  // already use for their own non-representable dimensions. Matches
  // generate_model_sdf.py's buildAerialObstacleCourseSdf exactly: gate i is
  // centered at x = course_start_x_m + i*gate_spacing_m,
  // z = gate_base_height_m + i*gate_height_step_m.
  renderAerialObstacleCourseDiagram(fields) {
    const get = (name) => Math.max(0, numericDimensionField(fields, AERIAL_OBSTACLE_COURSE_DIMENSION_FIELDS, name))
    const courseStartX = get('course_start_x_m')
    const gateCount = Math.max(1, Math.round(get('gate_count')))
    const gateSpacing = get('gate_spacing_m')
    const openingWidth = get('gate_opening_width_m')
    const openingHeight = get('gate_opening_height_m')
    const frameThickness = Math.max(get('gate_frame_thickness_m'), 0.02)
    const baseHeight = get('gate_base_height_m')
    const heightStep = get('gate_height_step_m')

    const lastX = courseStartX + (gateCount - 1) * gateSpacing
    const topGateCenterZ = baseHeight + (gateCount - 1) * heightStep
    const maxZ = topGateCenterZ + openingHeight / 2 + frameThickness

    const boundW = Math.max(lastX + gateSpacing * 0.6, 1)
    const boundH = Math.max(maxZ, 1)
    const viewW = 560
    const viewH = 190
    const padX = 24
    const padBottom = 20
    const padTop = 16
    const scale = Math.min((viewW - 2 * padX) / boundW, (viewH - padTop - padBottom) / boundH)
    const groundY = viewH - padBottom
    const toX = (worldX) => padX + worldX * scale
    const toZ = (worldZ) => groundY - worldZ * scale

    var gates = []
    for (var i = 0; i < gateCount; i++) {
      const cx = courseStartX + i * gateSpacing
      const cz = baseHeight + i * heightStep
      const outerHalf = openingHeight / 2 + frameThickness
      const x0 = toX(cx) - (frameThickness * scale) / 2
      const y0 = toZ(cz + outerHalf)
      const w = Math.max(frameThickness * scale, 3)
      const h = outerHalf * 2 * scale
      gates.push(<rect key={"gate_" + i} x={x0} y={y0} width={w} height={h}
                        fill="none" stroke={Styles.vars.colors.orange} strokeWidth="2" />)
      gates.push(<line key={"drop_" + i} x1={toX(cx)} y1={y0 + h} x2={toX(cx)} y2={groundY}
                        stroke={Styles.vars.colors.grey1} strokeDasharray="2,2" />)
    }

    return (
      <React.Fragment>
        <svg viewBox={`0 0 ${viewW} ${viewH}`} width="100%" height={viewH}
             style={{ background: DIAGRAM_BG, borderRadius: 4 }}>
          <line x1={toX(0)} y1={groundY} x2={toX(boundW)} y2={groundY}
                stroke={Styles.vars.colors.grey1} strokeWidth="1" />
          {gates}
        </svg>
        <div style={{ fontSize: 11, color: Styles.vars.colors.grey1, marginTop: Styles.vars.spacing.xs }}>
          {gateCount + " gate" + (gateCount === 1 ? "" : "s") + ", opening " + openingWidth.toFixed(2) +
           "m wide x " + openingHeight.toFixed(2) + "m tall, spaced " + gateSpacing.toFixed(2) +
           "m apart, climbing from " + baseHeight.toFixed(2) + "m to " + topGateCenterZ.toFixed(2) +
           "m (side view, x-z plane)"}
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
      if (role === 'robot') {
        return this.renderRobotDimensionsDiagram(previewFields)
      }
      const model = this.state.environment_dimensions_model
      if (model === 'aerial_obstacle_course') {
        return this.renderAerialObstacleCourseDiagram(previewFields)
      }
      if (model === ENVIRONMENT_MODEL_NONE) {
        return (
          <div style={{ fontSize: 11, color: Styles.vars.colors.grey1, marginTop: Styles.vars.spacing.xs }}>
            {"Flat ground -- no environment geometry to preview."}
          </div>
        )
      }
      return this.renderEnvironmentDimensionsDiagram(previewFields)
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
            <Button onClick={() => this.onDownloadDimensionsClicked(role)}>{"Download Dimensions (YAML)"}</Button>
            <Button onClick={() => this.onUploadModelSdfClicked(role)}>{"Upload Raw model.sdf"}</Button>
            {/* Requested live (2026-09-04): "the reset config should be
                reset dimensions instead there in the robot and environment
                dimensions editing area" -- moved here from the config-
                viewer panel above (renderRobotConfigSettings/
                renderEnvironmentConfigSettings), which no longer has a
                Reset button of its own. */}
            <Button onClick={() => this.onSelectDimensionConfig(role, FALLBACK_DIMENSION_CONFIG_NAME[role])}>
              {"Reset to " + FALLBACK_DIMENSION_CONFIG_NAME[role]}
            </Button>
          </ButtonMenu>
          {/* Names and persists the CURRENTLY EDITED fields (not just
              whichever config was last loaded), and makes the new name the
              active one -- same "save also means use" behavior as Save
              Dimensions above, just under a new name instead of overwriting
              the active config in place. Deleting a saved config lives at
              the top of the page instead (renderRobotConfigAndDimensionsButtons/
              renderEnvironmentConfigSelector), next to the controls that
              pick one -- not duplicated here. */}
          <Label title={"Name New Config"}>
            <Input
              id={"DimensionsSaveAsName_" + role}
              value={this.state[role + '_dimensions_save_as_name']}
              onChange={(event) => this.setState({ [role + '_dimensions_save_as_name']: event.target.value })}
              onKeyDown={(event) => { if (event.key === 'Enter') { this.onSaveDimensionConfigAsClicked(role) } }}
            />
          </Label>
          <ButtonMenu>
            <Button onClick={() => this.onSaveDimensionConfigAsClicked(role)}>
              {"Save As New Config"}
            </Button>
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
        {/* Edit box for horizontal FOV, right under its own read-only
            reading above -- reported live (2026-09-03): "camera horizontal
            and vertical fov dont seem to be editable yet." Horizontal FOV
            already WAS editable, just only reachable via Robot Config
            Settings -> Robot Dimensions (camera_horizontal_fov_deg is one
            of ROBOT_DIMENSION_FIELDS) -- this reuses that exact same field/
            save mechanism (onSaveDimensionsClicked('robot')) rather than
            inventing a second one, just placed where the reading itself is
            shown, so there's no need to go hunting for it. Vertical FOV has
            no edit box here on purpose: it's derived from horizontal FOV
            and the camera's aspect ratio (see generate_model_sdf.py's
            buildRoverSdf, which only ever takes camera_horizontal_fov_deg
            as an input), not an independent physical parameter -- adding a
            box that silently did nothing would be worse than not having
            one. */}
        {this.renderFieldPair(
          <Label title={"Set Horizontal FOV (deg)"}>
            <Input
              id={"SimDim_robot_camera_horizontal_fov_deg_quick"}
              value={this.state.robot_dimensions_fields.camera_horizontal_fov_deg}
              onChange={(event) => {
                const el = document.getElementById("SimDim_robot_camera_horizontal_fov_deg_quick")
                if (el) {
                  setElementStyleModified(el)
                }
                const value = event.target.value
                this.setState((prevState) => ({
                  robot_dimensions_fields: { ...prevState.robot_dimensions_fields, camera_horizontal_fov_deg: value }
                }))
              }}
              onKeyDown={(event) => {
                if (event.key !== 'Enter') {
                  return
                }
                const el = document.getElementById("SimDim_robot_camera_horizontal_fov_deg_quick")
                if (el) {
                  clearElementStyleModified(el)
                }
                this.onSaveDimensionsClicked('robot')
              }}
            />
          </Label>,
          <Label title={"Vertical FOV"}>
            <Input disabled value={"derived from horizontal + aspect ratio"} />
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

    // Split into two half-width Columns, matching every other NEPI panel's
    // own convention (System -> Device Manager's NepiSystemDevice.js splits
    // its own content into two <Column> siblings the exact same way --
    // Columns/Column's shared flex:1 style only actually halves the width
    // when there are two sibling Columns to split; a single Column just
    // fills 100%, which is what every section here rendered at before this
    // change). Requested live (2026-09-04): "most of the other nepi
    // windows are like this too, while sim connector takes the whole
    // horizontal width per section."
    //
    // Left: pick-and-deploy plus live data -- what to run and whether it's
    // actually running. Right: everything that shapes WHAT gets deployed
    // (Robot/Environment Config Settings' dimensions editors) and the
    // live/config controls for whatever robot is currently connected
    // (NepiIFSimControls). Not a strict alternative split (e.g.
    // alphabetical) -- grouped by "operate the sim" vs. "configure the
    // sim/robot/environment", the same kind of task-based grouping Device
    // Manager's own split uses (device/license/admin vs. network/time).
    const leftColumn = (
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
            {this.renderEnvironmentConfigSelector()}

            {/* Deploy/Kill/Install controls -- right after picking WHAT to
                run (simulator), WHICH robot config, and WHICH environment
                config. */}
            <NepiIFSimLauncher
              namespace={namespace}
              make_section={false}
              only={"deploy"}
              selected_target={this.state.selected_launch_target}
              onTargetSelected={this.onLaunchTargetSelected}
              selected_robot_config={this.getSelectedRobotConfig()}
              unsaved_robot_dimensions={this.state.robot_dimensions_selected_config === ''}
              unsaved_environment_dimensions={this.state.environment_dimensions_selected_config === ''}
              onSaveUnsavedDimensionsAs={this.saveDimensionsAsNamed}
            />
          </React.Fragment>
        : null}

        {(show_data === true) ?
          this.renderData()
        : null}
      </React.Fragment>
    )

    const rightColumn = (
      <React.Fragment>
        {(show_selectors === true) ?
          <React.Fragment>
            {this.renderRobotConfigSettings()}
            {this.renderEnvironmentConfigSettings()}
          </React.Fragment>
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
          ref={this.simControlsRef}
          namespace={namespace}
          make_section={false}
          show_live_controls={show_controls}
        />
      </React.Fragment>
    )

    const content = (
      <Columns>
        <Column>{leftColumn}</Column>
        <Column>{rightColumn}</Column>
      </Columns>
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
