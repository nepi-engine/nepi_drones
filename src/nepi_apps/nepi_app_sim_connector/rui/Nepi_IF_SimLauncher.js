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
import { Columns, Column } from "./Columns"

// "New Sim" (a second, concurrent simulator instance) is hidden for now: the
// NEPI Gazebo service runs a single Gazebo, so the backend can only refuse
// it. Everything behind it -- onNewSimClicked, renderSecondaryInstances, the
// backend's sim/launch_new_simulator -- is kept; set this true to bring the
// button (and its "Other Running Instances" list) back once multi-instance
// support exists.
const SHOW_NEW_SIM = false

@inject("ros")
@observer

// Additive simulator auto-launch control -- see
// docs/SIMULATOR_AUTO_LAUNCH_PLAN.md (nepi_drones). Deliberately a separate
// component/file from Nepi_IF_Sim, not an edit to it: this is a convenience
// trigger over the same passive sim/select_simulator + sim/select_robot_config
// flow Nepi_IF_Sim already owns, not a replacement for it. Takes the same
// sim device namespace prop (<app>/sim) Nepi_IF_Sim takes, and talks to the
// sibling sim/launcher_status, sim/redeploy_simulator, sim/stop_simulator
// topics sim_connector_app_node.py publishes/subscribes alongside SimStatus.
//
// available_launch_targets is empty on any deployment that hasn't set
// NEPI_SIM_LAUNCH_TARGETS_CONFIG (a real device, normally) -- rendered as an
// explicit "not configured" note rather than hidden entirely, so the control
// stays discoverable instead of silently disappearing.
class NepiIFSimLauncher extends Component {
  constructor(props) {
    super(props)

    this.state = {

      // Sim device namespace (<app>/sim), from the namespace prop -- the
      // same namespace Nepi_IF_Sim receives, since these topics are
      // siblings of sim/status under it.
      namespace: null,

      // SimLauncherStatus from that namespace.
      status_msg: null,
      statusListener: null,

      // sim/secondary_launcher_status -- JSON-in-String, see that
      // publisher's own comment in sim_connector_app_node.py for why this
      // is a separate topic rather than a new SimLauncherStatus field.
      // {"active_targets": [...], "states": {actual_target: state}}.
      // Tracks "New Sim" instances -- concurrent deployments alongside
      // whatever the primary slot (status_msg above) is running.
      secondary_states: {},
      secondaryStatusListener: null,

      // Local dropdown selection, applied on Deploy click -- not published
      // on every change, only Deploy/Kill actually send anything.
      // Only used when this instance isn't given a selected_target prop --
      // see getSelectedTarget/setSelectedTarget.
      selected_target: 'None',
    }

    this.getSimNamespace = this.getSimNamespace.bind(this)

    this.getSelectedTarget = this.getSelectedTarget.bind(this)
    this.setSelectedTarget = this.setSelectedTarget.bind(this)

    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

    this.onTargetSelected = this.onTargetSelected.bind(this)
    this.onDeployClicked = this.onDeployClicked.bind(this)
    this.onNewSimClicked = this.onNewSimClicked.bind(this)
    this.onKillClicked = this.onKillClicked.bind(this)

    this.renderTargetSelector = this.renderTargetSelector.bind(this)
    this.renderDeployControls = this.renderDeployControls.bind(this)
  }

  getSimNamespace() {
    return (this.props.namespace !== undefined) ? this.props.namespace : null
  }

  // Two instances of this component are commonly mounted at once (see the
  // `only` prop -- selector up top, deploy controls at the bottom), and the
  // dropdown selection has to be the same value in both. When a parent
  // passes selected_target/onTargetSelected props (Nepi_IF_Sim does), this
  // becomes a controlled value owned by the parent; otherwise it falls back
  // to this instance's own local state, for standalone use.
  getSelectedTarget() {
    return (this.props.selected_target !== undefined)
      ? this.props.selected_target : this.state.selected_target
  }

  setSelectedTarget(value) {
    if (this.props.onTargetSelected !== undefined) {
      this.props.onTargetSelected(value)
    } else {
      this.setState({ selected_target: value })
    }
  }

  componentDidMount() {
    this.updateStatusListener()
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getSimNamespace()
    if (namespace !== this.state.namespace) {
      this.updateStatusListener()
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    if (this.state.secondaryStatusListener) {
      this.state.secondaryStatusListener.unsubscribe()
    }
    this.setState({ statusListener: null, secondaryStatusListener: null })
  }

  // Subscribes to <namespace>/launcher_status, message type
  // SimLauncherStatus -- a sibling of Nepi_IF_Sim's own <namespace>/status
  // subscription under the same sim device namespace.
  updateStatusListener() {
    const namespace = this.getSimNamespace()
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null, status_msg: null })
    }
    if (namespace != null && namespace !== 'None') {
      var statusListener = this.props.ros.setupStatusListener(
        namespace + '/launcher_status',
        "nepi_app_sim_connector/SimLauncherStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    if (this.state.secondaryStatusListener != null) {
      this.state.secondaryStatusListener.unsubscribe()
      this.setState({ secondaryStatusListener: null, secondary_states: {} })
    }
    if (namespace != null && namespace !== 'None') {
      var secondaryStatusListener = this.props.ros.setupStatusListener(
        namespace + '/secondary_launcher_status', "std_msgs/String",
        (message) => {
          try {
            const parsed = JSON.parse(message.data)
            this.setState({ secondary_states: parsed.states || {} })
          } catch (e) {
            // Ignore -- latched topic can briefly hold an empty/default
            // String before the first real publish.
          }
        }
      )
      this.setState({ secondaryStatusListener: secondaryStatusListener })
    }
    this.setState({ namespace: namespace })
  }

  statusListener(message) {
    this.setState({ status_msg: message })
    // Take the device's reported selection only while the operator hasn't
    // picked anything yet (e.g. on page load). After that the dropdown is
    // the operator's: they can pick MuJoCo while Gazebo is running and
    // Deploy it, and an echo of the running simulator must not undo that.
    const current = this.getSelectedTarget()
    if ((current === 'None' || current === '') &&
        message.selected_launch_target !== undefined && message.selected_launch_target !== '') {
      this.setSelectedTarget(message.selected_launch_target)
    }
  }

  onTargetSelected(event) {
    this.setSelectedTarget(event.target.value)
  }

  // Explicitly re-sends the currently-displayed robot config immediately
  // before every launch-triggering action below, rather than trusting that
  // an earlier select_robot_config message (fired whenever the operator
  // used that dropdown) already landed and was processed. Found live
  // (2026-08-18): selecting "Quadcopter" then immediately clicking Deploy
  // on a freshly-loaded page could race the backend's own processing of
  // that earlier message, so Deploy read a stale selected_robot_config and
  // launched the wrong world -- only fixed itself after a kill + redeploy
  // gave the round trip time to land. Sending it again here, synchronously
  // right before the launch/redeploy/install message on the SAME websocket
  // connection, guarantees the backend's selection is fresh by the time it
  // resolves which actual target to launch, regardless of any earlier
  // message's timing. No-ops harmlessly if the parent didn't pass this
  // prop (e.g. a standalone/testing mount) or if the operator hasn't
  // touched the robot config selector at all yet.
  resendRobotConfigIfKnown(namespace) {
    const robot_config = this.props.selected_robot_config
    if (robot_config !== undefined && robot_config !== null
        && robot_config !== 'None' && robot_config !== '') {
      this.props.ros.sendStringMsg(namespace + '/select_robot_config', robot_config)
    }
  }

  // Builds the payload for every sim/*_simulator topic below: a JSON object
  // carrying target_key AND the currently-displayed robot config together,
  // atomically, in the ONE message that triggers the launch. Sending
  // select_robot_config first via resendRobotConfigIfKnown (still done,
  // below, so an already-live bridge picks up the change too) is NOT enough
  // on its own -- confirmed live (2026-08-28) that it still raced runLaunch
  // on a fresh page load (pick Quadcopter, immediate Deploy could still
  // launch the rover), because select_robot_config and redeploy_simulator are
  // different ROS topics, each dispatched on its own independent subscriber
  // thread backend-side with no ordering guarantee, even though both are
  // sent back-to-back over this one websocket connection. Embedding
  // robot_config directly in this payload gives the backend everything it
  // needs from the ONE message its launch callback actually receives, so
  // there is nothing left to race.
  buildLaunchPayload(target) {
    const robot_config = this.props.selected_robot_config
    const has_robot_config = (robot_config !== undefined && robot_config !== null
        && robot_config !== 'None' && robot_config !== '')
    return JSON.stringify({
      target_key: target,
      robot_config: has_robot_config ? robot_config : null,
    })
  }

  // Publishes the selected target key to sim/redeploy_simulator. The app node
  // does the rest: Gazebo service request, readiness poll, then applies whatever robot
  // config is already selected in the Sim Connector panel (the existing,
  // unmodified select_robot_config flow) -- so choosing a simulator here and
  // a model over there, then clicking Deploy, applies both in one action.
  // Nothing here talks to the VM directly.
  //
  // Also doubles as "Use Open Sim" when the selected target already matches
  // what's running. Routed through sim/redeploy_simulator so that case is a
  // guaranteed kill-and-relaunch,
  // not the old short-circuit that just re-pointed the bridge at whatever
  // was already up without touching the VM -- requested live (2026-09-18):
  // "use open sim should kill and redeploy the sim with the new settings
  // set... without creating an extra window." That old short-circuit also
  // never called pushDirtyDimensions (only a FRESH gzserver launch does --
  // see that method's own comment in sim_connector_app_node.py), which is
  // why an environment/robot-dimensions edit made while a sim was already
  // deployed silently never reached Gazebo: this button looked like it
  // applied the change but never actually relaunched anything. When
  // nothing is running yet, redeploySimulatorCb's own runRedeploy skips the
  // stop and falls straight through to the same runLaunch this used to call
  // directly, so the plain first-time "Deploy" case (nothing up yet) is
  // unaffected.
  // 2026-09-08 -- REMOVED the popup this comment used to describe
  // (confirmUnsavedDimensionsOrPrompt, a window.prompt() blocking every
  // launch action whenever a dimensions role had unsaved edits, forcing an
  // operator to name+save a preset or cancel). Requested live: "this
  // should never be a popup that shows up. if its edited but not saved,
  // the default, or whatever is selected for the robot config, should just
  // launch. only time it can be changed is if the user saves the changes,
  // it shows up on the robot config dropdown, and they select that." A
  // launch now always pushes the SELECTED preset's own saved content (see
  // sim_connector_app_node.py's pushDirtyDimensions, which reads the
  // selected config's file when one is selected, falling back to the
  // built-in default only when nothing is), never whatever unsaved edits
  // happen to be sitting in the active store -- so an edit that was never
  // saved simply has no effect on what gets deployed, exactly as asked,
  // with nothing here left to prompt about.
  onDeployClicked() {
    const namespace = this.getSimNamespace()
    const target = this.getSelectedTarget()
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      this.resendRobotConfigIfKnown(namespace)
      this.props.ros.sendStringMsg(namespace + '/redeploy_simulator', this.buildLaunchPayload(target))
    }
  }

  // "New Sim" -- launches the selected target as a genuinely SEPARATE,
  // concurrent instance alongside whatever the primary slot (Use Open Sim/
  // Deploy/Kill above) is already running, rather than replacing it.
  // sim/launch_new_simulator is a distinct topic/thread/state slot from
  // sim/redeploy_simulator on the backend (see sim_connector_app_node.py's
  // secondary_launch_states comment) -- only gazebo_rover and
  // gazebo_quadcopter actually have their own GAZEBO_MASTER_URI/
  // ROS_MASTER_URI (in the legacy simulator_launch_targets.yaml, no longer
  // read since 2026-09-28), so this only really
  // supports one OTHER target running concurrently with the primary one,
  // not arbitrary N.
  onNewSimClicked() {
    const namespace = this.getSimNamespace()
    const target = this.getSelectedTarget()
    if (namespace == null || namespace === 'None' || target === 'None' || target === '') {
      return
    }
    // Requested live (2026-09-18): "if new sim is hit even if the robot
    // being summoned is the same, it should still work. it should just
    // give a warning saying 'you already have this robot config open, are
    // you sure you want to open another?'... and do it." Checked
    // client-side against secondary_states (already subscribed for
    // renderSecondaryInstances below) and the primary status's own
    // active_launch_target -- an approximation using the selected target
    // key, not the backend's fully robot_config-resolved actual_target, but
    // good enough to catch the common "clicked New Sim twice for the same
    // thing" case this prompt exists for.
    const status_msg = this.state.status_msg
    const already_open = (this.state.secondary_states[target] !== undefined)
      || (status_msg != null && status_msg.active_launch_target === target
          && status_msg.launcher_state === 'running')
    if (already_open) {
      const proceed = window.confirm(
        "You already have this robot config open. Are you sure you want to open another?")
      if (!proceed) {
        return
      }
    }
    this.resendRobotConfigIfKnown(namespace)
    const payload = JSON.parse(this.buildLaunchPayload(target))
    payload.confirmed = already_open
    this.props.ros.sendStringMsg(namespace + '/launch_new_simulator', JSON.stringify(payload))
  }

  onStopSecondaryClicked(actual_target) {
    const namespace = this.getSimNamespace()
    if (namespace != null && namespace !== 'None') {
      this.props.ros.sendStringMsg(namespace + '/stop_new_simulator', actual_target)
    }
  }

  // "Other running instances" -- one line + Stop button per secondary
  // target currently tracked in secondary_states, regardless of the
  // primary slot's own selected/active target. Rendered underneath the
  // main Deploy/Kill/New Sim controls (see render() below), not gated on
  // busy/deploy_disabled -- stopping a secondary instance is independent
  // of whatever the primary slot is doing.
  renderSecondaryInstances() {
    const entries = Object.entries(this.state.secondary_states)
    if (SHOW_NEW_SIM === false || entries.length === 0) {
      return null
    }
    return (
      <div>
        <Label title={"Other Running Instances"} />
        {entries.map(([actual_target, state]) => (
          <Columns key={actual_target}>
            <Column>{actual_target + ": " + state}</Column>
            <Column>
              <Button onClick={() => this.onStopSecondaryClicked(actual_target)}>{"Stop"}</Button>
            </Column>
          </Columns>
        ))}
      </div>
    )
  }

  onKillClicked() {
    const namespace = this.getSimNamespace()
    if (namespace != null && namespace !== 'None') {
      this.props.ros.sendTriggerMsg(namespace + '/stop_simulator')
    }
  }

  // Launch-target selector, backed by the launcher status message's reported
  // list -- the same reported-list-plus-selection shape Nepi_IF_Sim's own
  // selectors already use.
  renderTargetSelector() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }

    const available = (status_msg.available_launch_targets !== undefined)
      ? status_msg.available_launch_targets : []
    const names = (status_msg.available_launch_target_names !== undefined)
      ? status_msg.available_launch_target_names : []

    if (available.length === 0) {
      return (
        <Label title={"Simulator"}>
          <Input disabled value={"Not configured on this deployment"} />
        </Label>
      )
    }

    var items = []
    items.push(<Option key={'None'} value={'None'}>{'None'}</Option>)
    for (var i = 0; i < available.length; i++) {
      const display = (names[i] !== undefined && names[i] !== '') ? names[i] : available[i]
      items.push(<Option key={available[i]} value={available[i]}>{display}</Option>)
    }

    return (
      <Label title={"Simulator"}>
        <Select
          onChange={this.onTargetSelected}
          value={this.getSelectedTarget()}
        >
          {items}
        </Select>
      </Label>
    )
  }

  // Everything below the target selector: the running-sim message and
  // whichever button set applies. Two mutually-exclusive cases:
  //   1. Something is currently running -- show "You have X open" plus Kill,
  //      New Sim, and (only when the selection matches what's running) Use
  //      Open Sim.
  //   2. Otherwise (nothing running) -- plain Deploy.
  renderDeployControls() {
    const status_msg = this.state.status_msg
    const target = this.getSelectedTarget()
    if (status_msg == null || target === 'None' || target === '') {
      return null
    }

    const state = (status_msg.launcher_state !== undefined && status_msg.launcher_state !== '')
      ? status_msg.launcher_state : 'idle'
    const available = (status_msg.available_launch_targets !== undefined)
      ? status_msg.available_launch_targets : []
    const names = (status_msg.available_launch_target_names !== undefined)
      ? status_msg.available_launch_target_names : []

    const busy = (state === 'launching') || (state === 'stopping')
    const running = (state === 'running')
    const last_error = (status_msg.last_error !== undefined) ? status_msg.last_error : ''

    // Plain wrapping text, not an <Input> -- these messages run long (the
    // launch_command refuse-guards in particular spell out exactly why and
    // what to do about it), and a single-line input box just clips them
    // instead of showing the whole thing. Stacked title-then-text, not the
    // usual two-column Label (title on the left half, value on the right
    // half) -- that split put a long wrapping message in the right half of
    // the row, far from its own "Last Error" title. Requested live
    // (2026-09-04): "for the 'last error' box, keep it close to the text."
    const error_row = (last_error !== '') ?
      <div style={{ marginTop: Styles.vars.spacing.regular }}>
        <div style={{ fontWeight: 'bold', textAlign: "left" }}>{"Last Error"}</div>
        <div style={{
          textAlign: "left",
          whiteSpace: "normal",
          wordBreak: "break-word",
          color: Styles.vars.colors.red,
        }}>
          {last_error}
        </div>
      </div>
    : null

    // Copy-paste fallback -- populated by the backend (publishLauncherStatus)
    // when a request fails because the Gazebo service isn't responding, for a
    // human to run directly on the sim machine. A <pre> block, not an
    // <Input> or the plain wrapping <div> error_row uses -- these are
    // multi-line shell commands meant to be selected and copied verbatim,
    // where preserved line breaks and a monospace font both matter.
    const manual_fallback_commands = (status_msg.manual_fallback_commands !== undefined)
      ? status_msg.manual_fallback_commands : ''
    const manual_fallback_row = (manual_fallback_commands !== '') ?
      <Label title={"Run These Commands Manually"}>
        <pre style={{
          textAlign: "left",
          whiteSpace: "pre-wrap",
          wordBreak: "break-word",
          userSelect: "text",
        }}>
          {manual_fallback_commands}
        </pre>
      </Label>
    : null

    if (running) {
      const running_target = status_msg.selected_launch_target
      const running_ind = available.indexOf(running_target)
      const running_name = (running_ind !== -1 && names[running_ind] !== undefined && names[running_ind] !== '')
        ? names[running_ind] : running_target
      const selection_matches_running = (running_target === target)

      // What actually launched can differ from the raw selector choice
      // (running_name/running_target above) -- a robot config can redirect
      // "Gazebo" to a completely different, hidden target (e.g. the
      // quadcopter's own ArduCopter SITL + iris world). Reported live
      // (2026-08-19) as genuine confusion -- "I selected Gazebo and
      // Quadcopter but it launched the rover instead" turned out to be
      // impossible to verify from this Status line alone, since it only
      // ever showed the selector's own name regardless of the real target.
      // active_launch_target_name is empty on a not-yet-redeployed backend
      // (older SimLauncherStatus.msg) -- falls back to the selector name
      // exactly as before in that case.
      const active_name = (status_msg.active_launch_target_name !== undefined
                           && status_msg.active_launch_target_name !== '')
        ? status_msg.active_launch_target_name : running_name
      const status_text = (status_msg.active_launch_target !== undefined
                           && status_msg.active_launch_target !== ''
                           && status_msg.active_launch_target !== running_target)
        ? "You have " + running_name + " open (actually running: " + active_name + ")"
        : "You have " + active_name + " open"

      return (
        <React.Fragment>

          <Label title={"Status"}>
            <Input disabled value={status_text} />
          </Label>

          {error_row}
          {manual_fallback_row}

          {/* A different simulator picked while this one runs: Deploy
              switches to it -- the backend stops the running one first. */}
          <ButtonMenu>
            {selection_matches_running ?
              <Button disabled={busy} onClick={this.onDeployClicked}>{"Use Open Sim"}</Button>
            : <Button disabled={busy} onClick={this.onDeployClicked}>{"Deploy"}</Button>}
            {SHOW_NEW_SIM ?
              <Button disabled={busy} onClick={this.onNewSimClicked}>{"New Sim"}</Button>
            : null}
            <Button disabled={busy} onClick={this.onKillClicked}>{"Kill"}</Button>
          </ButtonMenu>

          {this.renderSecondaryInstances()}

        </React.Fragment>
      )
    }

    const deploy_disabled = (available.length === 0) || busy

    return (
      <React.Fragment>
        {error_row}
        {manual_fallback_row}
        <ButtonMenu>
          <Button disabled={deploy_disabled} onClick={this.onDeployClicked}>{"Deploy"}</Button>
        </ButtonMenu>
        {this.renderSecondaryInstances()}
      </React.Fragment>
    )
  }

  // make_section defaults to false here (the opposite of every other
  // Nepi_IF_* component in this app): the normal mount point is directly
  // inside Nepi_IF_Sim's own render, right under its Robot Config selector,
  // with no separate "Simulator Launcher" heading of its own -- there is no
  // longer a standalone panel for this at all. A caller that does want its
  // own titled section (e.g. testing this component in isolation) can still
  // pass make_section={true}.
  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : false
    const title = (this.props.title !== undefined) ? this.props.title : "Simulator"
    const status_msg = this.state.status_msg
    // Lets a parent mount the selector and the deploy controls at two
    // different places on the page (selector up top, deploy controls below
    // the other sim controls) instead of always getting both together.
    // Undefined (the default) renders both, unchanged from before this prop
    // existed.
    const only = this.props.only

    // No status yet: render nothing, matching Nepi_IF_Sim's own not-ready
    // branch -- this is "haven't heard from the device yet", not "not
    // configured" (that case still renders, see renderTargetSelector).
    if (status_msg == null) {
      return (
        <Columns>
          <Column>

          </Column>
        </Columns>
      )
    }

    const content = (
      <React.Fragment>
        {(only !== "deploy") ? this.renderTargetSelector() : null}
        {(only !== "selector") ? this.renderDeployControls() : null}
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

export default NepiIFSimLauncher
