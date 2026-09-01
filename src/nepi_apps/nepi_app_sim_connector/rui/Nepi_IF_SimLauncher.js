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
import BooleanIndicator from "./BooleanIndicator"
import Styles from "./Styles"
import { Columns, Column } from "./Columns"

@inject("ros")
@observer

// Additive simulator auto-launch control -- see
// docs/SIMULATOR_AUTO_LAUNCH_PLAN.md (nepi_drones). Deliberately a separate
// component/file from Nepi_IF_Sim, not an edit to it: this is a convenience
// trigger over the same passive sim/select_simulator + sim/select_robot_config
// flow Nepi_IF_Sim already owns, not a replacement for it. Takes the same
// sim device namespace prop (<app>/sim) Nepi_IF_Sim takes, and talks to the
// sibling sim/launcher_status, sim/launch_simulator, sim/stop_simulator
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

      // Local dropdown selection, applied on Deploy click -- not published
      // on every change, only Deploy/Kill/Install actually send anything.
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
    this.confirmUnsavedDimensionsOrPrompt = this.confirmUnsavedDimensionsOrPrompt.bind(this)
    this.onDeployClicked = this.onDeployClicked.bind(this)
    this.onNewSimClicked = this.onNewSimClicked.bind(this)
    this.onKillClicked = this.onKillClicked.bind(this)
    this.onInstallClicked = this.onInstallClicked.bind(this)
    this.onLaunchNewClicked = this.onLaunchNewClicked.bind(this)
    this.onUseExistingClicked = this.onUseExistingClicked.bind(this)
    this.onKillAllGazeboClicked = this.onKillAllGazeboClicked.bind(this)

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
    this.setState({ statusListener: null })
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
    this.setState({ namespace: namespace })
  }

  statusListener(message) {
    this.setState({ status_msg: message })
    // Keep the dropdown following the device's own reported selection
    // (e.g. after a launch started from elsewhere) unless the operator
    // hasn't picked anything yet.
    if (message.selected_launch_target !== undefined && message.selected_launch_target !== '') {
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
  // launch the rover), because select_robot_config and launch_simulator are
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

  // Publishes the selected target key to sim/launch_simulator. The app node
  // does the rest: ssh launch, readiness poll, then applies whatever robot
  // config is already selected in the Sim Connector panel (the existing,
  // unmodified select_robot_config flow) -- so choosing a simulator here and
  // a model over there, then clicking Deploy, applies both in one action.
  // Nothing here talks to the VM directly.
  //
  // Also doubles as "Use Open Sim" (same topic, same handler) when the
  // selected target already matches what's running -- the app node itself
  // short-circuits to just applying the new model to the already-connected
  // bridge rather than touching the VM again, so this button never needs to
  // know which case it's in.
  // Shared by every action below that actually launches/redeploys the
  // simulator (Deploy, New Sim, Launch New, Use Existing) -- a launch
  // pushes whatever's CURRENTLY active in each dimensions role to the VM
  // regardless of whether it was ever saved as a named preset, so this
  // catches the case where that active state has unsaved edits (see
  // Nepi_IF_Sim.js's robot_dimensions_selected_config/
  // environment_dimensions_selected_config -- '' means unsaved) BEFORE
  // launching, and prompts to name+save each one as a new preset.
  // Requested live (2026-09-01): an operator's own unsaved test edit had
  // been mistaken for having permanently replaced a built-in preset (it
  // hadn't -- only the reported selection was wrong, already fixed
  // separately); prompting up front removes the ambiguity that raised the
  // question in the first place, and keeps the built-in presets from ever
  // being what an un-saved edit quietly rides along on. Returns false if
  // the operator cancels the naming prompt -- the caller must not proceed
  // with the launch in that case.
  confirmUnsavedDimensionsOrPrompt() {
    const axes = [
      { role: 'robot', unsaved: this.props.unsaved_robot_dimensions, label: 'Robot' },
      { role: 'environment', unsaved: this.props.unsaved_environment_dimensions, label: 'Environment' },
    ]
    for (var i = 0; i < axes.length; i++) {
      const { role, unsaved, label } = axes[i]
      if (!unsaved) {
        continue
      }
      const name = window.prompt(
        label + " dimensions have been edited but not saved as a preset. Name a new preset " +
        "to save them as before deploying (Cancel to stop without deploying):"
      )
      if (name == null) {
        return false
      }
      if (name.trim() === '') {
        window.alert("Please enter a name.")
        return false
      }
      if (this.props.onSaveUnsavedDimensionsAs != null) {
        this.props.onSaveUnsavedDimensionsAs(role, name.trim())
      }
    }
    return true
  }

  onDeployClicked() {
    const namespace = this.getSimNamespace()
    const target = this.getSelectedTarget()
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      if (!this.confirmUnsavedDimensionsOrPrompt()) {
        return
      }
      this.resendRobotConfigIfKnown(namespace)
      this.props.ros.sendStringMsg(namespace + '/launch_simulator', this.buildLaunchPayload(target))
    }
  }

  // "New Sim" -- explicit clean restart: stops whatever is currently
  // running (if anything) and launches the selected target fresh, even if
  // that's the very same target that's already up. Distinct from Deploy/
  // "Use Open Sim" above, which never touches the VM when nothing needs to
  // change on it.
  onNewSimClicked() {
    const namespace = this.getSimNamespace()
    const target = this.getSelectedTarget()
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      if (!this.confirmUnsavedDimensionsOrPrompt()) {
        return
      }
      this.resendRobotConfigIfKnown(namespace)
      this.props.ros.sendStringMsg(namespace + '/redeploy_simulator', this.buildLaunchPayload(target))
    }
  }

  onKillClicked() {
    const namespace = this.getSimNamespace()
    if (namespace != null && namespace !== 'None') {
      this.props.ros.sendTriggerMsg(namespace + '/stop_simulator')
    }
  }

  // Publishes the selected target key to sim/install_simulator -- runs that
  // target's install_command over ssh, then re-checks it. Same namespace
  // pattern as Deploy/Kill.
  onInstallClicked() {
    const namespace = this.getSimNamespace()
    const target = this.getSelectedTarget()
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      this.props.ros.sendStringMsg(namespace + '/install_simulator', target)
    }
  }

  // Offered specifically when launcher_state is 'gazebo_conflict' -- see
  // sim_connector_app_node.py's isGazeboConflictError/runLaunch. Force past
  // the gazebo that's in the way by killing it (and anything else's gazebo
  // on that host -- see SimulatorLauncher.kill_all_gazebo's own docstring
  // for why this is deliberately blunt) and launching the selected target
  // fresh.
  onLaunchNewClicked() {
    const namespace = this.getSimNamespace()
    const target = this.getSelectedTarget()
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      if (!this.confirmUnsavedDimensionsOrPrompt()) {
        return
      }
      this.resendRobotConfigIfKnown(namespace)
      this.props.ros.sendStringMsg(namespace + '/force_launch_simulator', this.buildLaunchPayload(target))
    }
  }

  // The other conflict-resolution choice: skip starting a new gazebo
  // entirely and just point this target's bridge (and, for the
  // quadcopter target, SITL) at whatever's already running. Only works
  // out if the already-running gazebo happens to have the right world
  // loaded -- if not, the app's own ready_check reports a clean failure
  // rather than a false success (see attach_launch_command's own comment
  // in simulator_launch_targets.yaml).
  onUseExistingClicked() {
    const namespace = this.getSimNamespace()
    const target = this.getSelectedTarget()
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      if (!this.confirmUnsavedDimensionsOrPrompt()) {
        return
      }
      this.resendRobotConfigIfKnown(namespace)
      this.props.ros.sendStringMsg(namespace + '/attach_simulator', this.buildLaunchPayload(target))
    }
  }

  // Standalone escape hatch, not specific to the currently selected
  // target -- kills every gzclient/gzserver on every configured target's
  // host, regardless of who started it. Offered here because the
  // gazebo_conflict state is the situation this exists for, but it isn't
  // itself part of resolving THIS target's own launch (Launch New already
  // does this as its first step) -- it's for clearing a stray instance
  // without immediately relaunching anything.
  onKillAllGazeboClicked() {
    const namespace = this.getSimNamespace()
    if (namespace != null && namespace !== 'None') {
      this.props.ros.sendTriggerMsg(namespace + '/kill_all_gazebo')
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

  // Everything below the target selector: dependency state, the running-sim
  // message, and whichever button set applies. Four mutually-exclusive
  // cases, checked in this order:
  //   1. The last launch hit launch_command's own "a gzserver is already
  //      running" refuse-to-launch guard -- show the error plus Launch New
  //      / Use Existing / Kill All Gazebo instead of a dead-end failure.
  //   2. Something is currently running (regardless of the selected
  //      target's own install state -- it's running, so it's plainly
  //      present) -- show "You have X open" plus Kill, New Sim, and (only
  //      when the selection matches what's running) Use Open Sim.
  //   3. Nothing running, and the SELECTED target is confirmed missing --
  //      show Install instead of Deploy. "checking"/"unknown" intentionally
  //      do NOT show Install: unknown means "couldn't reach the host to
  //      tell" (see SimulatorLauncher.is_installed's docstring), and
  //      offering to install onto a host that can't be confirmed reachable
  //      invites a confusing failure rather than a useful one.
  //   4. Otherwise (idle/installed/unknown, nothing running) -- plain Deploy.
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
    const check_states = (status_msg.available_launch_target_installed_check_state !== undefined)
      ? status_msg.available_launch_target_installed_check_state : []
    const target_ind = available.indexOf(target)
    const check_state = (target_ind !== -1 && check_states[target_ind] !== undefined)
      ? check_states[target_ind] : 'unknown'

    const busy = (state === 'launching') || (state === 'stopping') || (state === 'installing')
    const running = (state === 'running')
    const last_error = (status_msg.last_error !== undefined) ? status_msg.last_error : ''

    // Plain wrapping text, not an <Input> -- these messages run long (the
    // launch_command refuse-guards in particular spell out exactly why and
    // what to do about it), and a single-line input box just clips them
    // instead of showing the whole thing.
    const error_row = (last_error !== '') ?
      <Label title={"Last Error"}>
        <div style={{
          textAlign: "left",
          whiteSpace: "normal",
          wordBreak: "break-word",
          color: Styles.vars.colors.red,
        }}>
          {last_error}
        </div>
      </Label>
    : null

    // Worst-case copy-paste fallback -- populated by the backend
    // (publishLauncherStatus) only once a failure is confirmed
    // dependency-related, for a human to run directly in a terminal when
    // auto-install either failed or (like the ArduPilot SITL quadcopter
    // target) was never offered in the first place. A <pre> block, not an
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

    // A real choice, not a dead end: launch_command's own refuse-to-launch
    // guard means a gazebo is already up but isn't tracked as this app's
    // own launch (see runLaunch's own comment). Checked before the
    // running/not_installed branches below since it's mutually exclusive
    // with both (a launch that hit this guard never got as far as
    // "running", and never needed an install check to fail this way).
    if (state === 'gazebo_conflict') {
      return (
        <React.Fragment>

          {error_row}
          {manual_fallback_row}

          <ButtonMenu>
            <Button disabled={busy} onClick={this.onLaunchNewClicked}>{"Launch New"}</Button>
            <Button disabled={busy} onClick={this.onUseExistingClicked}>{"Use Existing"}</Button>
            <Button disabled={busy} onClick={this.onKillAllGazeboClicked}>{"Kill All Gazebo"}</Button>
          </ButtonMenu>

        </React.Fragment>
      )
    }

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

          <ButtonMenu>
            {selection_matches_running ?
              <Button disabled={busy} onClick={this.onDeployClicked}>{"Use Open Sim"}</Button>
            : null}
            <Button disabled={busy} onClick={this.onNewSimClicked}>{"New Sim"}</Button>
            <Button disabled={busy} onClick={this.onKillClicked}>{"Kill"}</Button>
          </ButtonMenu>

        </React.Fragment>
      )
    }

    if (check_state === 'not_installed') {
      return (
        <React.Fragment>

          <Label title={"Dependencies Installed"}>
            <BooleanIndicator value={false} />
          </Label>

          {error_row}
          {manual_fallback_row}

          <ButtonMenu>
            <Button disabled={busy} onClick={this.onInstallClicked}>{"Install"}</Button>
          </ButtonMenu>

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
