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
      selected_target: 'None',
    }

    this.getSimNamespace = this.getSimNamespace.bind(this)

    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

    this.onTargetSelected = this.onTargetSelected.bind(this)
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
      this.setState({ selected_target: message.selected_launch_target })
    }
  }

  onTargetSelected(event) {
    this.setState({ selected_target: event.target.value })
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
  onDeployClicked() {
    const namespace = this.getSimNamespace()
    const target = this.state.selected_target
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      this.props.ros.sendStringMsg(namespace + '/launch_simulator', target)
    }
  }

  // "New Sim" -- explicit clean restart: stops whatever is currently
  // running (if anything) and launches the selected target fresh, even if
  // that's the very same target that's already up. Distinct from Deploy/
  // "Use Open Sim" above, which never touches the VM when nothing needs to
  // change on it.
  onNewSimClicked() {
    const namespace = this.getSimNamespace()
    const target = this.state.selected_target
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      this.props.ros.sendStringMsg(namespace + '/redeploy_simulator', target)
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
    const target = this.state.selected_target
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
    const target = this.state.selected_target
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      this.props.ros.sendStringMsg(namespace + '/force_launch_simulator', target)
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
    const target = this.state.selected_target
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      this.props.ros.sendStringMsg(namespace + '/attach_simulator', target)
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
          value={this.state.selected_target}
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
    const target = this.state.selected_target
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

      return (
        <React.Fragment>

          <Label title={"Status"}>
            <Input disabled value={"You have " + running_name + " open"} />
          </Label>

          {error_row}

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
