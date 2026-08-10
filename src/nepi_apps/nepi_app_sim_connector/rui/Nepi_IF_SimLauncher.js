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
    this.onKillClicked = this.onKillClicked.bind(this)
    this.onInstallClicked = this.onInstallClicked.bind(this)

    this.renderTargetSelector = this.renderTargetSelector.bind(this)
    this.renderInstallState = this.renderInstallState.bind(this)
    this.renderStatus = this.renderStatus.bind(this)
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
  onDeployClicked() {
    const namespace = this.getSimNamespace()
    const target = this.state.selected_target
    if (namespace != null && namespace !== 'None' && target !== 'None' && target !== '') {
      this.props.ros.sendStringMsg(namespace + '/launch_simulator', target)
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
        <Label title={"Launch Target"}>
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
      <Label title={"Launch Target"}>
        <Select
          onChange={this.onTargetSelected}
          value={this.state.selected_target}
        >
          {items}
        </Select>
      </Label>
    )
  }

  // Whether the currently selected target's dependencies are known to be
  // present, plus an Install button when they're confirmed missing.
  // Independent of launcher_state -- a target can be checked/installed
  // whether or not anything is currently deployed. "checking"/"unknown"
  // intentionally do NOT show the Install button: unknown means "couldn't
  // reach the host to tell" (see SimulatorLauncher.is_installed), and
  // offering to install onto a host you can't confirm is reachable invites
  // a confusing failure rather than a useful one.
  renderInstallState() {
    const status_msg = this.state.status_msg
    const target = this.state.selected_target
    if (status_msg == null || target === 'None' || target === '') {
      return null
    }

    const available = (status_msg.available_launch_targets !== undefined)
      ? status_msg.available_launch_targets : []
    const check_states = (status_msg.available_launch_target_installed_check_state !== undefined)
      ? status_msg.available_launch_target_installed_check_state : []
    const target_ind = available.indexOf(target)
    const check_state = (target_ind !== -1 && check_states[target_ind] !== undefined)
      ? check_states[target_ind] : 'unknown'

    const state = (status_msg.launcher_state !== undefined) ? status_msg.launcher_state : 'idle'
    const busy = (state === 'launching') || (state === 'stopping') || (state === 'installing')

    return (
      <React.Fragment>

        <Label title={"Dependencies Installed"}>
          {(check_state === 'checking') ?
            <Input disabled value={"Checking..."} />
          : <BooleanIndicator value={check_state === 'installed'} />}
        </Label>

        {(check_state === 'not_installed') ?
          <ButtonMenu>
            <Button disabled={busy} onClick={this.onInstallClicked}>{"Install"}</Button>
          </ButtonMenu>
        : null}

      </React.Fragment>
    )
  }

  // Launcher state plus last error, and the Deploy/Kill buttons. Deploy is
  // disabled while a launch/stop/install is already in progress, nothing is
  // selected, or the target is confirmed not installed yet (Install is the
  // action to take instead, see renderInstallState) -- "unknown"/"checking"
  // do NOT block Deploy, since a target this app can't confirm the install
  // state of might still be perfectly launchable (e.g. check_installed_command
  // itself failing over a flaky connection shouldn't be mistaken for "this
  // won't work"). Kill is disabled unless something is actually running or on
  // its way up -- matching sim_connector_app_node.py's own launcher_thread
  // busy-check, just surfaced so the buttons don't invite a request the node
  // would ignore anyway.
  renderStatus() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }

    const state = (status_msg.launcher_state !== undefined && status_msg.launcher_state !== '')
      ? status_msg.launcher_state : 'idle'
    const available = (status_msg.available_launch_targets !== undefined)
      ? status_msg.available_launch_targets : []
    const check_states = (status_msg.available_launch_target_installed_check_state !== undefined)
      ? status_msg.available_launch_target_installed_check_state : []
    const target = this.state.selected_target
    const target_ind = available.indexOf(target)
    const check_state = (target_ind !== -1 && check_states[target_ind] !== undefined)
      ? check_states[target_ind] : 'unknown'

    const busy = (state === 'launching') || (state === 'stopping') || (state === 'installing')
    const deploy_disabled = (available.length === 0) || (target === 'None') || (target === '')
      || busy || (check_state === 'not_installed')
    const kill_disabled = (state !== 'running') && (state !== 'launching')

    return (
      <React.Fragment>

        <Label title={"Launcher State"}>
          <Input disabled value={state} />
        </Label>

        {(status_msg.last_error !== undefined && status_msg.last_error !== '') ?
          <Label title={"Last Error"}>
            <Input disabled value={status_msg.last_error} />
          </Label>
        : null}

        {this.renderInstallState()}

        <ButtonMenu>
          <Button disabled={deploy_disabled} onClick={this.onDeployClicked}>{"Deploy"}</Button>
          <Button disabled={kill_disabled} onClick={this.onKillClicked}>{"Kill"}</Button>
        </ButtonMenu>

      </React.Fragment>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const title = (this.props.title !== undefined) ? this.props.title : "Simulator Launcher"
    const status_msg = this.state.status_msg

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
        {this.renderTargetSelector()}
        {this.renderStatus()}
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
