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

@inject("ros")
@observer

// Additive multi-OS-instance deploy-target picker -- see
// docs/SIM_OS_INSTANCES_PLAN.md (nepi_drones). A separate file from
// Nepi_IF_SimLauncher, same reasoning that component is separate from
// Nepi_IF_Sim: this is additive over the existing single-hardcoded-VM
// launch/install/deploy flow, not a replacement for it. Takes the same sim
// device namespace prop (<app>/sim) every sibling component here takes, and
// talks to sim/os_instances/{status,register,verify,select,remove} --
// siblings of sim/launcher_status/launch_simulator under the same namespace.
//
// Mounted above <NepiIFSim> in NepiAppSimConnector.js -- this is the literal
// "button on top of the Sim Connector section" the OS-selected picker is
// meant to be. Selecting an instance here re-points EVERY
// simulator_launch_targets.yaml target's host/ssh_user/ssh_port at that
// machine (see os_instance_registry.py's select()); Nepi_IF_SimLauncher below
// this component keeps working completely unchanged, since it only ever
// reflects whatever SimLauncherStatus reports.
//
// Sentinel value for the "+ Add New OS Instance" selector entry -- never a
// real instance_id (those are all 'os_'-prefixed, see
// os_instance_registry.py's _instance_id_from_name), so it can't collide.
const ADD_NEW_VALUE = '__add_new_os_instance__'

class NepiIFSimOsInstances extends Component {
  constructor(props) {
    super(props)

    this.state = {
      namespace: null,
      status_msg: null,
      statusListener: null,

      // Local-only UI state -- nothing below is published until an explicit
      // button click.
      showing_add_form: false,
      new_instance_name: '',
      verify_ssh_user: '',
      verify_host: '',
    }

    this.getSimNamespace = this.getSimNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

    this.onOsSelected = this.onOsSelected.bind(this)
    this.onRegisterClicked = this.onRegisterClicked.bind(this)
    this.onTestConnectionClicked = this.onTestConnectionClicked.bind(this)
    this.onCancelAddClicked = this.onCancelAddClicked.bind(this)
    this.onRemoveInstance = this.onRemoveInstance.bind(this)

    this.renderOsSelector = this.renderOsSelector.bind(this)
    this.renderAddInstancePanel = this.renderAddInstancePanel.bind(this)
    this.renderInstanceList = this.renderInstanceList.bind(this)
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

  // Subscribes to <namespace>/os_instances/status, message type
  // SimOsInstancesStatus -- a sibling of Nepi_IF_SimLauncher's own
  // <namespace>/launcher_status subscription under the same sim device
  // namespace.
  updateStatusListener() {
    const namespace = this.getSimNamespace()
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null, status_msg: null })
    }
    if (namespace != null && namespace !== 'None') {
      var statusListener = this.props.ros.setupStatusListener(
        namespace + '/os_instances/status',
        "nepi_app_sim_connector/SimOsInstancesStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ namespace: namespace })
  }

  statusListener(message) {
    this.setState({ status_msg: message })
    // Once a register/verify round trip completes (setup_state back to
    // 'idle' or 'failed' no longer refers to a name the operator is still
    // typing), the local add-form's own name field is stale either way --
    // clearing it here (not on every message) avoids fighting the operator
    // while they're mid-typing before clicking Register.
    if (message.setup_state === 'idle' && this.state.showing_add_form
        && message.pending_instance_id === '') {
      this.setState({ showing_add_form: false, new_instance_name: '',
                     verify_ssh_user: '', verify_host: '' })
    }
  }

  onOsSelected(event) {
    const value = event.target.value
    const namespace = this.getSimNamespace()
    if (namespace == null || namespace === 'None') {
      return
    }
    if (value === ADD_NEW_VALUE) {
      this.setState({ showing_add_form: true })
      return
    }
    this.props.ros.sendStringMsg(namespace + '/os_instances/select', value)
  }

  onRegisterClicked() {
    const namespace = this.getSimNamespace()
    const name = this.state.new_instance_name.trim()
    if (namespace == null || namespace === 'None' || name === '') {
      return
    }
    this.props.ros.sendStringMsg(namespace + '/os_instances/register', name)
  }

  onTestConnectionClicked() {
    const namespace = this.getSimNamespace()
    const status_msg = this.state.status_msg
    if (namespace == null || namespace === 'None' || status_msg == null) {
      return
    }
    const instance_id = status_msg.pending_instance_id
    const ssh_user = this.state.verify_ssh_user.trim()
    if (instance_id === '' || ssh_user === '') {
      return
    }
    const payload = { instance_id: instance_id, ssh_user: ssh_user }
    const host = this.state.verify_host.trim()
    if (host !== '') {
      payload.host = host
    }
    this.props.ros.sendStringMsg(namespace + '/os_instances/verify', JSON.stringify(payload))
  }

  onCancelAddClicked() {
    const namespace = this.getSimNamespace()
    const status_msg = this.state.status_msg
    // A pending (not-yet-verified) registration is abandoned, not just
    // hidden -- otherwise cancelling would leave an orphaned 'pending'
    // instance cluttering the registry with no way back to it (re-clicking
    // "+ Add New" always creates a fresh instance_id, it can't resume this
    // one). Nothing to remove if registration never actually happened yet
    // (pending_instance_id still empty -- the operator only typed a name).
    if (namespace != null && namespace !== 'None' && status_msg != null
        && status_msg.pending_instance_id !== '') {
      this.props.ros.sendStringMsg(namespace + '/os_instances/remove', status_msg.pending_instance_id)
    }
    this.setState({ showing_add_form: false, new_instance_name: '',
                   verify_ssh_user: '', verify_host: '' })
  }

  onRemoveInstance(instance_id) {
    const namespace = this.getSimNamespace()
    if (namespace == null || namespace === 'None') {
      return
    }
    this.props.ros.sendStringMsg(namespace + '/os_instances/remove', instance_id)
  }

  // Header picker: every VERIFIED instance plus "+ Add New OS Instance" --
  // this is the "OS selected" control. Unverified instances don't appear
  // here (selecting one would silently point every launch target at a
  // machine that was never confirmed reachable) but are still listed,
  // removable, further down in renderInstanceList.
  renderOsSelector() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }
    const ids = status_msg.instance_ids || []
    const names = status_msg.instance_display_names || []
    const statuses = status_msg.instance_statuses || []
    const selected = status_msg.selected_instance_id || ''

    var items = []
    if (selected === '') {
      items.push(<Option key={'None'} value={'None'}>{'Default (this app\'s own config)'}</Option>)
    }
    for (var i = 0; i < ids.length; i++) {
      if (statuses[i] !== 'verified') {
        continue
      }
      items.push(<Option key={ids[i]} value={ids[i]}>{names[i]}</Option>)
    }
    items.push(<Option key={ADD_NEW_VALUE} value={ADD_NEW_VALUE}>{'+ Add New OS Instance'}</Option>)

    return (
      <Label title={"OS"}>
        <Select
          onChange={this.onOsSelected}
          value={(selected !== '') ? selected : 'None'}
        >
          {items}
        </Select>
      </Label>
    )
  }

  // The registration/verification wizard -- open only while showing_add_form
  // is set (the operator picked "+ Add New OS Instance") or a previous
  // registration is still pending/failed (so a page reload mid-setup doesn't
  // lose the in-progress state, since status_msg itself already reflects it).
  renderAddInstancePanel() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }
    const pending_id = status_msg.pending_instance_id || ''
    const setup_state = status_msg.setup_state || 'idle'
    const open = this.state.showing_add_form || pending_id !== ''
    if (!open) {
      return null
    }

    const last_error = (setup_state === 'failed' && status_msg.last_error) ? status_msg.last_error : ''
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

    // Step 1: nothing registered yet this round -- just a name + Register.
    if (pending_id === '') {
      return (
        <React.Fragment>
          <Label title={"New OS Instance Name"}>
            <Input
              value={this.state.new_instance_name}
              onChange={(e) => this.setState({ new_instance_name: e.target.value })}
            />
          </Label>
          <ButtonMenu>
            <Button onClick={this.onRegisterClicked}>{"Register"}</Button>
            <Button onClick={this.onCancelAddClicked}>{"Cancel"}</Button>
          </ButtonMenu>
        </React.Fragment>
      )
    }

    // Step 2: registered, not yet verified -- show the setup commands plus
    // the SSH username field and Test Connection.
    const verifying = (setup_state === 'verifying')
    return (
      <React.Fragment>

        <Label title={"Run These Commands On Your New Machine"}>
          <pre style={{
            textAlign: "left",
            whiteSpace: "pre-wrap",
            wordBreak: "break-word",
            userSelect: "text",
          }}>
            {status_msg.pending_setup_commands || ''}
          </pre>
        </Label>

        <Label title={"SSH Username On That Machine"}>
          <Input
            value={this.state.verify_ssh_user}
            onChange={(e) => this.setState({ verify_ssh_user: e.target.value })}
          />
        </Label>

        <Label title={"Host (leave blank if using the reverse tunnel above)"}>
          <Input
            value={this.state.verify_host}
            onChange={(e) => this.setState({ verify_host: e.target.value })}
          />
        </Label>

        {error_row}

        <ButtonMenu>
          <Button disabled={verifying} onClick={this.onTestConnectionClicked}>{"Test Connection"}</Button>
          <Button disabled={verifying} onClick={this.onCancelAddClicked}>{"Cancel"}</Button>
        </ButtonMenu>

      </React.Fragment>
    )
  }

  // Every registered instance (any status), each removable -- lets a
  // pending/unreachable instance be cleaned up even outside the add-flow
  // above (e.g. the operator navigated away mid-setup and came back later).
  renderInstanceList() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }
    const ids = status_msg.instance_ids || []
    if (ids.length === 0) {
      return null
    }
    const names = status_msg.instance_display_names || []
    const statuses = status_msg.instance_statuses || []
    const selected = status_msg.selected_instance_id || ''

    return (
      <React.Fragment>
        {ids.map((id, i) => {
          const label = names[i] + " (" + statuses[i] + ")" + ((id === selected) ? " -- selected" : "")
          return (
            <Columns key={id}>
              <Column>
                <Label title={label}>
                  <Button onClick={() => this.onRemoveInstance(id)}>{"Remove"}</Button>
                </Label>
              </Column>
            </Columns>
          )
        })}
      </React.Fragment>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const title = (this.props.title !== undefined) ? this.props.title : "Deploy-Target OS"
    const status_msg = this.state.status_msg

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
        {this.renderOsSelector()}
        {this.renderAddInstancePanel()}
        {this.renderInstanceList()}
      </React.Fragment>
    )

    if (make_section === false) {
      return (
        <React.Fragment>
          {content}
        </React.Fragment>
      )
    }
    return (
      <Section title={title}>
        {content}
      </Section>
    )
  }

}

export default NepiIFSimOsInstances
