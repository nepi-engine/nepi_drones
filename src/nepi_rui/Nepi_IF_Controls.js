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

import Toggle from "react-toggle"
import AsyncToggle from "./AsyncToggle"
import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import { SliderAdjustment } from "./AdjustmentWidgets"
import RangeAdjustment from "./RangeAdjustment"

import { setElementStyleModified, clearElementStyleModified, onChangeSwitchStateValue } from "./Utilities"


import Nepi_IF_Control from "./Nepi_IF_Control"

@inject("ros")
@observer

// Component that contains the ControlsIF controls. Renders one widget per
// control from a nepi_interfaces/ControlsStatus message.
class Nepi_IF_Controls extends Component {
  constructor(props) {
    super(props)

    this.state = {
      controlsNamespace: null,
      status_msg: null,

      // name -> in-progress edit string for editable text/number inputs
      editValues: {},

      // name -> { baseline, typed, type } for values we have sent but not yet
      // seen confirmed in an incoming status. Keeps the optimistic override in
      // editValues alive until statusListener() reconciles it (see below).
      pending: {},

      // "Show Controls" toggle state (Nepi_IF_Settings pattern). Defaults shown;
      // can be overridden via the show_controls prop or forced on via
      // allways_show_controls.
      show_controls: (this.props.show_controls !== undefined) ? this.props.show_controls : true,

      statusListener: null,
      needs_update: false
    }

    this.getNamespace = this.getNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.getControlValue = this.getControlValue.bind(this)
    this.renderControl = this.renderControl.bind(this)

  }

  getNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null) {
      if (this.props.namespace !== undefined) {
        namespace = this.props.namespace
      }
    }
    return namespace
  }

  // Read the current value a control reports in a status message, by name and
  // type. Returns null if the control isn't present or isn't an editable type.
  getControlValue(message, name, type) {
    if (message == null) { return null }
    const names = message.controls_name_list || []
    const i = names.indexOf(name)
    if (i === -1) { return null }
    const control_msg = message[i]

    const LIST_TYPES = ["Selections","Toggles","RangeSlider"]

    const STRING_TYPES = ["Selection","Selections","Toggles"]
    const BOOL_TYPES = ["Toggle", "Toggles"]
    const INT_TYPES = ["Int"]
    const FLOAT_TYPES = ["Float","FloatSlider","RangeSlider"]
    const EMPTY_TYPES = ['Trigger']

    if (control_msg == null) { return null }
    const msg_value = control_msg.value
    var values_list = null
    var value = null

    if (STRING_TYPES.indexOf(type) !== -1){
      values_list = msg_value
    }
    else if (BOOL_TYPES.indexOf(type) !== -1){
      values_list = msg_value.map(item => item === 'true')
    }
    else if (FLOAT_TYPES.indexOf(type) !== -1){
      value = msg_value.map(item => parseFloat(item))
    }
    else if (INT_TYPES.indexOf(type) !== -1){
      values_list = msg_value.map(item => parseInt(item))
    }
    else if (EMPTY_TYPES.indexOf(type) !== -1){
      values_list = msg_value.map(item => '')
    }

    if (values_list != null){
      if (LIST_TYPES.indexOf(type) !== -1) { 
        value = values_list
      }
      else if (values_list.length > 0){
        value = values_list[0]
      }
    }

    return value
  }

  statusListener(message) {
    // Reconcile any in-progress edits against the freshly received status.
    // While a value is being edited we keep showing the user's typed text (an
    // optimistic override in editValues) until this status confirms the change.
    // We drop the override when either the backend value has moved off what it
    // held when we sent (covers the node clamping/rejecting to a *different*
    // value, e.g. Int bounds [0,10]) or it now equals what the user typed.
    // Dropping the override in the same message that carries the new value lets
    // the input hand off from typed-text to backend-value with no stale frame.
    const pendingKeys = Object.keys(this.state.pending)
    if (pendingKeys.length === 0) {
      this.setState({ status_msg: message })
      return
    }
    const editValues = { ...this.state.editValues }
    const pending = { ...this.state.pending }
    let changed = false
    pendingKeys.forEach((name) => {
      const p = pending[name]
      const cur = this.getControlValue(message, name, p.type)
      if (cur == null) { return }
      var moved = false
      var matches = false
      if (p.type === "Int") {
        moved = cur !== p.baseline
        matches = cur === parseInt(p.typed, 10)
      } else if (p.type === "Float") {
        moved = cur !== p.baseline
        matches = cur === parseFloat(p.typed)
      } else { // String
        moved = String(cur) !== String(p.baseline)
        matches = String(cur) === String(p.typed)
      }
      if (moved || matches) {
        delete editValues[name]
        delete pending[name]
        changed = true
      }
    })
    if (changed) {
      this.setState({ status_msg: message, editValues: editValues, pending: pending })
    } else {
      this.setState({ status_msg: message })
    }
  }

  updateStatusListener(namespace) {
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null, status_msg: null })
    }
    if (namespace != null && namespace !== 'None' && namespace.indexOf('null') === -1) {
      const statusNamespace = namespace + '/status'
      var statusListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_interfaces/ControlsStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getNamespace()
    const props_status_msg = (this.props.status_msg !== undefined) ? this.props.status_msg : null
    const namespace_changed = (namespace !== this.state.controlsNamespace)
    if ((namespace != null && namespace_changed && props_status_msg == null) || this.state.needs_update === true) {
      this.updateStatusListener(namespace)
    }
    // Guarded: an unconditional setState here re-enters componentDidUpdate on
    // every render (mobx-react's observer SCU re-renders on any state identity
    // change), which is an infinite update loop, and it also cleared the
    // operator's in-progress edits on every frame.
    if (namespace_changed === true || this.state.needs_update === true) {
      this.setState({ controlsNamespace: namespace, needs_update: false, editValues: {}, pending: {} })
    }
  }

  componentDidMount() {
    this.setState({ needs_update: true })
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null })
    }
  }


  // Render a single control given its type and Control message.
  // Each block below maps one nepi_controls control type to its RUI widget and
  // the nepi_controls "set_*_control_value" topic it publishes to on change.
  renderControl(control_msg) {
    const namespace = this.getNamespace()
    const control_hidden = control_msg.hidden
      return (


         <Nepi_IF_Control
              namespace={namespace}
              control_msg={control_msg}
              control_hidden={control_hidden}
            />

      )

  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const status_msg = (this.props.status_msg !== undefined) ? this.props.status_msg : this.state.status_msg

    // Show Controls toggle (Nepi_IF_Settings pattern). allways_show_controls
    // forces the controls open and hides the toggle.
    const allways_show_controls = (this.props.allways_show_controls !== undefined) ? this.props.allways_show_controls : false
    const show_controls = (allways_show_controls === true) ? true : this.state.show_controls

    const show_controls_toggle = (allways_show_controls === false) ? (
      <Columns>
        <Column>
          <Label title="Show Controls">
            {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
            <Toggle
              checked={show_controls === true}
              onClick={() => onChangeSwitchStateValue.bind(this)("show_controls", show_controls)}>
            </Toggle>
          </Label>
        </Column>
        <Column>
        </Column>
      </Columns>
    ) : null

    // Controls widgets, one per non-hidden control. Only built when the section
    // is expanded and a status has arrived.
    var controls_body = null
    if (show_controls === true && status_msg != null) {
      const names = status_msg.controls_name_list || []
      const types = status_msg.controls_type_list || []
      const msgs = status_msg.controls_msg_list || []
      controls_body = (
        <Columns>
          <Column>
            {names.map((name, i) => {
              const control_msg = msgs[i]
              if (control_msg == null) { return null }
              // Hidden controls are not shown in the Controls box (they remain
              // manageable from the Controls Settings box).
              if (control_msg.hidden === true) { return null }
              return this.renderControl(control_msg)
            })}
          </Column>
        </Columns>
      )
    }

    const body = (
      <React.Fragment>
        {show_controls_toggle}
        {controls_body}
      </React.Fragment>
    )

    if (make_section === false) {
      return body
    }
    return (
      <Section title={(this.props.title !== undefined) ? this.props.title : "CONTROLS"}>
        {body}
      </Section>
    )
  }
}

export default Nepi_IF_Controls
