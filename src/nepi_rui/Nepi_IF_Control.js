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

//import Toggle from "react-toggle"
import AsyncToggle from "./AsyncToggle"
//import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import { SliderAdjustment } from "./AdjustmentWidgets"
import RangeAdjustment from "./RangeAdjustment"
//import BooleanIndicator from "./BooleanIndicator"
import ColoredIndicator from "./ColoredIndicator"

import { round, rgbToIindicatorColor, setElementStyleModified, clearElementStyleModified, onChangeSwitchStateValue } from "./Utilities"

@inject("ros")
@observer

// Component that contains the ControlIF control. Renders one widget per
// control from a nepierfaces/ControlStatus message.
class Nepi_IF_Control extends Component {
  constructor(props) {
    super(props)

    this.state = {
      // name -> in-progress edit string for editable text/number inputs
      editValues: {},

      // name -> { baseline, typed, control_type } for values we have sent but not yet
      // seen confirmed in an incoming status. Keeps the optimistic override in
      // editValues alive until statusListener() reconciles it (see below).

      // "Selections" only: whether its option list is expanded. One
      // Nepi_IF_Control renders exactly one control_msg, so this is a plain
      // boolean rather than a name-keyed map. Local view state with no backend
      // round trip -- deliberately not mirrored into the control.
      ddOpen: false,

    }

    this.getControlValue = this.getControlValue.bind(this)

    this.renderBounds = this.renderBounds.bind(this)

    this.renderDoubleControl = this.renderDoubleControl.bind(this)
    this.renderTripleControl = this.renderTripleControl.bind(this)

    this.renderIntSliderControl = this.renderIntSliderControl.bind(this)


    this.renderFloatSliderControl = this.renderFloatSliderControl.bind(this)
    this.onInputChange = this.onInputChange.bind(this)
    this.onInputKey = this.onInputKey.bind(this)
    this.toggleDropDown = this.toggleDropDown.bind(this)
  }

  toggleDropDown() {
    this.setState({ ddOpen: this.state.ddOpen === false })
  }

  // Read the current value a control reports in a status message, by name and
  // control_type. Returns null if the control isn't present or isn't an editable control_type.
  getControlValue() {
    const control_msg = this.props.control_msg !== undefined ? this.props.control_msg : null
    

    const LIST_TYPES = ["Menu","Selections","Toggles","IntDouble","IntTriple","IntSliders","FloatDouble","FloatTriple","FloatSliders","RangeSlider","ColorRGB"]

    const STRING_TYPES = ["Selection","Selections","Toggles"]
    const BOOL_TYPES = ["Toggle"]
    const INT_TYPES = ["Menu","Int","IntDouble","IntTriple","IntSlider","IntSliders","ColorRGB"]
    const FLOAT_TYPES = ["Float",,"FloatDouble","FloatTriple","FloatSliders","RangeSlider"]
    const EMPTY_TYPES = ['Trigger']

    if (control_msg == null) { return null }
    const msg_value = control_msg.value
    const control_type = control_msg.type
    const IS_LIST_TYPE = LIST_TYPES.indexOf(control_type)

    const IS_STRING_TYPE = STRING_TYPES.indexOf(control_type)
    const IS_BOOL_TYPE = BOOL_TYPES.indexOf(control_type)
    const IS_INT_TYPE = INT_TYPES.indexOf(control_type)
    const IS_FLOAT_TYPE = FLOAT_TYPES.indexOf(control_type)
    const IS_EMPTY_TYPE = EMPTY_TYPES.indexOf(control_type)


    var values_list = null
    var value = null

    if (IS_STRING_TYPE !== -1){
      values_list = msg_value
    }
    if (IS_BOOL_TYPE !== -1){
      values_list = msg_value.map(item => item === 'True')
    }
    if (IS_FLOAT_TYPE !== -1){
      values_list = msg_value.map(item => parseFloat(item))
    }
    if (IS_INT_TYPE !== -1){
      values_list = msg_value.map(item => parseInt(item))
    }
    if (IS_EMPTY_TYPE !== -1){
      values_list = msg_value.map(item => 'EMPTY')
    }

    if (values_list != null){
      if (IS_LIST_TYPE !== -1) { 
        value = values_list
      }
      else if (values_list.length > 0){
        value = values_list[0]
      }
    }

    return value
  }

  // Editable text/number input helpers (PTX control pattern)
  onInputChange(name, e) {
    const el = document.getElementById('csbx_' + name)
    if (el) { setElementStyleModified(el) }
    const editValues = { ...this.state.editValues }
    editValues[name] = e.target.value
    this.setState({ editValues: editValues })
  }

  onInputKey(name, control_type, e) {
    if (e.key !== 'Enter') { return }
    const namespace = this.props.namespace !== undefined ? this.props.namespace : null
    const topic = (this.props.topic !== undefined) ? this.props.topic : 'update_control'
    const { sendUpdateControlValue } = this.props.ros
    const el = document.getElementById('csbx_' + name)
    if (el) { clearElementStyleModified(el) }
    const raw = e.target.value
    // Value the control reports right now; statusListener() uses this baseline
    // to detect when the backend has acted on our change.
    const baseline = this.getControlValue()
    var sent = false
    if (control_type === "String") {
      sendUpdateControlValue(namespace  + "/" + topic, name, raw)
    } else if (control_type === "Int") {
      const val = parseInt(raw, 10)
      if (!Number.isNaN(val)) { sendUpdateControlValue(namespace  + "/" + topic, name, raw); sent = true }
    } else if (control_type === "Float") {
      const val = parseFloat(raw)
      if (!Number.isNaN(val)) { sendUpdateControlValue(namespace  + "/" + topic, name, raw); sent = true }
    }
    const editValues = { ...this.state.editValues }
    delete editValues[name]
    
    this.setState({ editValues: editValues})
  }



  // Editable text/number input helpers (PTX control pattern)
  onInputChangeIndex(name, values, index, e) {
    const el = document.getElementById('csbx_' + name)
    if (el) { setElementStyleModified(el) }
    const editValues = { ...this.state.editValues }
    var update_values = values
    if (update_values.length > index){
        update_values[index] = e.target.value
        editValues[name] = update_values
      this.setState({ editValues: editValues })
    }
  }

  onInputKeyIndex(name, control_type, index, e) {
    if (e.key !== 'Enter') { return }
    const namespace = this.props.namespace !== undefined ? this.props.namespace : null
    const topic = (this.props.topic !== undefined) ? this.props.topic : 'update_control'
    const { sendUpdateControlValue } = this.props.ros
    const el = document.getElementById('csbx_' + name)
    if (el) { clearElementStyleModified(el) }
    const raw = e.target.value
    // Value the control reports right now; statusListener() uses this baseline
    // to detect when the backend has acted on our change.
    const baseline = this.getControlValue()
    var sent = false
    if (control_type === "String") {
      sendUpdateControlValue(namespace  + "/" + topic, name, raw, index)
    } else if (control_type === "ColorRBG") {
      const val = parseInt(raw, 10)
      if (!Number.isNaN(val)) { 
        if (val >= 0 && val <= 255){
          sendUpdateControlValue(namespace  + "/" + topic, name, raw, index); sent = true 
        }
      }
    } else if (control_type === "Int" || control_type === "IntDouble" || control_type === "IntTriple") {
      const val = parseInt(raw, 10)
      if (!Number.isNaN(val)) { sendUpdateControlValue(namespace  + "/" + topic, name, raw, index); sent = true }
    } else if (control_type === "Float" || control_type === "FloatDouble" || control_type === "FloatTriple") {
      const val = parseFloat(raw)
      if (!Number.isNaN(val)) { sendUpdateControlValue(namespace  + "/" + topic, name, raw, index); sent = true }
    }
    const editValues = { ...this.state.editValues }
    delete editValues[name]
    
    this.setState({ editValues: editValues})
  }



  renderBounds(min,max){
       
        const min_bound = (min !== parseInt(-999) ) ? min : 'Nan'
        const max_bound = (max !== parseInt(-999) ) ? max : 'Nan'
        return (

          <React.Fragment>

                <Columns>
                <Column>

                  <label > {"Min"} </label>                
                  <Input disabled={true} value={min_bound} />

                </Column>
                <Column>

                  <label > {"Max"} </label>                
                  <Input disabled={true} value={max_bound} />
                  
                </Column>
              </Columns>

          </React.Fragment>              

        )
      }



  renderDoubleControl(name, control_type, value,options){

        return (

          <React.Fragment>

                <Columns>
                <Column>

                  <label > {options[0]} </label>                
                  <Input
                    id={'csbx_' + name + '_' + options[0]}
                    style={{ width: "100%" }}
                    value={value}
                    onChange={(e) => this.onInputChangeIndex(name, value, 0, e)}
                    onKeyDown={(e) => this.onInputKeyIndex(name, control_type, 0, e)}
                  />

                </Column>
                <Column>

                  <label > {options[1]} </label>                
                  <Input
                    id={'csbx_' + name + '_' + options[1]}
                    style={{ width: "100%" }}
                    value={value}
                    onChange={(e) => this.onInputChangeIndex(name, value, 1, e)}
                    onKeyDown={(e) => this.onInputKeyIndex(name, control_type, 1, e)}
                  />
                  
                </Column>
              </Columns>

          </React.Fragment>              

        )
      }


  renderTripleControl(name, control_type, value, options){

        return (

          <React.Fragment>

                <Columns>
                <Column>

                  <label > {options[0]} </label>                
                  <Input
                    id={'csbx_' + name + '_' + options[0]}
                    style={{ width: "100%" }}
                    value={value}
                    onChange={(e) => this.onInputChangeIndex(name, value, 0, e)}
                    onKeyDown={(e) => this.onInputKeyIndex(name, control_type, 0, e)}
                  />

                </Column>
                <Column>

                  <label > {options[1]} </label>                
                  <Input
                    id={'csbx_' + name + '_' + options[1]}
                    style={{ width: "100%" }}
                    value={value}
                    onChange={(e) => this.onInputChangeIndex(name, value, 1, e)}
                    onKeyDown={(e) => this.onInputKeyIndex(name, control_type, 1, e)}
                  />
                  
                </Column>
                <Column>

                  <label > {options[2]} </label>                
                  <Input
                    id={'csbx_' + name + '_' + options[2]}
                    style={{ width: "100%" }}
                    value={value}
                    onChange={(e) => this.onInputChangeIndex(name, value, 2, e)}
                    onKeyDown={(e) => this.onInputKeyIndex(name, control_type, 2, e)}
                  />

                </Column>
              </Columns>

          </React.Fragment>              

        )
      }


  renderIntSliderControl(name,value,min,max, index){
        const namespace = this.props.namespace !== undefined ? this.props.namespace : null
        const topic = (this.props.topic !== undefined) ? this.props.topic : 'update_control'

        return (
          <SliderAdjustment
            title={name}
            comp_name={name}
            comp_index={index}
            is_control={true}
            topic={namespace + "/" + topic}
            msgType={"std_msgs/Float32"}
            adjustment={value}
            min={min}
            max={max}
            step={1}
            displayDecimals={0}
            scaled={1}
            tooltip={name}
            unit={""}
          />
        )
      }





  renderFloatSliderControl(name,value,min,max,round_value, round_display, index){
        const namespace = this.props.namespace !== undefined ? this.props.namespace : null
        const topic = (this.props.topic !== undefined) ? this.props.topic : 'update_control'
        // Step size and display precision come off the control message the same
        // defensive way the bounds above do. Both MUST be passed: SliderAdjustment
        // defaults step to 1, and its render rounds the value to displayDecimals
        // before handing it to BOTH the slider handle and the (disabled) text box.
        // Left unset, a [0.0, 1.0] control is a two-position switch, and passing
        // only one of the two still is -- a display coarser than the step
        // re-quantizes the handle even when the step is right.
        //
        // round_value is how many decimals the node rounds a SET value to
        // (nepi_control default -1, meaning no rounding); round_display is how
        // many the RUI should show (default 2). Neither is trusted on its own:
        // both are int32, so a control message that never carried them arrives
        // with 0 rather than undefined, and round_value 0 is step 1 -- the defect
        // again. The range check below is what actually rules that out.
        const range = max - min
        const fallback_step = (range > 0) ? (range / 100) : 1
        var step = (round_value >= 0 && round_value <= 6) ? Math.pow(10, -round_value) : fallback_step
        // Fewer than three stops between the ends is not a slider, whatever the
        // message asked for. Also catches range <= 0 and any non-finite bound.
        if (!(step > 0) || !((range / step) >= 2)) { step = fallback_step }
        if (!Number.isFinite(step) || step <= 0) { step = 1 }
        // Never display coarser than the step -- see the note above -- and never
        // finer than the node asked for.
        const step_decimals = Math.min(6, Math.max(0, Math.ceil(-Math.log10(step))))
        const displayDecimals = Math.max(step_decimals, round_display)

        return (
          <SliderAdjustment
            title={name}
            comp_name={name}
            comp_index={index}
            is_control={true}
            topic={namespace + "/" + topic}
            msgType={"std_msgs/Float32"}
            adjustment={value}
            min={min}
            max={max}
            step={step}
            displayDecimals={displayDecimals}
            scaled={1}
            tooltip={name}
            unit={""}
          />
        )
      }


  // Render a single control given its control_type and Control message.
  // Each block below maps one nepi_control control control_type to its RUI widget and
  // the nepi_control "value_*_control_value" topic it publishes to on change.
  render() {
    const { sendUpdateControlValue } = this.props.ros
    const namespace = this.props.namespace !== undefined ? this.props.namespace : null
    const topic = (this.props.topic !== undefined) ? this.props.topic : 'update_control'
    const control_msg = this.props.control_msg !== undefined ? this.props.control_msg : null
    const control_hidden = this.props.control_hidden !== undefined ? this.props.control_hidden : false
    const show_bound = this.props.show_bound !== undefined ? this.props.show_bound : true
  
    if (namespace == null || control_msg == null || control_hidden === true) {
      return (
        <React.Fragment>
          
        </React.Fragment>
      )
    }
    else {

      const name = control_msg.name
      const control_type =  control_msg.type
      const display_name = (control_msg.display_name && control_msg.display_name !== '') ? control_msg.display_name : name
      const options = control_msg.options
      const min_bound = control_msg.min_bound
      const max_bound = control_msg.max_bound
      const value = this.getControlValue()
      const round_value =  (control_msg.round_value >= 0) ? control_msg.round_value : 6
      const round_display =  (control_msg.round_display >= 0) ? control_msg.round_display : 6
      const values = (value != null) ? value : []
      // Value inputs whose value tracks either the in-progress edit or the message
      const editing = (name in this.state.editValues)

      // MENU -- drop-down of string options; the control's value is the *index*
      // of the selected option. Sends the new index as an Int.
      if (control_type === "Menu") {
        const display_value = (options.length >= value) ? options[value] : 'Option_' + String(value)
        return (
          <Label title={display_name} key={name}>
            <Select
              id={'csbx_' + name}
              value={value}
              onChange={(e) => sendUpdateControlValue(namespace  + "/" + topic,  name, String(parseInt(e.target.value, 10)))}
            >
              {options.map((opt, i) => <Option key={name + '_' + i} value={i}>{opt}</Option>)}
            </Select>
          </Label>
        )
      }

      // SELECTION -- drop-down of string options; the control's value is the
      // selected option *text* (not its index). Sends the new text as a String.
      // "Discrete" is an alias of "Selection", not a separate control_type: it is the
      // spelling driver params yaml files use for the same named option list,
      // so it renders through this same branch. 
      if (control_type === "Selection" || control_type === "Discrete") {
        return (
          <Label title={display_name} key={name}>
            <Select
              id={'csbx_' + name}
              value={value}
              onChange={(e) => sendUpdateControlValue(namespace  + "/" + topic, name, e.target.value)}
            >
              {options.map((opt, i) => <Option key={name + '_' + i} value={opt}>{opt}</Option>)}
            </Select>
          </Label>
        )
      }

      // TOGGLES -- a multi-select: each option gets its own toggle. The value
      // is the full array of currently-selected option strings. On every toggle
      // we send the complete desired selection (declarative), not a single delta.
      if (control_type === "Toggles") {
        const na_options = ['NONE','ALL']
        const show_options =  [...na_options, ...options]
        return (
          <Label title={display_name} key={name}>
            <div>
              {show_options.map((opt, i) => (
                <div key={name + '_' + i} style={{ display: "inline-block", marginRight: Styles.vars.spacing.regular, textAlign: "center" }}>
                  <div style={{ fontSize: Styles.vars.fontSize.small, marginBottom: Styles.vars.spacing.xs }}>{opt}</div>
                  <AsyncToggle
                    checked={values.indexOf(opt) !== -1}
                    onClick={() => {
                      // Send the complete desired selection (declarative), not a toggle.
                      const next = (opt === 'ALL') ? options : 
                                      (opt === 'NONE') ? [] :
                                          values.indexOf(opt) !== -1
                                            ? values.filter((s) => s !== opt)
                                            : [...values, opt]
                    
                      sendUpdateControlValue(namespace  + "/" + topic, name, next)
                    }}
                  />
                </div>
              ))}
            </div>
          </Label>
        )
      }

    
      if (control_type === "Selections") {
        // "None" and "All" are actions, not selectable options: they are never
        // highlighted and are never sent as values -- they resolve to [] and to
        // the full option list respectively. Mixed case, and the collapsed
        // affordance below is a bare narrow <Select>, because this widget is
        // deliberately the same dropdown as the AI detector's class selector
        // (NepiMgrAiDetector.js renderDetectorSettings) rather than a lookalike.
        // Note the existing "Toggles" toggle branch above spells these NONE
        // and ALL; the difference is intentional, it follows its own source.
        const rows = ['None', 'All', ...options]
        return (
          <Label title={display_name} key={name}>
            <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
            <div
              id={'csbx_' + name}
              onClick={this.toggleDropDown}
              style={{backgroundColor: Styles.vars.colors.grey0}}
            >
              <Select style={{width: "10px"}}/>
            </div>
            <div hidden={this.state.ddOpen === false}>
              {rows.map((opt, i) => (
                <div
                  key={name + '_dd_' + i}
                  onClick={() => {
                    const next = (opt === 'All') ? options :
                                   (opt === 'None') ? [] :
                                     values.indexOf(opt) !== -1
                                       ? values.filter((s) => s !== opt)
                                       : [...values, opt]
                    sendUpdateControlValue(namespace + "/" + topic, name, next)
                  }}
                  style={{
                    textAlign: "center",
                    padding: `${Styles.vars.spacing.xs}`,
                    color: Styles.vars.colors.black,
                    backgroundColor: (values.indexOf(opt) !== -1)
                                       ? Styles.vars.colors.blue
                                       : Styles.vars.colors.grey0,
                    cursor: "pointer",
                  }}
                >
                  {opt}
                </div>
              ))}
            </div>
          </Label>
        )
      }

      // TRIGGER -- a momentary action. There is no persistent value; pressing the
      // button fires a one-shot trigger (an empty String payload).
      if (control_type === "Trigger") {
        return (
          <Label title={display_name} key={name}>
            <ButtonMenu>
              <Button onClick={() => sendUpdateControlValue(namespace  + "/" + topic, name, value)}>{"Trigger"}</Button>
            </ButtonMenu>
          </Label>
        )
      }

      // TOGGLE -- a single on/off switch. Sends the *opposite* of the current
      // value as a Toggle each time it is clicked.
      if (control_type === "Toggle") {
        const checked = (value === 'True' || value === 'true' || value === true)
        return (
          <Label title={display_name} key={name}>
            <AsyncToggle
              checked={checked}
              onClick={() => sendUpdateControlValue(namespace  + "/" + topic, name, !checked)}
            />
          </Label>
        )
      }

      // STRING  -- free-form typed values. These follow the PTX
      // editable-input pattern: the box shows an in-progress edit string while
      // the user types, and the value is sent (parsed to the right control_type) only on
      // Enter. See onInputChange / onInputKey above.
      if (control_type === "String" ) {
        const show_value = (editing === true) ? this.state.editValues[name] : value
        return (
          <Label title={display_name} key={name}>
            <Input
              id={'csbx_' + name}
              style={{ width: "120px" }}
              value={show_value}
              onChange={(e) => this.onInputChange(name, e)}
              onKeyDown={(e) => this.onInputKey(name, control_type, e)}
            />
          </Label>
        )
      }

      // INT -- free-form typed values. These follow the PTX
      // editable-input pattern: the box shows an in-progress edit string while
      // the user types, and the value is sent (parsed to the right control_type) only on
      // Enter. See onInputChange / onInputKey above.
      if (control_type === "Int") {
        const show_value = (editing === true) ? this.state.editValues[name] : value
        return (

            <React.Fragment>
              <Label title={display_name} key={name}>
              </Label>

                <div hidden={show_bound === false}>
                  {this.renderBounds(min_bound,max_bound)}
                </div>
                
                <Input
                  id={'csbx_' + name}
                  style={{ width: "120px" }}
                  value={show_value}
                  onChange={(e) => this.onInputChange(name, e)}
                  onKeyDown={(e) => this.onInputKey(name, control_type, e)}
                />

            </React.Fragment> 
        )
      }

      if (control_type === "IntDouble") {
        
        return (

        <React.Fragment>
          <Label title={display_name} key={name}>
          </Label>

            <div hidden={show_bound === false}>
              {this.renderBounds(min_bound,max_bound)}
            </div>
            
            {this.renderDoubleControl(name, control_type, value, options)}

        </React.Fragment> 
        )
      }

      if (control_type === "IntTriple") {
        
        return (

        <React.Fragment>
          <Label title={display_name} key={name}>
          </Label>

            <div hidden={show_bound === false}>
              {this.renderBounds(min_bound,max_bound)}
            </div>
            
            {this.renderTripleControl(name, control_type, value, options)}

        </React.Fragment> 
        )
      }


      // INTSLIDER -- a single decimal value dragged between a min and max.
      // bounds carries [min, max]; -999 in either slot means "no limit",
      // in which case we fall back to a sensible default (0 / 100).
      if (control_type === "IntSlider") {
        const min = (min_bound !== -999) ? min_bound : 0
        const max = (max_bound !== -999) ? max_bound : 255
      
        this.renderIntSliderControl(name,value,min,max,'')
      }


      // INTSLIDERS -- a multi-select: each option gets its own int slider. 
      // names come from the options list. On every toggle
      // we send the complete desired selection (declarative), not a single delta.
      if (control_type === "IntSliders") {
        const min = (min_bound !== -999) ? min_bound : 0
        const max = (max_bound !== -999) ? max_bound : 255
        return (
          <Label title={display_name} key={name}>
                  <div>
                    {/* Map over the device names array */}
                    {options.map((slider_name, index) => (
                      this.renderIntSliderControl(slider_name, values[index], min, max, index)
                    ))}
                  </div>
          </Label>
        )
      }



      // FLOAT -- free-form typed values. These follow the PTX
      // editable-input pattern: the box shows an in-progress edit string while
      // the user types, and the value is sent (parsed to the right control_type) only on
      // Enter. See onInputChange / onInputKey above.
      if (control_type === "Float") {
        const show_value = round((editing === true) ? this.state.editValues[name] : value, round_display)
        return (

        <React.Fragment>
          <Label title={display_name} key={name}>
          </Label>

            <div hidden={show_bound === false}>
              {this.renderBounds(min_bound,max_bound)}
            </div>
            
            <Input
              id={'csbx_' + name}
              style={{ width: "120px" }}
              value={show_value}
              onChange={(e) => this.onInputChange(name, e)}
              onKeyDown={(e) => this.onInputKey(name, control_type, e)}
            />

        </React.Fragment>
        )
      }

      if (control_type === "FloatDouble") {
        const show_value = round((editing === true) ? this.state.editValues[name] : value, round_display)
        return (

        <React.Fragment>
          <Label title={display_name} key={name}>
          </Label>

            <div hidden={show_bound === false}>
              {this.renderBounds(min_bound,max_bound)}
            </div>
            
            {this.renderDoubleControl(name, control_type, show_value, options)}

        </React.Fragment> 
        )
      }

      if (control_type === "FloatTriple") {
        const show_value = round((editing === true) ? this.state.editValues[name] : value, round_display)
        return (

        <React.Fragment>
          <Label title={display_name} key={name}>
          </Label>

            <div hidden={show_bound === false}>
              {this.renderBounds(min_bound,max_bound)}
            </div>
            
            {this.renderTripleControl(name, control_type, show_value, options)}

        </React.Fragment> 
        )
      }


      // FLOATSLIDER -- a single decimal value dragged between a min and max.
      // bounds carries [min, max]; -999 in either slot means "no limit",
      // in which case we fall back to a sensible default (0 / 100).
      if (control_type === "FloatSlider") {
        const min = (min_bound !== -999) ? min_bound : 0
        const max = (max_bound !== -999) ? max_bound : 1

        // Step size and display precision come off the control message the same
        // defensive way the bounds above do. Both MUST be passed: SliderAdjustment
        // defaults step to 1, and its render rounds the value to displayDecimals
        // before handing it to BOTH the slider handle and the (disabled) text box.
        // Left unset, a [0.0, 1.0] control is a two-position switch, and passing
        // only one of the two still is -- a display coarser than the step
        // re-quantizes the handle even when the step is right.
        //
        // round_value is how many decimals the node rounds a SET value to
        // (nepi_control default -1, meaning no rounding); round_display is how
        // many the RUI should show (default 2). Neither is trusted on its own:
        // both are int32, so a control message that never carried them arrives
        // with 0 rather than undefined, and round_value 0 is step 1 -- the defect
        // again. The range check below is what actually rules that out.
        this.renderFloatSliderControl(name,value,min,max,round_value,round_display,'')
      }


      // FLOATSLIDERS -- a multi-select: each option gets its own float slider. 
      // names come from the options list. On every toggle
      // we send the complete desired selection (declarative), not a single delta.
      if (control_type === "FloatSliders") {
        const min = (min_bound !== -999) ? min_bound : 0
        const max = (max_bound !== -999) ? max_bound : 1
        return (
          <Label title={display_name} key={name}>
                  <div>
                    {/* Map over the device names array */}
                    {options.map((slider_name, index) => (
                      this.renderFloatSliderControl(slider_name, values[index], min, max, round_value, round_display, index)
                    ))}
                  </div>
          </Label>
        )
      }

      // RANGESLIDER -- a min/max *range* dragged between two limits. values
      // holds the current [min, max] handles; bounds holds the outer
      // [min_limit, max_limit] the handles may move within.
      if (control_type === "RangeSlider") {
        const values = control_msg.values || [0, 1]
        const min_limit = (min_bound !== -999) ? min_bound : 0
        const max_limit = (max_bound !== -999) ? max_bound : 100
        return (
          <RangeAdjustment
            key={name}
            title={display_name}
            comp_name={name}
            is_control={true}
            topic={namespace + "/" + topic}
            min={values[0]}
            max={values[1]}
            min_limit_m={min_limit}
            max_limit_m={max_limit}
            tooltip={control_msg.description}
            unit={""}
          />
        )
      }


      // ColorRGB -- an multi-select: each option (R,G,B) gets its own int slider.
      // names come from the options list. On every toggle
      // we send the complete desired selection (declarative), not a single delta.
      if (control_type === "ColorRGB") {
        const min = 0
        const max = 255
        const indicator_color = rgbToIindicatorColor(value[0],value[1],value[2])
        return (

          <React.Fragment>
                


                <Columns>
                <Column>

                <Label title={display_name} key={name}> </Label>

                </Column>
                <Column>

                  <ColoredIndicator indicator_color={indicator_color} />
                  
                </Column>
              </Columns>

                  {this.renderTripleControl(name, control_type, value, options)}
          
                  <div>
                    {/* Map over the device names array */}
                    {options.map((slider_name, index) => (
                      this.renderIntSliderControl(slider_name, values[index], min, max, index)
                    ))}
                  </div>

          </React.Fragment>


         
        )
      }


      return null
    }
  }

}

export default Nepi_IF_Control
