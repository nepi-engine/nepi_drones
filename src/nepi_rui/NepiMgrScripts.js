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
import React, { Component } from 'react';
import { observer, inject } from "mobx-react"
import { toJS } from 'mobx';
import Toggle from "react-toggle"
import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import Button, { ButtonMenu } from "./Button"
import ListBox from './ListBox';
import './ListBox.css';
import './Scripts.css';
import Styles from "./Styles"


import NepiSystemMessages from "./Nepi_IF_Messages"
import NepiIFGenericPublisher from "./Nepi_IF_GenericPublisher"
import ImageViewer from "./Nepi_IF_ImageViewer"
import ScriptDocs from "./ScriptDocs"

// Utilities
function bytesToKBString(bytes) {
  return ((bytes/1024.0).toFixed(2) + "KB")
}

@inject("ros")
@observer
class ScriptsMgr extends Component {
  constructor(props) {
    super(props);

    this.state = {
      selectedScript: '',
      runningSelectedScript: '',
      needs_update: false,
      showAdvanced: false,
      publisherPreset: null,
      publisherPresetVersion: 0
    };

    this.handleScriptsScriptSelect = this.handleScriptsScriptSelect.bind(this)
    //this.handleRunningScriptSelect = this.handleRunningScriptSelect.bind(this)
    this.handleStopScriptClick = this.handleStopScriptClick.bind(this)
    this.handleStartScriptClick = this.handleStartScriptClick.bind(this)
    this.handleCheckboxChange = this.handleCheckboxChange.bind(this)
    this.handleTryTestCommand = this.handleTryTestCommand.bind(this)
    this.handleAutoStartClick = this.handleAutoStartClick.bind(this)

    this.renderSelection = this.renderSelection.bind(this)
    this.renderControls = this.renderControls.bind(this)
    this.renderUsageGuide = this.renderUsageGuide.bind(this)
    this.renderPublisher = this.renderPublisher.bind(this)
    this.renderLiveFeed = this.renderLiveFeed.bind(this)
    this.getRequirementStatus = this.getRequirementStatus.bind(this)
    this.getLiveFeedTopic = this.getLiveFeedTopic.bind(this)

    this.prevRunningScripts = null;
  }

  componentDidMount(){
    this.setState({needs_update: true})
  }

  handleScriptsScriptSelect = (item) => {
    this.setState({ 
        selectedScript: item, 
        runningSelectedScript: ''
    });
    this.props.ros.callGetSystemStatsQueryService(item) // get script and system status
    this.props.ros.callGetSystemStatsQueryService(item, false) // Fire off a one-shot request for faster feedback
  };

  handleStartScriptClick = () => {
    // Start the currently selected script
    const scriptToLaunch = (this.state.selectedScript !== '')? this.state.selectedScript : this.state.runningSelectedScript
    if (scriptToLaunch) {
      this.props.ros.startLaunchScriptService(scriptToLaunch);
      this.props.ros.callGetSystemStatsQueryService(scriptToLaunch, false) // Fire off a one-shot request for faster feedback
    }
  }

  handleStopScriptClick = () => {
    // Stop the currently selected script
    const scriptToStop = (this.state.selectedScript !== '')? this.state.selectedScript : this.state.runningSelectedScript
    if (scriptToStop) {
      this.props.ros.stopLaunchScriptService(scriptToStop);
      this.props.ros.callGetSystemStatsQueryService(scriptToStop, false) // Fire off a one-shot request for faster feedback
    }
  };

  handleCheckboxChange = (e) => {
    const script = (this.state.selectedScript !== '')? this.state.selectedScript : this.state.runningSelectedScript
    this.props.ros.onToggleAutoStartEnabled(this.state.selectedScript, e.target.checked)
    this.props.ros.callGetSystemStatsQueryService(script, false) // Fire off a one-shot request for faster feedback
  }

  handleTryTestCommand = (cmd) => {
    // Opens Advanced Debugging and hands the command off to the generic
    // publisher via a bumped preset version -- see Nepi_IF_GenericPublisher's
    // componentDidUpdate for why a version counter, not a value diff.
    this.setState((prevState) => ({
      showAdvanced: true,
      publisherPreset: cmd,
      publisherPresetVersion: prevState.publisherPresetVersion + 1
    }))
  }

  handleAutoStartClick = (scriptName) => {
    // Same service call the main Start button uses (see
    // handleStartScriptClick) -- just invoked directly against a named
    // dependency script from another script's own guide, e.g.
    // sim_ai_targeting_bridge_script.py from drone_follow_object_mission_script.py's
    // entry, so the user doesn't have to navigate away to start it by hand.
    this.props.ros.startLaunchScriptService(scriptName)
    this.props.ros.callGetSystemStatsQueryService(scriptName, false)
  }


  getRequirementStatus(doc) {
    // Live-checks doc.requiredTopics (see ScriptDocs.js) against the current
    // topic list -- same substring-match style as renderMessages()'s
    // check_topic, and reactive for free since topicNames is a MobX
    // @observable this component (an @observer) already reads on every
    // render. Scripts with no requiredTopics (undocumented, or only
    // non-checkable free-text requires) default to allSatisfied=true so
    // Start behaves exactly as it did before this feature for them.
    const { topicNames } = this.props.ros
    const names = topicNames || []
    const requiredTopics = (doc && doc.requiredTopics) || []
    const items = requiredTopics.map((rt) => ({
      label: rt.label,
      satisfied: rt.patterns.some((pattern) => names.some((n) => n.indexOf(pattern) !== -1)),
    }))
    const allSatisfied = items.every((item) => item.satisfied)
    return { items, allSatisfied }
  }

  getLiveFeedTopic(doc) {
    // Finds whichever live topic matches doc.liveFeedTopicPattern (see
    // ScriptDocs.js) -- SITL/Gazebo and a real physical drone both publish
    // under "<device_name>/color_2d_image", so a plain substring search
    // picks up whichever vehicle is actually connected right now with no
    // sim-vs-hardware branching. Reactive for the same reason
    // getRequirementStatus is: topicNames is a MobX observable this
    // @observer component already re-reads on every render.
    if (!doc || !doc.liveFeedTopicPattern) {
      return null
    }
    const { topicNames } = this.props.ros
    const names = topicNames || []
    return names.find((n) => n.indexOf(doc.liveFeedTopicPattern) !== -1) || null
  }

  renderSelection() {
    const { scripts, running_scripts } = this.props.ros;
    //const { scripts, running_scripts, systemStats} = this.props.ros;
    let filesForListBox = []
    let runningFilesForListBox = [];

    //console.log('Scripts scripts:', scripts);
    filesForListBox = toJS(scripts)
    //  console.log('Scripts scripts (filesForListBox):', filesForListBox);
    //console.log('systemStats:', systemStats);
    //_systemStats = toJS(systemStats)
    //console.log('_systemStats:', _systemStats);
    //console.log('_systemStats:', _systemStats && _systemStats.cpu_percent);
    //console.log('_systemStats:', _systemStats && _systemStats.disk_usage);
    //console.log('_systemStats:', _systemStats && _systemStats.memory_usage);
    //console.log('_systemStats:', _systemStats && _systemStats.swap_info);
    //console.log('_systemStats:', _systemStats && _systemStats.file_size);

    runningFilesForListBox = toJS(running_scripts);
    //console.log('Running scripts (runningFilesForListBox):', runningFilesForListBox);
    
    return (
     
      <Columns>
        <Column>
          <Section title={"Scripts"}>
            <ListBox 
              id="scriptsListBox" 
              items={filesForListBox.scripts} 
              selectedItem={this.state.selectedScript} 
              onSelect={this.handleScriptsScriptSelect} 
              style={{ color: 'black', backgroundColor: 'white' }}
            />
          </Section>
        </Column>
        <Column>
          <Section title={"Running Scripts"}>
            <ListBox 
              id="runningScriptsListBox" 
              items={runningFilesForListBox.running_scripts} 
              selectedItem={this.state.runningSelectedScript}
              onSelect={this.handleRunningScriptSelect} 
              style={{ color: 'black', backgroundColor: 'white' }} 
            />
          </Section>
        </Column>
        </Columns>
    )
  }



  renderControls() {
    const { scripts, running_scripts, systemStats} = this.props.ros;
    //const { scripts, running_scripts, systemStats} = this.props.ros;
    //console.log('Scripts scripts:', scripts);

    //console.log('systemStats:', systemStats);
    //_systemStats = toJS(systemStats)
    //console.log('_systemStats:', _systemStats);
    //console.log('_systemStats:', _systemStats && _systemStats.cpu_percent);
    //console.log('_systemStats:', _systemStats && _systemStats.disk_usage);
    //console.log('_systemStats:', _systemStats && _systemStats.memory_usage);
    //console.log('_systemStats:', _systemStats && _systemStats.swap_info);
    //console.log('_systemStats:', _systemStats && _systemStats.file_size);
    //console.log('Running scripts (runningFilesForListBox):', runningFilesForListBox);

    const selectedScript = (this.state.selectedScript !== '')?
      this.state.selectedScript : this.state.runningSelectedScript

    const doc = ScriptDocs[selectedScript]
    const { items: requirementItems, allSatisfied } = this.getRequirementStatus(doc)
    const missingLabels = requirementItems.filter((item) => !item.satisfied).map((item) => item.label)
    const startDisabled = selectedScript === '' || !allSatisfied

    return (

      <Columns>
        <Column>
             <Section title={"Control and Status"}>
            <Label title={"File name"} >
              <Input 
                disabled 
                value={selectedScript || ''} 
                style={{width: '100%'}} 
              />
            </Label>
            <Label title={"File size"} >
              <Input 
                disabled 
                value={systemStats && typeof systemStats.file_size_bytes !== 'undefined'? bytesToKBString(systemStats.file_size_bytes) : ''} 
                style={{width: '100%'}}
              />
            </Label>
            <Label title={"Log size"} >
              <Input 
                disabled 
                value={systemStats && typeof systemStats.log_size_bytes !== 'undefined'? bytesToKBString(systemStats.log_size_bytes) : ''} 
                style={{width: '100%'}}
              />
            </Label>
            <Label title={"CPU Usage"} >
              <Input 
                disabled 
                value={systemStats && typeof systemStats.cpu_percent !== 'undefined'? 
                  systemStats.cpu_percent.toFixed(1) + "%" 
                  : ''} 
                style={{width: '100%'}} 
              />
            </Label>
            <Label title={"Memory Usage"} >
              <Input 
                disabled 
                value={systemStats && typeof systemStats.memory_percent !== 'undefined'? 
                  systemStats.memory_percent.toFixed(1) + "%"
                  : ''} 
                  style={{width: '100%'}} 
                />
            </Label>
            <Label title={"Run Time"} >
              <Input 
                disabled 
                value={systemStats && typeof systemStats.run_time_s !== 'undefined'?
                  systemStats.run_time_s.toFixed(1) + "s" 
                  : ''} 
                style={{width: '100%'}} 
              />
            </Label>
            <Label title={"Cumulative Run Time"} >
              <Input 
                disabled 
                value={systemStats && typeof systemStats.cumulative_run_time_s !== 'undefined'?
                  systemStats.cumulative_run_time_s.toFixed(1) + "s" 
                  : ''} 
                style={{width: '100%'}} 
              />
            </Label>
            <Label title={"Started Count"} >
              <Input 
                disabled 
                value={systemStats && typeof systemStats.started_runs !== 'undefined'? 
                  systemStats.started_runs
                  : ''}
                style={{width: '100%'}} 
              />
            </Label>
            <Label title={"Completion Count"} >
              <Input 
                disabled 
                value={systemStats && typeof systemStats.completed_runs !== 'undefined'? 
                  systemStats.completed_runs
                  : ''}
                style={{width: '100%'}} 
              />
            </Label>
            <Label title={"Error Count"} >
              <Input 
                disabled 
                value={systemStats && typeof systemStats.error_runs !== 'undefined'? 
                  systemStats.error_runs
                  : ''}
                style={{width: '100%'}} 
              />
            </Label>            
            <Label title={"Stop Count"} >
              <Input 
                disabled 
                value={systemStats && typeof systemStats.stopped_manually !== 'undefined'?
                  systemStats.stopped_manually
                  : ''} 
                style={{width: '100%'}} 
              />
            </Label>
            {(selectedScript !== '')?
              <ButtonMenu>
                <Label title={"Auto Start"} marginTop={Styles.vars.spacing.medium}>
                <Toggle
                  checked={systemStats && typeof systemStats.auto_start_enabled !== 'undefined'?
                    systemStats.auto_start_enabled
                    : false}
                  onChange={this.handleCheckboxChange}
                  //onChange={onToggleAutoStartEnabled}
                />
                </Label>
                <Button disabled={startDisabled} onClick={this.handleStartScriptClick}>{"Start"}</Button>
                <Button disabled={selectedScript === ''} onClick={this.handleStopScriptClick}>{"Stop"}</Button>
              </ButtonMenu>
              : null
            }
            {(selectedScript !== '' && missingLabels.length > 0) ?
              <label style={{color: Styles.vars.colors.red}}>
                {"Waiting for: " + missingLabels.join(", ")}
              </label>
              : null
            }
          </Section>
        </Column>
      </Columns>
    )
  }





  renderUsageGuide() {
    const scriptFile = this.state.selectedScript || this.state.runningSelectedScript
    if (!scriptFile) {
      return null
    }
    const doc = ScriptDocs[scriptFile]
    const { items: requirementItems } = this.getRequirementStatus(doc)

    return (
      <Columns>
        <Column>
          <Section title={"How to Use This Script"}>
            {!doc ?
              <label>{"No usage guide has been written for this script yet. Use Advanced Debugging below to inspect its topics directly."}</label>
            :
            <React.Fragment>
              <p>{doc.summary}</p>

              {doc.knownIssue ?
                <p style={{color: Styles.vars.colors.red, fontWeight: 'bold'}}>{"Known issue: " + doc.knownIssue}</p>
                : null
              }

              {doc.requires && doc.requires.length > 0 ?
                <React.Fragment>
                  <label style={{fontWeight: 'bold'}}>{"Requires"}</label>
                  <ul>
                    {doc.requires.map((r, i) => <li key={i}>{r}</li>)}
                  </ul>
                </React.Fragment>
                : null
              }

              {requirementItems.length > 0 ?
                <React.Fragment>
                  <label style={{fontWeight: 'bold'}}>{"Peripheral Status"}</label>
                  <ul>
                    {requirementItems.map((item, i) => (
                      <li key={i} style={{color: item.satisfied ? Styles.vars.colors.green : Styles.vars.colors.red}}>
                        {(item.satisfied ? "✓ " : "✗ ") + item.label +
                          (item.satisfied ? " -- detected" : " -- not detected yet")}
                      </li>
                    ))}
                  </ul>
                </React.Fragment>
                : null
              }

              {doc.autoStartScript ?
                <Button onClick={() => this.handleAutoStartClick(doc.autoStartScript)}>
                  {"Auto-Start Sim Requirements"}
                </Button>
                : null
              }

              {doc.testCommands && doc.testCommands.length > 0 ?
                <React.Fragment>
                  <label style={{fontWeight: 'bold'}}>{"Test Commands"}</label>
                  {doc.testCommands.map((cmd, i) => (
                    <Columns key={i}>
                      <Column>
                        <label>{cmd.label}</label>
                        <br />
                        <label style={{fontStyle: 'italic'}}>{cmd.topicHint + "  (" + cmd.messageType + ")"}</label>
                        {cmd.notes ? <React.Fragment><br /><label>{cmd.notes}</label></React.Fragment> : null}
                      </Column>
                      <Column>
                        <Button onClick={() => this.handleTryTestCommand(cmd)}>{"Try It"}</Button>
                      </Column>
                    </Columns>
                  ))}
                </React.Fragment>
                : null
              }

              {doc.tips && doc.tips.length > 0 ?
                <React.Fragment>
                  <label style={{fontWeight: 'bold'}}>{"What Else You Can Do"}</label>
                  <ul>
                    {doc.tips.map((t, i) => <li key={i}>{t}</li>)}
                  </ul>
                </React.Fragment>
                : null
              }
            </React.Fragment>
            }
          </Section>
        </Column>
      </Columns>
    )
  }



  renderMessages() {
    const { namespacePrefix, deviceId} = this.props.ros
    const {topicNames} = this.props.ros
    const script_file = this.state.selectedScript
    const check_topic = "/" + namespacePrefix + "/" + deviceId + "/" + script_file.split('.')[0] + "/messages"
    const topic_publishing = topicNames ? topicNames.indexOf(check_topic) !== -1 : false
    const msg_namespace = topic_publishing ? check_topic  : "Waiting for Topic to Publish"

    return (
      <React.Fragment>

      <Columns>
        <Column>

        <label style={{fontWeight: 'bold'}}>
            {msg_namespace}
          </label>

        <NepiSystemMessages
        namespace={msg_namespace}
        title={"NepiSystemMessages"}
        />

      </Column>
      </Columns>

      </React.Fragment>
    )
  }



  renderLiveFeed() {
    // Auto-appears the instant a live vehicle's camera topic is detected
    // (see getLiveFeedTopic) and disappears if it goes away -- no manual
    // "connect" step, matching the same live-detection approach as the
    // Peripheral Status checklist above. Only scripts that declare
    // liveFeedTopicPattern in ScriptDocs.js (currently the drone mission
    // scripts) ever try to show this.
    const scriptFile = this.state.selectedScript || this.state.runningSelectedScript
    const doc = ScriptDocs[scriptFile]
    const feedTopic = this.getLiveFeedTopic(doc)
    if (!feedTopic) {
      return null
    }
    return (
      <Columns>
        <Column>
          <Section title={"Live Camera Feed"}>
            <ImageViewer
              image_topic={feedTopic}
              title={feedTopic}
              hideQualitySelector={false}
              show_topic_selector={false}
              show_browser_save_button={false}
              show_save_controls={false}
              streamingImageQuality={50}
              streamingImageRate={10}
            />
          </Section>
        </Column>
      </Columns>
    )
  }

  renderPublisher() {
    // Generic test-message publisher (see Nepi_IF_GenericPublisher.js) --
    // lets you manually feed a script's inputs (or any topic it depends on)
    // to actually exercise it, since scripts have no per-script RUI controls
    // of their own. Not scoped to the selected script's own namespace on
    // purpose -- a script's real inputs (e.g. an upstream detection topic)
    // are frequently outside its own node namespace. Collapsed by default and
    // toggled from How to Use This Script's "Try It" buttons -- raw
    // topic/message-type publishing is not what an average user should land
    // on first.
    return (
      <Columns>
        <Column>
          <Button onClick={() => this.setState((s) => ({showAdvanced: !s.showAdvanced}))}>
            {(this.state.showAdvanced ? "Hide" : "Show") + " Advanced Debugging"}
          </Button>
          {this.state.showAdvanced ?
            <NepiIFGenericPublisher
              presetVersion={this.state.publisherPresetVersion}
              presetTopicHint={this.state.publisherPreset ? this.state.publisherPreset.topicHint : ""}
              presetMessageType={this.state.publisherPreset ? this.state.publisherPreset.messageType : ""}
              presetJson={this.state.publisherPreset ? this.state.publisherPreset.json : "{}"}
            />
            : null
          }
        </Column>
      </Columns>
    )
  }

  render() {
    const hide_app = this.state.selected_topic === "Connecting"
    return (


      <div style={{ display: 'flex' }}>
        <div style={{ width: '60%' }}>
          {this.renderSelection()}

          {this.renderUsageGuide()}

          {this.renderLiveFeed()}

          {this.renderMessages()}

          {this.renderPublisher()}

        </div>

        <div style={{ width: '5%' }}>
          {}
        </div>

        <div hidden={hide_app} style={{ width: '35%' }}>

        {this.renderControls()}

        </div>
      </div>

    )
  }

}


export default ScriptsMgr
