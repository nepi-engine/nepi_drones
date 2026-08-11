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
import { toJS } from "mobx"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import Button from "./Button"
import Styles from "./Styles"

// Generic ROS topic publisher -- lets a user manually feed test data into any
// topic (a script's own inputs, or an upstream topic it subscribes to) without
// per-script custom UI. Message body is authored as raw JSON, matching
// roslib.js's own ROSLIB.Message shape -- this is deliberately not the
// dirty-tracked "Editable Input Box Pattern" used elsewhere for persistent
// device settings, since there is no external/device-side value to stay in
// sync with here; the form is a one-shot scratch pad, cleared by the user
// between test publishes, not a live-synced setting.
@inject("ros")
@observer
class NepiIFGenericPublisher extends Component {
  constructor(props) {
    super(props)

    this.state = {
      topicName: props.presetVersion ? (props.presetTopicHint || "") : "",
      messageType: props.presetVersion ? (props.presetMessageType || "") : "",
      jsonText: props.presetVersion ? (props.presetJson || "{}") : "{}",
      statusMsg: ""
    }

    this.onChangeTopicName = this.onChangeTopicName.bind(this)
    this.onChangeMessageType = this.onChangeMessageType.bind(this)
    this.onChangeJsonText = this.onChangeJsonText.bind(this)
    this.onClickPublish = this.onClickPublish.bind(this)
  }

  // A script's usage guide (NepiMgrScripts.js/ScriptDocs.js) can push a "Try
  // It" preset in here. presetVersion is a bump counter rather than diffing
  // the preset fields themselves, so clicking the same test command twice in
  // a row (e.g. after editing the topic and wanting the original json back)
  // still re-applies it.
  componentDidUpdate(prevProps) {
    if (this.props.presetVersion !== undefined && this.props.presetVersion !== prevProps.presetVersion) {
      this.setState({
        topicName: this.props.presetTopicHint || "",
        messageType: this.props.presetMessageType || "",
        jsonText: this.props.presetJson || "{}",
        statusMsg: "",
      })
    }
  }

  // Auto-fills messageType from the store's already-tracked topicNames/
  // topicTypes pair (Store.js populates both via a single ros.getTopics()
  // call already used for other topic filtering) whenever the typed topic
  // name exactly matches a live topic -- purely a convenience, the user can
  // still edit messageType directly for a topic that doesn't exist yet.
  onChangeTopicName(e) {
    const topicName = e.target.value
    const { topicNames, topicTypes } = this.props.ros
    const names = topicNames ? toJS(topicNames) : []
    const types = topicTypes ? toJS(topicTypes) : []
    const idx = names.indexOf(topicName)
    const messageType = idx !== -1 ? types[idx] : this.state.messageType
    this.setState({ topicName, messageType, statusMsg: "" })
  }

  onChangeMessageType(e) {
    this.setState({ messageType: e.target.value, statusMsg: "" })
  }

  onChangeJsonText(e) {
    this.setState({ jsonText: e.target.value, statusMsg: "" })
  }

  onClickPublish() {
    const { topicName, messageType, jsonText } = this.state
    if (!topicName || !messageType) {
      this.setState({ statusMsg: "Topic name and message type are both required" })
      return
    }
    let data
    try {
      data = JSON.parse(jsonText)
    } catch (err) {
      this.setState({ statusMsg: "Invalid JSON: " + err.message })
      return
    }
    try {
      this.props.ros.publishMessage({ name: topicName, messageType, data, noPrefix: true })
      this.setState({ statusMsg: "Published to " + topicName + " at " + new Date().toLocaleTimeString() })
    } catch (err) {
      this.setState({ statusMsg: "Publish failed: " + err.message })
    }
  }

  render() {
    const { topicNames } = this.props.ros
    const names = topicNames ? toJS(topicNames) : []

    return (
      <Section title={"Publish Test Message"}>
        <datalist id="genericPublisherTopicList">
          {names.map((n) => <option key={n} value={n} />)}
        </datalist>

        <Label title={"Topic Name"}>
          <Input
            id={"GenericPublisherTopicNameInput"}
            list={"genericPublisherTopicList"}
            value={this.state.topicName}
            onChange={this.onChangeTopicName}
            placeholder={"/nepi/device1/.../topic"}
            style={{ width: "100%" }}
          />
        </Label>

        <Label title={"Message Type"}>
          <Input
            id={"GenericPublisherMessageTypeInput"}
            value={this.state.messageType}
            onChange={this.onChangeMessageType}
            placeholder={"e.g. std_msgs/Bool"}
            style={{ width: "100%" }}
          />
        </Label>

        <Label title={"Message (JSON)"}>
          <textarea
            id={"GenericPublisherJsonTextArea"}
            value={this.state.jsonText}
            onChange={this.onChangeJsonText}
            rows={6}
            style={{ width: "100%", fontFamily: "monospace", color: Styles.vars.colors.black }}
          />
        </Label>

        <Columns>
          <Column>
            <Button onClick={this.onClickPublish}>{"Publish"}</Button>
          </Column>
          <Column>
            <label>{this.state.statusMsg}</label>
          </Column>
        </Columns>
      </Section>
    )
  }
}

export default NepiIFGenericPublisher
