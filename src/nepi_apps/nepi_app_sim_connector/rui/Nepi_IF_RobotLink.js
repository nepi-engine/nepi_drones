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
import Label from "./Label"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import { Columns, Column } from "./Columns"
import AsyncToggle from "./AsyncToggle"

@inject("ros")
@observer

// Requested live (2026-09-15): "if a drone and ardupilot sitl are both up,
// the user should be able to link both, and whatever motor commands and
// flying happens in the sim should happen with the physical drone too."
//
// Mirrors robot_link_relay.py's own status JSON over sim/robot_link/status
// (a std_msgs/String, not a new typed message -- see that file's own
// docstring for why) -- takes the same sim device namespace prop
// (<app>/sim) Nepi_IF_Sim and Nepi_IF_SimLauncher already take, since these
// are sibling topics under it.
//
// The props-off toggle and the enable toggle are both AsyncToggle: both
// read a value that only updates on the next status message (a real
// backend-confirmed acknowledgment / link state), matching the CONVERT
// test in this repo's Toggle Pattern convention.
class NepiIFRobotLink extends Component {
  constructor(props) {
    super(props)

    this.state = {
      namespace: null,
      statusListener: null,
      status: null,
      simStatusListener: null,
      sim_status: null,
    }

    this.getSimNamespace = this.getSimNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListenerCb = this.statusListenerCb.bind(this)
    this.updateSimStatusListener = this.updateSimStatusListener.bind(this)
    this.simStatusListenerCb = this.simStatusListenerCb.bind(this)
    this.onSelectPhysicalRobot = this.onSelectPhysicalRobot.bind(this)
    this.onSelectSimRobot = this.onSelectSimRobot.bind(this)
  }

  getSimNamespace() {
    return this.props.namespace
  }

  componentDidMount() {
    this.updateStatusListener()
    this.updateSimStatusListener()
  }

  componentDidUpdate(prevProps) {
    if (prevProps.namespace !== this.props.namespace) {
      this.updateStatusListener()
      this.updateSimStatusListener()
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
    }
    if (this.state.simStatusListener != null) {
      this.state.simStatusListener.unsubscribe()
    }
  }

  updateStatusListener() {
    const namespace = this.getSimNamespace()
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null, status: null })
    }
    if (namespace != null && namespace !== "None") {
      const statusListener = this.props.ros.setupStatusListener(
        namespace + "/robot_link/status",
        "std_msgs/String",
        this.statusListenerCb
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ namespace: namespace })
  }

  statusListenerCb(message) {
    try {
      this.setState({ status: JSON.parse(message.data) })
    } catch (e) {
      // Malformed/empty payload -- keep the last good status rather than
      // rendering a broken panel over one bad tick.
    }
  }

  // SimStatus (the sim device's own main status message, sibling of
  // robot_link/status) carries available_simulators/available_simulator_
  // names/selected_simulator -- the SAME selector Nepi_IF_Sim.js's own
  // simulator dropdown drives via sim/select_simulator (device_if_sim.py's
  // SimDeviceIF already wires this topic; this component just reuses it
  // rather than inventing a second, independent notion of "which sim").
  // Requested live (2026-09-16): "it says select both a sim and a physical
  // robot first - however, there is only a box for selecting the physical
  // robot" -- the sim side was previously auto-tracked with no visible
  // control of its own, which looked like the sim half of the picker was
  // simply missing whenever auto-select hadn't picked one (or had picked
  // the wrong one, with more than one sim-capable device on the graph).
  updateSimStatusListener() {
    const namespace = this.getSimNamespace()
    if (this.state.simStatusListener != null) {
      this.state.simStatusListener.unsubscribe()
      this.setState({ simStatusListener: null, sim_status: null })
    }
    if (namespace != null && namespace !== "None") {
      const simStatusListener = this.props.ros.setupStatusListener(
        namespace + "/status",
        "nepi_app_sim_connector/SimStatus",
        this.simStatusListenerCb
      )
      this.setState({ simStatusListener: simStatusListener })
    }
  }

  simStatusListenerCb(message) {
    this.setState({ sim_status: message })
  }

  onSelectSimRobot(event) {
    const namespace = this.getSimNamespace()
    if (namespace == null) {
      return
    }
    this.props.ros.sendStringMsg(namespace + "/select_simulator", event.target.value)
  }

  onSelectPhysicalRobot(event) {
    const namespace = this.getSimNamespace()
    if (namespace == null) {
      return
    }
    this.props.ros.sendStringMsg(namespace + "/robot_link/select_physical_robot", event.target.value)
  }

  render() {
    const namespace = this.getSimNamespace()
    const status = this.state.status
    if (namespace == null || status == null) {
      return null
    }

    const sim_status = this.state.sim_status
    const available_sims = sim_status ? (sim_status.available_simulators || []) : []
    const available_sim_names = sim_status ? (sim_status.available_simulator_names || []) : []
    const selected_sim = sim_status ? (sim_status.selected_simulator || "") : ""

    const available = status.available_physical_robots || []
    const available_names = status.available_physical_robot_names || []
    const selected_physical = status.physical_namespace || ""
    const props_off_acknowledged = status.props_off_acknowledged === true
    const enabled = status.enabled === true
    const can_enable = selected_physical !== "" && selected_sim !== "" && props_off_acknowledged

    return (
      <Section title={"Robot Link"}>
        <Columns>
          <Column>
            <p style={{ color: Styles.vars.colors.orange }}>
              {"Warning: propellers, blades, and any other object that could cause harm " +
                "or damage must be removed/disconnected from the physical robot's motors " +
                "before linking -- the motors themselves can stay connected and powered. " +
                "Once linked, motor commands, arming, mode changes and flight/movement " +
                "commands sent to the simulator are mirrored to the physical robot in " +
                "real time."}
            </p>
          </Column>
          <Column>
            <Label title={"Sim Robot"}>
              <Select
                id="RobotLinkSimRobotSelect"
                value={selected_sim}
                onChange={this.onSelectSimRobot}
              >
                <Option value={""}>{"None"}</Option>
                {available_sims.map((ns, i) => (
                  <Option key={ns} value={ns}>
                    {available_sim_names[i] || ns}
                  </Option>
                ))}
              </Select>
            </Label>

            {available_sims.length === 0 && (
              <Label title={""}>
                <p style={{ color: Styles.vars.colors.red }}>
                  {"No simulator currently detected. Deploy one above to link it."}
                </p>
              </Label>
            )}

            <Label title={"Physical Robot"}>
              <Select
                id="RobotLinkPhysicalRobotSelect"
                value={selected_physical}
                onChange={this.onSelectPhysicalRobot}
              >
                <Option value={""}>{"None"}</Option>
                {available.map((ns, i) => (
                  <Option key={ns} value={ns}>
                    {available_names[i] || ns}
                  </Option>
                ))}
              </Select>
            </Label>

            {available.length === 0 && (
              <Label title={""}>
                <p style={{ color: Styles.vars.colors.red }}>
                  {"No physical robots currently detected. Connect one to link it."}
                </p>
              </Label>
            )}

            <Label title={"Confirm propellers/blades are removed"}>
              <AsyncToggle
                checked={props_off_acknowledged}
                disabled={selected_physical === ""}
                onClick={() =>
                  this.props.ros.sendBoolMsg(
                    namespace + "/robot_link/set_props_off_acknowledged",
                    !props_off_acknowledged
                  )
                }
              />
            </Label>

            <Label title={"Link Enabled"}>
              <AsyncToggle
                checked={enabled}
                disabled={!can_enable && !enabled}
                onClick={() =>
                  this.props.ros.sendBoolMsg(namespace + "/robot_link/enable", !enabled)
                }
              />
            </Label>

            {status.last_error ? (
              <Label title={""}>
                <p style={{ color: Styles.vars.colors.red }}>{status.last_error}</p>
              </Label>
            ) : null}
          </Column>
        </Columns>
      </Section>
    )
  }
}

export default NepiIFRobotLink
