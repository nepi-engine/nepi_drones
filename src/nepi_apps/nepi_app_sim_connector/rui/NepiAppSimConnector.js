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

import { Columns, Column } from "./Columns"

import NepiIFSim from "./Nepi_IF_Sim"

@inject("ros")
@observer

// Sim Connector application page.
//
// Deliberately thin, following the connect-app pages: it resolves the sim device
// namespace (<app>/sim) and hands it to the reusable Nepi_IF_Sim component, which
// owns both selectors, the status display, and the controls. This page adds
// nothing of its own -- no control logic, and nothing specific to any simulator.
class NepiAppSimConnector extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_sim_connector",
      simName: "sim",
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getSimNamespace = this.getSimNamespace.bind(this)
  }

  getBaseNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    if (namespacePrefix !== null && deviceId !== null) {
      return "/" + namespacePrefix + "/" + deviceId
    }
    return null
  }

  getAppNamespace() {
    const base = this.getBaseNamespace()
    if (base !== null) {
      return base + "/" + this.state.appName
    }
    return null
  }

  // The sim device namespace the Nepi_IF_Sim component subscribes to, i.e.
  // <app>/sim, matching the sub-namespace SimDeviceIF registers under.
  getSimNamespace() {
    const appNamespace = this.getAppNamespace()
    if (appNamespace !== null) {
      return appNamespace + "/" + this.state.simName
    }
    return null
  }

  render() {
    const simNamespace = this.getSimNamespace()

    // One panel, full width. The Deploy/Kill/Install controls now render
    // inline inside Nepi_IF_Sim itself, directly under Robot Config, so
    // there's no separate "Simulator Launcher" panel to lay out beside it
    // anymore -- see Nepi_IF_Sim.js's mount of NepiIFSimLauncher.
    return (

      <Columns>
        <Column>

          {/* The reverse-SSH "OS instance" deploy-target picker
              (NepiIFSimOsInstances, see docs/SIM_OS_INSTANCES_PLAN.md) is
              deliberately not mounted here anymore -- requested live
              (2026-09-04): with the shared-storage deploy_state.yaml flag
              file now the real deploy/kill transport (see
              simulator_launcher.py's _launch_via_deploy_state), reverse SSH
              and per-machine registration aren't needed for deploy/kill to
              work. The component and its backend topics are untouched, just
              unmounted -- remount it here if reverse SSH deploy targets are
              needed again later. */}

          {/* show_controls is deliberately false, not a bug -- this panel's job is
              standing up the right sim/robot config, not direct control. Manual
              motor/goto/camera controls would duplicate what Devices -> Robots
              already provides once a sim is deployed and its RBX driver registers
              there. See commit "Remove manual robot controls from sim connector RUI
              panel" if this looks wrong again -- it isn't. */}
          <NepiIFSim
            namespace={simNamespace}
            show_selectors={true}
            show_data={true}
            show_controls={false}
            make_section={true}
            title={"Sim Connector"}
          />

        </Column>
      </Columns>

    )
  }
}

export default NepiAppSimConnector
