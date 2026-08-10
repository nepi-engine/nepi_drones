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
import NepiIFSimLauncher from "./Nepi_IF_SimLauncher"

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

    // Two real panels, side by side, each using a comfortable share of the
    // full width -- not the copy-pasted-from-NepiDeviceIDX.js 75/2/23 split
    // this page shipped with, whose 75% pane was a placeholder
    // (renderImageViewer(), in the IDX camera page this was templated from)
    // that never got filled in here and just sat empty. Sim Connector has no
    // second, independent camera feed of its own to put there -- the one
    // camera view it has is already live inside Nepi_IF_Sim-Controls' own
    // state/subscriptions, nested under Sim Connector on the right below --
    // so relocating it would mean duplicating that subscription rather than
    // fixing a layout number, for a change nobody asked for.
    return (

      <Columns>
        <Column>

          <div style={{ display: 'flex' }}>

            <div style={{ width: "49%" }}>

              <NepiIFSimLauncher
                namespace={simNamespace}
                make_section={true}
                title={"Simulator Launcher"}
              />

            </div>

            <div style={{ width: '2%' }}>
              {}
            </div>

            <div style={{ width: "49%" }}>

              <NepiIFSim
                namespace={simNamespace}
                show_selectors={true}
                show_data={true}
                show_controls={true}
                make_section={true}
                title={"Sim Connector"}
              />

            </div>

          </div>

        </Column>
      </Columns>

    )
  }
}

export default NepiAppSimConnector
