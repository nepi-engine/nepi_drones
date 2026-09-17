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

import BooleanIndicator from "./BooleanIndicator"
import Styles from "./Styles"
import Section from "./Section"
import Button, { ButtonMenu } from "./Button"
import { Columns, Column } from "./Columns"
import Input from "./Input"
import Label from "./Label"

import NepiIFAdminEnable from "./Nepi_IF_AdminEnable"
import NepiIFSettings from "./Nepi_IF_Settings"

function roundWithSuffix(value, decimals, suffix) {
  return value && (value.toFixed(decimals) + " " + suffix)
}

@inject("ros")
@observer
class SystemMgr extends Component {
  constructor(props) {
    super(props)
    this.state = {

      connected: false,
      needs_update: false,

      vehicle_subnet: "",
      update_gateway: "",
      current_gateway: "",
      subnet_initialized: false,

      settingsNamespace: 'None',
      settingsListener: null,
      settingsNamesList: [],
      settingsValuesList: [],

      expand_armed: false

    }

    this.expandArmTimer = null

    this.getBaseNamespace = this.getBaseNamespace.bind(this)

    this.getValidSubnetPrefix = this.getValidSubnetPrefix.bind(this)
    this.getDerivedIp = this.getDerivedIp.bind(this)
    this.onChangeVehicleSubnet = this.onChangeVehicleSubnet.bind(this)
    this.onKeyVehicleSubnet = this.onKeyVehicleSubnet.bind(this)

    this.onChangeGateway = this.onChangeGateway.bind(this)
    this.onKeyGateway = this.onKeyGateway.bind(this)

    this.updateSettingsListener = this.updateSettingsListener.bind(this)
    this.settingsListener = this.settingsListener.bind(this)
    this.getSettingValue = this.getSettingValue.bind(this)

    this.apply_updates = this.apply_updates.bind(this)
    this.onExpandStorageDrive = this.onExpandStorageDrive.bind(this)

    this.renderSystemConfig = this.renderSystemConfig.bind(this)
    this.renderSystemAdvanced = this.renderSystemAdvanced.bind(this)
    this.renderSystemNetlist = this.renderSystemNetlist.bind(this)


  }

  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId
    }
    return baseNamespace
  }

  // Returns the "a.b.c" /24 network prefix for a valid vehicle subnet entry,
  // or null if the entry is not a valid subnet.  Accepts entries like
  // "192.168.179", "192.168.179.0", or "192.168.179.0/24".
  getValidIP(ip){
    if (ip === null || ip === undefined){
      return null
    }
    var octets = ip.trim().split("/")[0].split('.')
    if (octets === ""){
      return null
    }
    if (octets.length < 4){
      return null
    }
    for (var i = 0; i < octets.length; i++){
      var oct = octets[i]
      if (!/^\d{1,3}$/.test(oct)){
        return null
      }
      var num = parseInt(oct, 10)
      if (num < 0 || num > 255){
        return null
      }
    }
    return octets.join(".")
  }

  getValidSubnetPrefix(subnet){
    if (subnet === null || subnet === undefined){
      return null
    }
    var cleaned = subnet.trim().split("/")[0]
    if (cleaned === ""){
      return null
    }
    var octets = cleaned.split(".")
    if (octets.length < 3){
      return null
    }
    var prefix_octets = octets.slice(0, 3)
    for (var i = 0; i < prefix_octets.length; i++){
      var oct = prefix_octets[i]
      if (!/^\d{1,3}$/.test(oct)){
        return null
      }
      var num = parseInt(oct, 10)
      if (num < 0 || num > 255){
        return null
      }
    }
    return prefix_octets.join(".")
  }

  // Re-prefix an IP onto a new "a.b.c" network prefix.  The host octet and any
  // /mask suffix are preserved from currentValue (e.g. "10.10.10.103/16" with a
  // new prefix "192.168.5" becomes "192.168.5.103/16").  When currentValue is
  // blank or not a parseable IP, defaultHostSuffix is used instead (e.g.
  // "103/16", "1", "8").
  getDerivedIp(currentValue, newPrefix, defaultHostSuffix){
    var hostSuffix = defaultHostSuffix
    if (currentValue !== null && currentValue !== undefined && currentValue !== ""){
      var parts = currentValue.trim().split("/")
      var octets = parts[0].split(".")
      if (octets.length === 4 && /^\d{1,3}$/.test(octets[3])){
        hostSuffix = octets[3] + (parts.length > 1 ? "/" + parts[1] : "")
      }
    }
    return newPrefix + "." + hostSuffix
  }

  onChangeVehicleSubnet(event){
    const el = document.getElementById("VehicleSubnet")
    el.style.color = "purple"
    el.style.fontWeight = "bold"
    this.setState({ vehicle_subnet: event.target.value })
  }

  onKeyVehicleSubnet(event){
    if (event.key === 'Enter'){
      const prefix = this.getValidSubnetPrefix(this.state.vehicle_subnet)
      // Only accept the entry if it is a valid subnet.  An invalid entry is
      // rejected: it is not committed and the box stays marked (purple/bold).
      if (prefix !== null){
        const el = document.getElementById("VehicleSubnet")
        el.style.color = Styles.vars.colors.black
        el.style.fontWeight = "normal"
        // Push the subnet plus the three IPs derived from it in a single batch
        // update.  Each IP keeps its current host octet and netmask and only
        // swaps the network prefix.  The disabled boxes below read the new
        // values back from the live config.
        const base_namespace = this.getBaseNamespace()
        const settingsList = [
          { nameStr: 'NEPI_VEHICLE_SUBNET', typeStr: 'String', value: prefix },
          { nameStr: 'NEPI_ALIAS_IP_1', typeStr: 'String',
            value: this.getDerivedIp(this.getSettingValue('NEPI_ALIAS_IP_1'), prefix, '103/16') },
          { nameStr: 'NEPI_NTP_IP', typeStr: 'String',
            value: this.getDerivedIp(this.getSettingValue('NEPI_NTP_IP'), prefix, '1') },
          { nameStr: 'NEPI_NAV_IP_HNAV', typeStr: 'String',
            value: this.getDerivedIp(this.getSettingValue('NEPI_NAV_IP_HNAV'), prefix, '8') },
          { nameStr: 'NEPI_NAV_IP_NMEA', typeStr: 'String',
            value: this.getDerivedIp(this.getSettingValue('NEPI_NAV_IP_NMEA'), prefix, '1') }
        ]
        // updateSettings now publishes one update_setting message per entry:
        // the batch 'update_settings' topic is retired, and the node applied
        // its entries in a sequential loop anyway.
        this.props.ros.updateSettings(base_namespace + '/settings', settingsList)
      }
    }
  }


  onChangeGateway(event){
    const el = document.getElementById("Gateway")
    el.style.color = "purple"
    el.style.fontWeight = "bold"
    this.setState({ update_gateway: event.target.value })
  }

  onKeyGateway(event){
    if (event.key === 'Enter'){
      const ip = this.getValidIP(this.state.update_gateway)
      // Only accept the entry if it is a valid subnet.  An invalid entry is
      // rejected: it is not committed and the box stays marked (purple/bold).
      if (ip !== null){
        const el = document.getElementById("Gateway")
        el.style.color = Styles.vars.colors.black
        el.style.fontWeight = "normal"
        // Push the subnet plus the three IPs derived from it in a single batch
        // update.  Each IP keeps its current host octet and netmask and only
        // swaps the network prefix.  The disabled boxes below read the new
        // values back from the live config.
        const base_namespace = this.getBaseNamespace()
        const settingsList = [
          { nameStr: 'NEPI_GATEWAY_IP', typeStr: 'String', value: ip }
        ]
        this.props.ros.updateSettings(base_namespace + '/settings', settingsList)
      }
    }
  }


  // Callback for the system-config Settings status message.  Caches the
  // setting name/value pairs so the disabled boxes can show live values.
  //
  // Settings are a nepi_controls controls set, so the status is a
  // nepi_interfaces/ControlsStatus: parallel name/type lists plus one Control
  // message per setting.  Every system-config setting is a String or an Int
  // (system_mgr derives the type from the config value), so the displayed
  // value comes off set_string or set_int.  There is no settings_topic field
  // to gate on -- this listener is already scoped to one namespace.
  settingsListener(message){
    const names = message.controls_name_list ? message.controls_name_list : []
    const types = message.controls_type_list ? message.controls_type_list : []
    const msgs = message.controls_msg_list ? message.controls_msg_list : []
    var namesList = []
    var valuesList = []
    for (let ind = 0; ind < names.length; ind++){
      const control_msg = msgs[ind]
      if (control_msg == null){ continue }
      namesList.push(names[ind])
      valuesList.push((types[ind] === "Int") ? String(control_msg.set_int) : control_msg.set_string)
    }
    var newState = {
      settingsNamesList: namesList,
      settingsValuesList: valuesList
    }
    // Initialize the Vehicle Subnet box from the config once, when the box
    // is still empty and the config holds a real (non-NONE) subnet.
    if (this.state.subnet_initialized === false && this.state.vehicle_subnet === ""){
      const subnetInd = namesList.indexOf('NEPI_VEHICLE_SUBNET')
      if (subnetInd !== -1){
        const subnetVal = valuesList[subnetInd]
        if (subnetVal !== "" && subnetVal !== "NONE"){
          newState.vehicle_subnet = subnetVal
          newState.subnet_initialized = true
        }
      }
    }
    this.setState(newState)
  }

  // Subscribe to the system-config Settings status once the base namespace
  // is known (and resubscribe if it changes).
  updateSettingsListener(){
    const base_namespace = this.getBaseNamespace()
    if (base_namespace === null){
      return
    }
    const settingsNamespace = base_namespace + '/settings'
    if (this.state.settingsNamespace !== settingsNamespace){
      if (this.state.settingsListener != null){
        this.state.settingsListener.unsubscribe()
      }
      const listener = this.props.ros.setupSettingsStatusListener(
        settingsNamespace + '/status',
        this.settingsListener
      )
      this.setState({ settingsNamespace: settingsNamespace, settingsListener: listener })
    }
  }

  getSettingValue(name){
    const namesList = this.state.settingsNamesList
    const valuesList = this.state.settingsValuesList
    const ind = namesList.indexOf(name)
    if (ind !== -1){
      const value = valuesList[ind]
      // Treat an unset (NONE) config value as blank.
      if (value === "NONE"){
        return ""
      }
      return value
    }
    return ""
  }



  async checkConnection() {
    const { connectedToNepi } = this.props.ros
    if (this.state.connectedToNepi !== connectedToNepi){
      this.setState({connected: connectedToNepi})
    }
    this.updateSettingsListener()
    this.setState({needs_update: !this.state.needs_update})
    setTimeout(async () => {
      await this.checkConnection()
    }, 500)
  }

  
  componentDidMount(){
    this.checkConnection()
  }
    
  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.settingsListener != null){
      this.state.settingsListener.unsubscribe()
    }
    if (this.expandArmTimer !== null){
      clearTimeout(this.expandArmTimer)
      this.expandArmTimer = null
    }
    this.setState({connected: false, settingsListener: null})
  }

  apply_updates(){
    const base_namespace = this.getBaseNamespace()
    this.props.ros.sendTriggerMsg(base_namespace + "/update_system_config")
  }

  // Two-click confirmation for the Expand Storage Drive button.  The first
  // click arms the button and changes its label to Confirm Expand.  A second
  // click within 10 seconds sends the expand trigger.  If the second click
  // does not come, the button disarms back to its safe state.
  onExpandStorageDrive(){
    if (this.state.expand_armed === false){
      if (this.expandArmTimer !== null){
        clearTimeout(this.expandArmTimer)
      }
      this.expandArmTimer = setTimeout(() => {
        this.expandArmTimer = null
        this.setState({ expand_armed: false })
      }, 10000)
      this.setState({ expand_armed: true })
    }
    else {
      clearTimeout(this.expandArmTimer)
      this.expandArmTimer = null
      this.setState({ expand_armed: false })
      const base_namespace = this.getBaseNamespace()
      this.props.ros.sendTriggerMsg(base_namespace + "/expand_storage_drive")
    }
  }


  renderSystemConfig(show_controls){


      // Disable the expand button while a config update is running or while
      // the status text shows an expansion is requested or in progress.
      // const status_msg_str = (systemMgrStatus.nepi_update_msg ? systemMgrStatus.nepi_update_msg : "")

      const vehicle_subnet = this.state.vehicle_subnet
      // Live values read from the system config.
      const ip_alias_1 = this.getSettingValue('NEPI_ALIAS_IP_1')
      const hnav = this.getSettingValue('NEPI_NAV_IP_HNAV')
      const ntp = this.getSettingValue('NEPI_NTP_IP')
      const nmea = this.getSettingValue('NEPI_NAV_IP_NMEA')

      const update_gateway = this.state.update_gateway
      const current_gateway = this.getSettingValue('NEPI_GATEWAY_IP')

      return (


      <React.Fragment>


            <Label title={"Vehicle Subnet"}>
              <Input
                disabled={show_controls === false}
                id={"VehicleSubnet"}
                value={vehicle_subnet}
                onChange={this.onChangeVehicleSubnet}
                onKeyDown={this.onKeyVehicleSubnet}
                placeholder={"e.g. 192.168.179"}
              />
            </Label>

            <Label title={"IP_ALIAS_1"}>
              <Input disabled value={ip_alias_1} />
            </Label>

            <Label title={"HNAV"}>
              <Input disabled value={hnav} />
            </Label>

            <Label title={"NTP"}>
              <Input disabled value={ntp} />
            </Label>

            <Label title={"NMEA"}>
              <Input disabled value={nmea} />
            </Label>


            <Label title={"Gateway"}>
              <Input
                id={"Gateway"}
                value={update_gateway}
                onChange={this.onChangeGateway}
                onKeyDown={this.onKeyGateway}
                placeholder={"e.g. 10.0.0.1"}
              />
            </Label>

            <Label title={""}>
              <Input disabled value={current_gateway} />
            </Label>

           
    

        </React.Fragment>

      )

  }

  renderSystemAdvanced(){
      const base_namespace = this.getBaseNamespace()
      const systemMgrStatus = this.props.ros.systemMgrStatus
      const nepi_update_requested = systemMgrStatus.nepi_update_requested
      const nepi_updating_config = systemMgrStatus.nepi_updating_config
      // const nepi_service_running = systemMgrStatus.nepi_service_running
      // const hide_update_config = (nepi_update_requested === true || nepi_updating_config === true)
      // var update_message = systemMgrStatus.nepi_update_msg

      // if (nepi_updating_config === true ){
        // update_message = 'NEPI CONFIG UPDATING.  DO NOT POWER OFF SYSTEM'
      // }
      // else if (nepi_update_requested === true){
        // update_message = 'UPDATE REQUEST SENT.  WAITING FOR RESPONSE'
      // }

      // Disable the expand button while a config update is running or while
      // the status text shows an expansion is requested or in progress.
      const status_msg_str = (systemMgrStatus.nepi_update_msg ? systemMgrStatus.nepi_update_msg : "")
      const expand_in_progress = (status_msg_str === 'STORAGE EXPANSION REQUESTED' ||
                                  status_msg_str.indexOf('EXPANDING STORAGE') !== -1)
      const expand_disabled = (nepi_updating_config === true ||
                               nepi_update_requested === true ||
                               expand_in_progress === true)
      const expand_button_label = (this.state.expand_armed === true) ? "Confirm Expand" : "Expand Drive"

      return (


          <Section title={("Advanced Settings")}>



            <div style={{ display: 'flex' }}>
                    <div style={{ width: '30%' }}>


                      <ButtonMenu>
                          <Button
                            disabled={expand_disabled}
                            onClick={this.onExpandStorageDrive}>{expand_button_label}
                          </Button>
                      </ButtonMenu>

                    </div>

                    <div style={{ width: '30%' }} >

                   <ButtonMenu>
                          <Button
                            disabled={false}
                            onClick={() => this.props.ros.sendTriggerMsg(base_namespace + "/restart_nepi_software")}>{"Restart Software"}
                          </Button>
                      </ButtonMenu>

                    </div>

                    <div style={{ width: '30%' }} >
                   <ButtonMenu>
                          <Button
                            disabled={false}
                            onClick={() => this.props.ros.sendTriggerMsg(base_namespace + "/restart_nepi_container")}>{"Restart Container"}
                          </Button>
                      </ButtonMenu>


                    </div>
            </div>

                <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                      {this.state.expand_armed === true ? 'Click Confirm Expand to expand the storage drive' : ''}
                    </label>

        </Section>

      )

  }


    renderSystemNetlist(){
      const base_namespace = this.getBaseNamespace()
      const systemMgrStatus = this.props.ros.systemMgrStatus
      const {
        hearbeatNepi,
        systemStatusDiskUsageMB,
        systemStatusTempC,
        systemDefsDiskCapacityMB,
        //Unused diskUsagePercent
      } = this.props.ros
      const netlist = systemMgrStatus.netlist_str
      // const { wifi_query_response } = this.props.ros
      const internet_connected = (systemMgrStatus != null) ? systemMgrStatus.internet_connected : false
      const date_time_str = (systemMgrStatus != null) ? systemMgrStatus.date_time_str : ''
      return (


        <React.Fragment>


      <Section title={"System Status"}>
   <div style={{ display: 'flex' }}>
        <div style={{ width: '30%' }}>

        <Label title={"Heartbeat"}>
          <BooleanIndicator value={hearbeatNepi} />
        </Label>

                <Label title={"Date Time"}></Label>
        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {date_time_str}
        </label>  

        <Label title={"Temperature"}>
          <Input disabled value={roundWithSuffix(systemStatusTempC, 1, "\u00B0C")} />
        </Label>

        <Label title={"Capacity"}>
          <Input disabled value={roundWithSuffix(systemDefsDiskCapacityMB / 1000.0, 1, "GB")} />
        </Label>


        <Label title={"Used"}>
          <Input disabled value={roundWithSuffix(systemStatusDiskUsageMB / 1000.0, 1, "GB")} />
        </Label>

          </div>

            <div style={{ width: '30%' }} >


            </div>



            <div style={{ width: '30%' }} >


            </div>
        </div>

    <div style={{ display: 'flex' }}>
        <div style={{ width: '30%' }}>


        <Label title={"Internet Connected"}>
          <BooleanIndicator value={internet_connected} />
        </Label>

          </div>

            <div style={{ width: '30%' }} >

              <ButtonMenu>
                  <Button
                  disabled={false}
                  onClick={() => this.props.ros.sendTriggerMsg(base_namespace + "/connect_internet")}>{"Connect Internet"}
                </Button>
            </ButtonMenu>

            </div>



            <div style={{ width: '30%' }} >


            </div>
        </div>               

                <Label title={"Network Connections"}></Label>
                <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                  {netlist}
                </label> 
                                  
        </Section>



        </React.Fragment>

      )

  }




  render() {
    const base_namespace = this.getBaseNamespace()
    const systemMgrStatus = this.props.ros.systemMgrStatus
    const nepi_service_running = systemMgrStatus.nepi_service_running
    const admin_mode_set = this.props.ros.systemAdminModeSet

    const nepi_update_requested = systemMgrStatus.nepi_update_requested
    const nepi_updating_config = systemMgrStatus.nepi_updating_config
    const hide_update_config = (nepi_update_requested === true || nepi_updating_config === true)
    var update_message = systemMgrStatus.nepi_update_msg

    if (nepi_updating_config === true ){
      update_message = 'NEPI CONFIG UPDATING.  DO NOT POWER OFF SYSTEM'
    }
    else if (nepi_update_requested === true){
      update_message = 'UPDATE REQUEST SENT.  WAITING FOR RESPONSE'
    }


    const { userRestricted} = this.props.ros
    const admin_view_restricted = userRestricted.indexOf('SYSTEM-ADMIN-VIEW') !== -1  
    const show_admin = (admin_mode_set === true || admin_view_restricted === false)

    const show_controls = (admin_mode_set === true && nepi_service_running === true)
    
    if (systemMgrStatus == null || base_namespace == null || show_admin === false){
      return (
  
        <Columns>
          <Column>
  
  
          </Column>
        </Columns>
      )
    }

    else {

      return (

          <React.Fragment>
                               


            <div style={{ display: 'flex' }}>
                    <div style={{ width: '30%' }} >


                          <Section title={("NEPI System")}>
                            

                                  {<NepiIFAdminEnable
                                    make_section={false}
                                    title={null}
                                    show_link_button={false}
                                    show_line={false}
                                    />}

                                  {(nepi_service_running === false) ?
                                    <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                                      {'NEPI SERVICE NOT RUNNING'}
                                    </label> 
                                  : null }

                                  { this.renderSystemConfig(show_controls) }

                                <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


                                <div hidden={show_controls === false}>

                                        <div style={{ display: 'flex' }}>
                                                    <div style={{ width: '30%' }}>


                                                    <div hidden={ hide_update_config === true}>
                                                        <ButtonMenu>
                                                          <Button 
                                                            onClick={() => this.props.ros.sendTriggerMsg(base_namespace + "/update_system_config")}>{"Apply Updates"}
                                                          </Button>
                                                      </ButtonMenu>

                                                    </div>

                                                    </div>

                                                    <div style={{ width: '5%' }} >
                                                      {}
                                                    </div>

                                                    <div style={{ width: '65%' }} >
                                                        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                                                          {update_message}
                                                        </label>

                                                    </div>
                                            </div>

                                  </div>

                                  <NepiIFSettings
                                    settingsNamespace={base_namespace + '/settings'}
                                    show_controls={show_controls}
                                    make_section={false}
                                    title={"System Config"}
                                    />

                                  
                            </Section>


                                  { show_controls === true ? 
                                      this.renderSystemAdvanced()
                                  : null}

                    </div>


                    <div style={{ width: '5%' }} >
                    </div>

                    <div style={{ width: '65%' }} >


                        { admin_mode_set ? 
                            this.renderSystemNetlist()
                            : null}


                    </div>
            </div>

                  

          </React.Fragment>

     )
   }
  }


}
export default SystemMgr
