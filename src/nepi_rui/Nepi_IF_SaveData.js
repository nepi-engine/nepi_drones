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

import Section from "./Section"
//import EnableAdjustment from "./EnableAdjustment"
import Button, { ButtonMenu } from "./Button"
import BooleanIndicator from "./BooleanIndicator"
import Toggle from "react-toggle"
import AsyncToggle from "./AsyncToggle"
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Styles from "./Styles"
import Select, { Option } from "./Select"
import Input from "./Input"

import {onChangeSwitchStateValue, round} from "./Utilities"

import NepiIFConfig from "./Nepi_IF_Config"

function roundWithSuffix(value, decimals, suffix) {
  return value && (value.toFixed(decimals) + " " + suffix)
}

@inject("ros")
@observer

// Component that contains the Save Data Controls
class NepiIFSaveData extends Component {
  constructor(props) {
    super(props)

    // these states track the values through the components Save Data Status messages
    this.state = {
      saveNamespace: 'None',
      status_msg: null,


      lastSaveNamespace: 'None',
      saveRatesMsg: "",
      saveAllEnabled: false,
      saveAllRate: 0.0,
      saveDataPrefix: "",
      saveDataPrefixModified: false,
      saveDataSubfolder: "",
      saveDataSubfolderModified: false,
      saveUtcTz: false,
      exp_filename: "",
      saveDataEnabled: false,
      saveDataRate: "",
      saveDataRateModified: false,
      saveNamesList: [],
      saveRatesList: [],
      selectedDataProducts: [],
      selectedNamespace: null,

      saveAll: false,

      show_active_settings: true,

      showControls: false,

      updatedNamespace: null,
      saveStatusListener: null,

      needs_update: false
    }

    this.getAllNamespace = this.getAllNamespace.bind(this)
    this.updateSaveStatusListener = this.updateSaveStatusListener.bind(this)
    this.saveStatusListener = this.saveStatusListener.bind(this)

    this.createTopicOptions = this.createTopicOptions.bind(this)
    this.onChangeTopicSelection = this.onChangeTopicSelection.bind(this)

    this.renderSaveBar = this.renderSaveBar.bind(this)
    this.renderTopicSelector = this.renderTopicSelector.bind(this)
    this.renderSaveControls = this.renderSaveControls.bind(this)
    this.renderControlOptions = this.renderControlOptions.bind(this)


    this.updateSaveLists = this.updateSaveLists.bind(this)
    this.getSaveNamesList = this.getSaveNamesList.bind(this)
    this.getSaveRatesList = this.getSaveRatesList.bind(this)
    this.getSaveConfigString = this.getSaveConfigString.bind(this)
    this.getActiveConfigString = this.getActiveConfigString.bind(this)
    this.getSaveRateValue = this.getSaveRateValue.bind(this)
    this.getSaveRateData = this.getSaveRateData.bind(this)
    this.onClickToggleShowControls = this.onClickToggleShowControls.bind(this)


    this.getSaveDataValue = this.getSaveDataValue.bind(this)
    this.onChangeBoolSaveDataValue = this.onChangeBoolSaveDataValue.bind(this)
    this.onChangeBoolSaveAllValue = this.onChangeBoolSaveAllValue.bind(this)
    this.onChangeBoolUtcTzValue = this.onChangeBoolUtcTzValue.bind(this)
    this.onUpdateSaveDataRateValue = this.onUpdateSaveDataRateValue.bind(this)
    this.onKeySaveDataRateValue = this.onKeySaveDataRateValue.bind(this)
    this.onUpdateInputSaveDataPrefixValue = this.onUpdateInputSaveDataPrefixValue.bind(this)
    this.onKeySaveInputSaveDataPrefixValue = this.onKeySaveInputSaveDataPrefixValue.bind(this)
    this.onUpdateInputSaveDataSubfolderValue = this.onUpdateInputSaveDataSubfolderValue.bind(this)
    this.onKeySaveInputSaveDataSubfolderValue = this.onKeySaveInputSaveDataSubfolderValue.bind(this)
    this.onToggleDataProductSelection = this.onToggleDataProductSelection.bind(this)
    this.sendSaveRateUpdate = this.sendSaveRateUpdate.bind(this)
    
    this.updateSelectedDataProducts = this.updateSelectedDataProducts.bind(this)
    this.getSelectedDataProducts = this.getSelectedDataProducts.bind(this)
    this.getDiskUsageRate = this.getDiskUsageRate.bind(this)


    this.convertStrListToMenuList = this.convertStrListToMenuList.bind(this)
    
    this.doNothing = this.doNothing.bind(this)

    this.onSnapshotTriggered = this.onSnapshotTriggered.bind(this)


  }

  getAllNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var allNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      allNamespace = "/" + namespacePrefix + "/" + deviceId + '/save_data'
    }
    return allNamespace
  }

  // CAllback for handling ROS Status messages
  saveStatusListener(message) {
    if (message.save_data_topic === this.state.saveNamespace){
      this.setState({
        status_msg: message,
        saveRatesMsg: message.save_data_rates,
        saveAllEnabled: message.save_all_enabled,
        saveAllRate: message.save_all_rate,
        saveUtcTz: message.save_data_utc,
        exp_filename: message.example_filename,
        saveDataEnabled: message.save_data_enabled
      })

      if (!this.state.saveDataPrefixModified) {
        this.setState({ saveDataPrefix: message.filename_prefix })
      }
      if (!this.state.saveDataSubfolderModified) {
        this.setState({ saveDataSubfolder: message.save_subfolder })
      }
      if (!this.state.saveDataRateModified) {
        const rates = message.save_data_rates
        const active = rates && rates.find(r => r.save_rate_hz > 0)
        const displayRate = active ? String(round(active.save_rate_hz, 2))
                          : (message.save_all_rate > 0 ? String(round(message.save_all_rate, 2)) : "")
        this.setState({ saveDataRate: displayRate })
      }
      this.updateSaveLists()
      this.updateSelectedDataProducts()
    }
  }

  // Function for configuring and subscribing to Status
  updateSaveStatusListener(saveNamespace) {
    if (this.state.saveStatusListener != null) {
      this.state.saveStatusListener.unsubscribe()
      this.setState({saveStatusListener: null, 
                    status_msg: null})
    }
    if (saveNamespace !== '' &&  saveNamespace !== 'None'){
      var saveStatusListener = this.props.ros.setupSaveDataStatusListener(
            saveNamespace,
            this.saveStatusListener
          )
      this.setState({ saveNamespace: saveNamespace})
      this.setState({ saveStatusListener: saveStatusListener})
    }
  }


    componentDidMount() {
      this.setState({needs_update: true})
    }

  // Lifecycle method cAlled when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const allNamespace = this.getAllNamespace()
    const allSaveNamespace = (allNamespace != null) ? allNamespace: 'None'
    const saveNamespace =  (this.state.updatedNamespace != null) ? this.state.updatedNamespace :
                                  (this.props.saveNamespace !== undefined) ? 
                                    (this.props.saveNamespace !== '' && this.props.saveNamespace !== 'None' && this.props.saveNamespace !== null) ?
                                        this.props.saveNamespace : allSaveNamespace : allSaveNamespace
    const show_all_data_options = (this.props.show_all_data_options !== undefined) ? this.props.show_all_data_options : true

    const selectedNamespace = (this.state.selectedNamespace != null) ? this.state.selectedNamespace :
                                    (show_all_data_options === false) ? saveNamespace : allSaveNamespace
    const needs_update = ((this.state.saveNamespace !== saveNamespace || this.state.needs_update === true))
  
    if (needs_update) {
      this.setState({saveNamespace: saveNamespace,
                    selectedNamespace: selectedNamespace,
                    needs_update: false
      })
      this.updateSaveStatusListener(saveNamespace)
    }
  }

  // Lifecycle method cAlled just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.saveStatusListener) {
      this.state.saveStatusListener.unsubscribe()
    }
    this.setState({saveStatusListener: null, 
                  status_msg: null})
  }



  // Function for creating configs options list from status msg
  updateSaveLists() {
    var saveRatesMsg = this.state.saveRatesMsg
    var NamesList = []
    var RatesList = []
    var name = ""
    var rate_int = 0
    var rate_str = ""
    var saveRateMsg = null
    // add None and All options to lists
    NamesList.push("None")
    RatesList.push("0.0")
    NamesList.push("All")
    RatesList.push(this.state.saveDataRate)
    for (let ind = 0; ind < saveRatesMsg.length; ind++){
        saveRateMsg =saveRatesMsg[ind]
        name = saveRateMsg.data_product
        rate_int = saveRateMsg.save_rate_hz
        rate_str = rate_int.toFixed(2)
        NamesList.push(name)
        RatesList.push(rate_str)
    }
    this.setState({saveNamesList:NamesList})
    this.setState({saveRatesList:RatesList})
  }

  onClickToggleShowControls(){
    const currentVal = this.state.showControls 
    this.setState({showControls: !currentVal})
    this.render()
  }


  getSaveNamesList(){
    const list = this.state.saveNamesList
    return list
  }

  getSaveRatesList(){
    const list = this.state.saveRatesList
    return list
  }


  getSaveConfigString() {
    const {saveDataNamespaces , saveDataCaps} = this.props.ros
    const { namespacePrefix, deviceId} = this.props.ros
    const allNamespace = this.getAllNamespace()
    const saveNamespace = this.state.saveNamespace
    const isAllNamespace = (saveNamespace === allNamespace)
    var topic = ''
    var namesList = []
    var ratesList = []
    var configsStr = ""
    var entryStr
    var configsStrList = ['\n']
    var i = 0
    var i2 = 0
    var save_rates_list = []
    var shortname = ''
    if (isAllNamespace === true){
      for (i = 0; i < saveDataNamespaces.length; i++) {
        topic = saveDataNamespaces[i]
        namesList = []
        ratesList = []
        if (topic !== "None" && topic !== allNamespace ){
                save_rates_list = saveDataCaps[topic].save_data_rates
                if (save_rates_list !== undefined){
                  for (i2 = 0; i2 < save_rates_list.length; i2++) {
                      namesList.push(save_rates_list[i2].data_product)
                      ratesList.push(round(save_rates_list[i2].save_rate_hz,2))
                  }
                  shortname = topic.replace("/" + namespacePrefix + "/" + deviceId + '/','' ).replace('/save_data','')  
                  configsStrList.push(shortname + '    \n')
                  configsStrList.push('----------------------    \n')
                  for (let ind = 0; ind < namesList.length; ind++) {
                        entryStr = "  " + namesList[ind] + " : " + ratesList[ind] +  " Hz    \n"
                        configsStrList.push(entryStr)
                    
                  }
                  configsStrList.push('\n')
                }
        }
      }
    } 
    else {

      namesList = this.state.saveNamesList
      ratesList = this.state.saveRatesList

      for (let ind = 0; ind < namesList.length; ind++) {
        if (namesList[ind] !== "None" && namesList[ind] !== "All" ){
         
            entryStr = namesList[ind] + " : " + ratesList[ind] +  " Hz    \n"
            configsStrList.push(entryStr)
        
        }
      }
    }
    configsStr = configsStrList.join("")
    return configsStr
  }



  getActiveConfigString() {
    const {saveDataNamespaces , saveDataCaps} = this.props.ros
    const { namespacePrefix, deviceId} = this.props.ros
    const allNamespace = this.getAllNamespace()
    const saveNamespace = this.state.saveNamespace
    const isAllNamespace = (saveNamespace === allNamespace)
    var topic = ''
    var namesList = []
    var ratesList = []
    var configsStr = ""
    var entryStr
    var configsStrList = ['\n']
    var save_rates_list = {}
    var i = 0
    var i2 = 0
    var shortname
    if (isAllNamespace === true){
      for (i = 0; i < saveDataNamespaces.length; i++) {
        topic = saveDataNamespaces[i]
        namesList = []
        ratesList = []
        if (topic !== "None" && topic !== allNamespace ){
                save_rates_list = saveDataCaps[topic].save_data_rates
                if (save_rates_list !== undefined){
                  for (i2 = 0; i2 < save_rates_list.length; i2++) {
                      namesList.push(save_rates_list[i2].data_product)
                      ratesList.push(round(save_rates_list[i2].save_rate_hz,2))
                  }
                  shortname = topic.replace("/" + namespacePrefix + "/" + deviceId + '/','' ).replace('/save_data','')  
                  configsStrList.push(shortname + '    \n')
                  configsStrList.push('----------------------    \n')
                  for (let ind = 0; ind < namesList.length; ind++) {
                    if (ratesList[ind] > 0){
                        entryStr = "  " + namesList[ind] + " : " + ratesList[ind] +  " Hz    \n"
                        configsStrList.push(entryStr)
                    }
                    
                  }
                  configsStrList.push('\n')
                }
        }
      }
    } 
    else {

      namesList = this.state.saveNamesList
      ratesList = this.state.saveRatesList

      for (let ind = 0; ind < namesList.length; ind++) {
        if (namesList[ind] !== "None" && namesList[ind] !== "All" ){
          if (ratesList[ind] > 0){
            entryStr = namesList[ind] + " : " + ratesList[ind] +  " Hz    \n"
            configsStrList.push(entryStr)
          }
        }
      }
    }
    configsStr = configsStrList.join("")
    return configsStr
  }

  getSaveRateValue(dataName) {
    const configsStrList = this.getsaveRatesAsList()
    const configInd = configsStrList.indexOf(dataName)
    return configsStrList[configInd + 1]
  }

  getSaveRateData(dataName) {
    const configValue = this.getSaveRateValue(dataName)
    return parseFloat(configValue)
  }

  sendSaveRateUpdate(data_product,rate) {
    const {updateSaveDataRate}  = this.props.ros
    if (isNaN(rate) === false){
      updateSaveDataRate(this.state.saveNamespace,data_product,rate)
    }
  }

  updateSelectedDataProducts() {
    const NamesList = this.state.saveNamesList
    const RatesList = this.state.saveRatesList
    var selectedList = []
    var name = ""
    for (let ind = 0; ind < NamesList.length; ind++){
      name = NamesList[ind]
      if (name !== "None" && name !== "All"){
        if (parseFloat(RatesList[ind]) > 0) {
          selectedList.push(NamesList[ind])
        }
      }
    }
    this.setState({selectedDataProducts:selectedList})
  }

  getSelectedDataProducts(){
    const list =  this.state.selectedDataProducts
    return list
  }

  doNothing(){
    var ret = false
    return ret
  }
 
  onToggleDataProductSelection(event){
    const data_product = event.currentTarget.getAttribute("data-product")
    if (!data_product) {
      return
    }
    const saveDataRate = this.state.saveDataRate
    // A select click means "save this", so a blank or 0 Hz rate box falls back to 1 Hz --
    // publishing 0.0 on a select reads as a dead control (the next status drops it again).
    const parsedRate = parseFloat(saveDataRate)
    const rate = (isNaN(parsedRate) || parsedRate <= 0.0) ? 1.0 : parsedRate

    // 'All' and 'None' are SaveDataRate sentinels (ALL_DATA_PRODUCTS / NONE_DATA_PRODUCTS).
    // One publish sets every product's rate backend-side; the local list is set optimistically
    // and corrected by the next save status message.
    if (data_product === "All") {
      const allProducts = this.getSaveNamesList().filter(
        (name) => (name !== "None" && name !== "All"))
      this.sendSaveRateUpdate("All", rate)
      this.setState({selectedDataProducts: allProducts})
      return
    }
    if (data_product === "None") {
      this.sendSaveRateUpdate("None", 0.0)
      this.setState({selectedDataProducts: []})
      return
    }

    const selectedList = this.getSelectedDataProducts().slice()
    const ind = selectedList.indexOf(data_product)
    if (ind !== -1) {
      selectedList.splice(ind, 1)
      this.sendSaveRateUpdate(data_product, 0.0)
    } else {
      selectedList.push(data_product)
      this.sendSaveRateUpdate(data_product, rate)
    }
    this.setState({selectedDataProducts: selectedList})
  }

  getSaveDataValue(){
    const saveData = (this.state.saveDataEnabled === true)
    return saveData
  }

  onChangeBoolSaveDataValue(){
    const {sendBoolMsg}  = this.props.ros
    const saveAll = this.state.saveAll
    const allNamespace = this.getAllNamespace()
    const enabled = (this.state.saveDataEnabled === false)
    const saveDataRate = this.state.saveDataRate
    const rate = parseFloat(saveDataRate)
    if (saveAll === false) {
      this.sendSaveRateUpdate('Active',rate)
      sendBoolMsg(this.state.saveNamespace + '/save_data_enable',enabled)      
    }
    else {
      sendBoolMsg(allNamespace + '/save_data_enable',enabled)
    }

  }


  onChangeBoolSaveAllValue(){
    const {sendBoolMsg}  = this.props.ros
    const saveNamespace = this.state.saveNamespace
    const lastSaveNamespace = (this.state.lastSaveNamespace !== 'None') ? this.state.lastSaveNamespace : saveNamespace
    const saveAll = this.state.saveAll
    const enabled = (saveAll === false)
    const allNamespace = this.getAllNamespace()
    if (enabled === false ) {
      sendBoolMsg(allNamespace + '/save_data_enable',false)
      this.setState({lastSaveNamespace: lastSaveNamespace,
                    updatedNamespace: lastSaveNamespace
      })          
    }
    else {
      this.setState({lastSaveNamespace: saveNamespace,
                   updatedNamespace: allNamespace
      })             
    }
    
    this.setState({saveAll: enabled})
  }


  onChangeBoolUtcTzValue(e){
    const {sendBoolMsg}  = this.props.ros
    const enabled = e.target.checked
    sendBoolMsg(this.state.saveNamespace + '/save_data_utc',enabled)
  }


  onUpdateSaveDataRateValue(event) {
    this.setState({ saveDataRate: event.target.value, saveDataRateModified: true })
    document.getElementById("save_rate").style.color = Styles.vars.colors.red
  }

  onKeySaveDataRateValue(event) {
    if(event.key === 'Enter'){
      const rate = parseFloat(event.target.value)
      if (!isNaN(rate)){
        this.setState({ saveDataRateModified: false })
        this.sendSaveRateUpdate('Active',rate)
      }
      document.getElementById("save_rate").style.color = Styles.vars.colors.black
    }
  }




  onUpdateInputSaveDataPrefixValue(event) {
    this.setState({ saveDataPrefix: event.target.value, saveDataPrefixModified: true })
    document.getElementById("input_prefix").style.color = Styles.vars.colors.red
  }

  onKeySaveInputSaveDataPrefixValue(event) {
    const { sendStringMsg} = this.props.ros
    const key = event.key
    const value = event.target.value
    if(key === 'Enter'){
      this.setState({ saveDataPrefixModified: false })
      sendStringMsg(this.state.saveNamespace + '/save_data_prefix',value)
      document.getElementById("input_prefix").style.color = Styles.vars.colors.black
    }
  }

  onUpdateInputSaveDataSubfolderValue(event) {
    this.setState({ saveDataSubfolder: event.target.value, saveDataSubfolderModified: true })
    document.getElementById("input_subfolder").style.color = Styles.vars.colors.red
  }

  onKeySaveInputSaveDataSubfolderValue(event) {
    const { sendStringMsg} = this.props.ros
    const key = event.key
    const value = event.target.value
    if(key === 'Enter'){
      this.setState({ saveDataSubfolderModified: false })
      sendStringMsg(this.state.saveNamespace + '/save_data_subfolder',value)
      document.getElementById("input_subfolder").style.color = Styles.vars.colors.black
    }
  }
  

  convertStrListToMenuList(strList){
    var menuList = []
    for (let ind = 0; ind < strList.length; ind++){
      menuList.push(<Option>{strList[ind]}</Option>)
    } 
    return menuList
  }

  getDiskUsageRate(){
    const {systemStatusDiskRate} = this.props.ros
    return systemStatusDiskRate
  }


  onSnapshotTriggered(){
    // Requested live 2026-09-16: "make sure the bottom snapshot button works
    // with the same logic as the top one" -- an image viewer instance passes
    // its own saveImageFrame (captures the current canvas frame and downloads
    // it as a PNG in-browser) as onSnapshotOverride, replacing this method's
    // own default of publishing a ROS snapshot_trigger to the device's save-
    // data mechanism. Non-image callers of this component (this file is
    // generic, used for every data type's Save Data panel) never pass this
    // prop, so they keep the original device-side trigger unchanged.
    if (this.props.onSnapshotOverride !== undefined && this.props.onSnapshotOverride !== null) {
      this.props.onSnapshotOverride()
      return
    }
    const { sendTriggerMsg} = this.props.ros

    const saveAll = this.state.saveAll
    const allNamespace = this.getAllNamespace()
    if ((saveAll === false)) {
      sendTriggerMsg(this.state.saveNamespace + '/snapshot_trigger')
    }
    else {
      sendTriggerMsg(allNamespace + '/snapshot_trigger')
    }
  }

  renderSaveBar() {

    const saveDataEnabled = this.getSaveDataValue()
    const diskUsage = this.getDiskUsageRate()


    const show_topic_selector = (this.props.show_topic_selector !== undefined) ? this.props.show_topic_selector : true
    const allSaveNamespace = this.getAllNamespace()
    const saveNamespace = this.state.saveNamespace
    const is_all_namespace = (saveNamespace === allSaveNamespace)
    const allways_show_controls = (this.props.allways_show_controls !== undefined) ? this.props.allways_show_controls : false
    const showControls = (allways_show_controls === true) ? true : this.state.showControls
    
    const show_all_data_options = (this.props.show_all_data_options !== undefined) ? this.props.show_all_data_options : true

    const saveAll = this.state.saveAll
    const save_enabled = saveDataEnabled

    const { userRestricted} = this.props.ros
    const save_controls_restricted = userRestricted.indexOf('SYSTEM-SAVE-CONTROL') !== -1

    return (

      <React.Fragment> 
                    <div style={{ display: 'flex' }}>

                            


                        
                        <div style={{ width: '15%' }}>

                            <div hidden={(allways_show_controls === true || save_controls_restricted === true)}>
                                <Label title="Show Save Controls">
                                  {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                                  <Toggle
                                    checked={showControls===true}
                                    onClick={() => {this.onClickToggleShowControls()}}>
                                  </Toggle>
                              </Label>
                            </div>
                        </div>

                        <div style={{ width: '5%' }}>
                          {}
                        </div>


                    <div style={{ width: '30%' }}>

                        { (show_topic_selector === true) ?
                          this.renderTopicSelector()
                        : null }


                        </div>

                        <div style={{ width: '8%' }}>
                          {}
                        </div>


                        <div style={{ width: '10%' }}>

                                { ((show_all_data_options === true) && (is_all_namespace === true)) ?
                                <Label title={"All Enabled"}>
                                  <BooleanIndicator value={(true)} />
                                </Label>
                                  : null }

                                  { ((show_all_data_options === true) && (is_all_namespace === false)) ?
                                      <Label title="Enable All">
                                        {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                                        <Toggle
                                          checked={ (saveAll === true) }
                                          onClick={() => {this.onChangeBoolSaveAllValue()}}
                                        />
                                  </Label>

                                  : null }

                        </div>





                        <div style={{ width: '6%' }}>
                          {}
                        </div>



                        <div style={{ width: '10%' }}>
                          <Label title={"Enable Save"}>
                            <AsyncToggle
                              checked={ (save_enabled === true) }
                              onClick={() => {this.onChangeBoolSaveDataValue()}}
                            />
                          </Label>
                        </div>



                        {/* Requested live 2026-09-16: "move the enable all
                            and enable save dials a bit to the side" -- the
                            spacers on either side of Enable All/Enable Save
                            widened (8%/10%/6%/10%, was 12%/8%/3%/8%) so those
                            two toggles have breathing room instead of sitting
                            flush against the topic selector and Snapshot
                            button; Snapshot's own slot narrowed slightly
                            (19% -> 16%) to compensate. Total still 100%. */}
                        <div style={{ width: '16%' }}>

                       <ButtonMenu >
                        <Button onClick={this.onSnapshotTriggered}>{"Snapshot"}</Button>
                      </ButtonMenu>

                        </div>

                  </div>


      <div hidden={(save_enabled === false)}>

            <div style={{ display: 'flex' }}>

              <div style={{ width: '15%' }}>



                              <Label title={"Save Rate"}>
                                
                                <Input disabled value={roundWithSuffix(diskUsage, 3, "MB/s")} />
                              </Label>

              </div>



              <div style={{ width: '85%' }}>
                {}
              </div>

            </div>

        </div>


      </React.Fragment>
    )
  }



  createTopicOptions() {
    const { namespacePrefix, deviceId} = this.props.ros
    const show_all_data_options = (this.props.show_all_data_options !== undefined) ? this.props.show_all_data_options : true
    const save_include_filters = (this.props.save_include_filters !== undefined) ? this.props.save_include_filters : []
    const save_exclude_filters = (this.props.save_exclude_filters !== undefined) ? this.props.save_exclude_filters : []
    const allNamespace = this.getAllNamespace()
    var items = []
    //items.push(<Option value={"None"}>{"None"}</Option>)
    const saveData_topics = this.props.ros.saveDataNamespaces
    var shortname_split = []
    var shortname_base = ''
    var shortname_desc = ''
    var shortname = ''
    var topic = ""
    var include = true
    var i
    var i2
    var i3
    for (i = 0; i < saveData_topics.length; i++) {
      topic = saveData_topics[i]
      include = true
      if (topic !== allNamespace && topic.indexOf("None") === -1){
        for (i2 = 0; i2 < save_exclude_filters.length; i2++) {
          if (topic.indexOf(save_exclude_filters[i2]) !== -1 ){
            include = false
          }
        }
        for (i3 = 0; i3 < save_include_filters.length; i3++) {
          if (topic.indexOf(save_include_filters[i3]) === -1 ){
            include = false
          }
        }
        if (include === true){
          shortname_split = topic.replace("/" + namespacePrefix + "/" + deviceId + '/','' ).replace('/save_data','').split('/')
          shortname_base = shortname_split[0]
          shortname_desc = shortname_split[shortname_split.length - 1]
          shortname = shortname_base
          if (shortname_desc !== shortname){
            shortname = shortname + '-' + shortname_desc
          }
          items.push(<Option value={topic}>{shortname}</Option>)
        }
      }
    }
    if (items.length === 0){
      items.unshift(<Option value={"None"}>{"None"}</Option>)
    }
    if (show_all_data_options === true && items.length > 0){
      items.unshift(<Option value={allNamespace}>{"All"}</Option>)
    }
    return items
  }


  onChangeTopicSelection(event){
    this.setState({lastSaveNamespace: this.saveNamespace})
    const selNamespace = event.target.value
    this.setState({updatedNamespace: selNamespace,
                   selectedNamespace: selNamespace,
                   needs_update: true
    })
  }

  renderTopicSelector() {
    const saveTopics = this.createTopicOptions()
    const selectedNamespace = (this.state.selectedNamespace != null) ? this.state.selectedNamespace : this.state.saveNamespace
    return (
      <React.Fragment>

      <Columns>
        <Column>


          <Label title={"Save Select"}>
                <Select onChange={this.onChangeTopicSelection}
                id="topicSelecor"
                value={selectedNamespace}>
                  {saveTopics}
                </Select>
            </Label>

      </Column>
      </Columns>

      </React.Fragment>
    )
  }


  renderSaveControls() {
    const allNamespace = this.getAllNamespace()
    const saveNamespace = this.state.saveNamespace
    const saveDataEnabled = this.getSaveDataValue()
    const isAllNamespace = (saveNamespace === allNamespace)
    const dataProdcutSources = this.getSaveNamesList()
    const selectedDataProducts = this.getSelectedDataProducts()
    const diskUsage = this.getDiskUsageRate()
    const saveUtcTz = this.state.saveUtcTz
    const show_all_data_options = (this.props.show_all_data_options !== undefined) ? this.props.show_all_data_options : true
    const allSaveNamespace = (allNamespace != null) ? allNamespace: 'None'
    const is_all_namespace = (saveNamespace === allSaveNamespace)
    const show_active_settings = this.state.show_active_settings
    const all_str = (isAllNamespace === true) ? 'ALL '  : ''

    const show_save_controls = (this.props.show_save_controls !== undefined) ? this.props.show_save_controls : true
    const status_msg = this.state.status_msg
    const config_topic = (status_msg != null) ? status_msg.config_topic : 
            (is_all_namespace === true) ? allSaveNamespace : ''

      return (

      <React.Fragment>
        

                <label style={{fontWeight: 'bold'}}>
                  {saveNamespace}
                </label>
              
                <Columns>
                  <Column>


                          <Label title={"Set Save Rate " + all_str + "(Hz)"}>
                            <Input id="save_rate" 
                                value={this.state.saveDataRate} 
                                onChange={this.onUpdateSaveDataRateValue} 
                                onKeyDown= {this.onKeySaveDataRateValue} />
                        </Label>

                        { ((show_all_data_options === true) && (is_all_namespace === true)) ?
                        <Label title={"Set Save Prefix " + all_str}>
                          <Input id="input_prefix" 
                              value={this.state.saveDataPrefix} 
                              onChange={this.onUpdateInputSaveDataPrefixValue} 
                              onKeyDown= {this.onKeySaveInputSaveDataPrefixValue} />
                        </Label>
                        : null }

                        { ((show_all_data_options === true) && (is_all_namespace === true)) ?
                        <Label title={"Set Save Subfolder " + all_str}>
                          <Input id="input_subfolder" 
                              value={this.state.saveDataSubfolder} 
                              onChange={this.onUpdateInputSaveDataSubfolderValue} 
                              onKeyDown= {this.onKeySaveInputSaveDataSubfolderValue} />
                        </Label>
                        : null }

                       { ((show_all_data_options === true) && (is_all_namespace === true)) ?
                        <Label title={"Use UTC Time " + all_str}>
                            <AsyncToggle
                              checked={ (saveUtcTz) }
                              onClick={this.onChangeBoolUtcTzValue}
                            />
                          </Label>
                       : null }

                  <div  hidden={isAllNamespace === true}>
                        <Label title={"Selected Message Topics"}>
                          <div onClick={this.doNothing} style={{backgroundColor: Styles.vars.colors.grey0}}>
                            <Select style={{width: "10px"}}/>
                          </div>

                            {dataProdcutSources.map((data_product) =>
                            <div onClick={this.onToggleDataProductSelection}
                                data-product={data_product}
                                style={{
                                    textAlign: "center",
                                    padding: `${Styles.vars.spacing.xs}`,
                                    color: Styles.vars.colors.black,
                                    backgroundColor: (selectedDataProducts.includes(data_product))? Styles.vars.colors.blue : Styles.vars.colors.grey0,
                                    cursor: "pointer",
                                  }}>
                                  <body style={{color: Styles.vars.colors.black}}>{data_product}</body>
                            </div>
                            )}
                        </Label>

                  </div>



              

                  </Column>
                  <Column>

                        <Label title={"Saving"}>
                          <BooleanIndicator value={(saveDataEnabled === true)} />
                        </Label>

                          <Label title={"Save Rate"}>
                            
                            <Input disabled value={roundWithSuffix(diskUsage, 3, "MB/s")} />
                          </Label>


                      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                        <Label title="Filter Active">
                            {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                            <Toggle
                              checked={show_active_settings===true}
                              onClick={() => onChangeSwitchStateValue.bind(this)("show_active_settings",show_active_settings)}>
                            </Toggle>
                        </Label>

                  <div  hidden={show_active_settings === true}>



                        {/* <Label title={"All Data Save Settings"}>
                        </Label> */}

                        <pre style={{ height: "400px", overflowY: "auto" }}>
                          {this.getSaveConfigString()}
                        </pre>

              </div>


                  <div  hidden={show_active_settings === false}>


                        {/* <Label title={"Active Data Save Settings"}>
                        </Label> */}

                        <pre style={{ height: "200px", overflowY: "auto" }}>
                          {this.getActiveConfigString()}
                        </pre>

              </div>


                  </Column>
                </Columns>



                <div align={"left"} textAlign={"left"} hidden={saveNamespace === 'None' || isAllNamespace === true}>


                {(show_save_controls === true && config_topic !== '') ?
                <NepiIFConfig
                    namespace={config_topic}
                    title={"Nepi_IF_Config"}
                />
                : null }


                </div>




              <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


         


        </React.Fragment>
      )

  }


 renderControlOptions() {
    const {sendBoolMsg}  = this.props.ros
    const allNamespace = this.getAllNamespace()
    const saveNamespace = this.state.saveNamespace
    const status_msg = this.state.status_msg
    const disabled = (status_msg != null) ? status_msg.disabled : true
    const allow_enable = (this.props.allow_enable !== undefined) ? this.props.allow_enable : true

    const allways_show_controls = (this.props.allways_show_controls !== undefined) ? this.props.allways_show_controls : false
    const showControls = (allways_show_controls === true) ? true : this.state.showControls
    const allSaveNamespace = (allNamespace != null) ? allNamespace: 'None'
    const selectedNamespace = (this.state.selectedNamespace != null) ? this.state.selectedNamespace : this.state.saveNamespace
    const is_all_namespace = (saveNamespace === allSaveNamespace) || (selectedNamespace === allSaveNamespace)
    const { userRestricted} = this.props.ros
    const save_controls_restricted = userRestricted.indexOf('SYSTEM-SAVE-CONTROL') !== -1

    var disable_topic = '/disable'
    if (is_all_namespace){
      disable_topic = '/saving_disable'
    }
    if (saveNamespace === 'None' || showControls === false || save_controls_restricted === true){
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

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


        <div style={{ display: 'flex' }}>

          <div style={{ width: '30%' }}>
            { ((is_all_namespace === false) ) ?
            <Label title={"Enabled"}>
                <AsyncToggle
                  disable={allow_enable === false}
                  checked={ (disabled === false) }
                  onClick={() => sendBoolMsg(saveNamespace + disable_topic,!disabled)}
                />
              </Label>
            : null }

          </div>

          <div style={{ width: '70%' }}>
            {}
          </div>

        </div>

        <div style={{ display: 'flex' }}>

          <div style={{ width: '65%' }}>
   
          { ((is_all_namespace === true || disabled === false)) ?
            this.renderSaveControls()
          : null }
          </div>

          <div style={{ width: '35%' }}>
            {}
          </div>

        </div>

        </React.Fragment>
      )
    }
 }




  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const saveNamespace = this.state.saveNamespace
    const { userRestricted} = this.props.ros
    const save_view_restricted = userRestricted.indexOf('SYSTEM-SAVE-VIEW') !== -1


    if (saveNamespace === 'None' || save_view_restricted === true){
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )
    }
    else if (make_section === false){
      return (
        <React.Fragment>
        {this.renderSaveBar()}
         {this.renderControlOptions()}
        </React.Fragment>
      )
    }
    else {
      return (

      <Section>

        {this.renderSaveBar()}
        {this.renderControlOptions()}

      </Section>
      )

    }
  }


}

export default NepiIFSaveData
