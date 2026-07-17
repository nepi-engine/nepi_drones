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
import { observable, action } from "mobx"
import moment from "moment"
import ROS from "roslib"
import yaml from "js-yaml"

const ROS_WS_URL = `ws://${window.location.hostname}:9090`
//const FLASK_URL = `http://${window.location.hostname}:5003`
const LICENSE_SERVER_WS_URL = `ws://${window.location.hostname}:9092`

const TRIGGER_MASKS = {
  OUTPUT_ENABLED: 0xffffffff,
  DEFAULT: 0x7fffffff
}

const NEPI_TIMEOUT_MSEC = 3000


// TODO: Would be better to query the display_name property of all nodes to generate
// this dictionary... requires a new SDKNode service to do so
const NODE_DISPLAY_NAMES = {
  config_mgr: "Config Manager",
  nav_pose_mgr: "Nav./Pose/GPS",
  network_mgr: "Network",
  ai_detector_mgr: "Classifier",
  system_mgr: "System",
  time_mgr: "Time Sync",
  trigger_mgr: "Triggering",
  nepi_link_ros_bridge: "NEPI Connect",
  gpsd_ros_client: "GPSD Client",
  illumination_mgr: "Illumination",
  scripts_mgr: "Scripts",
  app_image_sequencer: "Sequencer"
}

const UPDATE_PERIOD = 100 // ms between sending updates

let _ruiCryptoKey = null

function displayNameFromNodeName(node_name) {
  var display_name = NODE_DISPLAY_NAMES[node_name]
  if (display_name) {
    return display_name
  }
  return node_name
}

function nodeNameFromDisplayName(display_name) {
  for( var node_name in NODE_DISPLAY_NAMES ) {
    if (NODE_DISPLAY_NAMES[node_name] === display_name) {
      return node_name
    }
  }
  // Don't return anything if we don't find the display name -- callers can check for undefined
}

export { TRIGGER_MASKS, displayNameFromNodeName, nodeNameFromDisplayName }

/*
async function apiCall(endpoint) {
  try {
    const r = await fetch(`${FLASK_URL}/api/${endpoint}`, {
      method: "GET"
    })
    const json = await r.json()
    return json
  } catch (err) {
    console.error(err)
  }
}
*/

/*
// gets a file through the flask api and parses it as Json
async function getFileJson(filename) {
  try {
    const r = await fetch(`${FLASK_URL}/files/${filename}`, {
      method: "GET"
    })
    const json = await r.json()
    return json
  } catch (err) {
    console.error(err)
    return null
  }
}
*/

function getLocalTZ() {
  return Intl.DateTimeFormat().resolvedOptions().timeZone
}

//////////////////////////////////////////////////////////////
// from: https://stackoverflow.com/questions/7837456/how-to-compare-arrays-in-javascript
// Warn if overriding existing method
if (Array.prototype.equals)
  console.warn(
    "Overriding existing Array.prototype.equals. Possible causes: New API defines the method, there's a framework conflict or you've got double inclusions in your code."
  )
// attach the .equals method to Array's prototype to call it on any array
// eslint-disable-next-line no-extend-native
Array.prototype.equals = function(array) {
  // if the other array is a falsy value, return
  if (!array) return false

  // compare lengths - can save a lot of time
  if (this.length !== array.length) return false

  for (var i = 0, l = this.length; i < l; i++) {
    // Check if we have nested arrays
    if (this[i] instanceof Array && array[i] instanceof Array) {
      // recurse into the nested arrays
      if (!this[i].equals(array[i])) return false
    } else if (this[i] !== array[i]) {
      // Warning - two different object instances will never be equal: {x:20} != {x:20}
      return false
    }
  }
  return true
}
// Hide method from for-in loops
// eslint-disable-next-line no-extend-native
Object.defineProperty(Array.prototype, "equals", { enumerable: false })
//////////////////////////////////////////////////////////////

class ROSConnectionStore {

  @action.bound
  resetStates(){
      this.ros = null
      this.rosCheckStarted = false
      this.rosAutoReconnect = false
      this.connectedToRos = false

      this.connectedToNepi = false

      this.checkTopicsServices = false
      
      this.topicNames = []
      this.topicTypes = []
      this.serviceNames = []


      //////// System Mgr
      this.connectedToSystemMgr = false
      this.systemMgrStatus = null
      this.systemStatusTopics = null
      this.systemStatusTopicTypes = null
      this.systemStatusServices = null

      this.systemHwType = null
      this.systemHwModel = null
      this.systemInContainer = false
      this.systemManagesSSH = false
      this.systemManagesSHARE = false
      this.systemManagesTime = false
      this.systemManagesNetwork = false

      this.systemStatusDiskUsageMB = null
      this.systemStatusDiskRate = null
      this.systemStatusTempC = null
      this.systemStatusWarnings = []

      this.systemDefsFirmwareVersion = null
      this.systemDefsDiskCapacityMB = null

      this.diskUsagePercent = null

      this.saveFreqHz = null

      this.triggerStatus = false
      this.triggerAutoRateHz = 0
      this.triggerMask = TRIGGER_MASKS.DEFAULT



      this.systemAdminEnabled = false
      this.systemAdminPasswordValid = false
      this.systemAdminModeSet = false

      this.systemDebugEnabled = false

      this.managers_list = []
      this.managers_active_list = []
      this.managers_running_list = []


      this.userRestrictionOptions = []
      this.userRestrictions = []
      this.userRestricted = []

      this.userLoginEnabled = false
      this.userLoginPasswordValid = false
      this.userLoginModeSet = false

      this.systemRunModeOptions = []
      this.systemRunMode = null


      // App Manager
      this.connectedToAppsMgr = false
      this.appsMgrStatus = null

      this.apps_list = []
      this.apps_name_list = []
      this.apps_group_list = []
      this.apps_status_list = []
      
      this.apps_active_list = []

      this.apps_running_list = []
      this.apps_running_name_list = []
      this.apps_running_group_list = []


      // Time Manager
      this.connectedToTimeMgr = false
      this.timeMgrStatus = null

      this.timeStatusTime = null
      this.timeStatusTimeStr = null
      this.timeStatusTimezone = null
      this.timeStatusDateStr = null
      this.clockUTCMode = false
      this.available_timezones = []

      this.clockTZ = this.get_timezone_desc()
      this.clockNTP = false
      this.ntp_sources = []
      this.clockPPS = false


      // Drivers Manager
      this.connectedToDriversMgr = false
      this.driversMgrStatus = null

      this.drivers_list = []
      this.drivers_name_list = []
      this.drivers_type_list = []
      this.drivers_group_id_list = []
      this.drivers_status_list = []

      this.drivers_active_list = []
      this.drivers_active_name_list = []
      this.drivers_active_type_list = []

      this.drivers_running_list = []

      this.drivers_retry_enabled = []


      this.devices_name_list = []
      this.devices_alias_list = []

      this.devices_running_name_list = []
      this.devices_running_type_list = []

      // AI Model Manager
      this.connectedToAiModelsMgr = false
      this.aiModelsMgrStatus = null


      this.ai_frameworks_list = []
      this.ai_frameworks_folder_list = []
      this.ai_frameworks_active_list = []

      this.ai_models_ordered_list = []
      this.ai_models_ordered_name_list = []
      this.ai_models_ordered_framework_list = []
      this.ai_models_ordered_type_list = []
      this.ai_models_ordered_status_list = []


      this.ai_models_active_list = []


      this.ai_models_running_list = []
      this.ai_models_running_name_list = []
      this.ai_models_running_type_list = []
      this.ai_models_running_namespace_list = []


      // Software Manager
      this.connectedToSoftwareMgr = false
      this.softwareMgrStatus = null

      // Navpose Manager
      this.connectedToNavposeMgr = false
      this.navposeModelsMgrStatus = null

      this.navpose_save_data_topic = ''

      this.navposes_max_pub_rate = null

      this.frame_nav_options = []
      this.frame_nav = null
      this.frame_alt_options = []
      this.frame_alt = null
      this.frame_depth_options = []
      this.frame_depth = null

      this.navpose_sync_transforms = false
      this.navpose_frames = []
      this.navpose_frames_topics = []
      this.navpose_frames_solutions = []      


  }


  //////////////////////////////
  // ROS
  //////////////////////////////


  @observable ros = null
  @observable rosCheckDelay = 1000
  @observable rosQueryLock = false
  @observable connectedToRos = false
  @observable rosCheckStarted = false
  @observable rosAutoReconnect = true
  @observable messageLog = ""

  @observable topicQueryLock = false
  @observable checkTopicsServices = false
  @observable topicNames = null
  @observable topicNamesLast = null
  @observable topicTypes = null
  @observable topicTypesLast = null
  @observable serviceNames = null
  @observable serviceNamesLast = null

  rosListeners = []


  async checkROSConnection() {
    var update_time = 100
    if (this.rosQueryLock === false){
      this.rosQueryLock = true
      if (!this.connectedToRos) {
        try {
          // setup rosbridge connection
          if (this.ros == null) {
            this.rosCheckStarted = true
            this.ros = new ROS.Ros({
              url: ROS_WS_URL
            })
            this.ros.on("connection", this.onConnectedToROS)
            this.ros.on("error", this.onErrorConnectingToROS)
            this.ros.on("close", this.onDisconnectedToROS)
          }
          else {
            this.ros.connect(ROS_WS_URL)
            this.checkTopicsServices = true
            this.topicQueryLock = false
            this.updateTopicsServices()
          }


        
        } catch (e) {
          //console.error(e)
          // errored
        }
      }

      if (this.connectedToRos === true && this.connectedToNepi === false) {
          this.watchdogNepiCounter = 0
          this.checkTopicsServices = true
          this.topicQueryLock = false
          this.updateTopicsServices()
      }

      if (this.ros != null ) {
        update_time = (this.connectedToNepi === false) ? 3000 : NEPI_TIMEOUT_MSEC
        if (this.connectedToNepi === true && this.watchdogNepiCounter >= this.watchdogNepiMax ) {
            this.connectedToNepi = false
            this.destroyROSConnection()
        }
        this.watchdogNepiCounter++
      }
      this.rosQueryLock = false
    }

    if (this.rosAutoReconnect) {
      setTimeout(async () => {
        await this.checkROSConnection()
      }, update_time)
    }
    
  }

  @action.bound
  async updateTopicsServices() {

    var update_time = 100
    if (this.topicQueryLock === false){
      this.topicQueryLock = true
      if ((this.ros != null) && (this.connectedToRos === true)) {
        this.topicQueryLock = true

        // Update Topics and Services
        if (this.systemStatusTopics != null && this.systemStatusTopicTypes != null ){
              update_time = 100
              this.topicNames = this.systemStatusTopics
              this.topicTypes = this.systemStatusTopicTypes
        }
        else {
          update_time = 1000
          try {
            this.ros.getTopics(result => {
                this.topicNames = result.topics
                this.topicTypes = result.types
                })
          } catch (error) {
              this.topicNames = []
              this.topicTypes = []
          }
        }

        if (this.systemStatusServices != null){
              update_time = 100
              this.serviceNames = this.systemStatusServices
        }
        else{
          this.serviceNames = []
        }
        // else {
        //   try {
        //     this.ros.getServices(result => {
        //       update_time = 2000
        //       this.serviceNames = result
        //     })
        //   } catch (error) {
        //     this.serviceNames = []
        //   }
        // }


        const topicNames = (this.topicNamesLast != null) ? this.topicNamesLast : []
        const topicTypes = (this.topicTypesLast != null) ? this.topicTypesLast : []
        const serviceNames = (this.serviceNamesLast != null) ? this.serviceNamesLast : []

        if (this.topicNames != null && this.topicTypes != null && this.serviceNames != null){
          if (this.topicNames.length !== topicNames.length ||
              this.topicTypes.length !== topicTypes.length ||
              this.serviceNames.length !== serviceNames.length) {

                update_time = 2000
                var newPrefix = this.updatePrefix(this.topicNames, this.topicTypes)
                var newResetTopics = this.updateResetTopics(this.topicNames, this.topicTypes)
                var newSaveDataNamespaces = this.updateSaveDataNamespaces(this.topicNames, this.topicTypes)
                var newAiDetectorNamespaces = this.updateAiDetectorNamespaces(this.topicNames, this.topicTypes)
                var newImageTopics = this.updateImageTopics(this.topicNames, this.topicTypes)
                var newMessageTopics = this.updateMessageTopics(this.topicNames, this.topicTypes)
                var newPointcloudTopics = this.updatePointcloudTopics(this.topicNames, this.topicTypes)
                this.updateIDXDevices(this.topicNames, this.topicTypes)
                this.updatePTXDevices(this.topicNames, this.topicTypes)
                this.updateLXSDevices(this.topicNames, this.topicTypes)
                this.updateRBXDevices(this.topicNames, this.topicTypes)
                this.updateNPXDevices(this.topicNames, this.topicTypes)        

                if (newPrefix === true){
                  this.setupMgrSystemStatusListener()
                  this.setupRUISettingsListener()    // services
                }

                if ((this.connectedToNepi === true) && (newPrefix || newResetTopics || newAiDetectorNamespaces || newSaveDataNamespaces || newMessageTopics || newImageTopics || newPointcloudTopics)) {
                  this.initializeSystemListeners()
                }

              }
        }
      
        this.topicNamesLast = this.topicNames
        this.topicTypesLast = this.topicTypes
        this.serviceNamesLast = this.serviceNames

      }
      this.topicQueryLock = false
    }
    if (this.checkTopicsServices === true) {
      setTimeout(async () => {
        await this.updateTopicsServices()
      }, update_time)
    }
  }

  validPrefix() {
    return this.namespacePrefix && this.deviceId
  }



  @action.bound
  destroyROSConnection() {
    if (this.ros != null){
      //this.rosAutoReconnect = false
      this.ros.off("connection", this.onConnectedToROS)
      this.ros.off("error", this.onErrorConnectingToROS)
      this.ros.off("close", this.onDisconnectedToROS)

      this.ros = null
      this.rosListeners.forEach(listener => {
        listener.unsubscribe()
      })
      this.resetStates()
    }
  }



  @action.bound
  rosLog(text) {
    this.messageLog = `${text}\n${this.messageLog}`
  }

  get rosPrefix() {
    return `/${this.namespacePrefix}/${this.deviceId}`
  }

  publishMessage({ name, messageType, data, noPrefix = false }) {
    const publisher = new ROS.Topic({
      ros: this.ros,
      name: noPrefix ? name : `${this.rosPrefix}/${name}`,
      messageType
    })
    const message = new ROS.Message(data)
    publisher.publish(message)
  }

  addListener({
    name,
    messageType,
    callback,
    noPrefix = false,
    manageListener = false
  }) {
    const listener = new ROS.Topic({
      ros: this.ros,
      name: noPrefix ? name : `${this.rosPrefix}/${name}`,
      messageType
    })
    listener.subscribe(action(callback))

    // add to listeners that get unsubscribed
    if (manageListener) {
      this.rosListeners.push(listener)
    }

    // return listener for clients that manage their own
    return listener
  }

  callService({ name, messageType, args = {}, msgKey = null }) {
    const name_check = (name !== '')
    const valid_name = (name != null && name_check === true)
    if (valid_name === false || this.connectedToNepi === false){
      return null
    }
    else {
      const service_namespace = name.startsWith('/')? name : `${this.rosPrefix}/${name}`
      if ( this.serviceNames.indexOf(service_namespace) !== -1) {

        try {
                return new Promise(resolve => {
                  const client = new ROS.Service({
                    ros: this.ros,
                    name: service_namespace,
                    serviceType: messageType
                  })
                  const request = new ROS.ServiceRequest(args)
                  client.callService(
                    request,
                    action(result => {
                      resolve(msgKey ? result[msgKey] : result)
                    })
                  )
                })
        }
        catch (e) {
          return null
        }
      }
      else{
        return null
      }
    }
  }

  @action.bound
  onConnectedToROS() {
    this.connectedToRos = true
    this.rosLog("Connected Ros")
    this.checkLicense()
  }

  @action.bound
  initializeSystemListeners() {


    this.startPollingIPAddrQueryService()
    this.startPollingBandwidthUsageService()
    this.startPollingWifiQueryService()
    this.setupMgrSoftwareStatusListener()
    this.startPollingTimeMgrStatusService()
    this.setupDriversMgrStatusListener()
    this.setupAppsMgrStatusListener()
    this.setupAiModelsMgrStatusListener()
    this.setupNavposeStatusListener()
    
    // scripts manager services
    this.startPollingGetScriptsService()  // populate listbox with files
    this.startPollingGetRunningScriptsService()  // populate listbox with active files
    //this.startPollingLaunchScriptService() // invoke script execution
    //this.startPollingStopScriptService() // stop script execution
    //this.callGetSystemStatsQueryService() // get script and system status

    // sequential image mux services
    //this.callMuxSequenceQuery(true) // Start it polling


  }

  @action.bound
  onErrorConnectingToROS() {
    this.connectedToRos = false
    this.rosLog("Error connecting to NEPI device, retrying")
  }

  @action.bound
  onDisconnectedToROS() {
    this.connectedToRos = false

    this.namespacePrefix = null
    this.deviceType = null
    this.deviceId = null
    this.deviceSerial = null

    this.rosLog("Connection to NEPI device closed")
  }




  //////////////////////////////
  // NEPI 
  //////////////////////////////


  @observable connectedToNepi = false
  @observable hearbeatNepi = false
  @observable watchdogNepiMax = 5
  @observable watchdogNepiCounter = 0



  @observable namespacePrefix = null
  @observable deviceType = null
  @observable deviceId = null
  @observable deviceSerial = null
  @observable deviceInWater = false

  @observable navPoseTopics = []
  @observable navPoseCaps = {}
  @observable imageTopics = []
  @observable imageCaps = {}
  @observable imageDetectionTopics = []
  @observable depthMapTopics = []
  @observable depthMapCaps = {}
  @observable pointcloudTopics = []
  @observable pointcloudCaps = {}
  @observable settingCaps = {}
  @observable saveDataNamespaces = []
  @observable saveDataCaps = {}

  @observable resetTopics = []

  @observable aiDetectorNamespaces = []
  @observable aiDetectorCaps = {}


  @observable navSatFixTopics = []
  @observable orientationTopics = []
  @observable headingTopics = []
  @observable messageTopics = []

  @observable imageFilterDetection = null
  @observable imageFilterSequencer = null
  @observable imageFilterPTX = null
  @observable targLocalizerImgTopic = null
  
  @observable lastUpdate = new Date()

  @observable idxDevices = {}
  @observable ptxDevices = {}
  @observable lsxDevices = {}
  @observable rbxDevices = {}
  @observable npxDevices = {}



  //////////////////////////////
  // License Manager
  //////////////////////////////

  @observable license_server = null
  @observable license_valid = true // Default to true to avoid initial DEVELOPER message]
  @observable license_key = ''
  @observable hardware_key = ''

  @observable license_type = 'Unlicensed'
  @observable license_status = 'No License Found'
  @observable license_info = null
  @observable license_request_info = null
  @observable license_request_mode = false



  
  async checkLicense() {
    var retry_delay_ms = 3000
    if (!this.license_server) {
      try {
        this.license_server = new WebSocket(LICENSE_SERVER_WS_URL)

        this.license_server.onmessage = (event) => {
          var response_dict = yaml.load(event.data)         
          
          if ('licensed_components' in response_dict)
          {
            this.license_info = yaml.load(event.data)
            this.license_key = this.license_info['licensed_components']['nepi_base']['hardware_key']
            this.hardware_key = this.license_info['licensed_components']['hardware_key']
            this.license_type = this.license_info['licensed_components']['nepi_base']['commercial_license_type']
            this.license_status = this.license_info['licensed_components']['nepi_base']['status']
            if ( this.license_type === 'Unlicensed') {
              this.license_valid = false
            }
            else {
              if (this.license_request_mode && !this.license_valid) {
                this.license_request_mode = false
              }
              this.license_valid = true
            }
          }

          else if ('license_request' in response_dict)
          {
            this.license_request_info = yaml.load(event.data)
          }
        }

        retry_delay_ms = 250 // Fast to avoid a lot of latency while connecting
      } catch (e) {
        // Note: Failure to contact the server does not result in this exception, instead
        // an event is dispatched, so we don't get into this block just because the
        // server isn't present: 
        // https://stackoverflow.com/questions/31002592/javascript-doesnt-catch-error-in-websocket-instantiation
        //console.error(e)
        console.error("License server not running")
        this.license_server = null
        this.license_yaml = null
        this.license_valid = false 
        this.license_info = null
        this.license_request_info = null
        this.license_request_mode = false
      }
    }

    else if (this.license_server.readyState === 1) { // READY
      // Check for license updates
      this.license_server.send("license_check") 
      retry_delay_ms = 5000 // Slow down the updates now that we are connected
    }

    else if (this.license_server.readyState === 3) { // CLOSED
      this.license_server = null
      this.license_valid = false
      this.license_info = null
      this.license_request_info = null
      this.license_request_mode = false
    }

    setTimeout(async () => {
      await this.checkLicense()
    }, retry_delay_ms)
  }

  //////////////////////////////
  // SYSTEM MGR
  //////////////////////////////

  @observable connectedToSystemMgr = false
  @observable systemMgrStatus = null

  @observable systemStatusTopics = null
  @observable systemStatusTopicTypes = null
  @observable systemStatusServices = null
  @observable systemHwType = null
  @observable systemHwModel = null
  @observable systemInContainer = false
  @observable systemManagesSSH = false
  @observable systemManagesSHARE = false
  @observable systemManagesTime = false
  @observable systemManagesNetwork = false

  @observable systemStatusDiskUsageMB = null
  @observable systemStatusDiskRate = null
  @observable systemStatusTempC = null
  @observable systemStatusWarnings = []

  @observable systemDefsFirmwareVersion = null
  @observable systemDefsDiskCapacityMB = null

  @observable diskUsagePercent = null

  @observable saveFreqHz = null

  @observable triggerStatus = false
  @observable triggerAutoRateHz = 0
  @observable triggerMask = TRIGGER_MASKS.DEFAULT



  @observable systemAdminEnabled = false
  @observable systemAdminPasswordValid = false
  @observable systemAdminModeSet = false

  @observable systemDebugEnabled = false

  @observable managers_list = []
  @observable managers_active_list = []
  @observable managers_running_list = []


  @observable systemRunModeOptions = []
  @observable systemRunMode = null

  @observable userRestrictionOptions = []
  @observable userRestrictions = []
  @observable userRestricted = []



  @observable userLoginEnabled = false
  @observable userLoginPasswordValid = false
  @observable userLoginModeSet = false






  @action.bound
  setupMgrSystemStatusListener() {
    this.addListener({
      name: "status",
      messageType: "nepi_interfaces/MgrSystemStatus",
      manageListener: true,
      callback: message => {
        


        this.connectedToSystemMgr = true
        this.systemMgrStatus = message

        this.systemStatusTopics = message.active_topics
        this.systemStatusTopicTypes = message.active_topic_types
        this.systemStatusServices = message.active_services
        this.systemHwType = message.hw_type
        this.systemHwModel = message.hw_model
        this.systemInContainer = message.in_container
        this.systemManagesSSH = message.manages_ssh
        this.systemManagesSHARE = message.manages_share
        this.systemManagesTime = message.manages_time
        this.systemManagesNetwork = message.manages_network
        
        this.systemStatusDiskUsageMB = message.disk_usage
        this.systemStatusDiskRate = message.storage_rate
        this.deviceType = message.hw_type
        this.deviceSerial = message.serial_number
        this.systemDefsFirmwareVersion = message.firmware_version
        this.systemDefsDiskCapacityMB = message.disk_capacity

        
        this.diskUsagePercent = `${parseInt(
          100 * this.systemStatusDiskUsageMB / this.systemDefsDiskCapacityMB,
          10
        )}%`

        this.systemStatusTempC =
          message.temperatures.length && message.temperatures[0]
        this.systemStatusWarnings = message.warnings && message.warnings.flags
        //this.rosLog("Received Status Message:")
        var i
        for(i in message.info_strings) {
          this.rosLog(message.info_strings[i].payload)
        }

        ///////////////////
        // NEPI Configuration

        this.systemAdminEnabled=message.sys_admin_enabled
        this.systemAdminPasswordValid=message.sys_admin_password_valid
        this.systemAdminModeSet = message.sys_admin_mode_set


        this.systemRunModeOptions = message.sys_run_mode_options
        this.systemRunMode = message.sys_run_mode
        this.systemDebugEnabled = message.sys_debug_enabled


        this.managers_list = message.sys_managers_ordered_list
        this.managers_active_list = message.sys_managers_active_list
        this.managers_running_list = message.sys_managers_running_list



        this.userLoginEnabled=message.user_login_enabled
        this.userLoginPasswordValid=message.user_login_password_valid
        this.userLoginModeSet = message.user_login_mode_set

        this.userRestrictionOptions = message.user_restriction_options
        this.userRestrictions = message.user_restrictions
        this.userRestricted = message.user_restricted




        ///////////////////
        // NEPI connection updates

        // turn hearbeatNepi on for half a second
        this.hearbeatNepi = true
        setTimeout(() => {
          this.hearbeatNepi = false
        }, 500)

        this.watchdogNepiCounter = 0
        this.connectedToNepi = true
      


        ///////////////////

      }
    })
  }


  //////////////////////////////
  // SOFTWARE MGR
  //////////////////////////////

  @observable connectedToSoftwareMgr = false
  @observable softwareMgrStatus = null

  @observable softwareInstallOptions = []
 

  @action.bound
  setupMgrSoftwareStatusListener() {
    this.addListener({
      name: "software_mgr/status",
      messageType: "nepi_interfaces/MgrSoftwareStatus",
      manageListener: true,
      callback: message => {
        
        this.softwareMgrStatus = message

        this.connectedToSoftwareMgr = true
      }
    })
  }

  //////////////////////////////
  // TIME MGR
  //////////////////////////////

  @observable timeMgrStatus = null
  @observable connectedToTimeMgr = null


  @observable timeStatusTime = null
  @observable timeStatusTimeStr = null
  @observable timeStatusTimezone = null
  @observable timeStatusDateStr = null
  @observable clockUTCMode = false
  @observable available_timezones = []

  @observable clockTZ = this.get_timezone_desc()
  @observable clockNTP = false
  @observable ntp_sources = []
  @observable clockPPS = false
 

  @observable gpsClockSyncEnabled = false







  startPollingTimeMgrStatusService() {
    const _pollOnce = async () => {
         const response = await this.callService({
          name: "time_status_query",
          messageType: "nepi_interfaces/TimeStatusQuery",
        })

        if (response != null){
          // if last_ntp_sync is 10y, no sync has happened
          this.timeMgrStatus = response.time_status
          this.connectedToTimeMgr = true

          this.available_timezones = response.available_timezones
          this.ntp_sources = this.timeMgrStatus.ntp_sources
          this.clockNTP = false
          const currentlySyncd = this.timeMgrStatus.currently_syncd
          currentlySyncd &&
          currentlySyncd.length &&
          currentlySyncd.forEach(syncd => {
              if (syncd !== false) {
                this.clockNTP = true
              }
            })
          // if last_pps – current_time < 1 second no sync has happened
          this.clockPPS = true
          const lastPPSTS = moment.unix(this.timeMgrStatus.last_pps).unix()
          this.timeStatusTime = moment.unix(this.timeMgrStatus.current_time)
          this.timeStatusTimeStr =this.timeMgrStatus.time_str
          this.timeStatusDateStr = this.timeMgrStatus.date_str
          this.timeStatusTimezone = this.timeMgrStatus.timezone
          this.timeStatusTimezoneDesc = this.timeMgrStatus.timezone_description
          const currTS = this.timeStatusTime && this.timeStatusTime.unix()
          if (currTS && lastPPSTS - currTS < 1) {
            this.clockPPS = false
          }     
          const IS_LOCAL = window.location.hostname === "localhost"
          const clock_synced = this.timeMgrStatus.clock_synced

          const auto_sync_clocks = this.timeMgrStatus.auto_sync_clocks

          const should_sync = (IS_LOCAL === false && this.systemManagesTime === true && clock_synced === false && auto_sync_clocks === true)

          if (should_sync === true ){
            this.syncTime2Device()
          }
        }
      }


      if (this.connectedToNepi === true) {
        setTimeout(_pollOnce, 1000)
      }

    _pollOnce()
  }


 //////////////////////////////
  // DRIVERS MGR
  //////////////////////////////


  @observable driversMgrName = "drivers_mgr"

  @observable connectedToDriversMgr = false
  @observable driversMgrStatus = null

  @observable drivers_list = []
  @observable drivers_name_list = []
  @observable drivers_type_list = []
  @observable drivers_group_id_list = []
  @observable drivers_status_list = []

  @observable drivers_active_list = []
  @observable drivers_active_name_list = []
  @observable drivers_active_type_list = []

  @observable drivers_running_list = []

  @observable drivers_retry_enabled = []


  @observable devices_name_list = []
  @observable devices_alias_list = []

  @observable devices_running_name_list = []
  @observable devices_running_type_list = []

  


  @action.bound
  setupDriversMgrStatusListener() {
    this.addListener({
      name: this.driversMgrName + '/status',
      messageType: "nepi_interfaces/MgrDriversStatus",
      manageListener: true,
      callback: message => {
        
      this.driversMgrStatus = message

      this.drivers_list = message.drivers_ordered_list
      this.drivers_name_list = message.drivers_ordered_name_list
      this.drivers_type_list = message.drivers_ordered_type_list
      this.drivers_group_id_list = message.drivers_ordered_group_id_list
      this.drivers_status_list = message.drivers_ordered_status_list

      this.drivers_active_list = message.drivers_active_list
      this.drivers_active_name_list = message.drivers_active_name_list
      this.drivers_active_type_list = message.drivers_active_type_list

      this.drivers_running_list = message.drivers_running_list

      this.drivers_retry_enabled = message.retry_enabled

      this.devices_name_list = message.devices_name_list
      this.devices_alias_list = message.devices_alias_list

      this.devices_running_name_list = message.devices_running_name_list
      this.devices_running_type_list = message.devices_running_type_list


      this.connectedToDriversMgr = true

      }
    })
  }


  //////////////////////////////
  // APPS Manager
  //////////////////////////////


  @observable appsMgrName  = "apps_mgr"

  @observable connectedToAppsMgr = false
  @observable appsMgrStatus = null

  @observable apps_list =  []
  @observable apps_name_list = []
  @observable apps_group_list = []
  @observable apps_status_list = []

  @observable apps_active_list = []

  @observable apps_running_list = []
  @observable apps_running_name_list = []
  @observable apps_running_group_list = []


  
 
  @action.bound
  setupAppsMgrStatusListener() {
    this.addListener({
      name: this.appsMgrName + '/status',
      messageType: "nepi_interfaces/MgrAppsStatus",
      manageListener: true,
      callback: message => {

      this.appsMgrStatus = message

      this.apps_list = message.apps_ordered_list
      this.apps_name_list = message.apps_ordered_name_list
      this.apps_group_list = message.apps_ordered_group_list
      this.apps_status_list = message.apps_ordered_status_list
      
      this.apps_active_list = message.apps_active_list

      this.apps_running_list = message.apps_running_list
      this.apps_running_name_list = message.apps_running_name_list
      this.apps_running_group_list = message.apps_running_group_list


      this.connectedToAppsMgr = true

      }
    })
  }



  //////////////////////////////
  // AI Model Manager
  ////////////////////////////// 

  @observable aiModelsMgrName = 'ai_models_mgr'

  @observable connectedToAiModelsMgr = false
  @observable aiModelsMgrStatus = null
  

  @observable ai_frameworks_list = []
  @observable ai_frameworks_folder_list = []
  @observable ai_frameworks_active_list = []

  @observable ai_models_list = []
  @observable ai_models_name_list = []
  @observable ai_models_framework_list = []
  @observable ai_models_type_list = []
  @observable ai_models_status_list = []


  @observable ai_models_active_list = []


  @observable ai_models_running_list = []
  @observable ai_models_running_name_list = []
  @observable ai_models_running_type_list = []
  @observable ai_models_running_namespace_list = []


  setupAiModelsMgrStatusListener() {
    this.addListener({
      name: this.aiModelsMgrName + '/status',
      messageType: "nepi_interfaces/MgrAiModelsStatus",
      manageListener: true,
      callback: message => {

      this.aiModelsMgrStatus = message

      this.ai_frameworks_list = message.ai_frameworks_list
      this.ai_frameworks_folder_list = message.ai_frameworks_folder_list
      this.ai_frameworks_active_list = message.ai_frameworks_active_list

      this.ai_models_list = message.ai_models_ordered_list
      this.ai_models_name_list = message.ai_models_ordered_name_list
      this.ai_models_framework_list = message.ai_models_ordered_framework_list
      this.ai_models_type_list = message.ai_models_ordered_type_list
      this.ai_models_status_list = message.ai_models_ordered_status_list


      this.ai_models_active_list = message.ai_models_active_list


      this.ai_models_running_list = message.ai_models_running_list
      this.ai_models_running_name_list = message.ai_models_running_name_list
      this.ai_models_running_type_list = message.ai_models_running_type_list
      this.ai_models_running_namespace_list = message.ai_models_running_namespace_list

      this.connectedToAiModelsMgr = true


      }
    })
  }




  //////////////////////////////
  // Network Manager
  //////////////////////////////


  @observable ip_query_response = null
  @observable bandwidth_usage_query_response = null
  @observable wifi_query_response = null
  /*
  @observable NUID = "INVALID"
  @observable NEPIConnectStatus = null
  @observable alias = ""
  @observable ssh_public_key = ""
  @observable bot_running = null
  @observable lb_last_connection_time = null
  @observable hb_last_connection_time = null
  @observable lb_do_msg_count = "1"
  @observable lb_dt_msg_count = null
  @observable hb_do_transfered_mb = null
  @observable hb_dt_transfered_mb = null
  @observable lb_data_sets_per_hour = null
  @observable lb_enabled = null
  @observable hb_enabled = null
  @observable lb_available_data_sources = null
  @observable lb_selected_data_sources = null
  @observable lb_comms_types = null
  @observable auto_attempts_per_hour = null
  @observable lb_data_queue_size_kb = null
  @observable hb_data_queue_size_mb = null
  @observable hb_auto_data_offloading_enabled = null
  @observable log_storage_enabled = null
  */

  @observable streamingImageQuality = 95
  @observable nepiLinkHbAutoDataOffloadingCheckboxVisible = false

  @observable scripts = []
  @observable running_scripts = []
  @observable launchScript = false
  @observable stopScript = false
  @observable systemStats = null
  @observable scriptForPolledStats = null

  @observable imgMuxSequences = null
  @observable drivers_list_query



  //////////////////////////////
  // NavPose Manager
  //////////////////////////////

  @observable blankNavpose = {
      navpose_frame: 'nepi_frame',
      frame_nav: 'ENU',
      frame_altitude: 'WGS84',
      frame_depth: 'MSL',

      has_location: false,
      // Location Lat,Long
      latitude: -999,
      longitude: -999,
  
      has_heading: false,
      // Heading should be provided in Degrees True North
      heading_deg: -999,
  
      has_position: false,
      // Position should be provided in Meters in specified 3d frame (x,y,z) with x forward, y right/left, and z up/down
      x_m: -999,
      y_m: -999,
      z_m: -999,
  
      has_orientation: false,
      time_orientation: moment.utc().unix(),
      // Orientation should be provided in Degrees in specified 3d frame
      roll_deg: -999,
      pitch_deg: -999,
      yaw_deg: -999,
  
      has_altitude: false,
      // Altitude should be provided in postivie meters in specified alt frame
      altitude_m: -999,
      geoid_height_meters: -999,
  
      has_depth: false,
      // Depth should be provided in positive meters
      depth_m: -999,

      has_pan_tilt: false,
      // Pan and Titl should be provided in ENU frame
      pan_deg: -999,
      tilt_deg: -999
  }

  @observable blankTransform = {
      source_ref_description: '',
      end_ref_description: '',

      x_m: 0.0,
      y_m: 0.0,
      z_m: 0.0,

      x_invert: false,
      y_invert: false,
      z_invert: false,

      roll_deg: 0.0,
      pitch_deg: 0.0,
      yaw_deg: 0.0,

      roll_invert: false,
      pitch_invert: false,
      yaw_invert: false,

      heading_deg: 0.0,

      heading_invert: false,

  }


  @observable navposeMgrName = 'navpose_mgr'
  @observable connectedToNavposeMgr = false
  @observable navposeModelsMgrStatus = null

  @observable navpose_save_data_topic = ''

  @observable navposes_max_pub_rate = null

  @observable frame_nav_options = []
  @observable frame_nav = null
  @observable frame_alt_options = []
  @observable frame_alt = null
  @observable frame_depth_options = []
  @observable frame_depth = null

  @observable navpose_sync_transforms = false
  @observable navpose_frames = []
  @observable navpose_frames_topics = []
  @observable navpose_frames_solutions = [] 


  setupNavposeStatusListener() {
    this.addListener({
      name: this.navposeMgrName + '/status',
      messageType: "nepi_interfaces/MgrNavPoseStatus",
      manageListener: true,
      callback: message => {

      this.navposeModelsMgrStatus = message

      this.navpose_save_data_topic = message.save_data_topic

      this.navposes_max_pub_rate = message.navposes_max_pub_rate

      this.frame_nav_options = message.frame_nav_options
      this.frame_nav = message.frame_nav
      this.frame_alt_options = message.frame_alt_options
      this.frame_alt = message.frame_alt

      this.navpose_sync_transforms = message.sync_transforms
      this.navpose_frames = message.navpose_frames
      this.navpose_frames_topics = message.navpose_frames_topics
      this.navpose_frames_solutions = message.navpose_frames_solutions

      this.connectedToNavposeMgr = true


      }
    })
  }




  ////////////////////////////////
  /////  Service Calls
  ///////////////////////////////



  @action.bound
  async callSaveDataCapabilitiesQueryService(namespace) {
    this.saveDataCaps[namespace] = []
    const response = await this.callService({
      name: namespace + "/capabilities_query",
      messageType: "nepi_interfaces/SaveDataCapabilitiesQuery",  
    })
    if (response != null){
      this.saveDataCaps[namespace] = response
    }
  }



  @action.bound
  async callSettingsCapabilitiesQueryService(namespace) {
    this.settingCaps[namespace] = []
    const response = await this.callService({
      name: namespace + "/capabilities_query",
      messageType: "nepi_interfaces/SettingsCapabilitiesQuery",  
    })
    if (response != null){
      this.settingCaps[namespace] = response
    }
  }

  @action.bound
  async callImageCapabilitiesQueryService(namespace) {
    const response = await this.callService({
      name: namespace + "/capabilities_query",
      messageType: "nepi_interfaces/ImageCapabilitiesQuery",  
    })
    if (response != null){
    this.imageCaps[namespace] = response
    }
  }

  @action.bound
  async callNavPoseCapabilitiesQueryService(namespace) {

      const response = await this.callService({
        name: namespace + "/capabilities_query",
        messageType: "nepi_interfaces/NavPoseCapabilitiesQuery",  
      })
      if (response != null){
      this.navPoseCaps[namespace] = response
      }
  }
  

  @action.bound
  async callIDXCapabilitiesQueryService(namespace) {

    const response = await this.callService({
      name: namespace + "/capabilities_query",
      messageType: "nepi_interfaces/IDXCapabilitiesQuery",  
    })
    if (response != null){
    this.idxDevices[namespace] = response
    }

  }


  @action.bound
  async callPTXCapabilitiesQueryService(namespace) {

    const response = await this.callService({
      name: namespace + "/capabilities_query",
      messageType: "nepi_interfaces/PTXCapabilitiesQuery",
    })
    if (response != null){
    this.ptxDevices[namespace] = response
    }

  }

  @action.bound
  async callLSXCapabilitiesQueryService(namespace) {

    const response = await this.callService({
      name: namespace + "/capabilities_query",
      messageType: "nepi_interfaces/LSXCapabilitiesQuery",
    })
    if (response != null){
    this.lsxDevices[namespace] = response
    }

  }


  @action.bound
  async callRBXCapabilitiesQueryService(namespace) {

    const response = await this.callService({
      name: namespace + "/capabilities_query",
      messageType: "nepi_interfaces/RBXCapabilitiesQuery",  
    })
    if (response != null){
    this.rbxDevices[namespace] = response
    }

  }

  @action.bound
  async callNPXCapabilitiesQueryService(namespace) {
    const response = await this.callService({
      name: namespace + "/capabilities_query",
      messageType: "nepi_interfaces/NPXCapabilitiesQuery",  
    })
    if (response != null){
    this.npxDevices[namespace] = response
    }
  }

  // @action.bound
  // async callSoftwareStatusQueryService() {
  //   this.systemSoftwareStatus = await this.callService({
  //     name: "sw_update_status_query",
  //     messageType: "nepi_interfaces/SystemSoftwareStatusQuery"
  //   })
  // }



  @action.bound
  async callAppStatusQueryService(app_name) {
      var response = null
      if (this.apps_list.indexOf(app_name) !== -1){
        response = await this.callService({
          name: 'apps_mgr/app_status_query',
          messageType: "nepi_interfaces/AppStatusQuery",
          args: {app_name : app_name},
        })
      }
      return response
  }

  @action.bound
  async callDriverStatusQueryService(driver_name) {
      var response = null
      if (this.drivers_list.indexOf(driver_name) !== -1){
        response = await this.callService({
        name: 'drivers_mgr/driver_status_query',
        messageType: "nepi_interfaces/DriverStatusQuery",
        args: {driver_name : driver_name},
      })
      }
      return response

  }


    @action.bound
  async callAiDetectorCapabilitiesQueryService(namespace) {
    this.aiDetectorCaps[namespace] = []
      const response = await this.callService({
        name: namespace + "/detector_info_query",
        messageType: "nepi_interfaces/SaveDataCapabilitiesQuery",  
      })
      if (response != null ) {
      this.aiDetectorCaps[namespace] = response
      }
  }

  /*******************************/
  // System Data Update Functions
  /*******************************/



  @action.bound
  updatePrefix(topics,types) {
    // Function for testing if we need to update the device prefix variables.
    // It loops though the topics and uses the testTopicForPrefix to test and
    // perform the update.  If it updates we are done, break.

    // we return true if the prefix was updated

    var ret = false
    if (topics != null){
      for (var i = 0; i < topics.length; i++) {
        var topic_name_parts = topics[i].split("/")
        if (
          topic_name_parts[topic_name_parts.length - 1] === "status" &&
          types[i] === "nepi_interfaces/MgrSystemStatus"
        ) {
          if (
            this.namespacePrefix !== topic_name_parts[1] &&
            this.deviceId !== topic_name_parts[2]
          ) {
            this.namespacePrefix = topic_name_parts[1]
            this.deviceId = topic_name_parts[2]
            if (this.validPrefix()) {
              ret = true
              this.rosLog(
                `Fetched device info ${this.namespacePrefix}/${this.deviceId}`
              )
            }
          }
        }
      }
    }
    return ret
  }



  @action.bound
  updateSaveDataNamespaces(topics,types) {
    // Function for updating image topics list
    var newSaveDataNamespaces = []
    if (this.connectedToNepi === true) {
      for (var i = 0; i < topics.length; i++) {
        if (types[i] === "nepi_interfaces/SaveDataStatus"){
          newSaveDataNamespaces.push(topics[i].replace('/status',''))
        }
      }


      // sort the save topics for comparison to work
      newSaveDataNamespaces.sort()    
    }  
    else {
      newSaveDataNamespaces = []
    }
      if (!this.saveDataNamespaces.equals(newSaveDataNamespaces)) {
        this.saveDataNamespaces = newSaveDataNamespaces
        for (var i2 = 0; i2 < newSaveDataNamespaces.length; i2++) {
              this.callSaveDataCapabilitiesQueryService(newSaveDataNamespaces[i2])
            }
        return true
      } else {
        return false
      }

  }

    @action.bound
  updateAiDetectorNamespaces(topics,types) {
    // Function for updating image topics list
    var newAiDetectorNamespaces = []
    if (this.connectedToNepi === true) {
      
      for (var i = 0; i < topics.length; i++) {
        if (types[i] === "nepi_interfaces/AiDetectorStatus"){
          newAiDetectorNamespaces.push(topics[i].replace('/status',''))
        }
      }

      // sort the save topics for comparison to work
      newAiDetectorNamespaces.sort()    
    }  
    else {
      newAiDetectorNamespaces = []
    }

      if (!this.aiDetectorNamespaces.equals(newAiDetectorNamespaces)) {
        this.aiDetectorNamespaces = newAiDetectorNamespaces
        for (var i2 = 0; i2 < newAiDetectorNamespaces.length; i2++) {
              this.callAiDetectorCapabilitiesQueryService(newAiDetectorNamespaces[i2])
            }
        return true
      } else {
        return false
      }

  }

  @action.bound
  updateMessageTopics(topics,types) {
    // Function for updating image topics list
    var newMessageTopics = []
    if (this.connectedToNepi === true) {

      for (var i = 0; i < topics.length; i++) {
        if (types[i] === "nepi_interfaces/Message") {
          newMessageTopics.push(topics[i])
        }
      }

      // sort the image topics for comparison to work
      newMessageTopics.sort()
    }
    else {
      newMessageTopics = []
    }

    if (!this.messageTopics.equals(newMessageTopics)) {
      this.messageTopics = newMessageTopics
      return true
    } else {
      return false
    }

  }


  @action.bound
  updateNavPoseTopics(topics,types) {
    // Function for updating image topics list
    var newNavPoseTopics = []
    if (this.connectedToNepi === true) {
      
      for (var i = 0; i < topics.length; i++) {
        if (types[i] === "nepi_interfaces/NavPose" && topics[i].indexOf("zed_node") === -1) {
          newNavPoseTopics.push(topics[i])
        }
      }

      // sort the image topics for comparison to work
      newNavPoseTopics.sort()  
    }  
    else {
      newNavPoseTopics = []
    }
    if (!this.navposeTopics.equals(newNavPoseTopics)) {
      this.navposeTopics = newNavPoseTopics
      for (var i2 = 0; i < newNavPoseTopics.length; i2++) {
            this.callNavPoseCapabilitiesQueryService(newNavPoseTopics[i2])
          }
      return true
    } else {
      return false
    }

  }

  @action.bound
  updateImageTopics(topics,types) {
    // Function for updating image topics list
    var newImageTopics = []
    var newImageDetectionTopics = []
    if (this.connectedToNepi === true) {

      for (var i = 0; i < topics.length; i++) {
        if (types[i] === "sensor_msgs/Image" && topics[i].indexOf("zed_node") === -1) {
          newImageTopics.push(topics[i])
          if (topics[i].indexOf('detection_image') !== -1){
            newImageDetectionTopics.push(topics[i])
          }
        }
      }

      // sort the image topics for comparison to work
      newImageTopics.sort()    
    }  
    else {
      newImageTopics = []
    }
      if (!this.imageTopics.equals(newImageTopics)) {
        this.imageTopics = newImageTopics
        this.imageDetectionTopics = newImageDetectionTopics
        for (var i2 = 0; i2 < newImageTopics.length; i2++) {
              this.callImageCapabilitiesQueryService(newImageTopics[i2])
            }
        return true
      } else {
        return false
      }

  }



  @action.bound
  updatePointcloudTopics(topics,types) {
    // Function for updating image topics list
      var newPointcloudTopics = []
    if (this.connectedToNepi === true) {

      for (var i = 0; i < topics.length; i++) {
        if (types[i] === "sensor_msgs/PointCloud2") {
          newPointcloudTopics.push(topics[i])
        }
      }

      // sort the image topics for comparison to work
      newPointcloudTopics.sort()
    }  
    else {
      newPointcloudTopics = []
    }
      if (!this.pointcloudTopics.equals(newPointcloudTopics)) {
        this.pointcloudTopics = newPointcloudTopics
        return true
      } else {
        return false
      }

  }

  
  @action.bound
  updateIDXDevices(topics,types) {    
    var idx_devices_changed = false
    var devices_detected = []
    for (var i = 0; i < topics.length; i++) {
      if (topics[i].endsWith("/idx/status")) {
        const idx_device_namespace = topics[i].replace("/status","")
        if (!(devices_detected.includes(idx_device_namespace))) {
          this.callIDXCapabilitiesQueryService(idx_device_namespace) // Testing
          this.callSettingsCapabilitiesQueryService(idx_device_namespace + "/settings")
          const idxDevices = this.idxDevices[idx_device_namespace]
          if (idxDevices) { // Testing
            devices_detected.push(idx_device_namespace)
          }
          idx_devices_changed = true // Testing -- always declare changed
        }
      }
    }

    // Now clean out any devices that are no longer detected
    const previously_known = Object.keys(this.idxDevices)
    for (i = 0; i < previously_known.length; ++i) {
      if (!(devices_detected.includes(previously_known[i]))) {
        delete this.idxDevices[previously_known[i]]
        idx_devices_changed = true
      }
    }
    return idx_devices_changed
  }

  @action.bound
  updatePTXDevices(topics,types) {
    var ptx_devices_changed = false
    var ptx_devices_detected = []
    for (var i = 0; i < topics.length; i++) {
      if (topics[i].endsWith("/ptx/status")) {
        const ptx_device_namespace = topics[i].replace("/status","")
        if (!(ptx_devices_detected.includes(ptx_device_namespace))) {
          this.callPTXCapabilitiesQueryService(ptx_device_namespace)
          this.callSettingsCapabilitiesQueryService(ptx_device_namespace + "/settings")
          const ptxUnit = this.ptxDevices[ptx_device_namespace]
          if (ptxUnit)
          {
            ptx_devices_detected.push(ptx_device_namespace)
          }
        }
        ptx_devices_changed = true
      }
    }

    // Now clean out any units that are no longer detected
    const previously_known = Object.keys(this.ptxDevices)
    for (i = 0; i < previously_known.length; ++i) {
      if (!(ptx_devices_detected.includes(previously_known[i]))) {
        delete this.ptxDevices[previously_known[i]]
        ptx_devices_changed = true
      }
    }
    return ptx_devices_changed
  }
  
  @action.bound
  updateLXSDevices(topics,types) {
    var lsx_devices_changed = false
    var lsx_devices_detected = []
    for (var i = 0; i < topics.length; i++) {
      if (topics[i].endsWith("/lsx/status")) {
        const lsx_device_namespace = topics[i].replace("/status","")
        if (!(lsx_devices_detected.includes(lsx_device_namespace))) {
          this.callLSXCapabilitiesQueryService(lsx_device_namespace)
          this.callSettingsCapabilitiesQueryService(lsx_device_namespace + "/settings")
          const lsxDevices = this.lsxDevices[lsx_device_namespace]
          if (lsxDevices)
          {
            lsx_devices_detected.push(lsx_device_namespace)
          }
        }
        lsx_devices_changed = true
      }
    }

    // Now clean out any units that are no longer detected
    const previously_known = Object.keys(this.lsxDevices)
    for (i = 0; i < previously_known.length; ++i) {
      if (!(lsx_devices_detected.includes(previously_known[i]))) {
        delete this.lsxDevices[previously_known[i]]
        lsx_devices_changed = true
      }
    }
    return lsx_devices_changed
  }  
  
  @action.bound
  updateRBXDevices(topics,types) {
    var rbx_devices_changed = false
    var devices_detected = []
    for (var i = 0; i < topics.length; i++) {
      if (topics[i].endsWith("/rbx/status")) {
        const rbx_device_namespace = topics[i].replace("/status","")
        if (!(devices_detected.includes(rbx_device_namespace))) {
          this.callRBXCapabilitiesQueryService(rbx_device_namespace)
          this.callSettingsCapabilitiesQueryService(rbx_device_namespace + "/settings")
          const rbxDevice = this.rbxDevices[rbx_device_namespace]
          if (rbxDevice) {
            devices_detected.push(rbx_device_namespace)
          }
          rbx_devices_changed = true // Testing -- always declare changed
        }
      }
    }

    // Now clean out any devices that are no longer detected
    const previously_known = Object.keys(this.rbxDevices)
    for (i = 0; i < previously_known.length; ++i) {
      if (!(devices_detected.includes(previously_known[i]))) {
        delete this.rbxDevices[previously_known[i]]
        rbx_devices_changed = true
      }
    }
    return rbx_devices_changed
  }

  @action.bound
  updateNPXDevices(topics,types) {
    var npx_devices_changed = false
    var devices_detected = []
    for (var i = 0; i < topics.length; i++) {
      if (topics[i].endsWith("/npx/status")) {
        const npx_device_namespace = topics[i].replace("/status","")
        if (!(devices_detected.includes(npx_device_namespace))) {
          this.callNPXCapabilitiesQueryService(npx_device_namespace) // Testing
          this.callSettingsCapabilitiesQueryService(npx_device_namespace + "/settings")
          const npxSensor = this.npxDevices[npx_device_namespace]
          if (npxSensor) { // Testing
            devices_detected.push(npx_device_namespace)
          }
          npx_devices_changed = true // Testing -- always declare changed
        }
      }
    }

    // Now clean out any devices that are no longer detected
    const previously_known = Object.keys(this.npxDevices)
    for (i = 0; i < previously_known.length; ++i) {
      if (!(devices_detected.includes(previously_known[i]))) {
        delete this.npxDevices[previously_known[i]]
        npx_devices_changed = true
      }
    }
    return npx_devices_changed
  }


  @action.bound
  updateResetTopics(topics,types) {
    var newResetTopics = []
    for (var i = 0; i < topics.length; i++) {
      var topic_name_parts = topics[i].split("/")
      var last_element = topic_name_parts.pop()
      var topic_base = topic_name_parts.join("/")
      if (
        last_element === "system_reset" &&
        topics[i] === "nepi_interfaces/Reset"
      ) {
        newResetTopics.push(topic_base)
      }
    }

    // sort the topics for comparison to work
    newResetTopics.sort()

    if (!this.resetTopics.equals(newResetTopics)) {
      this.resetTopics = newResetTopics
      return true
    } else {
      return false
    }
  }

  /*******************************/
  // Generic Listener Functions
  /*******************************/

  @action.bound
  setupStatusListener(namespace, msg_type, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: msg_type,
        noPrefix: true,
        callback: callback,

      })
    }
  }

  @action.bound
  setupDataListener(namespace, msg_type, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: msg_type,
        noPrefix: true,
        callback: callback,

      })
    }
  }

  @action.bound
  setupStringListener(namespace,callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: "std_msgs/String",
        noPrefix: true,
        callback: callback,

      })
    }
  }

  @action.bound
  setupVector3Listener(namespace, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: "geometry_msgs/Vector3",
        noPrefix: true,
        callback: callback,

      })
    }
  }

  @action.bound
  setupFloatListener(namespace, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: "std_msgs/Float32",
        noPrefix: true,
        callback: callback,

      })
    }
  }

  /*******************************/
  // Custom Listener Functions
  /*******************************/

  @action.bound
  setupPTXStatusListener(ptxNamespace, callback) {
    if (ptxNamespace) {
      return this.addListener({
        name: ptxNamespace + "/status",
        messageType: "nepi_interfaces/DevicePTXStatus",
        noPrefix: true,
        callback: callback,

      })
    }
  }
    
  @action.bound
  setupLSXStatusListener(lsxNamespace, callback) {
    if (lsxNamespace) {
      return this.addListener({
        name: lsxNamespace + "/status",
        messageType: "nepi_interfaces/DeviceLSXStatus",
        noPrefix: true,
        callback: callback,

      })
    }
  }

  setupRUISettingsListener() {
    this.addListener({
      name: "rui_config_mgr/settings",
      messageType: "nepi_interfaces/RUISettings",
      callback: message => {
        this.streamingImageQuality = message.streaming_image_quality
        //this.streamingImageRate = message.streaming_image_rate
        this.nepiLinkHbAutoDataOffloadingCheckboxVisible = message.nepi_hb_auto_offload_visible
      }
    })
  }


  @action.bound
  setupIDXStatusListener(idxDeviceNamespace, callback) {
    if (idxDeviceNamespace) {
      return this.addListener({
        name: idxDeviceNamespace + "/status",
        messageType: "nepi_interfaces/DeviceIDXStatus",
        noPrefix: true,
        callback: callback,

      })
    }
  }

  @action.bound
  setupNPXStatusListener(npxDeviceNamespace, callback) {
    if (npxDeviceNamespace) {
      return this.addListener({
        name: npxDeviceNamespace + "/status",
        messageType: "nepi_interfaces/DeviceNPXStatus",
        noPrefix: true,
        callback: callback,

      })
    }
  }

  @action.bound
  setupRBXStatusListener(rbxDeviceNamespace, callback) {
    if (rbxDeviceNamespace) {
      return this.addListener({
        name: rbxDeviceNamespace + "/status",
        messageType: "nepi_interfaces/DeviceRBXStatus",
        noPrefix: true,
        callback: callback,

      })
    }
  }



  @action.bound
  setupSaveDataStatusListener(namespace, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace + "/status",
        messageType: "nepi_interfaces/SaveDataStatus",
        noPrefix: true,
        callback: callback,

      })
    }
  }


  @action.bound
  setupMessagesListener(namespace, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: "nepi_interfaces/Message",
        noPrefix: true,
        callback: callback,

      })
    }
  }

  @action.bound
  setupSettingsStatusListener(namespace, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: "nepi_interfaces/SettingsStatus",
        noPrefix: true,
        callback: callback,

      })
    }
  }

  @action.bound
  setupTransformListener(namespace, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: "nepi_interfaces/Transform",
        noPrefix: true,
        callback: callback,

      })
    }
  }

  @action.bound
  setupTrackingStatusListener(namespace, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: "nepi_interfaces/TrackingStatus",
        noPrefix: true,
        callback: callback,

      })
    }
  }

  /*******************************/
  // Data Polling Functions
  /*******************************/


  async startPollingIPAddrQueryService() {
    const _pollOnce = async () => {
        const response = await this.callService({
          name: "ip_addr_query",
          messageType: "nepi_interfaces/IPAddrQuery"
        })
        if (response != null ) {
            this.ip_query_response = response
        }
      }

      if (this.connectedToNepi === true) {
        setTimeout(_pollOnce, 1000)
      }

    _pollOnce()
  }

  async startPollingBandwidthUsageService() {
    const _pollOnce = async () => {
        const response = await this.callService({
          name: "bandwidth_usage_query",
          messageType: "nepi_interfaces/BandwidthUsageQuery",
        })
        if (response != null ) {
            this.bandwidth_usage_query_response = response
        }
    }
    if (this.connectedToNepi === true) {
      setTimeout(_pollOnce, 3000)
    }

    _pollOnce()
  }

  async startPollingWifiQueryService() {
    const _pollOnce = async () => {
        const response = await this.callService({
          name: "wifi_query",
          messageType: "nepi_interfaces/WifiQuery",
        })
        if (response != null ) {
            this.wifi_query_response = response
        }
      }

      if (this.connectedToNepi === true) {
        setTimeout(_pollOnce, 3000)
      }


    _pollOnce()    
  }



  async startPollingGetScriptsService() {
    const _pollOnce = async () => {
        const response = await this.callService({
          name: "get_scripts",
          messageType: "nepi_interfaces/GetScriptsQuery"
        })
        if (response != null ) {
            this.scripts = response
        }
      }

      if (this.connectedToNepi === true) {
        setTimeout(_pollOnce, 3000)
      }

    _pollOnce()
  }

  async startPollingGetRunningScriptsService() {
    const _pollOnce = async () => {
        const response  = await this.callService({
          name: "get_running_scripts",
          messageType: "nepi_interfaces/GetRunningScriptsQuery"
        })
        if (response != null ) {
            this.running_scripts = response
        }
      }

      if (this.connectedToNepi === true) {
        setTimeout(_pollOnce, 3000)
      }

    _pollOnce()
  }

  async startLaunchScriptService(item) {
    // One-shot action -- launch exactly once. (Do NOT add a setTimeout re-call here:
    // the launch_script service is not a poller, and a second call lands as a
    // "is already running... will not start another instance" rejection.)
    if (this.connectedToNepi === true) {
      const response = await this.callService({
        name: "launch_script",
        messageType: "nepi_interfaces/LaunchScript",
        args: {script : item}
      })
      if (response != null ) {
          this.launchScript = response
      }
    }
  }


  async stopLaunchScriptService(item) {
    // One-shot action -- stop exactly once. (See startLaunchScriptService: a second
    // delayed call just lands as a "Script not running" warning.)
    if (this.connectedToNepi === true) {
      const response = await this.callService({
        name: "stop_script",
        messageType: "nepi_interfaces/StopScript",
        args: {script : item}
      })
      if (response != null ) {
          this.stopScript = response
      }
    }
  }

  async callGetSystemStatsQueryService(item, poll = true) {
    const _pollOnce = async () => {
        const response = await this.callService({
          name: "get_system_stats",
          messageType: "nepi_interfaces/GetSystemStatsQuery",
          args: {script : this.scriptForPolledStats}
        })
        if (response != null ) {
            this.systemStats = response
        }
      }

      if (this.connectedToNepi && poll) {
        setTimeout(_pollOnce, 3000)
      }


    const firstCall = (this.scriptForPolledStats === null)
    this.scriptForPolledStats = item
    
    // Only launch this once, then just change state to start polling a different script
    if (firstCall === true || !poll) {
      _pollOnce()
    }
    
  }





  


  /*******************************/
  // Generic Send Data Functions
  /*******************************/
  @action.bound
  sendTriggerMsg(namespace) {
    this.publishMessage({
      name: namespace,
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })
  }





  @action.bound
  sendBoolMsg(namespace, value) {
    this.publishMessage({
      name: namespace,
      messageType: "std_msgs/Bool",
      data: {data: value},
      noPrefix: true
    })
    
  }

  @action.bound
  sendUpdateBoolMsg(namespace, name, value, name2 = '', name3 = '') {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/UpdateBool",
      data: {
        name: name,
        name2: name2,
        name3: name3,
        value: value
      },
      noPrefix: true
    })
  }


  // RUI_KEY_PATH = '/opt/nepi/nepi_rui/src/rui_webserver/rui-app/src/keys/rui_key'

  @action.bound
  async stringEncript(str) {
    try {
      const hexKey = process.env.REACT_APP_RUI_KEY
      if (!hexKey) {
        console.warn("REACT_APP_RUI_KEY is not set — encryption disabled")
        return str
      }
      if (!_ruiCryptoKey) {
        const keyBytes = new Uint8Array(hexKey.match(/.{1,2}/g).map(b => parseInt(b, 16)))
        _ruiCryptoKey = await crypto.subtle.importKey(
          "raw", keyBytes, { name: "AES-GCM" }, false, ["encrypt", "decrypt"]
        )
      }
      const nonce = crypto.getRandomValues(new Uint8Array(12))
      const ct = await crypto.subtle.encrypt(
        { name: "AES-GCM", iv: nonce }, _ruiCryptoKey, new TextEncoder().encode(str)
      )
      const payload = new Uint8Array(nonce.byteLength + ct.byteLength)
      payload.set(nonce, 0)
      payload.set(new Uint8Array(ct), nonce.byteLength)
      return btoa(String.fromCharCode(...payload))
    } catch (err) {
      console.error(err)
      return str
    }
  }


  @action.bound
  async stringDecript(str) {
    try {
      const hexKey = process.env.REACT_APP_RUI_KEY
      if (!hexKey) {
        console.warn("REACT_APP_RUI_KEY is not set — decryption disabled")
        return str
      }
      if (!_ruiCryptoKey) {
        const keyBytes = new Uint8Array(hexKey.match(/.{1,2}/g).map(b => parseInt(b, 16)))
        _ruiCryptoKey = await crypto.subtle.importKey(
          "raw", keyBytes, { name: "AES-GCM" }, false, ["encrypt", "decrypt"]
        )
      }
      const raw = Uint8Array.from(atob(str), c => c.charCodeAt(0))
      const nonce = raw.slice(0, 12)
      const ct = raw.slice(12)
      const plaintext = await crypto.subtle.decrypt(
        { name: "AES-GCM", iv: nonce }, _ruiCryptoKey, ct
      )
      return new TextDecoder().decode(plaintext)
    } catch (err) {
      console.error(err)
      return str
    }
  }


  @action.bound
  sendStringMsg(namespace,str) {
    this.publishMessage({
      name: namespace,
      messageType: "std_msgs/String",
      data: {'data':str},
      noPrefix: true
    })
  }




  

  @action.bound
  sendUpdateStringMsg(namespace, name, value, name2 = '', name3 = '') {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/UpdateString",
      data: {
        name: name,
        name2: name2,
        name3: name3,
        value: value
      },
      noPrefix: true
    })
  }

  @action.bound
  sendStringArrayMsg(namespace,strArray) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/StringArray",
      data: {'array':strArray},
      noPrefix: true
    })
  }


  @action.bound
  sendIntMsg(namespace, int_str) {
    let intVal = parseInt(int_str, 10)
    if (!isNaN(intVal)) {
      this.publishMessage({
        name: namespace,
        messageType: "std_msgs/Int32",
        data: {data: intVal},
        noPrefix: true
      })
    }
  }

  @action.bound
  sendUpdateIntMsg(namespace, name, value, name2 = '', name3 = '') {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/UpdateInt",
      data: {
        name: name,
        name2: name2,
        name3: name3,
        value: value
      },
      noPrefix: true
    })
  }

  @action.bound
  sendInt8Msg(namespace, int_str) {
    let intVal = parseInt(int_str, 10)
    if (!isNaN(intVal)) {
      this.publishMessage({
        name: namespace,
        messageType: "std_msgs/Int8",
        data: {data: intVal},
        noPrefix: true
      })
    }
  }

  @action.bound
  sendFloatMsg(namespace, float_str) {
    let floatVal = parseFloat(float_str)
    if (!isNaN(floatVal)) {
      this.publishMessage({
        name: namespace,
        messageType: "std_msgs/Float32",
        data: {data: floatVal},
        noPrefix: true
      })
    }
  }

  @action.bound
  sendUpdateFloatMsg(namespace, name, value, name2 = '', name3 = '') {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/UpdateFloat",
      data: {
        name: name,
        name2: name2,
        name3: name3,
        value: value
      },
      noPrefix: true
    })
  }


  @action.bound
  sendErrorBoundsMsg(namespace, max_m,max_d,min_stab) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/ErrorBounds",
      data: { 
        data: {
          max_distance_error_m: max_m,
          max_rotation_error_deg: max_d,
          min_stabilize_time_s: min_stab
        }
      },
      noPrefix: true
    })
  }


  @action.bound
  sendUpdateRangeWindowMsg(namespace, comp_name, min, max, throttle = true) {
    if (throttle){
      if (throttle && this.isThrottled()) {
        return
      }
    }
    if (namespace) {
      this.publishMessage({
        name: namespace,
        messageType: "nepi_interfaces/UpdateRangeWindow",
        noPrefix: true,
        data: {
          name: comp_name,
          start_range: min,
          stop_range: max
        }
      })
    } else {
      console.warn("publishRangeWindow: namespace not set")
    }
  }


  @action.bound
  sendUpdateOrderMsg(namespace, name, move_cmd, name2 = '', name3 = '') {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/UpdateOrder",
      data: {    
        name: name,
        name2: name2,
        name3: name3,
        move_cmd: move_cmd
      },
      noPrefix: true
    })
  }

  @action.bound
  sendImageSelectionMsg(namespace, image_index, image_topic) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/ImageSelection",
      data: {
        image_index: image_index,
        image_topic: image_topic
      },
      noPrefix: true
    })
  }



  @action.bound
  sendFloatVector3Msg(namespace, float1_str,float2_str,float3_str) {
    let float1Val = parseFloat(float1_str)
    let float2Val = parseFloat(float2_str)
    let float3Val = parseFloat(float3_str)
    if (!isNaN(float1Val) && !isNaN(float2Val) && !isNaN(float3Val)) {
      this.publishMessage({
        name: namespace,
        messageType: "geometry_msgs/Vector3",
        data: { 
          x: float1Val,
          y: float2Val,
          z: float3Val
        },
        noPrefix: true
      })
    }
  }


  @action.bound
  sendGeoPointMsg(namespace, lat_str,long_str,alt_str) {
    let latVal = parseFloat(lat_str)
    let longVal = parseFloat(long_str)
    let altVal = parseFloat(alt_str)
    if (!isNaN(latVal) && !isNaN(longVal) && !isNaN(altVal)) {
      this.publishMessage({
        name: namespace,
        messageType: "geographic_msgs/GeoPoint",
        data: { 
          latitude: latVal,
          longitude: longVal,
          altitude: altVal
        },
        noPrefix: true
      })
    }
  }

  @action.bound
  sendFloatGotoPoseMsg(namespace, float1_str,float2_str,float3_str) {
    let float1Val = parseFloat(float1_str)
    let float2Val = parseFloat(float2_str)
    let float3Val = parseFloat(float3_str)
    if (!isNaN(float1Val) && !isNaN(float2Val) && !isNaN(float3Val)) {
      this.publishMessage({
        name: namespace,
        messageType: "nepi_interfaces/GotoPose",
        data: { 
          roll_deg: float1Val,
          pitch_deg: float2Val,
          yaw_deg: float3Val
        },
        noPrefix: true
      })
    }
  }

  @action.bound
  sendFloatGotoPositionMsg(namespace, float1_str,float2_str,float3_str,float4_str) {
    let float1Val = parseFloat(float1_str)
    let float2Val = parseFloat(float2_str)
    let float3Val = parseFloat(float3_str)
    let float4Val = parseFloat(float4_str)
    if (!isNaN(float1Val) && !isNaN(float2Val) && !isNaN(float3Val) && !isNaN(float4Val)) {
      this.publishMessage({
        name: namespace,
        messageType: "nepi_interfaces/GotoPosition",
        data: { 
          x_meters: float1Val,
          y_meters: float2Val,
          z_meters: float3Val,
          yaw_deg: float4Val
        },
        noPrefix: true
      })
    }
  }

  @action.bound
  sendFloatGotoLocationMsg(namespace, float1_str,float2_str,float3_str,float4_str) {
    let float1Val = parseFloat(float1_str)
    let float2Val = parseFloat(float2_str)
    let float3Val = parseFloat(float3_str)
    let float4Val = parseFloat(float4_str)
    if (!isNaN(float1Val) && !isNaN(float2Val) && !isNaN(float3Val) && !isNaN(float4Val)) {
      this.publishMessage({
        name: namespace,
        messageType: "nepi_interfaces/GotoLocation",
        data: { 
          lat: float1Val,
          long: float2Val,
          altitude_meters: float3Val,
          yaw_deg: float4Val
        },
        noPrefix: true
      })
    }
  }

  @action.bound
  sendMotorControlMsg(namespace, motor_ind, speed_ratio) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/MotorControl",
      data: {
        motor_ind: motor_ind,
        speed_ratio: speed_ratio
      },
      noPrefix: true
    })
  }

  @action.bound
  sendImagePixelMsg(namespace,x,y,r,g,b,a) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/ImagePixel",
      data: {x: x,
          y: y,
          r: r,
          g: g,
          b: b,
          a: a},
      noPrefix: true
    })
  }

  @action.bound
  sendImageDragMsg(namespace,x,y,r,g,b,a) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/ImagePixel",
      data: {x: x,
          y: y,
          r: r,
          g: g,
          b: b,
          a: a},
      noPrefix: true
    })
  }


    @action.bound
  sendImageWindowMsg(namespace,x_min,x_max,y_min,y_max) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/ImageWindow",
      data: {x_min: x_min,
        x_max: x_max,
        y_min: y_min,
        y_max: y_max},
      noPrefix: true
    })
  }


@action.bound
saveConfigTriggered(namespace) {
  this.publishMessage({
    name: namespace + "/save_config",
    messageType: "std_msgs/Empty",
    data: {},
    noPrefix: true
  })
}


@action.bound
sendSaveConfigTrigger(namespace) {
  this.publishMessage({
    name: namespace + "/save_config",
    messageType: "std_msgs/Empty",
    data: {},
    noPrefix: true
  })
}




  sendMouseClickEventMsg(namespace, image_topic, image_index, mouse_click, click_count, status_msg ) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/ImageMouseEvent",
      data: { 
          image_topic: image_topic,
          image_index: image_index,

          click_event: true, 
          click_count: click_count,
          click: mouse_click,
          drag_event: false, 
          drag_start: {x:0,y:0,r:0,g:0,b:0,a:0},
          drag_stop: {x:0,y:0,r:0,g:0,b:0,a:0},
          window_event: false, 
          window: {x_min:0, x_max:0, y_min:0, y_max:0},
          image_status_msg: status_msg
      },
      noPrefix: true

    })
  }

  sendMouseDragEventMsg(namespace, image_topic, image_index, mouse_drag_start, mouse_drag_stop, status_msg ) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/ImageMouseEvent",

        data: {
          image_topic: image_topic,
          image_index: image_index,
          click_event: false, 
          click_count: 0,
          click: {x:0,y:0,r:0,g:0,b:0,a:0},
          drag_event: true, 
          drag_start: mouse_drag_start,
          drag_stop: mouse_drag_stop,
          window_event: false, 
          window: {x_min:0, x_max:0, y_min:0, y_max:0},
          image_status_msg: status_msg
        },
      noPrefix: true
    })
  }

  sendMouseWindowEventMsg(namespace, image_topic, image_index, mouse_window, status_msg ) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/ImageMouseEvent",
      data: { 
          image_topic: image_topic,
          image_index: image_index,
          click_event: false, 
          click_count: 0,
          click: {x:0,y:0,r:0,g:0,b:0,a:0},
          drag_event: false, 
          drag_start: {x:0,y:0,r:0,g:0,b:0,a:0},
          drag_stop: {x:0,y:0,r:0,g:0,b:0,a:0},
          window_event: true, 
          window: mouse_window,

          image_status_msg: status_msg
      },
      noPrefix: true

    })
  }



///// System IF Calls

@action.bound
updateCapSetting(namespace,nameStr,typeStr,optionsStrList,default_value_str) {
  this.publishMessage({
    name: namespace + "/update_setting",
    messageType: "nepi_interfaces/SettingCap",
    data: {type_str:typeStr,
      name_str:nameStr,
      options_list:optionsStrList,
      default_value_str:default_value_str
    },
    noPrefix: true
  })
}

@action.bound
updateSetting(namespace,nameStr,typeStr,valueStr) {
    this.publishMessage({
      name: namespace + "/update_setting",
      messageType: "nepi_interfaces/Setting",
      data: {type_str:typeStr,
        name_str:nameStr,
        value_str:valueStr
      },
      noPrefix: true
    })
  }

  // Update several settings with a single message.  settingsList is an array
  // of {nameStr, typeStr, valueStr} entries.  The backend applies each entry
  // in order.
  @action.bound
  updateSettings(namespace,settingsList) {
    this.publishMessage({
      name: namespace + "/update_settings",
      messageType: "nepi_interfaces/Settings",
      data: {
        settings: settingsList.map(s => ({
          type_str: s.typeStr,
          name_str: s.nameStr,
          value_str: s.valueStr
        }))
      },
      noPrefix: true
    })
  }

 
  @action.bound
  updateSaveDataRate(namespace,data_product,rate_hz) {
    this.publishMessage({
      name: namespace + "/save_data_rate",
      messageType: "nepi_interfaces/SaveDataRate",
      data: {
        data_product: data_product,
        save_rate_hz: rate_hz,
      },
      noPrefix: true
    })
  }





  @action.bound
  sendNavPoseMsg(namespace,navpose_data){
      this.publishMessage({
        name: namespace,
        messageType: "nepi_interfaces/NavPose",
        data: navpose_data,
        noPrefix: true
      })
    }

  @action.bound
  sendUpdateNavposeMsg(namespace,name, navpose_msg, name2 = '', name3 = '') {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_interfaces/UpdateNavPose",
      data: {         
        name: name,
        name2: name2,
        name3: name3,
        navpose: navpose_msg
      },
      noPrefix: true
    })
  }

    @action.bound
    sendNavPoseLocationMsg(namespace,lat,long,init_np){
      var np_msg = this.blankNavpose
      np_msg.has_location = true
      np_msg.time_location = moment.utc().unix()
      np_msg.latitude = parseFloat(lat)
      np_msg.longitude = parseFloat(long)
      const init = init_np ? init_np : false
      if (init === false){
        this.sendNavPoseMsg(namespace,np_msg)
      }
      else {
        this.sendInitNavPoseMsg(namespace,np_msg)
      }
    }

    @action.bound
    sendNavPoseHeadingMsg(namespace,heading,init_np){
      var np_msg = this.blankNavpose
      np_msg.has_heading = true
      np_msg.time_heading = moment.utc().unix()
      np_msg.heading_deg = parseFloat(heading)
      const init = init_np ? init_np : false
      if (init === false){
        this.sendNavPoseMsg(namespace,np_msg)
      }
      else {
        this.sendInitNavPoseMsg(namespace,np_msg)
      }
    }


    @action.bound
    sendNavPoseOrienationMsg(namespace,roll,pitch,yaw,init_np){
      var np_msg = this.blankNavpose
      np_msg.has_orientation = true
      np_msg.time_orientation = moment.utc().unix()
      np_msg.roll_deg = parseFloat(roll)
      np_msg.pitch_deg = parseFloat(pitch)
      np_msg.yaw_deg = parseFloat(yaw)
      const init = init_np ? init_np : false
      if (init === false){
        this.sendNavPoseMsg(namespace,np_msg)
      }
      else {
        this.sendInitNavPoseMsg(namespace,np_msg)
      }
    }



    @action.bound
    sendNavPosePositionMsg(namespace,x,y,z,init_np){
      var np_msg = this.blankNavpose
      np_msg.has_position = true
      np_msg.time_position = moment.utc().unix()
      np_msg.x_m = parseFloat(x)
      np_msg.y_m = parseFloat(y)
      np_msg.z_m = parseFloat(z)
      const init = init_np ? init_np : false
      if (init === false){
        this.sendNavPoseMsg(namespace,np_msg)
      }
      else {
        this.sendInitNavPoseMsg(namespace,np_msg)
      }
    }





    @action.bound
    sendNavPoseAltitudeMsg(namespace,alt,init_np){
      var np_msg = this.blankNavpose
      np_msg.has_altitude = true
      np_msg.time_altitude = moment.utc().unix()
      np_msg.altitude_m = parseFloat(alt)
      const init = init_np ? init_np : false
      if (init === false){
        this.sendNavPoseMsg(namespace,np_msg)
      }
      else {
        this.sendInitNavPoseMsg(namespace,np_msg)
      }
    }


    @action.bound
    sendNavPoseDepthMsg(namespace,depth,init_np){
      var np_msg = this.blankNavpose
      np_msg.has_depth = true
      np_msg.time_depth = moment.utc().unix()
      np_msg.depth_m = parseFloat(depth)
      const init = init_np ? init_np : false
      if (init === false){
        this.sendNavPoseMsg(namespace,np_msg)
      }
      else {
        this.sendInitNavPoseMsg(namespace,np_msg)
      }
    }



    @action.bound
    sendUpdateTransformMsg(namespace, transform_msg, name = '', name2 = '', name3 = '') {
        this.publishMessage({
          name: namespace,
          messageType: "nepi_interfaces/UpdateTransform",
          data: {
            name: name,
            name2: name2,
            name3: name3,
            transform: transform_msg

          },
          noPrefix: true
        })

    }


    // Publish a plain nepi_interfaces/Transform directly to a device's
    // Transform3DIF set topic (e.g. <transform_topic>/set_3d_transform).
    @action.bound
    sendTransformMsg(namespace, transform_msg) {
        this.publishMessage({
          name: namespace,
          messageType: "nepi_interfaces/Transform",
          data: transform_msg,
          noPrefix: true
        })

    }




  @action.bound
  async onInstallFullSysImg(new_img_filename) {
    this.publishMessage({
      name: "install_nepi_image",
      messageType: "std_msgs/String",
      data: { data: new_img_filename }
    })    
  }

  @action.bound
  async onSwitchNepitImage() {
    this.publishMessage({
      name: "switch_nepi_image",
      messageType: "std_msgs/Empty",
      data: {}
    })
  }

  @action.bound
  async onStartSysBackup() {
    this.publishMessage({
      name: "save_nepi_image",
      messageType: "std_msgs/Empty",
      data: {}
    })
  }  



  @action.bound
  setDeviceID({newDeviceID}) {
    this.publishMessage({
      name: "set_device_id",
      messageType: "std_msgs/String",
      data: { data: newDeviceID }
    })
  }


  get_timezone_desc(){
    var timezone = getLocalTZ()
    timezone = timezone.replace(' ','_')
    return timezone
  }


  @action.bound
  onToggleClockUTCMode() {
    this.clockUTCMode = !this.clockUTCMode
    this.clockTZ = this.get_timezone_desc()
  }

  @action.bound
  onSyncTimezone() {
    this.publishMessage({
      name: "set_time",
      messageType: "nepi_interfaces/TimeUpdate",
      data: {
          update_time: false,
          secs: 0,
          nsecs: 0,
          update_timezone: true,
          timezone: this.get_timezone_desc()
      }
    })
  }

  @action.bound
  setTimezone(timezone) {
    this.publishMessage({
      name: "set_time",
      messageType: "nepi_interfaces/TimeUpdate",
      data: {
          update_time: false,
          secs: 0,
          nsecs: 0,
          update_timezone: true,
          timezone:  timezone
      }
    })
  }

  @action.bound
  setTimezoneUTC() {
    this.publishMessage({
      name: "set_time",
      messageType: "nepi_interfaces/TimeUpdate",
      data: {
          update_time: false,
          secs: 0,
          nsecs: 0,
          update_timezone: true,
          timezone:  'Europe/London'
      }
    })
  }



  @action.bound
  syncTime2Device() {
    const utcTS = moment.utc()
      .unix()

    if (this.timeMgrStatus != null) {
      const auto_sync_timezones = this.timeMgrStatus.auto_sync_timezones

      if ( auto_sync_timezones === true){
          this.publishMessage({
            name: "set_time",
            messageType: "nepi_interfaces/TimeUpdate",
            data: {
                update_time: true,
                secs: Math.floor(utcTS),
                nsecs: 0,
                update_timezone: true,
                timezone:  this.timeStatusTimezoneDesc
            }
        })
      }
      else {
          this.publishMessage({
                    name: "set_time",
                    messageType: "nepi_interfaces/TimeUpdate",
                    data: {
                        update_time: true,
                        secs: Math.floor(utcTS),
                        nsecs: 0,
                        update_timezone: false,
                        timezone:  ""
                    }
                })

      }
    }
  }


  @action.bound
  syncTz2Device() {
    this.clockTZ = this.get_timezone_desc()
    this.publishMessage({
      name: "set_time",
      messageType: "nepi_interfaces/TimeUpdate",
      data: {
          update_time: false,
          secs: 0,
          nsecs: 0,
          update_timezone: true,
          timezone:  this.clockTZ
      }
    })
  }

  @action.bound
  syncTimeTz2Device() {
    const utcTS = moment.utc().unix()
    this.clockTZ = this.get_timezone_desc()
    this.publishMessage({
      name: "set_time",
      messageType: "nepi_interfaces/TimeUpdate",
      data: {
          update_time: true,
          secs: Math.floor(utcTS),
          nsecs: 0,
          update_timezone: true,
          timezone:  this.clockTZ
      }
    })
  }

  @action.bound
  onChangeTriggerRate(rate) {
    let freq = parseFloat(rate)

    if (isNaN(freq)) {
      freq = 0
    }

    this.publishMessage({
      name: "set_periodic_sw_trig",
      messageType: "nepi_interfaces/PeriodicSwTrig",
      data: {
        enabled: freq > 0,
        sw_trig_mask: this.triggerMask,
        rate_hz: freq
      }
    })

    // Update the state variable -- if rejected, will get set back on next periodic update
    // This allows the text entry box to immediately switch to using the state variable
    this.triggerAutoRateHz = freq
  }

  @action.bound
  onChangeTXRateLimit(limit) {
    let lim = parseInt(limit, 10)

    if (isNaN(lim)) {
      lim = -1
    }

    this.publishMessage({
      name: "set_tx_bw_limit_mbps",
      messageType: "std_msgs/Int32",
      data: { data: lim }
    })

    // Update locally for display purposes... will be corrected on next periodic update if value is rejected
    this.bandwidth_usage_query_response.tx_limit_mbps = lim
  }

  @action.bound
  onToggleHWTriggerOutputEnabled(e) {
    const checked = e.target.checked

    // If HW Trig Output Enable selected, trig mask is 0xFFFFFFFF, otherwise trig mask is 0x7FFFFFFF
    this.triggerMask = checked
      ? TRIGGER_MASKS.OUTPUT_ENABLED
      : TRIGGER_MASKS.DEFAULT

    // republish rate change with new mask
    this.onChangeTriggerRate({
      target: {
        value: this.triggerAutoRateHz
      }
    })
  }

  @action.bound
  onToggleHWTriggerInputEnabled(e) {
    const checked = e.target.checked

    this.publishMessage({
      name: "hw_trigger_in_enab",
      messageType: "std_msgs/UInt32",
      data: { data: checked ? this.triggerMask : 0 }
    })
  }

  @action.bound
  onToggleDHCPEnabled(e) {
    const checked = e.target.checked

    this.publishMessage({
      name: "enable_dhcp",
      messageType: "std_msgs/Bool",
      data: { data: checked }
    })

    // Set local immediately, will correct on next update if values is rejected
    this.ip_query_response.dhcp_enabled = checked
  }

  @action.bound
  onToggleWifiAPEnabled(e) {
    const checked = e.target.checked

    this.publishMessage({
      name: "enable_wifi_access_point",
      messageType: "std_msgs/Bool",
      data: { data: checked }
    })
  }

  @action.bound
  onToggleWifiClientEnabled(e) {
    const checked = e.target.checked

    this.publishMessage({
      name: "enable_wifi_client",
      messageType: "std_msgs/Bool",
      data: { data: checked }
    })
  }

  @action.bound
  onUpdateWifiClientCredentials(new_ssid, new_passphrase) {
    this.publishMessage({
      name: "set_wifi_client_credentials",
      messageType: "nepi_interfaces/NetworkWifiCredentials",
      data: { ssid: new_ssid, passphrase: new_passphrase }
    })
  }

  @action.bound
  onUpdateWifiAPCredentials(new_ssid, new_passphrase) {

    this.publishMessage({
      name: "set_wifi_access_point_credentials",
      messageType: "nepi_interfaces/NetworkWifiCredentials",
      data: { ssid: new_ssid, passphrase: new_passphrase }
    })
  }  

  @action.bound
  onRefreshWifiNetworks() {
    this.publishMessage({
      name: "refresh_available_wifi_networks",
      messageType: "std_msgs/Empty",
      data: {}
    })    
  }

  @action.bound
  onGenerateLicenseRequest() {
    if (this.license_server && (this.license_server.readyState === 1)) { // Connected
      this.license_server.send("license_request")
      this.license_request_mode = true
    }
    else {
      this.license_request_mode = false
    }
  }

  @action.bound
  onPressManualTrigger() {
    // Pressing Manual Trigger publishes mask on the sw_trigger topic.
    this.publishMessage({
      name: "sw_trigger",
      messageType: "std_msgs/UInt32",
      data: { data: this.triggerMask }
    })
  }

  @action.bound
  onToggleSaveDataAll(value) {
      this.publishMessage({
      name: "save_data_enable",
      messageType: "std_msgs/Bool",
      data: {data: value},
      noPrefix: true
    })
  }

  @action.bound
  onToggleSaveUTCAll(value) {
      this.publishMessage({
      name: "save_data_utc",
      messageType: "std_msgs/Bool",
      data: {data: value},
      noPrefix: true
    })
  }

    @action.bound
  onToggleLogNavPoseEnable(value) {
      this.publishMessage({
      name: "log_navpose_enable",
      messageType: "std_msgs/Bool",
      data: {data: value},
      noPrefix: true
    })
  }


  @action.bound
  onChangeSaveRateAll(rate) {
    let freq = parseFloat(rate)

    if (isNaN(freq)) {
      freq = 0
    }

    this.publishMessage({
      name: "save_data_rate",
      messageType: "nepi_interfaces/SaveDataRate",
      data: {
        data_product: "Active",
        save_rate_hz: freq,
      }
    })

    this.saveFreqHz = freq
  }




  @action.bound
  addIPAddr(addr) {
    this.publishMessage({
      name: "add_ip_addr",
      messageType: "std_msgs/String",
      data: { data: addr }
    })
  }

  @action.bound
  removeIPAddr(addr) {
    this.publishMessage({
      name: "remove_ip_addr",
      messageType: "std_msgs/String",
      data: { data: addr }
    })
  }





  @action.bound
  saveSettingsFilePrefix({newFilePrefix}) {
    this.publishMessage({
      name: "save_data/save_data_prefix",
      messageType: "std_msgs/String",
      data: { data: newFilePrefix }
    })
  }

  @action.bound
  deleteAllData() {
    this.publishMessage({
      name: "clear_data_folder",
      messageType: "std_msgs/Empty",
      data: {}
    })
  }

  @action.bound
  onToggleAutoStartEnabled(autoStartScriptName, isEnabled) {
    this.publishMessage({
      name: "enable_script_autostart",
      messageType: "nepi_interfaces/AutoStartEnabled",
      data: { 
        script: autoStartScriptName,
        enabled: isEnabled
       }
    })
  }

  // Control methods //////////////////////////////////////////////
  @action.bound
  isThrottled() {
    var now = new Date()
    if (now - this.lastUpdate < UPDATE_PERIOD) {
      return true
    }
    this.lastUpdate = now
    return false
  }
  
  @action.bound
  publishValue(
    topic,
    msgType,
    value,
    throttle = true,
    noPrefix = false,
  ) {
    if (throttle && this.isThrottled()) {
      return
    }

    this.publishMessage({
      name: topic,
      messageType: msgType,
      data: {data: Number(value)},
      noPrefix: noPrefix,
    })
  }


  @action.bound
  publishRangeWindow(topic, min, max, throttle = true) {
    if (throttle){
      if (throttle && this.isThrottled()) {
        return
      }
    }
    if (topic) {
      this.publishMessage({
        name: topic,
        messageType: "nepi_interfaces/RangeWindow",
        noPrefix: true,
        data: {
          start_range: min,
          stop_range: max
        }
      })
    } else {
      console.warn("publishRangeWindow: topic not set")
    }
  }



  /////////////////////////////////////////////////////////////////////////

  // Image Control methods /////////////////////////////////////////////
 

  @action.bound
  onChangeStreamingImageQuality(quality) {
    this.streamingImageQuality = quality
    this.publishMessage({
      name: "rui_config_mgr/set_streaming_image_quality",
      messageType: "std_msgs/UInt8",
      data: { data: quality }
    })
  }

    @action.bound
  onChangeStreamingImageRate(rate) {
    this.streamingImageRate = rate
    this.publishMessage({
      name: "rui_config_mgr/set_streaming_image_rate",
      messageType: "std_msgs/UInt8",
      data: { data: rate }
    })
  }

  /////////////////////////////////////////////////////////////////////////
  // Nav/Pose Control methods /////////////////////////////////////////////




  /////////////////////////////////////////////////////////////////////////

  // PTX
  @action.bound
  onPTXGoHome(ptxNamespace) {
    this.publishMessage({
      name: ptxNamespace + "/go_home",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })    
  }

  @action.bound
  onPTXSetHomeHere(ptxNamespace) {
    this.publishMessage({
      name: ptxNamespace + "/set_home_position_here",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })    
  }  


  @action.bound
  onPTXStop(ptxNamespace) {
    this.publishMessage({
      name: ptxNamespace + "/stop_moving",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    }) 
  }

  @action.bound
  onPTXPanStop(ptxNamespace) {
    this.publishMessage({
      name: ptxNamespace + "/stop_pan",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    }) 
  }

  @action.bound
  onPTXTiltStop(ptxNamespace) {
    this.publishMessage({
      name: ptxNamespace + "/stop_tilt",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    }) 
  }



  @action.bound
  onSetPTXHomePos(ptxNamespace, panHomePos, tiltHomePos) {
    this.publishMessage({
      name: ptxNamespace + "/set_home_position",
      messageType: "nepi_interfaces/PanTiltPosition",
      data: {"pan_deg": panHomePos, "tilt_deg": tiltHomePos},
      noPrefix: true
    })
  }

  @action.bound
  onSetPTXGotoPos(ptxNamespace, panPos, tiltPos) {
    this.publishMessage({
      name: ptxNamespace + "/goto_position",
      messageType: "nepi_interfaces/PanTiltPosition",
      data: {"pan_deg": panPos, "tilt_deg": tiltPos},
      noPrefix: true
    })
  }

  @action.bound
  onSetPTXGotoPanPos(ptxNamespace, panPos) {
    this.publishMessage({
      name: ptxNamespace + "/goto_pan_position",
      messageType: "std_msgs/Float32",
      data: {"data": panPos},
      noPrefix: true
    })
  }

  @action.bound
  onSetPTXGotoTiltPos(ptxNamespace, tiltPos) {
    this.publishMessage({
      name: ptxNamespace + "/goto_tilt_position",
      messageType: "std_msgs/Float32",
      data: {"data": tiltPos},
      noPrefix: true
    })
  }

  @action.bound
  onSetPTXSoftStopPos(ptxNamespace, panMin, panMax, tiltMin, tiltMax) {
    this.publishMessage({
      name: ptxNamespace + "/set_soft_limits",
      messageType: "nepi_interfaces/PanTiltLimits",
      data: {"min_pan_deg": panMin,
             "max_pan_deg": panMax,
             "min_tilt_deg": tiltMin,
             "max_tilt_deg": tiltMax},
      noPrefix: true
    })
  }

  @action.bound
  onSetPTXHardStopPos(ptxNamespace, panMin, panMax, tiltMin, tiltMax) {
    this.publishMessage({
      name: ptxNamespace + "/set_hard_limits",
      messageType: "nepi_interfaces/PanTiltLimits",
      data: {"min_pan_deg": panMin,
             "max_pan_deg": panMax,
             "min_tilt_deg": tiltMin,
             "max_tilt_deg": tiltMax},
      noPrefix: true
    })
  }

  @action.bound
  onPTXJogPan(ptxNamespace, direction) {
    this.publishMessage({
      name: ptxNamespace + "/jog_timed_pan",
      messageType: "nepi_interfaces/SingleAxisTimedMove",
      data: {"direction": direction,
             "duration_s": -1},
      noPrefix: true
    })
  }

  @action.bound
  onPTXJogTilt(ptxNamespace, direction) {
    this.publishMessage({
      name: ptxNamespace + "/jog_timed_tilt",
      messageType: "nepi_interfaces/SingleAxisTimedMove",
      data: {"direction": direction,
             "duration_s": -1},
      noPrefix: true
    })
  }

  @action.bound
  onPTXJogSpeedPan(ptxNamespace, direction, speed_ratio) {
    this.publishMessage({
      name: ptxNamespace + "/jog_timed_pan_speed_ratio",
      messageType: "nepi_interfaces/SingleAxisTimedSpeedMove",
      data: {"direction": direction,
             "speed_ratio": speed_ratio,
             "duration_s": -1},
      noPrefix: true
    })
  }

  @action.bound
  onPTXJogSpeedTilt(ptxNamespace, direction, speed_ratio) {
    this.publishMessage({
      name: ptxNamespace + "/jog_timed_tilt_speed_ratio",
      messageType: "nepi_interfaces/SingleAxisTimedSpeedMove",
      data: {"direction": direction,
             "speed_ratio": speed_ratio,
             "duration_s": -1},
      noPrefix: true
    })
  }

  
  @action.bound
  onLSXSetStandby(lsxNamespace, enable) {
    this.publishMessage({
      name: lsxNamespace + "/set_standby",
      messageType: "std_msgs/Bool",
      data: {"data" : enable},
      noPrefix: true
    })
  }

  @action.bound
  onLSXSetStrobeEnable(lsxNamespace, enable) {
    this.publishMessage({
      name: lsxNamespace + "/set_strobe_enable",
      messageType: "std_msgs/Bool",
      data: {"data" : enable},
      noPrefix: true
    })
  }

}
const stores = {
  ros: new ROSConnectionStore(),
}

export default stores
