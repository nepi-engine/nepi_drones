#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
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

import os
import time 
import copy


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header

from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryResponse, DeviceInfoQueryRequest

from nepi_interfaces.msg import DeviceLSXStatus
from nepi_interfaces.srv import LSXCapabilitiesQuery, LSXCapabilitiesQueryRequest, LSXCapabilitiesQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SettingsIF

# NavPose ConnectNavPosesIF retired (class removed from connect_data_if); LSX has no navpose source.
#from nepi_api.connect_data_if import ConnectNavPosesIF


class LSXDeviceIF:
    STATUS_UPDATE_RATE_HZ = 1
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]

    #Factory Control Values 
    FACTORY_CONTROLS_DICT = dict( standby_enabled = False,
    on_off_state = False,
    intensity_ratio = 0.0,
    color_selection = "None",
    kelvin_val = 4000,
    strobe_enbled = False,
    blink_interval_sec = 0,
    )

    # Define class variables

    ready = False

    status_msg = DeviceLSXStatus()
    info_report = DeviceInfoQueryResponse()
    caps_report = LSXCapabilitiesQueryResponse()

    getStatusFunction = None
    device_name = ""
  
    has_standby_mode = False
    has_on_off_control = False
    has_intensity_control = False
    has_color_control = False
    color_options_list = ["None"]
    has_kelvin_control = False
    kelvin_limits_list = [1000,5000]
    has_blink_control = False
    has_hw_strobe = False
    reports_temp = False
    reports_power = False


    node_if = None
    settings_if = None
    save_data_if = None
    navpose_if = None
    
    rbx_status_pub_interval = float(1)/float(STATUS_UPDATE_RATE_HZ)

    navpose_frame = 'nepi_frame'

    data_source_description = 'lighting_device'
    data_ref_description = 'device'


    #######################
    ### IF Initialization    
    def __init__(self, device_info, 
                 getSettingsFunction=None, setSettingFunction=None, 
                 factoryControls = None, 
                 data_source_description = 'lighting_device',
                 data_ref_description = 'device',
                 getStatusFunction = None,
                 standbyEnableFunction = None, turnOnOffFunction = None,
                 setIntensityRatioFunction = None, 
                 color_options_list =  None, setColorFunction = None,
                 kelvin_limits_list = None, setKelvinFunction = None,
                 enableStrobeFunction = None,
                 blinkOnOffFunction = None,
                 reports_temp = False, reports_power = False,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,'lsx')

        ##############################  
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting LSX Device IF Initialization Processes", log_name_list = self.log_name_list)


        ##############################
        # Initialize Class Variables
        self.device_name = device_info["device_name"]
        self.path = device_info["path"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]
        
        self.status_msg.device_name = self.device_name
        self.status_msg.device_path = self.path
        self.status_msg.device_node_name = self.node_name
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

        
        self.info_report.device_name = self.device_name
        self.info_report.device_path = self.path
        self.info_report.node_name = self.node_name
        self.info_report.node_namespace = self.node_namespace
        self.info_report.serial_num = self.serial_num
        self.info_report.hw_version = self.hw_version
        self.info_report.sw_version = self.sw_version
        self.info_report.type = 'LSX'


        self.caps_report.device_name = self.device_name
        self.caps_report.device_path = self.path
        self.caps_report.device_node_name = self.node_name

        # Create and update factory controls dictionary
        self.factory_controls_dict = self.FACTORY_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]


        ##############################
        self.initCb(do_updates = False)


        
        # Start LSX node control subscribers
        self.standbyEnableFunction = standbyEnableFunction
        if self.standbyEnableFunction is not None:
            self.has_standby_mode = True
            
        self.turnOnOffFunction = turnOnOffFunction
        if self.turnOnOffFunction is not None:
            self.has_on_off_control = True
            
        self.setIntensityRatioFunction = setIntensityRatioFunction
        if self.setIntensityRatioFunction is not None:
            self.has_intensity_control = True
            
        self.setColorFunction = setColorFunction
        if self.setColorFunction is not None:
            self.has_color_control = True
            if color_options_list != None:
              self.color_options_list = color_options_list
            else:
              self.color_options_list = ["None"]
        else:
            self.color_options_list = ["None"]
            
        self.setKelvinFunction = setKelvinFunction
        
        if self.setKelvinFunction is not None:
            self.has_kelvin_control = True
            if kelvin_limits_list != None:
              self.kelvin_limits_list = kelvin_limits_list
            else:
              self.kelvin_limits_list = [1000, 5000]
        else:
            self.kelvin_limits_list = [1000, 5000]
            

        self.blinkOnOffFunction = blinkOnOffFunction
        if self.blinkOnOffFunction is not None:
            self.has_blink_control = True
            

        self.enableStrobeFunction = enableStrobeFunction
        if self.enableStrobeFunction is not None:
            self.has_hw_strobe = True
           

        if reports_temp:
          self.reports_temp = True
    
        if reports_power:
          self.reports_power = True


        # Populate and advertise LSX node capabilities report
        self.caps_report.has_standby_mode = self.has_standby_mode
        self.caps_report.has_on_off_control = self.has_on_off_control
        self.caps_report.has_intensity_control = self.has_intensity_control
        self.caps_report.has_color_control = self.has_color_control
        self.caps_report.color_options_list = self.color_options_list
        self.caps_report.has_kelvin_control = self.has_kelvin_control
        self.caps_report.kelvin_min = self.kelvin_limits_list[0]
        self.caps_report.kelvin_max = self.kelvin_limits_list[1]
        self.caps_report.has_blink_control = self.has_blink_control
        self.caps_report.has_hw_strobe = self.has_hw_strobe
        self.caps_report.reports_temperature = self.reports_temp
        self.caps_report.reports_power = self.reports_power


        self.getStatusFunction = getStatusFunction


        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description
        ##################################################
        ### Node Class Setup

        self.msg_if.pub_debug("Starting Node IF Initialization", log_name_list = self.log_name_list)
        alt_namespace = None
        if self.device_name != self.node_name:
            alt_namespace = self.node_namespace.replace(self.node_name,self.device_name)
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.namespace,
                'alt_namespace': alt_namespace
        }



        # Params Config Dict ####################

        self.PARAMS_DICT = {
            'standby_enabled': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("standby_enabled")
            },
            'on_off_state': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("on_off_state")
            },
            'intensity_ratio': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("intensity_ratio")
            },
            'color_selection': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("color_selection")
            },
            'kelvin_val': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("kelvin_val")
            }, 
            'strobe_enbled': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("strobe_enbled")
            },
            'blink_interval_sec': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("blink_interval_sec")
            }
        }

        # Services Config Dict ####################

        self.SRVS_DICT = {
            'device_info_query': {
                'namespace': self.namespace,
                'topic': 'device_info_query',
                'srv': DeviceInfoQuery,
                'req': DeviceInfoQueryRequest(),
                'resp': DeviceInfoQueryResponse(),
                'callback': self.info_query_callback
            },
            'capabilities_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': LSXCapabilitiesQuery,
                'req': LSXCapabilitiesQueryRequest(),
                'resp': LSXCapabilitiesQueryResponse(),
                'callback': self.capabilities_query_callback
            }
        }



        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DeviceLSXStatus,
                'qsize': 1,
                'latch': True
            }
        }



        self.SUBS_DICT = {
            'set_standby': {
                'namespace': self.namespace,
                'topic': 'set_empty',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setStandbyCb, 
                'callback_args': ()
            },
            'turn_on_off': {
                'namespace': self.namespace,
                'topic': 'turn_on_off',
                'msg': Bool,
                'qsize': 1,
                'callback': self.turnOnOffCb, 
                'callback_args': ()
            },
            'set_intensity_ratio': {
                'namespace': self.namespace,
                'topic': 'set_intensity_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setIntensityRatioCb, 
                'callback_args': ()
            },
            'set_color': {
                'namespace': self.namespace,
                'topic': 'set_color',
                'msg': String,
                'qsize': 1,
                'callback': self.setColorCb, 
                'callback_args': ()
            },
            'set_kelvin': {
                'namespace': self.namespace,
                'topic': 'set_kelvin',
                'msg': Int32,
                'qsize': 1,
                'callback': self.setKelvinCb, 
                'callback_args': ()
            },
            'set_blink_interval': {
                'namespace': self.namespace,
                'topic': 'set_blink_interval',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setBlinkIntervalCb, 
                'callback_args': ()
            },
            'set_strobe_enable': {
                'namespace': self.namespace,
                'topic': 'set_strobe_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setStrobeEnableCb, 
                'callback_args': ()
            },
            'reset_controls': {
                'namespace': self.namespace,
                'topic': 'reset_controls',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetControlsCb, 
                'callback_args': ()
            },
            'set_navpose_frame': {
                'namespace': self.namespace,
                'topic': 'set_navpose_frame',
                'msg': String,
                'qsize': 1,
                'callback': self.setMountDescCb, 
                'callback_args': ()
            }



        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.initCb(do_updates = True)
        self.publish_status()


        status_update_time = float(1)/self.STATUS_UPDATE_RATE_HZ
        nepi_sdk.start_timer_process(status_update_time, self.statusTimerCb) 
        #nepi_sdk.start_timer_process(delay, self._publishNavPoseCb, oneshot = True)

        self.publish_status()



        ###############################
        # Setup Settings IF Class ####################
        self.getSettingsFunction = getSettingsFunction
        self.setSettingFunction = setSettingFunction
        if self.getSettingsFunction is not None and self.setSettingFunction is not None:
            self.msg_if.pub_debug("Starting Settings IF Initialization", log_name_list = self.log_name_list)
            settings_ns = self.namespace

            self.settings_if = SettingsIF(namespace = settings_ns,
                            getSettingsFunction=self.getSettingsFunction, 
                            setSettingFunction=self.setSettingFunction, 
                            use_nodename_prefix=False,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if,
                            node_if = self.node_if
                            )

        
    ####################
        # Setup NavPose IF Class
        # NavPose ConnectNavPosesIF retired (class removed from connect_data_if).
        # LSX has no navpose source, so no navpose IF is created (navpose_if stays None).
        # self.msg_if.pub_info("Starting NavPose IF Initialization")
        # np_namespace = self.namespace
        # self.navpose_if = ConnectNavPosesIF(namespace = np_namespace,
        #                             save_data_if = self.save_data_if,
        #                         log_name_list = self.log_name_list,
        #                         msg_if = self.msg_if,
        #                    node_if = self.node_if)
        

        #####################
        # Update Status Message
        nepi_sdk.sleep(1)
        if self.settings_if is not None:
            self.status_msg.settings_topic = self.settings_if.get_namespace()
            self.msg_if.pub_info("Using settings namespace: " + str(self.status_msg.settings_topic))
        # if self.save_data_if is not None:
        #     self.status_msg.save_data_topic = self.save_data_if.get_namespace()
        #     self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic))
        if self.navpose_if is not None:            
            self.status_msg.navpose_topic = self.navpose_if.get_namespace()
            self.msg_if.pub_info("Using navpose namespace: " + str(self.status_msg.navpose_topic))


        ####################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ####################################


        nepi_sdk.sleep

        ###############################
        # Class Methods

    def check_ready(self):
        """Returns the current ready state of the LSX device interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Blocks until the LSX device interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to
                float('inf'), which waits indefinitely.

        Returns:
            bool: True if the device became ready within the timeout, False if it did not.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready   


    def resetControlsCb(self, msg):
        self.msg_if.pub_info("Resetting LSX Device Controls", log_name_list = self.log_name_list)
        self.node_if.set_param('standby_enabled', self.init_standby_enabled)
        self.node_if.set_param('on_off_state', self.init_on_off_state)
        self.node_if.set_param('intensity_ratio', self.init_intensity_ratio)
        self.node_if.set_param('color_selection', self.init_color_selection)
        self.node_if.set_param('kelvin_val', self.init_kelvin_val)
        self.node_if.set_param('strobe_enbled', self.init_strobe_enbled)
        self.node_if.set_param('blink_interval_sec', self.init_blink_interval_sec)
        self.updateDevice()
        self.publish_status()


    def setMountDescCb(self,msg):
        self.msg_if.pub_info("Recived set navpose_frame message: " + str(msg))
        self.navpose_frame = msg.data
        self.publish_status(do_updates=False) # Updated inline here 
        self.node_if.set_param('navpose_frame', self.navpose_frame)

    def resetMountDescCb(self,msg):
        self.msg_if.pub_info("Recived reset navpose_frame message: " + str(msg))
        self.navpose_frame = 'None'
        self.publish_status(do_updates=False) # Updated inline here 
        self.node_if.set_param('navpose_frame', self.navpose_frame)

    def initConfig(self):
        self.initCb(do_updates = True)

    def initCb(self,do_updates = False):
      if self.node_if is not None:
        self.init_standby_enabled = self.node_if.get_param('standby_enabled')
        self.init_on_off_state = self.node_if.get_param('on_off_state')
        self.init_intensity_ratio = self.node_if.get_param('intensity_ratio')
        self.init_color_selection = self.node_if.get_param('color_selection')
        self.init_kelvin_val = self.node_if.get_param('kelvin_val')
        self.init_strobe_enbled = self.node_if.get_param('strobe_enbled')
        self.init_blink_interval_sec = self.node_if.get_param('blink_interval_sec')

      if do_updates == True:
        self.updateDevice()
      self.publish_status()

    def resetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.reset_params()
      if self.save_data_if is not None:
          self.save_data_if.reset()
      if do_updates == True:
        pass
      self.initCb(do_updates = True)

    def factoryResetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.factory_reset_params()
      if self.save_data_if is not None:
          self.save_data_if.factory_reset()
      if self.settings_if is not None:
          self.settings_if.factory_reset()
      if do_updates == True:
        pass
      self.initCb(do_updates = True)


    def updateDevice(self):
        # Reachable before self.node_if is bound: NodeClassIF.__init__ builds a
        # NodeConfigsIF, whose __init__ calls reset_config() -> this class's
        # resetCb -> initCb(do_updates=True) -> here, all while the
        # 'self.node_if = NodeClassIF(...)' assignment is still evaluating.
        # initCb and resetCb each guard on node_if for that reason; this did not,
        # so every param read below raised AttributeError on None. The premature
        # call is redundant anyway -- initCb(do_updates=True) runs again right
        # after node_if is assigned, which is when the device actually gets pushed.
        if self.node_if is None:
            return
        if self.standbyEnableFunction is not None:
          val = self.node_if.get_param('standby_enabled')
          self.standbyEnableFunction(val)
        if self.turnOnOffFunction is not None:
          val = self.node_if.get_param('on_off_state')
          self.turnOnOffFunction(val)
        if self.setIntensityRatioFunction is not None:
          val = self.node_if.get_param('intensity_ratio')
          self.setIntensityRatioFunction(val)
        if self.setColorFunction is not None:
          val = self.node_if.get_param('color_selection')
          self.setColorFunction(val)
        if self.setKelvinFunction is not None:
          val = self.node_if.get_param('kelvin_val')
          self.setKelvinFunction(val)
        if self.enableStrobeFunction is not None:
          val = self.node_if.get_param('strobe_enbled')
          self.enableStrobeFunction(val)
        


    def setCurrentAsDefault(self):
        if self.settings_if is not None:
          # initialize_settings() has never existed on SettingsIF -- this raised
          # AttributeError on every setCurrentAsDefault(). init() is the method.
          self.settings_if.init(do_updates = False)
        pass # We only use the param server, no member variables to apply to param server
    
    ## Callback to regulary check device comms, track failures, and kill unresponsive device connections
    def statusTimerCb(self,timer):
      #Update the status message
      self.publish_status()

    ### Status callback
    def publish_status(self):
        """Builds and publishes the DeviceLSXStatus message to the status topic.

        Calls the driver-supplied getStatusFunction to obtain the current device state,
        overlays the on/off state from the parameter server, and publishes the resulting
        DeviceLSXStatus message on the latched status publisher.
        """
        self.status_msg.device_name = self.device_name
        # update status values from device
        if self.node_if is not None:
            blink_interval = self.node_if.get_param('blink_interval_sec')
            if self.getStatusFunction is not None:
                status_msg=self.getStatusFunction()
                status_msg.on_off_state = self.node_if.get_param('on_off_state')
                try:
                    if self.node_if is not None:
                        self.node_if.publish_pub('status_pub',status_msg)

                except Exception as e:
                    self.msg_if.pub_info("Failed to publish status msg with exception: " + str(e))



    ### Info callback
    def info_query_callback(self, _):
        """Service handler that returns the device info report.

        Returns:
            DeviceInfoQueryResponse: Response containing device name, path, node name,
                namespace, serial number, hardware version, software version, and type.
        """
        return self.info_report


    ### Capabilities callback
    def capabilities_query_callback(self, _):
        """Service handler that returns the LSX capabilities report.

        Returns:
            LSXCapabilitiesQueryResponse: Response describing which LSX capabilities
                are supported, including standby mode, on/off control, intensity control,
                color control, kelvin control, blink control, hardware strobe,
                temperature reporting, and power reporting.
        """
        return self.caps_report

    ### LSX callbacks
    def setStandbyCb(self, standby_msg):
      self.msg_if.pub_info("Recieved standby message: " + str(standby_msg), log_name_list = self.log_name_list)
      standby=standby_msg.data
      if self.standbyEnableFunction is not None:
        self.standbyEnableFunction(standby)
      self.node_if.set_param('standby_enabled', standby)
      self.publish_status()


    def turnOnOffCb(self, on_off_msg):
      self.msg_if.pub_info("Recieved on off message: " + str(on_off_msg), log_name_list = self.log_name_list)
      on_off=on_off_msg.data
      if self.turnOnOffFunction is not None:
        self.turnOnOffFunction(on_off)
      self.node_if.set_param('on_off_state', on_off)
      self.publish_status()

    ### Set intensity callback
    def setIntensityRatioCb(self, intensity_msg):
      self.msg_if.pub_info("Recieved intensity message: " + str(intensity_msg), log_name_list = self.log_name_list)
      intensity=intensity_msg.data
      if self.setIntensityRatioFunction is not None:
        self.setIntensityRatioFunction(intensity)
      self.node_if.set_param('intensity_ratio', intensity)
      self.publish_status()

    ### Set color selection callback
    def setColorCb(self, color_msg):
      self.msg_if.pub_info("Recieved color selection message: " + str(color_msg), log_name_list = self.log_name_list)
      color = color_msg.data
      if color in self.color_options_list:
        if self.setColorFunction is not None:
          self.setColorFunction(color)
        self.node_if.set_param('color_selection', color)
      self.publish_status()

    ### Set kelvin value callback
    def setKelvinCb(self, kelvin_msg):
      self.msg_if.pub_info("Recieved set kelvin message: " + str(kelvin_msg), log_name_list = self.log_name_list)
      kelvin = kelvin_msg.data
      if kelvin >= self.kelvin_limits_list[0] and kelvin <= self.kelvin_limits_list[1]:
        if self.setKelvinFunction is not None:
          self.setKelvinFunction(kelvin)
        self.node_if.set_param('kelvin_val', kelvin)
      self.publish_status()

    def setStrobeEnableCb(self, strobe_enable_msg):
      self.msg_if.pub_info("Recieved strobe enable message: " + str(strobe_enable_msg), log_name_list = self.log_name_list)
      strobe_enable=strobe_enable_msg.data
      if self.enableStrobeFunction is not None:
        self.enableStrobeFunction(strobe_enable)
      self.node_if.set_param('strobe_enbled', strobe_enable)
      self.publish_status()


    ### Set blink interval callback
    def setBlinkIntervalCb(self, blink_int_msg):
      self.msg_if.pub_info("Recieved blink interval message: " + str(blink_int_msg), log_name_list = self.log_name_list)
      blink_interval = blink_int_msg.data
      if self.blinkOnOffFunction is not None:
        self.blinkOnOffFunction(blink_interval)
      self.node_if.set_param('blink_interval_sec', blink_interval)
      self.publish_status()




      
