#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com


### Set the namespace before importing nepi_sdk
import os
import copy
import serial
import serial.tools.list_ports
import time
import re
import sys

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls

from nepi_interfaces.msg import DeviceLSXStatus

from nepi_api.device_if_lsx import LSXDeviceIF
from nepi_api.messages_if import MsgIF




PKG_NAME = 'LSX_SIDUS_SS182_SERIAL'
FILE_TYPE = 'NODE'


DEFAULT_MIN = '1'
DEFAULT_MAX = '50'
DEFAULT_CURVE = [DEFAULT_MIN,DEFAULT_MAX,'1','5','70','95','4.5']



#########################################
# Sealite LSX Driver Node Class
#########################################

class SidusSS182SerialNode(object):
  ### LXS Driver Settings
  # Set driver capability parameters

  #######################
  DEFAULT_NODE_NAME='sealite'


  CAP_SETTINGS = dict(
    min_intensity_percent = {"type":"Int","name":"min_intensity_percent","options":["1","100"]},
    max_intensity_percent =  {"type":"Int","name":"max_intensity_percent","options":["1","100"]}
  )

  FACTORY_SETTINGS = dict(
    min_intensity_percent = {"type":"Int","name":"min_intensity_percent","value":str(DEFAULT_MIN)},
    max_intensity_percent =  {"type":"Int","name":"max_intensity_percent","value": str(DEFAULT_MAX)}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  settingFunctions = dict(
    min_intensity_percent = {'get':'getMinIntensityPercent', 'set': 'setMinIntensityPercent'},
    max_intensity_percent = {'get':'getMaxIntensityPercent', 'set': 'setMaxIntensityPercent'}
  )
  
  #Factory Control Values 
  FACTORY_CONTROLS = dict( standby_enabled = False,
  on_off_state = False,
  intensity_ratio = 0.0,
  strobe_enabled = False,
  blink_interval_sec = 2,
  blink_enabled = False
  )

  device_info_dict = dict(device_name = "",
                        path = "",
                        serial_number = "",
                        hw_version = "",
                        sw_version = "")

  init_settings_dict = dict()
  settings_dict = dict()
  
  # Initialize some parameters
  serial_num = ""
  hw_version = ""
  sw_version = ""

  connect_attempts = 0


  serial_port = None
  serial_busy = False
  connected = False

  on_off_state = False
  standby_state = False
  intensity_ratio = 0.0
  strobe_state = False
  
  temp_c = 0.0

  self_check_count = 5
  self_check_counter = 0 # track sequencial message failures

  lsx_if = None

  addr_str = ""

  cur_curve = DEFAULT_CURVE

  CONFIGS_DICT = {
        'Standard' : {'data_len': 4},
  }
  config_dict = CONFIGS_DICT['Standard']
  data_len = 4
  ### LXS Driver NODE Initialization
  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
  drv_dict = dict()                                                    
  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################  
    # Create Msg Class
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting Node Initialization Processes")

  ##############################  
  # Initialize Class Variables

    # Get required drv driver dict info

    try:
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict()) 
        #self.msg_if.pub_warn("Got Drivers_Dict from param server: " + str(self.drv_dict))
        self.device_name = self.drv_dict['DRIVER_DICT']['device_name']
        self.device_name = self.drv_dict['DRIVER_DICT']['device_path']
        self.port_str = self.drv_dict['DEVICE_DICT']['device_path'] 
        self.baud_str = self.drv_dict['DEVICE_DICT']['baud_str'] 
        self.baud_int = int(self.baud_str)
        self.addr_str = self.drv_dict['DEVICE_DICT']['addr_str'] 

        system_config = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['system_config']['value']
        if system_config in self.CONFIGS_DICT.keys():
            self.config_dict = self.CONFIGS_DICT[system_config]
        self.data_len = self.config_dict['data_len']
    except Exception as e:
        self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
        return

    ################################################  
    self.msg_if.pub_info("Connecting to Device on port " + self.port_str + " with baud " + self.baud_str)
    ### Try and connect to device
    while self.connected == False and self.connect_attempts < 5:
        self.connected = self.connect() 
        if self.connected == False:
            nepi_sdk.sleep(1)
    if self.connected == False:
        self.msg_if.pub_info("Shutting down node")
        self.msg_if.pub_info("Specified serial port not available")
        nepi_sdk.signal_shutdown("Serial port not available")   
    else:
        ################################################
        self.msg_if.pub_info("... Connected!")
        self.dev_info = self.driver_getDeviceInfo()
        self.logDeviceInfo()
        # Initialize settings
        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()
          
        # Launch the LSX interface --  this takes care of initializing all the camera settings from config. file
        self.msg_if.pub_info("Launching NEPI LSX () interface...")
        self.msg_if.pub_info("0")
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version
        self.msg_if.pub_info("4")

        self.lsx_if = LSXDeviceIF(
                    device_info = self.device_info_dict, 
                    getStatusFunction = self.getStatus,
                    getSettingsFunction=self.getSettingsFunction,
                    setSettingFunction=self.setSettingFunction,
                    factoryControls = self.FACTORY_CONTROLS,
                    standbyEnableFunction = None,
                    turnOnOffFunction = self.turnOnOff,
                    setIntensityRatioFunction = self.setIntensityRatio, 
                    blinkOnOffFunction = None,
                    reports_temp = True, 
                    reports_power = False
                    )
        self.msg_if.pub_info("5")

        #self.turnOnOff(False)
        # Start an sealite activity check process that kills node after some number of failed comms attempts
        self.msg_if.pub_info("Starting an activity check process")
        nepi_sdk.start_timer_process((0.2), self.check_timer_callback)
        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever (until object is detected)
        nepi_sdk.spin()
        """         else:
        self.msg_if.pub_info("Shutting down node")
        self.msg_if.pub_info("Specified serial port not available")
        nepi_sdk.signal_shutdown("Serial port not available")    """




      #**********************
      # Device setting functions

  def logDeviceInfo(self):
      dev_info_string = self.node_name + " Device Info:\n"
      dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
      dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
      dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
      dev_info_string += "Serial Number: " + self.dev_info["SerialNum"] + "\n"
      self.msg_if.pub_info(dev_info_string)


  def initSettingsDict(self):
      init_settings_dict = dict()
      for setting_name in self.CAP_SETTINGS.keys():
        cap_setting = self.CAP_SETTINGS[setting_name]
        setting_dict = dict()
        setting_dict['type'] = cap_setting['type']
        # The retired cap-settings form carried an Int control's min and max in
        # an 'options' pair. The controls contract calls that 'bounds'.
        if 'options' in cap_setting.keys():
          try:
            setting_dict['bounds'] = [int(cap_setting['options'][0]),int(cap_setting['options'][1])]
          except Exception as e:
            self.msg_if.pub_warn("Invalid bounds for setting: " + setting_name + " : " + str(e))
        default = None
        if setting_name in self.FACTORY_SETTINGS.keys():
          try:
            default = int(self.FACTORY_SETTINGS[setting_name]['value'])
          except Exception as e:
            self.msg_if.pub_warn("Invalid factory value for setting: " + setting_name + " : " + str(e))
        if setting_name in self.FACTORY_SETTINGS_OVERRIDES.keys():
          default = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
        if default is None:
          continue
        setting_dict['default'] = default
        init_settings_dict[setting_name] = setting_dict

      self.init_settings_dict = init_settings_dict
      settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
      settings_dict_values = nepi_controls.get_controls_values_dict(settings_dict)
      self.msg_if.pub_info("Initialized Settings: " + str(settings_dict_values))
      return settings_dict


  def refreshSettingsDict(self):
      # This is a serial device whose settings are held on the device rather
      # than reported as a live capability report, so this only reads current
      # values back. Bounds and options do not move.
      settings_dict = copy.deepcopy(self.settings_dict)
      for setting_name in settings_dict.keys():
        if setting_name not in self.settingFunctions.keys():
          continue
        try:
          get_function = globals()[self.settingFunctions[setting_name]['get']]
          val = get_function(self)
        except Exception as e:
          self.msg_if.pub_warn("Failed to read setting " + setting_name + " : " + str(e))
          continue
        if val is None:
          continue
        try:
          settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, int(val))
        except Exception as e:
          self.msg_if.pub_warn("Failed to apply read setting " + setting_name + " : " + str(e))
      return settings_dict


  def getSettingsFunction(self):
      return self.settings_dict


  def setSettingFunction(self,setting_name,setting_value):
      setting_str = setting_name + ":" + str(setting_value)
      success = False
      msg = 'Success'
      if setting_name not in self.settings_dict.keys():
        msg = (self.node_name + " Setting name " + setting_str + " is not supported")
        return False, msg, self.settings_dict
      if setting_name not in self.settingFunctions.keys():
        msg = (self.node_name + " Setting name " + setting_str + " has no set function")
        return False, msg, self.settings_dict

      try:
        set_function = globals()[self.settingFunctions[setting_name]['set']]
        # The device set functions return a bare success flag, not a
        # (success, msg) pair, and return None on the reject path.
        success = (set_function(self,setting_value) == True)
        if success == True:
          msg = ( self.node_name + " UPDATED SETTINGS " + setting_str)
        else:
          msg = ( self.node_name + " device rejected setting " + setting_str)
      except Exception as e:
        msg = "Failed to set " + setting_str + " : " + str(e)
        self.msg_if.pub_warn(msg)

      self.settings_dict = self.refreshSettingsDict()
      return success, msg, self.settings_dict

    ##############
    ### Settings Functions

  def getMinIntensityPercent(self):
    min_intensity = 0
    return min_intensity



  def getMaxIntensityPercent(self):
    data_str = '000'
    success = False
    ser_msg= ('&' + self.addr_str + "LPMX" + data_str + "R")
    response = self.send_msg(ser_msg)

    if success:
      if response is not None and len(response) >= 9:
          try:
              data_str = response[6:9]
              max_intensity = int(data_str) * 0.1
              self.msg_if.pub_warn("getMaxIntensityPercent: " + str(max_intensity))

              return max_intensity
          except ValueError:
              self.msg_if.pub_warn("Invalid response format")

  def setMaxIntensityPercent(self, val):
      success = False
      data_str = create_zero_prefix_str(val)
      ser_msg= ("&" + self.addr_str + 'LMX' + data_str + 'W')
      response = self.send_msg(ser_msg)
      return success



  #######################
  ### LSX IF Get Status Function

  ### Status callback
  def getStatus(self):
    # update status values from device
    success=self.update_status_values()
    # Create LSX status message
    status_msg= DeviceLSXStatus()
    status_msg.device_node_name = self.node_name
    status_msg.device_name = self.device_info_dict["device_name"]
    status_msg.device_path= self.device_info_dict["path"]
    status_msg.serial_num = self.device_info_dict["serial_number"]
    status_msg.hw_version = self.device_info_dict["hw_version"]
    status_msg.sw_version = self.device_info_dict["sw_version"]
    status_msg.on_off_state = self.on_off_state
    status_msg.standby_state = self.standby_state
    status_msg.intensity_ratio = self.intensity_ratio
    status_msg.strobe_state = self.strobe_state
    status_msg.blink_state = False
    status_msg.blink_interval = 0
    status_msg.temp_c = self.temp_c
    status_msg.power_w = 0
    return(status_msg)


  def update_status_values(self):
    success = True
    '''
    # Update standby status
    #self.msg_if.pub_info("Updating standby status")
    ser_msg= ('!' + self.addr_str + ':STBY?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      if response == "0":
        self.standby_state = False
      elif response == "1":
        self.standby_state = True
      else:
        success = False
      #self.msg_if.pub_info("Standby: " + str(self.standby_state))
    else:
      success = False
    '''
    # Update intensity_ratio status
    #self.msg_if.pub_info("Updating intensity_ratio status")
    ser_msg= ('&' + self.addr_str + 'LIN0000R')
    response = self.send_msg(ser_msg)
    if response is not None and response[0:5] == ser_msg[0:5]:
      try:
        self.intensity_ratio = float(response[6:9])/100
        #self.msg_if.pub_info("Intensity Ratio: " + str(self.intensity_ratio))
      except Exception as i:
        self.intensity_ratio = -999
        self.msg_if.pub_warn("Level response was not valid number")
        success = False
      #self.msg_if.pub_info("Intensity: " + str(self.intensity_ratio))
    else:
      self.intensity_ratio = -999
      success = False
    # Update strobe enable status
    '''
    #self.msg_if.pub_info("Updating strobe enable status")
    ser_msg= ('!' + self.addr_str + ':PMOD?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      if response == "0":
        self.strobe_state = False
      elif response == "1" or response == "2":
       self.strobe_state = True
      else:
        success = False
      #self.msg_if.pub_info("Strobe Enable: " + str(self.strobe_state))
    else:
      success = False
    '''
    # Update temp status
    #self.msg_if.pub_info("Updating temp status")
    ser_msg= ('&' + self.addr_str + '0003R')
    response = self.send_msg(ser_msg)
    if response == ser_msg[0:5]:
      try:
        temp_c = int(float(response[6:9]))
        success = True
        #self.msg_if.pub_info("Temp Deg C: " + str(temp_c))
      except Exception as t:
        temp_c = 255
        self.msg_if.pub_warn("Temp response was not valid number")
        success = False
      #self.msg_if.pub_info("Temp C: " + str(temp_c))
    else:
      temp_c = 255
      success = False
    if temp_c < 0 or temp_c > 255:
      temp_c = 255
    self.temp_c = temp_c
    return success

  #######################
  ### LSX IF Interface Functions
  def turnOnOff(self,turn_on_off):
    self.on_off_state = turn_on_off
    if turn_on_off == False:
      self.setOnFunction()
    else:
      self.setOffFunction()
    
  def blinkOnOff(self,blink_on_off):
    self.on_off_state = turn_on_off
    if turn_on_off == False:
      self.setIntensityFunction(0)
    else:
      self.setIntensityFunction(self.intensity_ratio)


  def setIntensityRatio(self,intensity_ratio):
    success = False
    if intensity_ratio < 0:
      intensity_ratio = 0
    elif intensity_ratio > 1:
      intensity_ratio = 1
    success = self.setIntensityFunction(intensity_ratio)
    if success:
      self.intensity_ratio = intensity_ratio
    return success 

  def setIntensityFunction(self,intensity_ratio):
    success = False
    level_val = int(100*intensity_ratio) * int(self.on_off_state)
    level_str = str(level_val)
    zero_prefix_len = 3-len(level_str)
    for z in range(zero_prefix_len):
      level_str = ('0' + level_str)
    ser_msg = ('&' + self.addr_str + 'LIN' + "0" + level_str + "W")
    self.msg_if.pub_warn("ser_msg: " + str(ser_msg))
    response = self.send_msg(ser_msg)
    self.msg_if.pub_warn("response: " + str(response))

    if response != None and response == ser_msg:
      success = True
    return success 
  
  def setOnFunction(self):
    success = False
    ser_msg = ('&' + self.addr_str + 'LON' + "0000" + "W")
    self.msg_if.pub_warn("ser_msg: " + str(ser_msg))
    response = self.send_msg(ser_msg)
    self.msg_if.pub_warn("response: " + str(response))

    if response != None and response[0:5] == ser_msg[0:5]:
      success = True
    return success  

  def setOffFunction(self):
    success = False
    ser_msg = ('&' + self.addr_str + 'LOF' + "0000" + "W")
    self.msg_if.pub_warn("ser_msg: " + str(ser_msg))
    response = self.send_msg(ser_msg)
    self.msg_if.pub_warn("response: " + str(response))

    if response != None and response[0:5] == ser_msg[0:5]:
      success = True
    return success  
  
  #######################
  ### Class Functions
  def check_timer_callback(self, timer):
      success = False
      # Use DSN command for heartbeat
      ser_msg = ('&' + self.addr_str + 'DSN0000R')
      expected_prefix = ('&' + self.addr_str + 'DSN')
      
      ser_str = (ser_msg + '\r\n')
      b = bytearray()
      b.extend(map(ord, ser_str))
      
      try:
          while self.serial_busy == True and not nepi_sdk.is_shutdown():
              time.sleep(0.01)
          self.serial_busy = True
          self.serial_port.write(b)
      except Exception as e:
          self.msg_if.pub_warn("Failed to send heartbeat message")
      
      time.sleep(.01)
      
      try:
          bs = self.serial_port.readline()
      except Exception as e:
          self.msg_if.pub_warn("Failed to receive heartbeat message")
      
      self.serial_busy = False
      response = bs.decode().strip()
      
      # Check for valid response 
      if response is not None and len(response) >= 9:
          if response[0:len(expected_prefix)] == expected_prefix:
              success = True
      if success:
          self.serial_busy = False
          self.self_check_counter = 0
      else:
          self.serial_busy = True
          self.self_check_counter = self.self_check_counter + 1
          
      if self.self_check_counter > self.self_check_count:
          self.msg_if.pub_warn("Shutting down device: " + self.addr_str + " on port " + self.port_str)
          self.msg_if.pub_warn("Too many comm failures")
          nepi_sdk.signal_shutdown("Too many comm failures")


  ### Function to try and connect to device at given port and baudrate
  def connect(self):
      success = False
      self.connect_attempts += 1

      port_check = self.check_port(self.port_str)
      if port_check is True:
        try:
          # Try and open serial port
          self.msg_if.pub_info("Opening serial port " + self.port_str + " with baudrate: " + self.baud_str)
          self.serial_port = serial.Serial(self.port_str,self.baud_int,timeout = 0.1)
          self.msg_if.pub_info("Serial port opened")
          # Send Message
          self.msg_if.pub_info("Requesting info for device: " + self.addr_str)
          ser_msg = ('&' + self.addr_str + 'DSN0000R')
          #self.msg_if.pub_info("Sending serial string: " + ser_msg)
          response = self.send_msg(ser_msg)
          #self.msg_if.pub_info("Got response message: " + response)
          if response is not None and len(response) == 12:
                expected_prefix = ('&' + self.addr_str + 'DSN')
                if response[0:len(expected_prefix)] == expected_prefix:
                  self.serial_num = response[5:9]
                  self.msg_if.pub_info("Got serial number: " + self.serial_num)
                  self.msg_if.pub_info("Requesting firmware P/N for device: " + self.addr_str)

                  ser_msg = ('&' + self.addr_str + 'DFW0000R')
                  fw_response = self.send_msg(ser_msg)
                  
                  if fw_response is not None and len(fw_response) >= 9:
                      expected_fw_prefix = ('&' + self.addr_str + 'DFW')
                      if fw_response[0:len(expected_fw_prefix)] == expected_fw_prefix:
                          self.sw_version = fw_response[5:9] 
                          self.msg_if.pub_info("Got Software Version: " + self.sw_version)

                      else:
                          self.msg_if.pub_warn("Device returned unexpected Software response format")
                          self.sw_version = "Unknown"
                  else:
                      self.msg_if.pub_warn("Device returned invalid DFW response")
                      self.sw_version = "Unknown"
                  
                  # Set hardware version as unknown since we don't have a specific command for it
                  self.hw_version = "Unknown"
                  
                  self.msg_if.pub_info("Connected to device at address: " + self.addr_str)
                  self.msg_if.pub_info("Serial Number: " + self.serial_num)
                  self.msg_if.pub_info("Firmware P/N: " + self.sw_version)
                  success = True
                    
                else:
                    self.msg_if.pub_warn("Device returned unexpected Serial Number response format")
          else:
              self.msg_if.pub_warn("Device returned invalid Serial Number response: " + str(response) + ' length: ' + str(len(response)))
                
        except Exception as e:
            self.msg_if.pub_warn("Something went wrong with connect function at serial port: " + self.port_str + " (" + str(e) + ")")
      else:
          self.msg_if.pub_warn("serial port not active")
      return success

      ret_addr = response[0:3]
      #self.msg_if.pub_info("Returned address value: " + ret_addr)
      if ret_addr == self.addr_str:
        self.msg_if.pub_info("Connected to device at address: " +  self.addr_str)
        res_split = response.split(',')
        if len(res_split) > 5:
        # Update serial, hardware, and software status values
          self.serial_num = res_split[2]
          self.hw_version = res_split[3]
          self.sw_version = res_split[4]
          success = True


        else:
          self.msg_if.pub_warn("Device returned address: " + ret_addr + " does not match: " +  self.addr_str)
      else:
        self.msg_if.pub_warn("Device returned invalid response")
      """   else:
              self.msg_if.pub_warn("Device returned empty response")
          else:
            self.msg_if.pub_warn("Device returned invalid response")
        except Exception as e:
          self.msg_if.pub_warn("Something went wrong with connect function at serial port at: " + self.port_str + "(" + str(e) + ")" )
      else:
        self.msg_if.pub_warn("serial port not active") """
      return success





  def send_msg(self,ser_msg):
    response = None
    if self.serial_port is not None and not nepi_sdk.is_shutdown():
      ser_str = (ser_msg + '\r\n')
      b=bytearray()
      b.extend(map(ord, ser_str))
      
      sleep_time = .1
      timeout = 2
      timer = 0
      while self.serial_busy == True and timer < timeout and not nepi_sdk.is_shutdown():
          time.sleep(sleep_time ) # Wait for serial port to be available
          timer += sleep_time 
      if timer < timeout:
        self.serial_busy = True
        #print("Sending " + ser_msg + " message")
        try:
          self.serial_port.write(b)
          time.sleep(.01)
          try:
            bs = self.serial_port.readline()
            self.serial_busy = False
            response = bs.decode()
            #print("Send response received: " + response[0:-2])
          except Exception as e1:
            print("Failed to recieve message")
        except Exception as e2:
          print("Failed to send message")
      else:
        print("Serial port write timed out on busy state")
    else:
      print("serial port not defined, returning empty string")
    return response

  ### Function for checking if port is available
  def check_port(self,port_str):
    success = False
    ports = serial.tools.list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      if loc == port_str:
        success = True
    return success

  def create_blank_str(self):
      data_str = ""
      zero_prefix_len = self.data_len-len(data_str)
      for z in range(self.data_len):
          data_str += '0'
      return data_str

  def create_zero_prefix_str(self,count_val):
    data_str = str(count_val)
    zero_prefix_len = self.data_len-len(data_str)
    for z in range(zero_prefix_len):
        data_str = ('0' + data_str)
    return data_str

   #######################
    ### Driver Interface Functions

  def driver_getDeviceInfo(self):
      method_name = sys._getframe().f_code.co_name
      dev_info = dict()
      dev_info["Manufacturer"] = 'Sidus'
      dev_info["Model"] = 'SS182'


      data_str = self.create_blank_str()
      ser_msg= ('&' + self.addr_str + 'ADF' + data_str + 'R')
      response = self.send_msg(ser_msg)

      firmware = ""
      if response is not None:
          firmware = response[5:8]
      dev_info["FirmwareVersion"] = firmware

      dev_info["SerialNum"] = ""
      return dev_info

  #######################
  ### Cleanup processes on node shutdown
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    if self.serial_port is not None:
      self.serial_port.close()
      
if __name__ == '__main__':
  SidusSS182SerialNode()





