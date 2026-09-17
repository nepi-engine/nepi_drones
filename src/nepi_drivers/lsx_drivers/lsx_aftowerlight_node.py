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


import os
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

PKG_NAME = 'LSX_AFTOWERLIGHT'
FILE_TYPE = 'NODE'


#########################################
# Sealite LSX Driver Node Class
#########################################

class AfTowerLightNode(object):

  # This device exposes no device settings, so it registers no
  # getSettingsFunction/setSettingFunction with LSXDeviceIF. Its adjustable
  # values are LSX controls, not settings.

  #Factory Control Values 
  FACTORY_CONTROLS = dict( standby_enabled = False,
    on_off_state = False,
    intensity_ratio = 0.0,
    color_selection = "None",
    kelvin_val = 4000,
    strobe_enabled = False,
    blink_interval_sec = 2,
    blink_enabled = False
  )
  color_options = ['GREEN','YELLOW','RED']
  kelvin_limits = [1000,5000]

  device_info_dict = dict(device_name = "",
                          path = "",
                          serial_number = "",
                          hw_version = "",
                          sw_version = "")
  
  # Initialize some parameters
  serial_num = "Unknown"
  hw_version = "Unknown"
  sw_version = "Unknown"

  serial_port = None
  serial_busy = False
  connected = False

  on_off_state = False
  blink_state = False
  standby_state = False
  strobe_state = False
  intensity_ratio = 0.0
  current_color = 'GREEN'
  last_color = 'GREEN'
  kelvin_val = 4000
  temp_c = 0.0
  power_w = 0.0

  lsx_if = None
  
  last_color = 'GREEN'
  on_off_dict = dict()
  on_off_dict['GREEN'] = [0x14,0x24] # Off,On
  on_off_dict['YELLOW'] = [0x12,0x22]
  on_off_dict['RED'] = [0x11,0x21]



  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
  drv_dict = dict()   
  ### LXS Driver NODE Initialization
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
          #self.msg_if.pub_warn("Nex_Dict: " + str(self.drv_dict))
          self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
          self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
          self.ser_port_str = self.drv_dict['DEVICE_DICT']['device_path'] 
          ser_baud_str = self.drv_dict['DEVICE_DICT']['baud_rate_str'] 
          self.ser_baud_int = int(ser_baud_str)
          self.msg_if.pub_info("Connecting to Device on port " + self.ser_port_str + " with baud " + ser_baud_str)
      except Exception as e:
          self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
          nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
          return  


      ################################################
      ### Try and connect to device
      self.connected = self.connect()
      if self.connected:
        # Create LSX  node
        self.msg_if.pub_info('Connected')
      else:
        self.msg_if.pub_info("Shutting down node")
        self.msg_if.pub_info("Specified serial port not available")
        nepi_sdk.signal_shutdown("Serial port not available")  


      #Start with off. IF will set to saved value
      self.turnOnOff(False)

      # Launch the LSX interface --  this takes care of initializing all the camera settings from config. file
      self.msg_if.pub_info("Launching NEPI LSX () interface...")
      self.device_info_dict["device_name"] = self.device_name
      self.device_info_dict["path"] = self.device_path
      self.device_info_dict["serial_number"] = self.serial_num
      self.device_info_dict["hw_version"] = self.hw_version
      self.device_info_dict["sw_version"] = self.sw_version



      self.lsx_if = LSXDeviceIF(

                  device_info = self.device_info_dict, 
                  getStatusFunction = self.getStatus,
                  factoryControls = self.FACTORY_CONTROLS,
                  standbyEnableFunction = None, #self.setStandby,
                  turnOnOffFunction = self.turnOnOff,
                  setIntensityRatioFunction = None, #self.setIntensityRatio, 
                  color_options_list =  self.color_options, 
                  setColorFunction = self.setColorOption,
                  kelvin_limits_list = None, #self.kelvin_limits, 
                  setKelvinFunction =None, #self.setKelvinVal,
                  blinkOnOffFunction = None,
                  enableStrobeFunction = None, #self.setStrobeEnable,
                  reports_temp = False, 
                  reports_power = False
                 )
    
      # Initialization Complete
      self.msg_if.pub_info("Initialization Complete")
      #Set up node shutdown
      nepi_sdk.on_shutdown(self.cleanup_actions)
      # Spin forever (until object is detected)
      nepi_sdk.spin() 




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
    status_msg.standby_state = self.standby_state
    status_msg.on_off_state = self.on_off_state
    status_msg.strobe_state = self.strobe_state
    status_msg.intensity_ratio = self.intensity_ratio
    status_msg.blink_state = False
    status_msg.blink_interval = 0
    status_msg.color_setting = self.current_color
    status_msg.kelvin_setting = int(self.kelvin_val)
    status_msg.temp_c = int(self.temp_c)
    status_msg.power_w = self.power_w
    return(status_msg)


  def update_status_values(self):
    success = True
    return success

  #######################
  ### LSX IF Interface Functions


  def turnOnOff(self,turn_on_off):
    ON = self.on_off_dict[self.current_color][0]
    OFF = self.on_off_dict[self.current_color][1]
    if turn_on_off == True:
      self.send_msg(ON)
    else:
      self.send_msg(OFF)
    self.on_off_state = turn_on_off

  def setColorOption(self,color_option):
    last_color = self.current_color
    cur_state = self.on_off_state or self.blink_state
    if color_option in self.color_options:
      if last_color != color_option:
          if cur_state == True:
            CUR_OFF = self.on_off_dict[self.current_color][1]
            self.send_msg(CUR_OFF)
          self.current_color = color_option
          if cur_state == True:
            CUR_ON = self.on_off_dict[self.current_color][0]
            self.send_msg(CUR_ON)
  
  #######################
  ### Class Functions


  ### Function to try and connect to device at given port and baudrate
  def connect(self):
    success = False
    check_ser_port = self.check_port(self.ser_port_str)
    if check_ser_port == True:
      try: 
        self.serial_port = serial.Serial(self.ser_port_str,self.ser_baud_int,timeout = 0.1)
        #self.msg_if.pub_warn("Serial port open")
        success = True
      except:
        self.msg_if.pub_warn("Failed to open serial port")
      self.serial_port
    else: 
      self.msg_if.pub_warn("Failed to find serial port")
    return success


  def send_msg(self,ser_msg):
    response = None
    if self.serial_port is not None and not nepi_sdk.is_shutdown():
      sleep_time = .1
      timeout = 2
      timer = 0
      while self.serial_busy == True and timer < timeout and not nepi_sdk.is_shutdown():
          time.sleep(sleep_time ) # Wait for serial port to be available
          timer += sleep_time 
      self.serial_busy = True
      try:
        
        self.serial_port.write(bytes([ser_msg]))
      except Exception as e:
        self.msg_if.pub_info("Failed to send message")
    else:
      self.msg_if.pub_info("serial port not defined, returning empty string")
    self.serial_busy = False
    return response

  ### Function for checking if port is available
  def check_port(self,port_str):
    success = False
    ports = serial.tools.list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      self.msg_if.pub_info("Checking port: " + loc)
      self.msg_if.pub_info("For port: " + port_str)
      if loc == port_str:
        success = True
    return success

  #######################
  ### Cleanup processes on node shutdown
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    if self.serial_port is not None:
      self.serial_port.close()
      
if __name__ == '__main__':
  AfTowerLightNode()






