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
import inspect
import math
import glob
import copy

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_nav

from nepi_api.device_if_ptx import PTXActuatorIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'PTX_SIDUS_SS109_SERIAL' # Use in display menus
FILE_TYPE = 'NODE'


class SidusSS109SerialPTXNode:

    start_time = nepi_utils.get_time()
    debug_update_delay = -1 # -1 to disable
    debug_times = []
    debug_delays = []
    debug_send_msgs = []
    debug_response_msgs = []
    debug_process_results = []
    debug_process_times = []
    debug_last_time = 0
    debugs_dict = dict()

    BLANK_DEBUG_DICT = dict(
    last_success_time = 0,
    num_requests = 0,
    num_skips = 0,
    num_successes = 0,
    num_fails = 0,
    request_delays = [],
    send_msgs = [],
    response_msgs = [],
    process_results = [],
    process_times = [],
    success_delays = []
    )


    SET_SPEED = False

    MAX_POSITION_UPDATE_RATE = 5
    SERIAL_RECEIVE_DELAY = 0.005

    MIN_SERIAL_SEND_DELAY = 0.01
    MAX_SERIAL_SEND_DELAY = 0.01
    serial_send_delay = 0.01 # Adjusted Atuomatically based on serial_fail_attempts

    MAX_SERIAL_ATTEMPTS=10  # Com Loss Point
    serial_fail_attempts = 0 # Current failed com attempts. Reset on success
    max_serial_fail_attempts = 0 # Tracked Max Attempts
    

    PAN_DEG_DIR = -1
    TILT_DEG_DIR = -1

    LIMITS_DICT = dict()
    LIMITS_DICT['max_pan_hardstop_deg'] = 175
    LIMITS_DICT['min_pan_hardstop_deg'] = -175
    LIMITS_DICT['max_tilt_hardstop_deg'] = 175
    LIMITS_DICT['min_tilt_hardstop_deg'] = -175
    LIMITS_DICT['max_pan_softstop_deg'] = 165
    LIMITS_DICT['min_pan_softstop_deg'] = -165
    LIMITS_DICT['max_tilt_softstop_deg'] = 74
    LIMITS_DICT['min_tilt_softstop_deg'] = -74



    CONFIGS_DICT = {
         'Standard' : {'data_len': 4, 'home':5000, 'deg_per_count':0.0879, 'degpsec_per_count': 0.5, 'max_degpsec': 20, 'hasjogspeed': False},
         'HighTorque' : {'data_len': 4, 'home':5000, 'deg_per_count':0.0879, 'degpsec_per_count': 0.5, 'max_degpsec': 10, 'hasjogspeed': False},
         'HighSpeed' : {'data_len': 4, 'home':5000, 'deg_per_count':0.0879, 'degpsec_per_count': 0.5, 'max_degpsec': 40, 'hasjogspeed': True},
    }
    config_dict = CONFIGS_DICT['Standard']


    PT_DIRECTION_POSITIVE = 1
    PT_DIRECTION_NEGATIVE = -1

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")
    
    # Initialize some parameters
    serial_num = "Unknown"
    hw_version = "Unknown"
    sw_version = "Unknown"
    ptx_if = None

    both_str = '!'
    pan_str = '#'
    tilt_str = '$'


    serial_port = None
    serial_lock = False
    serial_busy = False
    
    connected = False


    self_check_count = 100

    current_position = [0.0,0.0]
    position_times = [0.0,0.0]

    speed_ratio = 0.5
    speed_max_dps = 20

    drv_dict = dict()    


    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
                                                
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
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict()) 
        #self.msg_if.pub_warn("Got Drivers_Dict from param server: " + str(self.drv_dict))
        try:
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']            
            self.port_str = self.drv_dict['DEVICE_DICT']['device_path'] 
            self.baud_str = self.drv_dict['DEVICE_DICT']['baud_str'] 
            self.baud_int = int(self.baud_str)
            self.addr_str = self.drv_dict['DEVICE_DICT']['addr_str'] 

            system_config = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['system_config']['value']
            if system_config in self.CONFIGS_DICT.keys():
                self.config_dict = self.CONFIGS_DICT[system_config]

            self.speed_max_dps = self.config_dict['max_degpsec']

            try:
                system_config = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['system_config']['value']
                if system_config in self.CONsystem_configFIG_DICT.keys():
                    self.config_dict = self.CONFIGS_DICT[system_config]
            except Exception:
                pass
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        ################################################  
        self.msg_if.pub_info("Connecting to Device on port " + self.port_str + " with baud " + self.baud_str)
        ### Try and connect to device
        while self.connected == False and self.serial_fail_attempts < 5:
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
        

            # Launch the PTX interface --  this takes care of initializing all the ptx settings from config. file, subscribing and advertising topics and services, etc.
            # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
            self.msg_if.pub_info("Launching NEPI PTX () interface...")
            self.device_info_dict["device_name"] = self.device_name
            self.device_info_dict["path"] = self.device_path

            #Factory Control Values 
            self.FACTORY_CONTROLS = {
                'frame_id' : self.node_name + '_frame',
                'pan_joint_name' : self.node_name + '_pan_joint',
                'tilt_joint_name' : self.node_name + '_tilt_joint',
                'reverse_pan_control' : False,
                'reverse_tilt_control' : False,
                'speed_ratio' : 0.5,
                'status_update_rate_hz' : 10
            }
            

            # This device exposes no device settings, so it registers no
            # getSettingsFunction/setSettingFunction with PTXActuatorIF. Its
            # adjustable values (speed ratios, soft limits, home position) are
            # PTX controls, not settings. PTXActuatorIF also accepts a
            # getCapSettingsFunction argument, but that argument is inert --
            # it is never forwarded to SettingsIF, which has no such parameter;
            # a control's type, bounds and options now ride in the controls
            # dict itself.

            self.home_pan_deg = 0.0
            self.home_tilt_deg = 0.0


            
            self.ptx_if = PTXActuatorIF(device_info = self.device_info_dict,
                                        factoryControls = self.FACTORY_CONTROLS,
                                        factoryLimits = self.LIMITS_DICT,
                                        stopMovingCb = None, #self.stopMoving,
                                        movePanCb = self.movePan,  # Stop command not working on jog
                                        moveTiltCb = self.moveTilt, # Stop command not working on jog
                                        movePanSpeedRatioCb = self.movePanSpeedRatio,  # Stop command not working on jog
                                        moveTiltSpeedRatioCb = self.moveTiltSpeedRatio, # Stop command not working on jog
                                        getSoftLimitsCb = self.getSoftLimits, # 109 does not return response
                                        setSoftLimitsCb = self.setSoftLimits,
                                        getSpeedMaxCb = self.getSpeedMax,
                                        setSpeedMaxCb = None, #self.setSpeedMax,
                                        getSpeedRatioCb = self.getSpeedRatio,
                                        setSpeedRatioCb = self.setSpeedRatio,
                                        getPanSpeedRatioCb = self.getPanSpeedRatio,
                                        setPanSpeedRatioCb = self.setPanSpeedRatio,
                                        getTiltSpeedRatioCb = self.getTiltSpeedRatio,
                                        setTiltSpeedRatioCb = self.setTiltSpeedRatio,
                                        getPositionCb = self.getPosition,
                                        getPositionTimesCb = self.getPositionTimes,
                                        gotoPositionCb = self.gotoPosition,
                                        gotoPanPositionCb = self.gotoPanPosition,
                                        gotoTiltPositionCb = self.gotoTiltPosition,
                                        goHomeCb = self.goHome,
                                        setHomePositionCb = self.setHomePosition,
                                        setHomePositionHereCb = self.setHomePositionHere,
                                        getNavPoseCb = self.getNavPoseDict,
                                        navpose_update_rate = self.MAX_POSITION_UPDATE_RATE,
                                        deviceResetCb = self.resetDevice,
                                        calibrateCenterCB = self.calibrateCenter,
                                        

                                        )
            self.msg_if.pub_info(" ... PTX interface running")

            # Start an ptx activity check process that kills node after some number of failed comms attempts
            self.msg_if.pub_info("Starting an activity check process")
            update_interval = float(1.0) / self.MAX_POSITION_UPDATE_RATE
            nepi_sdk.start_timer_process(update_interval, self.updatePositionHandler, oneshot = True)
            # Initialization Complete
            self.msg_if.pub_info("Initialization Complete")
            #Set up node shutdown
            nepi_sdk.on_shutdown(self.cleanup_actions)
            # Spin forever (until object is detected)
            nepi_sdk.spin()


    def updatePositionHandler(self,timer):
        stime=nepi_utils.get_time()
        current_position = self.driver_getPosition(wait_on_busy = False)
        if current_position[0] is not None:
            self.current_position[0] = current_position[0]
            self.position_times[0] = nepi_utils.get_time()
        if current_position[1] is not None:
            self.current_position[1] = current_position[1]
            self.position_times[1] = nepi_utils.get_time()
        #self.msg_if.pub_info("Got current position :" + str(self.current_position))
        #self.msg_if.pub_info("Got position times :" + str(self.position_times))
        gtime = nepi_utils.get_time() - stime
        next_delay = max(0.01, float(1.0) / self.MAX_POSITION_UPDATE_RATE - gtime)
        nepi_sdk.start_timer_process(next_delay, self.updatePositionHandler, oneshot = True)

    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["SerialNum"] + "\n"
        self.msg_if.pub_info(dev_info_string)

       
    def getNavPoseDict(self):
        pan_deg, tilt_deg = self.current_position
        navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
        navpose_dict['has_orientation'] = True
        navpose_dict['time_oreantation'] = nepi_utils.get_time()
        navpose_dict['roll_deg'] = 0.0
        navpose_dict['yaw_deg'] = pan_deg * self.PAN_DEG_DIR
        navpose_dict['pitch_deg'] = tilt_deg * self.TILT_DEG_DIR
        return navpose_dict


    #######################
    ### PTX IF Functions

    def stopMoving(self):
        self.driver_stopMotion()

    def movePan(self, direction, duration):
        axis_str = self.pan_str
        if self.ptx_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == 1 else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.PAN_DEG_DIR
            success = self.driver_jog(axis_str = axis_str, direction = direction)

            if success:
                if duration > 0:
                    nepi_sdk.sleep(duration)
                    while success == False:
                        self.driver_stopAxisMotion(axis_str = axis_str)
                        nepi_sdk.sleep(self.serial_send_delay)


    def moveTilt(self, direction, duration):
        axis_str = self.tilt_str
        if self.ptx_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == 1 else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.TILT_DEG_DIR
            success = self.driver_jog(axis_str = axis_str, direction = direction)

            if success:
                if duration > 0:
                    nepi_sdk.sleep(duration)
                    while success == False:
                        self.driver_stopAxisMotion(axis_str = axis_str)
                        nepi_sdk.sleep(self.serial_send_delay)


    def movePanSpeedRatio(self, direction, speed_ratio, duration):

        axis_str = self.pan_str
        if self.ptx_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == 1 else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.PAN_DEG_DIR
            speed_dps = speed_ratio * self.speed_max_dps
            hasjogspeed = self.config_dict['hasjogspeed']
            if hasjogspeed == True:
                success = self.driver_jog_speed_dps(axis_str = axis_str, speed_dps = speed_dps, direction = direction)
            else:
                self.driver_setSpeedRatio(speed_ratio, axis_str=self.pan_str)
                success = self.driver_jog(axis_str=axis_str, direction=direction)
            if success:
                if duration > 0:
                    nepi_sdk.sleep(duration)
                    while success == False:
                        self.driver_stopAxisMotion(axis_str = axis_str)
                        nepi_sdk.sleep(self.serial_send_delay)



    def moveTiltSpeedRatio(self, direction, speed_ratio, duration):
        axis_str = self.tilt_str
        if self.ptx_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == 1 else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.TILT_DEG_DIR
            speed_dps = speed_ratio * self.speed_max_dps

            hasjogspeed = self.config_dict['hasjogspeed']
            if hasjogspeed == True:
                success = self.driver_jog_speed_dps(axis_str = axis_str, speed_dps = speed_dps, direction = direction)
            else:
                self.driver_setSpeedRatio(speed_ratio, axis_str=self.pan_str)
                success = self.driver_jog(axis_str=axis_str, direction=direction)

            

            if success:
                if duration > 0:
                    nepi_sdk.sleep(duration)
                    while success == False:
                        self.driver_stopAxisMotion(axis_str = axis_str)
                        nepi_sdk.sleep(self.serial_send_delay)

        

    def setSoftLimits(self, min_pan,max_pan,min_tilt,max_tilt):
        # TODO: Limits checking and driver unit conversion?
        if self.PAN_DEG_DIR == -1:
            min_pan_adj = max_pan * -1
            max_pan_adj = min_pan * -1
        else:
            min_pan_adj = min_pan
            max_pan_adj = max_pan
        if self.TILT_DEG_DIR == -1:
            min_tilt_adj = max_tilt * -1
            max_tilt_adj = min_tilt * -1
        else:
            min_tilt_adj = min_tilt
            max_tilt_adj = max_tilt

        if (min_pan_adj < max_pan_adj) and (min_tilt_adj < max_tilt_adj):
            self.driver_setSoftLimits(min_pan_adj, max_pan_adj, min_tilt_adj, max_tilt_adj)

    def getSoftLimits(self):
        # TODO: Driver unit conversion?
        [min_pan,max_pan,min_tilt,max_tilt] = self.driver_getSoftLimits()
        soft_limits = [min_pan,max_pan,min_tilt,max_tilt]
        return soft_limits




    def setSpeedRatio(self, ratio):
        # TODO: Limits checking and driver unit conversion?
        self.speed_ratio = ratio
        self.driver_setSpeedRatios(ratio)

    def getSpeedMax(self):
        return self.config_dict['max_degpsec']
    
    def setSpeedMax(self, speed):
        if speed >= 10 and speed <= 40:
            self.config_dict['max_degpsec'] = speed

    def getSpeedRatio(self):
        # TODO: Driver unit conversion?
        ratio = self.driver_getSpeedRatio()
        return ratio

    def setPanSpeedRatio(self, ratio):
        self.driver_setSpeedRatio(ratio, axis_str=self.pan_str)

    def setTiltSpeedRatio(self, ratio):
        self.driver_setSpeedRatio(ratio, axis_str=self.tilt_str)

    def getPanSpeedRatio(self):
        return self.driver_getSpeedRatio(axis_str=self.pan_str)

    def getTiltSpeedRatio(self):
        return self.driver_getSpeedRatio(axis_str=self.tilt_str)
          

    def getPosition(self):
        return self.current_position
        
    def getPositionTimes(self):
        return self.position_times
    
    def gotoPosition(self, pan_deg, tilt_deg):
        self.driver_moveToPosition(pan_deg * self.PAN_DEG_DIR, tilt_deg * self.TILT_DEG_DIR)

    def gotoPanPosition(self, pan_deg):
        #self.msg_if.pub_warn("gotoPanPosition: " + str(pan_deg) + " direction: " + str(self.PAN_DEG_DIR))
        self.driver_moveToPanPosition(pan_deg * self.PAN_DEG_DIR)

    def gotoTiltPosition(self, tilt_deg):
        #self.msg_if.pub_warn("gotoTiltPosition: " + str(tilt_deg) + " direction: " + str(self.TILT_DEG_DIR))
        self.driver_moveToTiltPosition(tilt_deg * self.TILT_DEG_DIR)
        
    def goHome(self):
        self.driver_moveToPosition(self.home_pan_deg, self.home_tilt_deg)

    def setHomePosition(self, pan_deg, tilt_deg):
        self.home_pan_deg = pan_deg * self.PAN_DEG_DIR
        self.home_tilt_deg = tilt_deg * self.TILT_DEG_DIR

    def setHomePositionHere(self):
        if self.driver_reportsPosition() is True:
            pan_deg, tilt_deg = self.getPosition()
            self.home_pan_deg = pan_deg * self.PAN_DEG_DIR
            self.home_tilt_deg = tilt_deg * self.TILT_DEG_DIR 

    def resetDevice(self):
        self.driver_resetDevice()


   #######################
   ###Calibration Functions

    def zeroPrefix(self, count_val):
        data_str = str(count_val)
        zero_prefix_len = 4-len(data_str)
        for z in range(zero_prefix_len):
            data_str = ('0' + data_str)
        return data_str

    def Reset(self):
        #move motor to limit

        #gets position
        ser_msg = (self.pan_str + self.addr_str + 'MRS0000R')
        [success,response] = self.send_msg(ser_msg)
        if success:
            pan_max_limit = int(response[5:9])
            self.msg_if.pub_warn('Reset complete')
        return pan_max_limit

    def calibrateTilt(self):
        self.msg_if.pub_warn('Calibrating Tilt')

        tilt_limit_max = self.LIMITS_DICT['max_tilt_hardstop_deg']
        tilt_limit_min = self.LIMITS_DICT['min_tilt_hardstop_deg']
        ser_msg = (self.tilt_str + self.addr_str + 'MLF9999W')
        self.send_msg(ser_msg)
        time.sleep(1)
        ser_msg = (self.tilt_str + self.addr_str + 'MLB0000W')
        self.send_msg(ser_msg)
        time.sleep(1)
        ser_msg = (self.tilt_str + self.addr_str + 'MMB0000W')
        self.send_msg(ser_msg)
        time.sleep(10)
        ser_msg = (self.tilt_str + self.addr_str + 'MMF0000W')
        self.send_msg(ser_msg)
        time.sleep(10)
        ser_msg = (self.tilt_str + self.addr_str + 'MML5000W')
        self.send_msg(ser_msg)
        time.sleep(5)
        ser_msg = (self.tilt_str + self.addr_str + 'MLB' + str((tilt_limit_min/.0879 +5000)) + 'W')
        self.send_msg(ser_msg)
        time.sleep(1)
        ser_msg = (self.tilt_str + self.addr_str + 'MLF' + str((tilt_limit_max/.0879 +5000)) + 'W')
        self.send_msg(ser_msg)
        time.sleep(1)
        self.msg_if.pub_warn('Tilt Calibration Complete')

    def calibratePan(self):
        self.msg_if.pub_warn('Calibrating Pan')

        pan_limit_max = self.LIMITS_DICT['max_pan_hardstop_deg']
        pan_limit_min = self.LIMITS_DICT['min_pan_hardstop_deg']
        ser_msg = (self.pan_str + self.addr_str + 'MLF9999W')
        self.send_msg(ser_msg)
        time.sleep(1)
        ser_msg = (self.pan_str + self.addr_str + 'MLB0000W')
        self.send_msg(ser_msg)
        time.sleep(1)
        ser_msg = (self.pan_str + self.addr_str + 'MMB0000W')
        self.send_msg(ser_msg)
        time.sleep(10)
        ser_msg = (self.pan_str + self.addr_str + 'MMF0000W')
        self.send_msg(ser_msg)
        time.sleep(10)
        ser_msg = (self.pan_str + self.addr_str + 'MML5000W')
        self.send_msg(ser_msg)
        time.sleep(5)
        ser_msg = (self.pan_str + self.addr_str + 'MLB' + str((pan_limit_min/.0879 +5000)) + 'W')
        self.send_msg(ser_msg)
        time.sleep(1)
        ser_msg = (self.pan_str + self.addr_str + 'MLF' + str((pan_limit_max/.0879 +5000)) + 'W')
        self.send_msg(ser_msg)
        time.sleep(1)
        self.msg_if.pub_warn('Pan Calibration Complete')


 
    def calibrateCenter(self):
        self.calibrateTilt()
        time.sleep(5)
        self.calibratePan()
        self.setHomePositionHere()
        self.msg_if.pub_warn('Position Set as Home')



   #######################
    ### Driver Interface Functions

    def driver_getDeviceInfo(self):
        method_name = sys._getframe().f_code.co_name
        dev_info = dict()
        dev_info["Manufacturer"] = 'Sidus'
        dev_info["Model"] = 'SS109'


        data_str = self.create_blank_str()
        ser_msg= (self.pan_str + self.addr_str + 'MRV' + data_str + 'R')
        [success,response] = self.send_msg(ser_msg)

        firmware = ""
        if success:
            firmware = response[5:8]
        dev_info["FirmwareVersion"] = firmware

        dev_info["SerialNum"] = ""
        return dev_info

               

    def driver_getSoftLimit(self,axis_str = '#', direction = 1):
        method_name = sys._getframe().f_code.co_name
        softLimit = -999
        success = False
        data_str = self.create_blank_str()
        if direction > 0:
            dir_str = 'MRF'
        else:
            dir_str = 'MRB'
        if axis_str == self.pan_str:
            ser_msg= (self.pan_str + self.addr_str + dir_str + data_str + 'R')
        elif axis_str == self.tilt_str:
            ser_msg= (self.tilt_str + self.addr_str + dir_str + data_str + 'R')
        elif axis_str == self.both_str:
            ser_msg= (self.both_str + self.addr_str + dir_str + data_str + 'R')
        else:
            return False
        #self.msg_if.pub_warn(method_name + ": Sending Get Soft Stop serial msg: " + ser_msg)
        [success,response] = self.send_msg(ser_msg, verbose = True)

        if success:
            try:
                data_str = response[5:(5 + self.config_dict['data_len'])]
                #self.msg_if.pub_warn(method_name + ": Will convert soft limit data str: " + data_str)
                pos_count = int(data_str)
                softLimit = self.pos_count2deg(pos_count)
                success = True
            except Exception as e:
                self.msg_if.pub_warn(method_name + ": Failed to convert message to int: " + data_str + " " + str(e))

        # softLimit = -999
        # if axis_str == self.pan_str:
        #     if direction > 0:
        #          softLimit = self.LIMITS_DICT['max_pan_softstop_deg']
        #     else:
        #         softLimit = self.LIMITS_DICT['min_pan_softstop_deg']
        # elif axis_str == self.tilt_str:
        #     if direction > 0:
        #          softLimit = self.LIMITS_DICT['max_tilt_softstop_deg']
        #     else:
        #         softLimit = self.LIMITS_DICT['min_tilt_softstop_deg']

        return softLimit

    def driver_getSoftLimits(self): 
        method_name = sys._getframe().f_code.co_name
        min_pan = self.driver_getSoftLimit(axis_str = self.pan_str, direction = -1)
        nepi_sdk.sleep(self.serial_send_delay)
        max_pan = self.driver_getSoftLimit(axis_str = self.pan_str, direction = 1)
        nepi_sdk.sleep(self.serial_send_delay)
        min_tilt = self.driver_getSoftLimit(axis_str = self.tilt_str, direction = -1)
        nepi_sdk.sleep(self.serial_send_delay)
        max_tilt = self.driver_getSoftLimit(axis_str = self.tilt_str, direction = 1)
        return [min_pan,max_pan,min_tilt,max_tilt] 

    def driver_setSoftLimit(self, limit_deg, axis_str = '#', direction = 1):
        method_name = sys._getframe().f_code.co_name
        success = False
        self
        pos_count = self.deg2pos_count(limit_deg)
        data_str = self.create_pos_str(pos_count)
        if direction > 0:
            dir_str = 'MLF'
        else:
            dir_str = 'MLB'
        if axis_str == self.pan_str:
            ser_msg= (self.pan_str + self.addr_str + dir_str + data_str + 'W')
        elif axis_str == self.tilt_str:
            ser_msg= (self.tilt_str + self.addr_str + dir_str + data_str + 'W')
        elif axis_str == self.both_str:
            ser_msg= (self.both_str + self.addr_str + dir_str + data_str + 'W')
        else:
            return False
        #self.msg_if.pub_warn(method_name + ": Sending Set Soft Stop serial msg: " + ser_msg)
        [success,response] = self.send_msg(ser_msg, verbose = True)  
        return success 

    def driver_setSoftLimits(self, min_pan,max_pan,min_tilt,max_tilt): 
        method_name = sys._getframe().f_code.co_name
        success_list = []
        nepi_sdk.sleep(self.serial_send_delay)
        success_list.append(self.driver_setSoftLimit(min_pan, axis_str = self.pan_str, direction = -1))
        nepi_sdk.sleep(self.serial_send_delay)
        success_list.append(self.driver_setSoftLimit(max_pan, axis_str = self.pan_str, direction = 1))
        nepi_sdk.sleep(self.serial_send_delay)
        success_list.append(self.driver_setSoftLimit(min_tilt, axis_str = self.tilt_str, direction = -1))
        nepi_sdk.sleep(self.serial_send_delay)
        success_list.append(self.driver_setSoftLimit(max_tilt, axis_str = self.tilt_str, direction = 1))
        return False not in success_list



    def driver_getSpeedRatio(self, axis_str = '#'):
        method_name = sys._getframe().f_code.co_name
        speedRatio = 0.0
        success = False
        data_str = self.create_blank_str()

        ser_msg= (axis_str + self.addr_str + 'MRS' + data_str + 'R')
        [success,response] = self.send_msg(ser_msg)
        if success:
            try:
                data_str = response[5:(5 + self.config_dict['data_len'])]
                #self.msg_if.pub_warn(method_name + ": Will convert speed str: " + data_str)
                speed_count = int(data_str)
                speedRatio = self.speed_count2ratio(speed_count)
            except Exception as e:
                self.msg_if.pub_warn(method_name + ": Failed to convert message to int: " + data_str + " " + str(e))

        return speedRatio

    def driver_setSpeedRatios(self,speedRatio):
        success = self.driver_setSpeedRatio(speedRatio, axis_str = "!")
        return success

    def driver_setSpeedRatio(self,speedRatio, axis_str = '!'):
        pan_success = False
        tilt_success = False
        method_name = sys._getframe().f_code.co_name
        self.serial_lock = True
        speed_count = None
        try:
            speed_count = self.ratio2speed_count(speedRatio)
            #self.msg_if.pub_warn(method_name + ": Updating Speed Count Data to: " + str(speed_count))
        except Exception as e:
            self.msg_if.pub_warn(method_name + ": Failed to convert message: " + str(speedRatio) + " " + str(e))
            return False
        if speed_count is not None:
            if speed_count == 0:
                speed_count = 1
            data_str = self.create_speed_str(speed_count)
            if axis_str == self.tilt_str or axis_str == self.both_str:
                ser_msg= (self.tilt_str  + self.addr_str + 'MSP' + data_str + 'W')
                #self.msg_if.pub_warn("Set Tilt Speed Msg: " + str(data_str))

                [success,response] = self.send_msg(ser_msg)
                # self.msg_if.pub_warn("")
                # dps=int(speed_count * 0.5) 
                # self.msg_if.pub_warn("Set Tilt Speed DPS: " + str(dps))
                # self.msg_if.pub_warn("Set Tilt Speed Msg: " + str(ser_msg))
                # self.msg_if.pub_warn("Set Tilt Speed Response: " + str(response))
                tilt_success = success
            else:
                tilt_success = True
            if axis_str == self.pan_str or axis_str == self.both_str:
                data_str = self.create_speed_str(speed_count)
                ser_msg= (self.pan_str  + self.addr_str + 'MSP' + data_str + 'W')
                #self.msg_if.pub_warn("Set Pan Speed Msg: " + str(data_str))

                [success,response] = self.send_msg(ser_msg)
                # self.msg_if.pub_warn("")
                # dps=int(speed_count * 0.5) 
                # self.msg_if.pub_warn("Set Pan Speed DPS: " + str(dps))
                # self.msg_if.pub_warn("Set Pan Speed Msg: " + str(ser_msg))
                # self.msg_if.pub_warn("Set Pan Speed Response: " + str(response))
                tilt_success = success
                pan_success = success
            else:
                pan_success = True
            return pan_success and tilt_success
        else:
            return False

    def driver_resetDevice(self):
        success = False
        # data_str = self.create_blank_str()
        # ser_msg= (self.tilt_str  + self.addr_str + 'MFR' + data_str + 'W')
        # [success,response] = self.send_msg(ser_msg)

        # nepi_sdk.sleep(self.serial_send_delay)
        # data_str = self.create_blank_str()
        # ser_msg= (self.pan_str  + self.addr_str + 'MFR' + data_str + 'W')
        # [success,response] = self.send_msg(ser_msg)

        return success




    def driver_reportsPosition(self):
        method_name = sys._getframe().f_code.co_name
        reportsPos = True
        return reportsPos

    def driver_getPosition(self, wait_on_busy = True, verbose = False):
        caller_method = inspect.currentframe().f_back.f_code.co_name
        method_name = sys._getframe().f_code.co_name
        pan_deg = self.getCurrentPanPosition(wait_on_busy,verbose)
        if verbose == True:
            self.msg_if.pub_warn(caller_method + ": " + method_name + ": Got pan degs: " + str(pan_deg))

        tilt_deg = self.getCurrentTiltPosition(wait_on_busy,verbose)
        if verbose == True:
            self.msg_if.pub_warn(caller_method + ": " + method_name + ": Got tilt degs: " + str(tilt_deg))
        return pan_deg, tilt_deg

        


    def getCurrentPanPosition(self, wait_on_busy = False, verbose = False):
        method_name = sys._getframe().f_code.co_name
        pan_deg = None #self.current_position[0]
        success = False
        #self.msg_if.pub_warn(method_name + ": Got pan position serial lock: " + str(self.serial_lock))
        if True: #self.serial_lock == False:
            data_str = self.create_blank_str()
            ser_msg= (self.pan_str + self.addr_str + 'MRL' + data_str + 'R')
            [success,response] = self.send_msg(ser_msg, wait_on_busy = wait_on_busy, verbose = verbose)

            if success == True:
                try:
                    data_str = response[5:(5 + self.config_dict['data_len'])]
                    #self.msg_if.pub_warn(method_name + ": Will convert pan position str: " + data_str)
                    pan_count = int(data_str) 
                    pan_deg = self.pos_count2deg(pan_count) * -1
                except Exception as e:
                    self.msg_if.pub_warn(method_name + ": Failed to convert message to int: " + data_str + " " + str(e))
        #self.msg_if.pub_warn(method_name + ": Got pan position str: " + str(pan_deg))
        return pan_deg


    def getCurrentTiltPosition(self, wait_on_busy = False, verbose = False):
        method_name = sys._getframe().f_code.co_name
        tilt_deg = None #self.current_position[1]
        success = False
        data_str = self.create_blank_str()
        ser_msg= (self.tilt_str + self.addr_str + 'MRL' + data_str + 'R')
        [success,response] = self.send_msg(ser_msg, wait_on_busy = wait_on_busy, verbose = verbose)

        if success:
            try:
                data_str = response[5:(5 + self.config_dict['data_len'])]
                #self.msg_if.pub_warn(method_name + ": Will convert tilt position str: " + data_str)
                tilt_count = int(data_str)
                tilt_deg = self.pos_count2deg(tilt_count) * -1
            except Exception as e:
                self.msg_if.pub_warn(method_name + ": Failed to convert message to int: " + data_str + " " + str(e))

        return tilt_deg


    def driver_moveToPosition(self,pan_deg, tilt_deg):
        self.serial_lock = True
        success = self.driver_moveToPanPosition(pan_deg)
        success = self.driver_moveToTiltPosition(tilt_deg)
        #self.msg_if.pub_warn("driver_moveToPosition: " + str(success))
        self.serial_lock = False
        return success


    def driver_moveToPanPosition(self,pan_deg):

        method_name = sys._getframe().f_code.co_name
        success = False
        pos_count = self.deg2pos_count(pan_deg)
        data_str = self.create_pos_str(pos_count)
        ser_msg= (self.pan_str + self.addr_str + 'MML' + data_str + 'W')
        #self.msg_if.pub_warn(" Will send move to pan pos with pos_count: " + str(pos_count) + "data_str: " + str(data_str))
        #self.msg_if.pub_warn("ser_msg: " + str(ser_msg))
        [success,response] = self.send_msg(ser_msg)
        
        return success


    def driver_moveToTiltPosition(self, tilt_deg):
        method_name = sys._getframe().f_code.co_name
        success = False
        pos_count = self.deg2pos_count(tilt_deg)
        data_str = self.create_pos_str(pos_count)
        ser_msg= (self.tilt_str + self.addr_str + 'MML' + data_str + 'W')
        #self.msg_if.pub_warn("pos_count: " + str(pos_count) + "data_str: " + str(data_str))
        #self.msg_if.pub_warn("ser_msg: " + str(ser_msg))
        [success,response] = self.send_msg(ser_msg)

        return success


    def driver_stopMotion(self):
        method_name = sys._getframe().f_code.co_name
        self.driver_stopAxisMotion(axis_str = self.both_str)


    def driver_stopAxisMotion(self, axis_str = '#'):
        method_name = sys._getframe().f_code.co_name
        self.serial_lock = True
        success = False
        data_str = self.create_blank_str()
        if axis_str == self.pan_str:
            ser_msg= (self.pan_str + self.addr_str + 'MST' + data_str + 'W')
            #self.msg_if.pub_warn(method_name + ": Sending Stop Pan serial msg: " + ser_msg)
        elif axis_str == self.tilt_str:
            ser_msg= (self.tilt_str + self.addr_str + 'MST' + data_str + 'W')
            #self.msg_if.pub_warn(method_name + ": Sending Stop Tilt serial msg: " + ser_msg)
        elif axis_str == self.both_str:
            ser_msg= (self.pan_str + self.addr_str + 'MST' + data_str + 'W')
            #self.msg_if.pub_warn(method_name + ": Sending Stop Pan serial msg: " + ser_msg)

            nepi_sdk.sleep(self.serial_send_delay)

            ser_msg= (self.tilt_str + self.addr_str + 'MST' + data_str + 'W')
            #self.msg_if.pub_warn(method_name + ": Sending Stop Tilt serial msg: " + ser_msg)
        else:
            return False
        self.serial_lock = False
        [success,response] = self.send_msg(ser_msg)
        return success 


    def driver_jog(self,axis_str, direction):

        method_name = sys._getframe().f_code.co_name
        success = False
        data_str = self.create_blank_str()
        if direction == 1:
            cmd_str = 'MMF'
        else:
            cmd_str = 'MMB'
        ser_msg= (axis_str + self.addr_str + cmd_str + data_str + 'W')
        [success,response] = self.send_msg(ser_msg, wait_on_busy = False)
        return success
    

    def driver_jog_speed_dps(self,axis_str, speed_dps, direction):

        method_name = sys._getframe().f_code.co_name
        success = False
        data_str = self.create_velocity_str(speed_dps) 
        if direction == 1:
            cmd_str = 'MMV0'
        else:
            cmd_str = 'MMV-'
        ser_msg= (axis_str + self.addr_str + cmd_str + data_str + 'W')
        [success,response] = self.send_msg(ser_msg, wait_on_busy = False)

        # self.msg_if.pub_warn("")
        # dps=round(speed_dps,2)
        # self.msg_if.pub_warn("Set Jog Speed DPS: " + str(dps))
        # self.msg_if.pub_warn("Set Jog Speed Msg: " + str(ser_msg))
        # self.msg_if.pub_warn("Set Jog Speed Response: " + str(response))
        return success

    #######################
    ### Driver Util Functions



    
    ### Function to try and connect to device at given port and baudrate
    def connect(self):
        success = False
        port_check = self.check_port(self.port_str)
        if port_check is True:
            try:
                # Try and open serial port
                self.msg_if.pub_info("Opening serial port " + self.port_str + " with baudrate: " + self.baud_str)
                self.serial_port = serial.Serial(self.port_str,self.baud_int,timeout = 50)
                self.msg_if.pub_info("Serial port opened")
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Something went wrong with connecting to serial port at: " + self.port_str + "(" + str(e) + ")" )
            if success == True:
                success = False
                response = ""
                # Send Message
                self.msg_if.pub_info("Requesting info for device: " + self.addr_str)
                # Test message
                data_str = self.create_blank_str()
                ser_msg= (self.pan_str + self.addr_str + 'MRA' + data_str + 'R')
                #self.msg_if.pub_warn("Sending serial string: " + ser_msg)
                [success,response] = self.send_msg(ser_msg)

                if success:
                    self.msg_if.pub_info("Connected to device at address: " +  self.addr_str)
                    # Update serial, hardware, and software status values
                    self.serial_num = "unknown"
                    self.hw_version = "unknown"
                    self.sw_version = "unknown"
                    success = True

                    # Factory Reset Device
                    nepi_sdk.sleep(self.serial_send_delay)
                    reset_success = self.driver_resetDevice()

            else:
                self.msg_if.pub_warn("serial port not active")
        return success


    def send_msg(self,ser_msg, wait_on_busy = True, verbose = False):
        start_time = round(nepi_utils.get_time() - self.start_time,3)
        caller_method = inspect.currentframe().f_back.f_code.co_name
        success = False
        response = "-999"
        
        if self.serial_port is not None:


            if self.serial_busy == True and wait_on_busy == True:
                while self.serial_busy == True and not nepi_sdk.is_shutdown():
                    nepi_sdk.sleep(self.MIN_SERIAL_SEND_DELAY / 2)  

            skip = True
            if self.serial_busy == False:
                skip = False
                self.serial_busy = True
                self.serial_port.reset_input_buffer()
                if verbose == True:
                    #self.msg_if.pub_warn(caller_method + ": send_msg: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
                    #self.msg_if.pub_warn(caller_method + ": send_msg: Locked serial with send msg: " + ser_msg)
                    pass
                ser_str = (ser_msg + '\r\n')
                b=bytearray()
                b.extend(map(ord, ser_str))
                try:
                    if verbose == True:
                        self.msg_if.pub_warn(caller_method + ": send_msg: Sending message " + str(ser_str))
                    self.serial_port.write(b)
                except Exception as e:
                    self.msg_if.pub_warn(caller_method + ": send_msg: Failed to send message " + str(e))
                time.sleep(self.SERIAL_RECEIVE_DELAY)
                try:
                    bs = self.serial_port.readline()
                    response = bs.decode()
                    if verbose == True:
                        self.msg_if.pub_warn(caller_method + ": send_msg: Device returned: " + str(response) + " for: " +  ser_str)
                except Exception as e:
                    self.msg_if.pub_warn(caller_method + ": send_msg: Failed to recieve message " + str(e))
                if verbose == True:
                    #self.msg_if.pub_warn(caller_method + ": send_msg: Unlocking serial")
                    #self.msg_if.pub_warn(caller_method + ": send_msg: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                    pass
                success = self.check_valid_response(ser_msg,response)
                if success == False:
                    # # Try again
                    # try:
                    #     bs = self.serial_port.readline()
                    #     response = bs.decode()
                    #     if verbose == True:
                    #         self.msg_if.pub_debug(caller_method + ": send_msg: Fialed \ Device returned: " + str(response) + " for: " +  ser_str)
                    # except Exception as e:
                    #     self.msg_if.pub_warn(caller_method + ": send_msg: Failed to recieve message " + str(e))
                    # if verbose == True:
                    #     #self.msg_if.pub_warn(caller_method + ": send_msg: Unlocking serial")
                    #     #self.msg_if.pub_warn(caller_method + ": send_msg: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                    #     pass
                    #self.msg_if.pub_warn(caller_method + ": Serial send " + str(ser_msg) + " returned bad response " + str(response))
                    self.serial_fail_attempts += 1
                    #self.msg_if.pub_warn(caller_method + ": Updated serial_fail_attempts to " + str(self.serial_fail_attempts))
    


                else:
                    self.serial_fail_attempts = 0

                self.serial_busy = False
                
            # else:
            #     #self.msg_if.pub_warn(caller_method + ": Serial port busy, can't send msg: " + ser_msg)
            #     self.serial_fail_attempts += 1
            #     #self.msg_if.pub_warn(caller_method + ": Updated serial_fail_attempts to " + str(self.serial_fail_attempts))

            # if self.serial_fail_attempts > self.max_serial_fail_attempts:
            #     self.max_serial_fail_attempts = copy.deepcopy(self.serial_fail_attempts)
            #     if self.serial_send_delay < self.MAX_SERIAL_SEND_DELAY:
            #         self.serial_send_delay = self.MIN_SERIAL_SEND_DELAY + self.MAX_SERIAL_SEND_DELAY * (self.max_serial_fail_attempts / self.MAX_SERIAL_ATTEMPTS)
            #     #self.msg_if.pub_warn(caller_method + ": Serial send delay updated to : " +  str(self.serial_send_delay))
            # if self.serial_fail_attempts > self.MAX_SERIAL_ATTEMPTS:
            #     nepi_sdk.signal_shutdown(caller_method + ": Exceeded Max Serial Fail attempts in a row, Shutting Down")


            #############################
            update_delay = self.debug_update_delay
            if update_delay != -1:
                debug_time = start_time
                process_time = nepi_utils.get_time() - start_time
                debug_key = str(ser_msg[:5])
                if success == True:
                    result = "Success"
                elif skip == True:
                    result = "Skip"
                else:
                    result = "Failed"

                try:
                    last_time = copy.deepcopy(self.debug_last_time)
                    debug_delay = round(debug_time - last_time,3)

                    debug_times = copy.deepcopy(self.debug_times)
                    debug_times.append(debug_time)
                    debug_delays = copy.deepcopy(self.debug_delays)
                    debug_delays.append(debug_delay)
                    debug_send_msgs = copy.deepcopy(self.debug_send_msgs)
                    debug_send_msgs.append(ser_msg)
                    debug_response_msgs = copy.deepcopy(self.debug_response_msgs)
                    debug_response_msgs.append(response)
                    debug_process_results = copy.deepcopy(self.debug_process_results)
                    debug_process_results.append(result)
                    debug_process_times = copy.deepcopy(self.debug_process_times)
                    debug_process_times.append(process_time)


                    if debug_delay > update_delay:
                        self.debug_last_time = copy.deepcopy(debug_time)
                        self.debug_times = []
                        self.debug_delays = []
                        self.debug_send_msgs = []
                        self.debug_response_msgs = []
                        self.debug_process_results = []
                        self.debug_process_times = []

                        self.msg_if.pub_warn("")
                        self.msg_if.pub_warn("----------------------------")
                        self.msg_if.pub_warn("Debug Time: "  + str(debug_time))
                        self.msg_if.pub_warn("Debug Times: "  + str(debug_times))
                        for i, dtime in enumerate(debug_times):
                            self.msg_if.pub_warn(str(debug_key) +  \
                                                " : " + str(debug_times[i]) + \
                                                " : " + str(debug_delays[i]) + \
                                                " : " + str(debug_process_results[i]) + \
                                                " : " + str(debug_process_times[i]) + \
                                                " : " + str(debug_send_msgs[i]) + \
                                                " : " + str(debug_response_msgs[i])
                                                )
                        self.msg_if.pub_warn("----------------------------")
                    else:
                        self.debug_times = debug_times
                        self.debug_delays = debug_delays
                        self.debug_send_msgs = debug_send_msgs
                        self.debug_response_msgs = debug_response_msgs
                        self.debug_process_results = debug_process_results
                        self.debug_process_times = debug_process_times
                except Exception as e:
                    self.msg_if.pub_warn("Debug Process Failed: " + str(e))

            ###############################


        return [success, response]


    def check_valid_response(self,ser_msg, response):
        caller_method = inspect.currentframe().f_back.f_code.co_name
        valid = False
        if len(response) >= 5 + self.config_dict['data_len'] + 1:
            if response[0:4] == ser_msg[0:4]:
                valid = True
        if valid == False:
            pass
            #self.msg_if.pub_warn(caller_method + ": Failed to get valid response message from: " + ser_msg + " : " + str(response))
        return valid




    ### Function for checking if port is available
    def check_port(self,port_str):
        success = False
            # Try pyserial first
        ports = list(serial.tools.list_ports.comports())
        add_ports = sorted(set(
                glob.glob('/dev/ttyTHS0')
            ))
        for add_port in add_ports:
            if add_port not in ports:
                ports.append(add_port)
        #self.msg_if.pub_warn("Node Port Check: " + str(ports))
        for p in ports:
            loc = getattr(p, 'device', p)
            if loc == port_str:
                success = True
        return success


    def pos_count2deg(self, count):
        dpc = self.config_dict['deg_per_count'] 
        home = self.config_dict['home'] 
        deg = float(count - home)*dpc  
        return deg

    def deg2pos_count(self, deg):
        dpc = self.config_dict['deg_per_count']
        home = self.config_dict['home'] 
        count = int(deg/dpc + home)
        return count


    def speed_count2dps(self, count):
        dps_per_count = self.config_dict['degpsec_per_count']
        dps = math.floor(count * dps_per_count)
        return dps

    def dps2speed_count(self, dps):
        dps_per_count = self.config_dict['degpsec_per_count']
        count = math.floor(dps / dps_per_count)
        return count

    def ratio2speed_count(self,ratio):
        max_count =  self.config_dict['max_degpsec'] / self.config_dict['degpsec_per_count']
        count = int(math.floor(ratio * max_count))
        return count

    def speed_count2ratio(self,count):
        max_count =  math.floor(self.config_dict['max_degpsec'] / self.config_dict['degpsec_per_count'])
        ratio = float(count/max_count)
        return ratio

    def create_blank_str(self):
        data_str = ""
        zero_prefix_len = self.config_dict['data_len']-len(data_str)
        for z in range(self.config_dict['data_len']):
            data_str += '0'
        return data_str

    def create_pos_str(self,count_val):
        data_str = str(count_val)
        zero_prefix_len = self.config_dict['data_len']-len(data_str)
        for z in range(zero_prefix_len):
            data_str = ('0' + data_str)
        return data_str

    def create_speed_str(self,count_val):
        data_str = str(count_val)
        zero_suffix_len = self.config_dict['data_len']-len(data_str)
        for z in range(zero_suffix_len):
            data_str = ('0' + data_str)
        return data_str
    
    def create_velocity_str(self,velocity_val):
        data_str = str(math.floor(velocity_val/self.speed_max_dps * self.speed_max_dps * 10))
        zero_suffix_len = self.config_dict['data_len']-len(data_str)-1
        for z in range(zero_suffix_len):
            data_str = ('0' + data_str)
        return data_str


    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        if self.serial_port is not None:
            self.serial_port.close()


if __name__ == '__main__':
	node = SidusSS109SerialPTXNode()
