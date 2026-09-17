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
import math
import numpy as np


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_targets

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header


from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryResponse, DeviceInfoQueryRequest

from nepi_interfaces.msg import DevicePTXStatus, PanTiltLimits, PanTiltPosition, SingleAxisTimedMove, SingleAxisTimedSpeedMove
from nepi_interfaces.msg import MotorStatus, MotorsStatus, MotorCommand
from nepi_interfaces.srv import PTXCapabilitiesQuery, PTXCapabilitiesQueryRequest, PTXCapabilitiesQueryResponse
from nepi_interfaces.msg import NavPosePanTilt



from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SettingsIF




class PTXActuatorIF:
    MAX_STATUS_UPDATE_RATE = 3
    MIN_LIMIT_ANGLE = 10
    # Backup Factory Control Values 
    FACTORY_CONTROLS_DICT = {
                'reverse_pan_enabled' : False,
                'reverse_tilt_enabled' : False,
                'speed_ratio' : 0.5
    }


    orientation_dict = {
        'time_orientation': nepi_utils.get_time(),
        # Orientation should be provided in Degrees ENU
        'roll_deg': 0.0,
        'tilt_deg': 0.0,
        'pan_deg': 0.0,
    }

    ready = False

    node_if = None
    settings_if = None
    save_data_if = None

    status_msg = DevicePTXStatus()
    info_report = DeviceInfoQueryResponse()
    caps_report = PTXCapabilitiesQueryResponse()

    has_position_feedback = False
    has_absolute_positioning = False
    has_timed_positioning = False
    has_timed_speed_positioning =False
    has_seperate_pan_tilt_control = False
    has_adjustable_limits = False
    has_adjustable_speed = False
    has_seperate_pan_tilt_speed = False
    has_homing = False
    has_set_home = False
    has_limit_controls = False
    has_calibration = False
    

    move_decimal_place = 0
    report_decimal_place = 2


    # Define some member variables
    pan_now_deg = 0.0
    pan_goal_deg = 0.0
    pan_home_pos_deg = 0.0
    min_pan_softstop_deg = 0.0
    max_pan_softstop_deg = 0.0
    tilt_now_deg = 0.0
    tilt_goal_deg = 0.0
    tilt_home_pos_deg = 0.0
    min_tilt_softstop_deg = 0.0
    max_tilt_softstop_deg = 0.0

    home_pan_deg = 0.0
    home_tilt_deg = 0.0

    has_calibration : False


    reverse_pan_enabled = False
    rpi = 1
    reverse_tilt_enabled = False
    rti = 1

    last_pan = 0
    last_tilt = 0

    max_pan_hardstop_deg = 0
    min_pan_hardstop_deg = 0

    max_tilt_hardstop_deg = 0
    min_tilt_hardstop_deg = 0

    max_pan_softstop_deg = 0
    min_pan_softstop_deg = 0

    max_tilt_softstop_deg = 0
    min_tilt_softstop_deg = 0

    current_position = [0.0,0.0]
    last_position = current_position
    position_times = [0.0,0.0]
    speed_ratio = 0.5
    speed_pan_ratio = speed_ratio
    speed_tilt_ratio = speed_ratio
    set_max_dps_disabled = False
    speed_max_dps = 10

    # Intended continuous-motion direction per motor for the standard motor
    # command interface. 1 = clockwise (toward the max soft-limit angle),
    # -1 = counter-clockwise (toward the min soft-limit angle). Set by the
    # standard 'set_direction' command and used by 'go_direction'.
    cmd_dir_pan = 1
    cmd_dir_tilt = 1


    tr_source_ref_description = 'tilt_axis_center'
    tr_end_ref_description = 'nepi_frame'


    data_source_description = 'pan_tilt'
    data_ref_description = 'tilt_axis_center'
    data_end_description = 'nepi_frame'

    
    is_moving = False

    speed_pan_dps = 0
    pan_last_time = 0

    speed_tilt_dps = 0
    tilt_last_time = 0

    last_time = 0

    pan_pos_history = []
    tilt_pos_history = []
    pan_time_history = [0]
    tilt_time_history = [0]
    SPEED_SMOOTHING_WINDOW = 3
    delay = False





    ### IF Initialization
    def __init__(self,  device_info,
                 factoryControls , # Dictionary to be supplied by parent, specific key set is required
                 getSettingsFunction=None, setSettingFunction=None, 
                 getCapSettingsFunction = None,
                 factoryLimits = None,
                 data_source_description = 'pan_tilt',
                 data_ref_description = 'tilt_axis_center',
                 stopMovingCb = None, # Required; no args
                 movePanCb = None, # Required; direction and optional time args
                 moveTiltCb = None, # Required; direction and optional time args
                 movePanSpeedRatioCb = None, # Required; direction, speed ratio, and optional time args
                 moveTiltSpeedRatioCb = None, # Required; direction, speed ratio, and optional time args
                 setSoftLimitsCb=None,
                 getSoftLimitsCb=None,
                 getSpeedMaxCb = None,
                 setSpeedMaxCb = None,
                 setSpeedRatioCb=None, # None ==> No speed adjustment capability; Speed ratio arg
                 getSpeedRatioCb=None, # None ==> No speed adjustment capabilitiy; Returns speed ratio
                 setPanSpeedRatioCb=None,
                 setTiltSpeedRatioCb=None,
                 getPanSpeedRatioCb=None,
                 getTiltSpeedRatioCb=None,
                 getPositionCb=None,
                 getPositionTimesCb=None,
                 gotoPositionCb=None, # None ==> No absolute positioning capability (pan_deg, tilt_deg, speed, float move_timeout_s) 
                 gotoPanPositionCb=None, # None ==> No absolute positioning capability (pan_deg, tilt_deg, speed, float move_timeout_s) 
                 gotoTiltPositionCb=None, # None ==> No absolute positioning capability (pan_deg, tilt_deg, speed, float move_timeout_s) 
                 goHomeCb=None, # None ==> No native driver homing capability, can still use homing if absolute positioning is supported
                 setHomePositionCb=None, # None ==> No native driver home absolute setting capability, can still use it if absolute positioning is supported
                 setHomePositionHereCb=None, # None ==> No native driver home instant capture capability, can still use it if absolute positioning is supported
                 getNavPoseCb=None,
                 navpose_update_rate = 10,
                 deviceResetCb = None,
                 calibrateCenterCB = None,
                 log_name = None,
                 log_name_list = [],
                 msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,'ptx')
        self.data_products_list = []


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
        self.msg_if.pub_info("Starting PTX Device IF Initialization Processes", log_name_list = self.log_name_list)



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
        self.info_report.type = 'PTX'

        self.caps_report.device_name = self.device_name
        self.caps_report.device_path = self.path
        self.caps_report.device_node_name = self.node_name

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description

        # Create and update factory controls dictionary
        self.factory_controls_dict = self.FACTORY_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]

        self.deviceResetCb = deviceResetCb


       # Configure PTX Capabilities

        # STOP MOVE #############
        self.stopMovingCb = stopMovingCb


        # GET POSITION #############
        self.getPositionCb = getPositionCb
        if self.getPositionCb is not None:
            self.has_position_feedback = True
            self.has_limit_controls = True
        self.getPositionTimesCb = getPositionTimesCb

        self.getNavPoseCb = getNavPoseCb
        if navpose_update_rate < 1:
           navpose_update_rate = 1
        if navpose_update_rate > 10:
            navpose_update_rate = 10
        self.navpose_update_rate = navpose_update_rate



        # Soft Limits are handled by PTX IF at top level, 
        # these are for updating the hardware if required
        self.setSoftLimitsCb = setSoftLimitsCb
        self.getSoftLimitsCb = getSoftLimitsCb

        # POSITION MOVE ############
        self.gotoPositionCb = gotoPositionCb
        self.gotoPanPositionCb = gotoPanPositionCb
        self.gotoTiltPositionCb = gotoTiltPositionCb


        if self.gotoPositionCb is not None or self.gotoPanPositionCb is not None:
            self.has_absolute_positioning = True
    
        if gotoPanPositionCb is not None and gotoTiltPositionCb is not None:
            self.has_seperate_pan_tilt_control = True

        # JOG MOVE ############
        self.movePanCb = movePanCb
        self.moveTiltCb = moveTiltCb
        if self.movePanCb is not None or self.moveTiltCb is not None:
            self.has_timed_positioning = True

        # JOG MOVE at Speed Ratio ############
        self.movePanSpeedRatioCb = movePanSpeedRatioCb
        self.moveTiltSpeedRatioCb = moveTiltSpeedRatioCb
        if self.movePanSpeedRatioCb is not None or self.moveTiltSpeedRatioCb is not None:
            self.has_timed_speed_positioning = True

        # SPEED SETTINGS  #############
        self.setSpeedMaxCb = setSpeedMaxCb
        self.getSpeedMaxCb = getSpeedMaxCb
        if self.getSpeedMaxCb is not None:
            self.set_max_dps_disabled = True
            self.speed_max_dps = self.getSpeedMaxCb()


        self.setSpeedRatioCb = setSpeedRatioCb
        if setSpeedRatioCb is None:
            self.getSpeedRatioCb = self.getZeroCb #None
        else:
            self.setSpeedRatioCb = setSpeedRatioCb
        self.getSpeedRatioCb = getSpeedRatioCb
        if getSpeedRatioCb is not None and setSpeedRatioCb is not None:
            self.has_adjustable_speed = True

        # PER-AXIS SPEED SETTINGS  #############
        self.setPanSpeedRatioCb = setPanSpeedRatioCb
        self.setTiltSpeedRatioCb = setTiltSpeedRatioCb
        self.getPanSpeedRatioCb = getPanSpeedRatioCb
        self.getTiltSpeedRatioCb = getTiltSpeedRatioCb
        if (setPanSpeedRatioCb is not None and setTiltSpeedRatioCb is not None and
                getPanSpeedRatioCb is not None and getTiltSpeedRatioCb is not None):
            self.has_seperate_pan_tilt_speed = True


        # Homing  #############
        self.goHomeCb = goHomeCb
        self.setHomePositionCb = setHomePositionCb
        self.setHomePositionHereCb = setHomePositionHereCb

        if self.goHomeCb is not None:
            self.has_homing = True
        if self.setHomePositionHereCb is not None:
            self.has_set_home = True
               
        # Calibration  #############
        self.calibrateCenterCB = calibrateCenterCB
        if self.calibrateCenterCB is not None:
            self.has_calibration = True




        # Create Capabilities Report
        self.caps_report = PTXCapabilitiesQueryResponse()
        self.caps_report.has_absolute_positioning = self.has_absolute_positioning
        self.caps_report.has_timed_positioning = self.has_timed_positioning
        self.caps_report.has_timed_speed_positioning = self.has_timed_speed_positioning
        self.caps_report.has_seperate_pan_tilt_control = self.has_seperate_pan_tilt_control
        self.caps_report.has_position_feedback = self.has_position_feedback
        self.caps_report.has_adjustable_speed = self.has_adjustable_speed
        self.caps_report.has_seperate_pan_tilt_speed = self.has_seperate_pan_tilt_speed
        self.caps_report.has_limit_controls = self.has_limit_controls
        self.caps_report.has_homing = self.has_homing
        self.caps_report.has_set_home = self.has_set_home
        self.caps_report.has_calibration = self.has_calibration


        #######################################
        # Set up factory limits

        if factoryLimits is not None:
            self.factoryLimits = factoryLimits  
        else:
            # Hard limits
            self.factoryLimits['min_pan_hardstop_deg'] = 0
            self.factoryLimits['max_pan_hardstop_deg'] = 0
            self.factoryLimits['min_tilt_hardstop_deg'] = 0
            self.factoryLimits['max_tilt_hardstop_deg'] = 0
  
            # Soft limits
            self.factoryLimits['min_pan_softstop_deg'] = 0
            self.factoryLimits['max_pan_softstop_deg'] = 0
            self.factoryLimits['min_tilt_softstop_deg'] = 0
            self.factoryLimits['max_tilt_softstop_deg'] = 0

        self.min_pan_hardstop_deg =  self.factoryLimits['min_pan_hardstop_deg']
        self.max_pan_hardstop_deg = self.factoryLimits['max_pan_hardstop_deg']

        self.min_tilt_hardstop_deg = self.factoryLimits['min_tilt_hardstop_deg']
        self.max_tilt_hardstop_deg = self.factoryLimits['max_tilt_hardstop_deg']


        self.min_pan_softstop_deg =  self.factoryLimits['min_pan_softstop_deg']
        self.max_pan_softstop_deg = self.factoryLimits['max_pan_softstop_deg']


        self.min_tilt_softstop_deg = self.factoryLimits['min_tilt_softstop_deg']
        self.max_tilt_softstop_deg = self.factoryLimits['max_tilt_softstop_deg']


        ########################       
        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description

        self.status_msg.has_absolute_positioning = self.has_absolute_positioning
        self.status_msg.has_timed_positioning = self.has_timed_positioning
        self.status_msg.has_timed_speed_positioning = self.has_timed_speed_positioning
        self.status_msg.has_seperate_pan_tilt_control = self.has_seperate_pan_tilt_control
        self.status_msg.has_position_feedback = self.has_position_feedback
        self.status_msg.has_adjustable_speed = self.has_adjustable_speed
        self.status_msg.has_seperate_pan_tilt_speed = self.has_seperate_pan_tilt_speed
        self.status_msg.has_limit_controls = self.has_limit_controls

        self.status_msg.has_homing = self.has_homing
        self.status_msg.has_set_home = self.has_set_home
        self.status_msg.has_calibration = self.has_calibration


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
            'speed_max_dps': {
                'namespace': self.namespace,
                'factory_val': self.speed_max_dps
            },
            'speed_ratios': {
                'namespace': self.namespace,
                'factory_val': [self.speed_pan_ratio,self.speed_tilt_ratio]
            },
            'home_position/pan_deg': {
                'namespace': self.namespace,
                'factory_val': 0.0
            },
            'home_position/tilt_deg': {
                'namespace': self.namespace,
                'factory_val': 0.0
            },
            'reverse_pan_enabled': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict['reverse_pan_enabled']
            },            
            'reverse_tilt_enabled': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict['reverse_tilt_enabled']
            },
            'max_pan_softstop_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['max_pan_softstop_deg']
            },            
            'min_pan_softstop_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['min_pan_softstop_deg']
            },            
            'max_tilt_softstop_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['max_tilt_softstop_deg']
            },           
            'min_tilt_softstop_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['min_tilt_softstop_deg']
            },
            'move_decimal_place': {
                'namespace': self.namespace,
                'factory_val': 0
            },
            'report_decimal_place': {
                'namespace': self.namespace,
                'factory_val': 2
            }
        }

        if self.has_seperate_pan_tilt_speed:
            self.PARAMS_DICT['speed_pan_ratio'] = {
                'namespace': self.namespace,
                'factory_val': 0.5
            }
            self.PARAMS_DICT['speed_tilt_ratio'] = {
                'namespace': self.namespace,
                'factory_val': 0.5
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
                'srv': PTXCapabilitiesQuery,
                'req': PTXCapabilitiesQueryRequest(),
                'resp': PTXCapabilitiesQueryResponse(),
                'callback': self.capabilities_query_callback
            }
        }


        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DevicePTXStatus,
                'qsize': 10,
                'latch': False
            },
            'pan_tilt_pub': {
                'msg': NavPosePanTilt,
                'namespace': self.namespace,
                'topic': 'pan_tilt',
                'qsize': 1,
                'latch': False
            },
            'motor_status_pub': {
                'msg': MotorsStatus,
                'namespace': self.namespace,
                'topic': 'motor_status',
                'qsize': 10,
                'latch': False
            }

        }



        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_speed_max_dps': {
                'namespace': self.namespace,
                'topic': 'set_speed_max_dps',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setMaxSpeedCb,
                'callback_args': ()
            },
            'speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setSpeedRatioCb,
                'callback_args': ()
            },
            'set_pan_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_pan_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setPanSpeedRatioCb,
                'callback_args': ()
            },
            'set_tilt_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_tilt_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setTiltSpeedRatioCb,
                'callback_args': ()
            },
            'stop_moving': {
                'namespace': self.namespace,
                'topic': 'stop_moving',
                'msg': Empty,
                'qsize': 1,
                'callback': self._stopMovingCb, 
                'callback_args': ()
            },
            'stop_pan': {
                'namespace': self.namespace,
                'topic': 'stop_pan',
                'msg': Empty,
                'qsize': 1,
                'callback': self._stopPanCb, 
                'callback_args': ()
            },
            'stop_tilt': {
                'namespace': self.namespace,
                'topic': 'stop_tilt',
                'msg': Empty,
                'qsize': 1,
                'callback': self._stopTiltCb, 
                'callback_args': ()
            },
            'goto_to_position': {
                'namespace': self.namespace,
                'topic': 'goto_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self._gotoPositionCb, 
                'callback_args': ()
            },
            'goto_to_pan_position': {
                'namespace': self.namespace,
                'topic': 'goto_pan_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoPanPositionCb, 
                'callback_args': ()
            },
            'goto_to_tilt_position': {
                'namespace': self.namespace,
                'topic': 'goto_tilt_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoTiltPositionCb, 
                'callback_args': ()
            },
            'goto_pan_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_pan_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoToPanRatioCb, 
                'callback_args': ()
            },
            'goto_tilt_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoToTiltRatioCb, 
                'callback_args': ()
            },
            'jog_timed_pan': {
                'namespace': self.namespace,
                'topic': 'jog_timed_pan',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self._jogTimedPanCb, 
                'callback_args': ()
            },
            'jog_timed_tilt': {
                'namespace': self.namespace,
                'topic': 'jog_timed_tilt',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self._jogTimedTiltCb, 
                'callback_args': ()
            },
            'jog_timed_pan_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'jog_timed_pan_speed_ratio',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
                'callback': self._jogTimedPanSpeedRatioCb, 
                'callback_args': ()
            },
            'jog_timed_tilt_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'jog_timed_tilt_speed_ratio',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
                'callback': self._jogTimedTiltSpeedRatioCb, 
                'callback_args': ()
            },
            'reverse_pan_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_pan_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setReversePanEnableCb, 
                'callback_args': ()
            },
            'reverse_tilt_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_tilt_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setReverseTiltEnableCb, 
                'callback_args': ()
            },
            'set_soft_limits': {
                'namespace': self.namespace,
                'topic': 'set_soft_limits',
                'msg': PanTiltLimits,
                'qsize': 1,
                'callback': self._setSoftstopCb, 
                'callback_args': ()
            },
            'go_home': {
                'namespace': self.namespace,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
                'callback': self._goHomeCb, 
                'callback_args': ()
            },
            'set_home_position': {
                'namespace': self.namespace,
                'topic': 'set_home_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self._setHomePositionCb, 
                'callback_args': ()
            },
            'set_home_position_here': {
                'namespace': self.namespace,
                'topic': 'set_home_position_here',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setHomePositionHereCb, 
                'callback_args': ()
            },
            'calibrate_center': {
                'namespace': self.namespace,
                'topic': 'calibrate_center',
                'msg': Empty,
                'qsize': 1,
                'callback': self._calibrateCenterCB,
                'callback_args': ()
            },
            'set_move_decimal_place': {
                'namespace': self.namespace,
                'topic': 'set_move_decimal_place',
                'msg': Int32,
                'qsize': 1,
                'callback': self._setMoveDecimalPlaceCb,
                'callback_args': ()
            },
            'set_report_decimal_place': {
                'namespace': self.namespace,
                'topic': 'set_report_decimal_place',
                'msg': Int32,
                'qsize': 1,
                'callback': self._setReportDecimalPlaceCb,
                'callback_args': ()
            },
            # Standard NEPI motor command interface. Carried on the four standard
            # per-motor command topics alongside the 'motor_status' topic, all
            # using nepi_interfaces/MotorCommand keyed by motor_name
            # (motor_0 = pan, motor_1 = tilt, "all" = both).
            'set_speed': {
                'namespace': self.namespace,
                'topic': 'set_speed',
                'msg': MotorCommand,
                'qsize': 1,
                'callback': self._setMotorSpeedCb,
                'callback_args': ()
            },
            'set_direction': {
                'namespace': self.namespace,
                'topic': 'set_direction',
                'msg': MotorCommand,
                'qsize': 1,
                'callback': self._setMotorDirectionCb,
                'callback_args': ()
            },
            'go_direction': {
                'namespace': self.namespace,
                'topic': 'go_direction',
                'msg': MotorCommand,
                'qsize': 1,
                'callback': self._goMotorDirectionCb,
                'callback_args': ()
            },
            'stop_motor': {
                'namespace': self.namespace,
                'topic': 'stop_motor',
                'msg': MotorCommand,
                'qsize': 1,
                'callback': self._stopMotorCb,
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

        ##############################
        # Start Node Processes

        # Periodic publishing
        status_update_rate = self.navpose_update_rate  
        if status_update_rate < 1:
            status_update_rate = 1
        if status_update_rate > 10:
            status_update_rate = 10
        self.status_update_rate = status_update_rate
        self.msg_if.pub_warn("Starting pt status publisher at hz: " + str(self.status_update_rate))    
        status_pub_delay = float(1.0) / self.status_update_rate
        nepi_sdk.start_timer_process(status_pub_delay, self._publishStatusCb)

        ##############################
        # Start Additional System Processes

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


        #####################
        # Update Status Message
        nepi_sdk.sleep(1)
        if self.settings_if is not None:
            self.status_msg.settings_topic = self.settings_if.get_namespace()
            self.msg_if.pub_info("Using settings namespace: " + str(self.status_msg.settings_topic))

        ####################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ####################################



  

    def initCb(self, do_updates = False):
        if do_updates == True and self.node_if is not None:
            # This one comes from the parent
            if self.getSoftLimitsCb is not None:
                    [min_pan,max_pan,min_tilt,max_tilt] = self.getSoftLimitsCb()
                    if min_pan != -999:
                        self.min_pan_softstop_deg = min_pan
                        self.node_if.set_param('min_pan_softstop_deg', self.min_pan_softstop_deg)
                    if max_pan != -999:
                        self.max_pan_softstop_deg = max_pan
                        self.node_if.set_param('max_pan_softstop_deg',self.max_pan_softstop_deg)

                    if min_tilt != -999:
                        self.min_tilt_softstop_deg = min_tilt
                        self.node_if.set_param('min_tilt_softstop_deg', self.min_tilt_softstop_deg)
                    if max_tilt != -999:
                        self.max_tilt_softstop_deg = max_tilt
                        self.node_if.set_param('max_tilt_softstop_deg',self.max_tilt_softstop_deg)




        if self.node_if is not None:
            self.min_pan_softstop_deg =  self.node_if.get_param('min_pan_softstop_deg')
            self.max_pan_softstop_deg = self.node_if.get_param('max_pan_softstop_deg')

            self.min_tilt_softstop_deg = self.node_if.get_param('min_tilt_softstop_deg')
            self.max_tilt_softstop_deg = self.node_if.get_param('max_tilt_softstop_deg')

            if None not in [self.min_pan_softstop_deg,self.max_pan_softstop_deg,self.min_tilt_softstop_deg,self.max_tilt_softstop_deg]:
                if self.setSoftLimitsCb is not None:
                    self.setSoftLimitsCb(self.min_pan_softstop_deg,
                                        self.max_pan_softstop_deg,
                                        self.min_tilt_softstop_deg,
                                        self.max_tilt_softstop_deg)
            if self.getSpeedMaxCb is None:
                self.speed_max_dps = self.node_if.get_param('speed_max_dps')
            if self.setSpeedMaxCb is not None:
                self.setSpeedMaxCb(self.speed_max_dps) 

            # Restore on the same predicates the three write paths are gated on:
            # _setSpeedRatio writes when has_adjustable_speed is True, and
            # _setPanSpeedRatio/_setTiltSpeedRatio write when
            # has_seperate_pan_tilt_speed is True. Gating the restore on
            # has_adjustable_speed == False was the exact inverse, so on the units
            # where the operator can move the speed it was saved and never read back.
            # It also left speed_ratios assigned under one guard and consumed under a
            # sibling guard -- a NameError on any driver with per-axis speed callbacks.
            if self.has_adjustable_speed == True or self.has_seperate_pan_tilt_speed == True:
                speed_ratios = self.node_if.get_param('speed_ratios')
                if speed_ratios is not None and len(speed_ratios) == 2:
                    self.speed_pan_ratio = speed_ratios[0]
                    self.speed_tilt_ratio = speed_ratios[1]
                    self.speed_ratio = max(speed_ratios)
                    self.msg_if.pub_warn("Initializing speed_ratios: " + str(speed_ratios))
                    if self.has_seperate_pan_tilt_speed == True:
                        self.setPanSpeedRatioCb(self.speed_pan_ratio)
                        self.setTiltSpeedRatioCb(self.speed_tilt_ratio)
                    elif self.setSpeedRatioCb is not None:
                        self.setSpeedRatioCb(self.speed_ratio)

            self.home_pan_deg = self.node_if.get_param('home_position/pan_deg')
            self.home_tilt_deg = self.node_if.get_param('home_position/tilt_deg')

            move_decimal_place = self.node_if.get_param('move_decimal_place')
            if move_decimal_place is not None:
                self.move_decimal_place = move_decimal_place
            report_decimal_place = self.node_if.get_param('report_decimal_place')
            if report_decimal_place is not None:
                self.report_decimal_place = report_decimal_place

            # self.goHome()

        
    
            # Restore the reverse flags BEFORE deriving the direction multipliers from
            # them. Reading the params after this derivation left rpi/rti computed from
            # the pre-restore value while status_msg reported the restored one, so a
            # restored reverse axis read correct on the page and still moved the wrong
            # way.
            reverse_pan_enabled = self.node_if.get_param('reverse_pan_enabled')
            if reverse_pan_enabled is not None:
                self.reverse_pan_enabled = reverse_pan_enabled
            reverse_tilt_enabled = self.node_if.get_param('reverse_tilt_enabled')
            if reverse_tilt_enabled is not None:
                self.reverse_tilt_enabled = reverse_tilt_enabled

            # Set reverse int values
            rpi = 1
            if self.reverse_pan_enabled:
                rpi = -1
            self.rpi = rpi
            rti = 1
            if self.reverse_tilt_enabled:
                rti = -1
            self.rti = rti
        if do_updates == True:
            pass
        self.publish_status()

    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass # self.node_if.reset_params()
        if self.save_data_if is not None:
            self.save_data_if.reset()
        if self.settings_if is not None:
            self.settings_if.reset()
        if do_updates == True:
            pass
        self.initCb(do_updates = True)

    def factoryResetCb(self,do_updates = True):
        if self.node_if is not None:
            pass # self.node_if.factory_reset_params()
        if self.save_data_if is not None:
            self.save_data_if.factory_reset()
        if self.settings_if is not None:
            self.settings_if.factory_reset()
        if do_updates == True:
            pass
        self.initCb(do_updates = True)



    ###############################
    # Class Methods

    def getPanAdj(self,pan_deg):
        if pan_deg is None:
            pan_deg = 0.0
        return pan_deg * self.rpi

    def getPanRatioAdj(self,ratio):
        if self.reverse_pan_enabled == True:
            ratio = 1 - ratio
        return ratio

    def getTiltAdj(self,tilt_deg):
        if tilt_deg is None:
            tilt_deg = 0.0
        return tilt_deg * self.rti

    def getPanTiltAdj(self,pan_deg,tilt_deg):
        adj_pan = self.getPanAdj(pan_deg)
        adj_tilt = self.getTiltAdj(tilt_deg)
        return adj_pan,adj_tilt

    def getTiltRatioAdj(self,ratio):
        if self.reverse_tilt_enabled == True:
            ratio = 1 - ratio
        return ratio

    def getPanMinMaxAdj(self,min_deg,max_deg):
        if self.reverse_pan_enabled == True:
            adj_min = copy.deepcopy(max_deg) * -1
            adj_max = copy.deepcopy(min_deg) * -1
        else:
            adj_min = min_deg
            adj_max = max_deg
        return adj_min,adj_max

    def getTiltMinMaxAdj(self,min_deg,max_deg):
        if self.reverse_tilt_enabled == True:
            adj_min = copy.deepcopy(max_deg) * -1
            adj_max = copy.deepcopy(min_deg) * -1
        else:
            adj_min = min_deg
            adj_max = max_deg
        return adj_min,adj_max

    def getLimitsAdj(self,pan_min,pan_max,tilt_min,tilt_max):
        [adj_pan_min,adj_pan_max] = self.getPanMinMaxAdj(pan_min,pan_max)
        [adj_tilt_min,adj_tilt_max] = self.getTiltMinMaxAdj(tilt_min,tilt_max)
        return adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max

    def getLimitsHardstopAdj(self):
        pan_min = self.min_pan_hardstop_deg
        pan_max = self.max_pan_hardstop_deg
        [adj_pan_min,adj_pan_max] = self.getPanMinMaxAdj(pan_min,pan_max)

        tilt_min = self.min_tilt_hardstop_deg
        tilt_max = self.max_tilt_hardstop_deg
        [adj_tilt_min,adj_tilt_max] = self.getTiltMinMaxAdj(tilt_min,tilt_max)

        return adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max

    def getLimitsSoftstopAdj(self):
        pan_min = self.min_pan_softstop_deg
        pan_max = self.max_pan_softstop_deg
        [adj_pan_min,adj_pan_max] = self.getPanMinMaxAdj(pan_min,pan_max)

        tilt_min = self.min_tilt_softstop_deg
        tilt_max = self.max_tilt_softstop_deg
        [adj_tilt_min,adj_tilt_max] = self.getTiltMinMaxAdj(tilt_min,tilt_max)

        return adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max




    def getPtPosition(self, orien_dict):
        x = 0.0
        y = 0.0
        z = 0.0
        if orien_dict is not None:
            
            ho = 0
            xo = 0
            yo = 0
            zo = 0

            # calculate pos from tilt axis
            tilt_rad = -1 * math.radians(orien_dict['pitch_deg'])
            x = (xo * np.cos(tilt_rad) - zo * np.sin(tilt_rad))
            z = (zo * np.cos(tilt_rad) + xo * np.sin(tilt_rad)) * self.rpi

            # Add tilt axis height
            z += (ho * self.rpi)

            # Add left right offset
            y += yo * self.rpi

            # Rotate about center pan axis
            pan_deg = orien_dict['yaw_deg'] * self.rpi
            [x,y,z] = nepi_utils.rotate_3d([x,y,z], 'z', pan_deg) 

        return x,y,z


    # def getNpPositionAdjustedCb(self):
    #     pos_dict = dict()
    #     pos_dict['time_position'] = nepi_utils.get_time()
    #     pos_dict['x_m'] = 0.0
    #     pos_dict['y_m'] = 0.0     
    #     pos_dict['z_m'] = 0.0

    #     if self.getNpPositionCb is not None:
    #         [x,y,z] = self.getPtPosition(orien_dict)
    #         pos_dict['x_m'] = round(x,5)
    #         pos_dict['y_m'] = round(y,5)   
    #         pos_dict['z_m'] = round(z,5)
    #         self.msg_if.pub_debug("Calculate navpose x,y,z: " + str([x,y,z]), log_name_list = self.log_name_list, throttle_s = 3.0)
    #     return pos_dict

    # def getNpOrientationAdjustedCb(self):
    #     orien_dict = dict()
    #     orien_dict['time_orientation'] = nepi_utils.get_time()
    #     orien_dict['yaw_deg'] = self.current_position[0]
    #     orien_dict['pitch_deg'] = self.current_position[1]      

    #     if self.getNpOrientationCb is not None:
    #         orien_dict = self.getNpOrientationCb()
    #         pan_deg = orien_dict['yaw_deg']
    #         orien_dict['yaw_deg'] = self.getPanAdj(pan_deg)
    #         tilt_deg = orien_dict['pitch_deg']
    #         orien_dict['pitch_deg'] = self.getTiltAdj(tilt_deg)
    #     return orien_dict
           

    def check_ready(self):
        """Returns the current ready state of the PTX device interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Blocks until the PTX device interface is ready or the timeout expires.

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





    def panRatioToDeg(self, ratio):
        pan_deg = 0
        if self.reverse_pan_enabled == False:
           pan_deg =  self.min_pan_softstop_deg + (1-ratio) * (self.max_pan_softstop_deg - self.min_pan_softstop_deg)
        else:
           pan_deg =  self.max_pan_softstop_deg - (1-ratio)  * (self.max_pan_softstop_deg - self.min_pan_softstop_deg)
        return  pan_deg
    
    def panDegToRatio(self, deg):
        ratio = 0.5
        if self.reverse_pan_enabled == False:
          ratio = 1 - (deg - self.min_pan_softstop_deg) / (self.max_pan_softstop_deg - self.min_pan_softstop_deg)
        else:
          ratio = (deg - self.min_pan_softstop_deg) / (self.max_pan_softstop_deg - self.min_pan_softstop_deg)
        return (ratio)
     
    def tiltDegToRatio(self, deg):
        ratio = 0.5
        if self.reverse_tilt_enabled == False:
          ratio = 1 - (deg - self.min_tilt_softstop_deg) / (self.max_tilt_softstop_deg - self.min_tilt_softstop_deg)
        else:
          ratio = (deg - self.min_tilt_softstop_deg) / (self.max_tilt_softstop_deg - self.min_tilt_softstop_deg)
        return ratio
    
    def tiltRatioToDeg(self, ratio):
        tilt_deg = 0
        if self.reverse_tilt_enabled == False:
           tilt_deg =  self.min_tilt_softstop_deg + (1-ratio) * (self.max_tilt_softstop_deg - self.min_tilt_softstop_deg)
        else:
           tilt_deg =  (self.max_tilt_softstop_deg) - (1-ratio) * (self.max_tilt_softstop_deg - self.min_tilt_softstop_deg)
        return  tilt_deg



    def positionWithinSoftLimits(self, pan_deg, tilt_deg):
        valid = False
        [adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max] = self.getLimitsSoftstopAdj()
        if (pan_deg <= adj_pan_max) or (pan_deg >= adj_pan_min) or \
           (tilt_deg <= adj_tilt_max) or (tilt_deg >= adj_tilt_min):
            valid = True
        
        return valid


    def getPositionConditioned(self, pan_deg, tilt_deg):
        [adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max] = self.getLimitsSoftstopAdj()
        #self.msg_if.pub_info("Checking Positions against soft limits: " + str([pan_deg, tilt_deg]), log_name_list = self.log_name_list)
        #self.msg_if.pub_info("Got Soft Limits: " + str([adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max]), log_name_list = self.log_name_list)
        if (pan_deg > adj_pan_max):
            pan_deg = adj_pan_max
        if (pan_deg < adj_pan_min):
            pan_deg = adj_pan_min
        if (tilt_deg > adj_tilt_max):
            tilt_deg = adj_tilt_max
        if (tilt_deg < adj_tilt_min):
            tilt_deg = adj_tilt_min

        place = self.move_decimal_place
        pan_deg = nepi_utils.get_sign(pan_deg) * (math.floor( abs(pan_deg * 10**(place)) ) / 10**(place))
        tilt_deg = nepi_utils.get_sign(tilt_deg) * (math.floor( abs(tilt_deg * 10**(place)) ) / 10**(place))
        return pan_deg,tilt_deg



 
    def _setSoftstopCb(self, msg):
        min_pan = msg.min_pan_deg
        max_pan = msg.max_pan_deg
        min_tilt = msg.min_tilt_deg
        max_tilt = msg.max_tilt_deg

        [min_pan_adj,max_pan_adj,min_tilt_adj,max_tilt_adj] = self.getLimitsAdj(min_pan,max_pan,min_tilt,max_tilt)

        valid = False
        if min_pan_adj >= self.min_pan_hardstop_deg and max_pan_adj <= self.max_pan_hardstop_deg and min_pan_adj < max_pan_adj:  
            if min_tilt_adj >= self.min_tilt_hardstop_deg and max_tilt_adj <= self.max_tilt_hardstop_deg and min_tilt_adj < max_tilt_adj:
                if abs(max_pan_adj - min_pan_adj) >= self.MIN_LIMIT_ANGLE and abs(max_tilt_adj - min_tilt_adj) >= self.MIN_LIMIT_ANGLE:
                    valid = True

                    self.min_pan_softstop_deg =  min_pan_adj
                    self.max_pan_softstop_deg = max_pan_adj
                    self.min_tilt_softstop_deg = min_tilt_adj
                    self.max_tilt_softstop_deg = max_tilt_adj

                    if None not in [min_pan_adj,max_pan_adj,min_tilt_adj,max_tilt_adj]:
                        if self.setSoftLimitsCb is not None:
                            self.setSoftLimitsCb(min_pan_adj,max_pan_adj,min_tilt_adj,max_tilt_adj)

                        self.node_if.set_param('max_pan_softstop_deg', max_pan_adj)
                        self.node_if.set_param('min_pan_softstop_deg', min_pan_adj)
                        self.node_if.set_param('max_tilt_softstop_deg', max_tilt_adj)
                        self.node_if.set_param('min_tilt_softstop_deg', min_tilt_adj)


        if valid == False:
            self.msg_if.pub_warn("Invalid softstop requested " + str(msg))
        
  
    def _stopMovingCb(self, _):
        self.msg_if.pub_warn("Got Stop Moving All msg")
        self.stopPanTilt('All')

    def _stopPanCb(self, _):
        self.msg_if.pub_warn("Got Stop Moving Pan msg")
        self.stopPanTilt('pan')

    def _stopTiltCb(self, _):
        self.msg_if.pub_warn("Got Stop Moving Tilt msg")
        self.stopPanTilt('tilt')

    def stopPanTilt(self,axis = 'All'):
        self.msg_if.pub_warn("Stopping Move for axis: " + str(axis))
        if axis == 'pan' or axis == 'All':
            pan_stop_deg = self.status_msg.pan_now_deg * self.rpi
        else:
            pan_stop_deg = self.status_msg.pan_goal_deg * self.rpi
        if axis == 'tilt' or axis == 'All':
            tilt_stop_deg = self.status_msg.tilt_now_deg * self.rpi
        else:
            tilt_stop_deg = self.status_msg.tilt_goal_deg * self.rpi

        self.msg_if.pub_info("Stopping motion by request", log_name_list = self.log_name_list)
        if self.stopMovingCb is not None:
            self.stopMovingCb()
        elif self.gotoPositionCb is not None:
            self.gotoPositionCb(pan_stop_deg,tilt_stop_deg)
        self.publish_status()



    def _setMaxSpeedCb(self, msg):
            max_speed = msg.data
            if (max_speed > 0.0):
                
                if self.setSpeedMaxCb is not None:
                    self.setSpeedMaxCb(max_speed) 
                    self.speed_max_dps = max_speed
                self.publish_status()
                self.node_if.set_param('speed_max_dps',max_speed)
                self.msg_if.pub_warn("Updated max speed to " + str(max_speed))

    def _setSpeedRatioCb(self, msg):
        ratio = nepi_utils.check_ratio(msg.data)
        self._setSpeedRatio(ratio)

    def _setSpeedRatio(self,speed_ratio):
        if self.caps_report.has_adjustable_speed == True:
            speed_cur = math.floor(self.getSpeedRatioCb())
            
            self.msg_if.pub_warn("new speed ratio " + "%.2f" % speed_ratio)
            self.msg_if.pub_warn("cur speed ratio " + "%.2f" % speed_cur)

            self.speed_ratio = speed_ratio
            self.speed_pan_ratio = speed_ratio
            self.speed_tilt_ratio = speed_ratio
            self.publish_status()
            
            self.setSpeedRatioCb(speed_ratio)
            self.node_if.set_param('speed_ratios',[speed_ratio,speed_ratio])
    
        

    def _setPanSpeedRatioCb(self, msg):
        ratio = nepi_utils.check_ratio(msg.data)
        self._setPanSpeedRatio(ratio)

    def _setPanSpeedRatio(self,speed_ratio):
        if self.caps_report.has_seperate_pan_tilt_speed == True:
            speed_cur = math.floor(self.getPanSpeedRatioCb())
            if speed_cur != speed_ratio and self.setPanSpeedRatioCb is not None:
                self.speed_pan_ratio = speed_ratio
                self.setPanSpeedRatioCb(speed_ratio)
                self.speed_ratio = max([speed_ratio,self.speed_tilt_ratio])
                self.node_if.set_param('speed_ratios', [speed_ratio,self.speed_tilt_ratio])
                self.publish_status()
           

    def _setTiltSpeedRatioCb(self, msg):
        ratio = nepi_utils.check_ratio(msg.data)
        self._setTiltSpeedRatio(ratio)

    def _setTiltSpeedRatio(self,speed_ratio):
        if self.caps_report.has_seperate_pan_tilt_speed == True:
            speed_cur = math.floor(self.getTiltSpeedRatioCb())
            if speed_cur != speed_ratio and self.setTiltSpeedRatioCb is not None:
                self.speed_tilt_ratio = speed_ratio
                self.setTiltSpeedRatioCb(speed_ratio)
                self.speed_ratio = max([self.speed_pan_ratio,speed_ratio])
                self.node_if.set_param('speed_ratios', [self.speed_pan_ratio,speed_ratio])
                self.publish_status()

    def _setHomePositionCb(self, msg):
        [pan_deg_adj,tilt_deg_adj] = self.getPanTiltAdj(msg.pan_deg, msg.tilt_deg)
        if not self.positionWithinSoftLimits(pan_deg_adj, tilt_deg_adj):
            self.msg_if.pub_warn("Requested home position is invalid... ignoring", log_name_list = self.log_name_list)
            return

        self.home_pan_deg = pan_deg_adj
        self.home_tilt_deg = tilt_deg_adj
        if self.node_if is not None:
            self.node_if.set_param('home_position/pan_deg', self.home_pan_deg)
            self.node_if.set_param('home_position/tilt_deg', self.home_tilt_deg)
        self.msg_if.pub_info("Updated home position to " + "%.2f" % self.home_pan_deg + " " + "%.2f" %  self.home_tilt_deg)
        self.publish_status()
        if self.setHomePositionCb is not None:
            # Driver supports absolute positioning, so just let it operate
            self.setHomePositionCb(self.home_pan_deg, self.home_tilt_deg)
        else:
            self.msg_if.pub_warn("Absolution position home setpoints not available... ignoring", log_name_list = self.log_name_list)
            return
        
        
            

    def _goHomeCb(self, _):
        self.msg_if.pub_warn("Got Go Home msg")
        self.goHome()

    def goHome(self):
        if self.gotoPositionCb is not None:
            self.pan_goal_deg = self.home_pan_deg
            self.tilt_goal_deg = self.home_tilt_deg
            self.gotoPositionCb(self.pan_goal_deg,self.tilt_goal_deg)
            self.publish_status()
        elif self.goHomeCb is not None:
            self.goHomeCb()
        

    def _gotoPositionCb(self, msg):
        #self.msg_if.pub_warn("Got goto position msg: " + str(msg))
        [pan_deg_adj,tilt_deg_adj] = self.getPanTiltAdj(msg.pan_deg,msg.tilt_deg)
        self.gotPosition(pan_deg_adj,tilt_deg_adj)

    def gotPosition(self,pan_deg,tilt_deg):
            [pan_deg,tilt_deg] = self.getPositionConditioned(pan_deg, tilt_deg)
            self.pan_goal_deg = pan_deg
            self.tilt_goal_deg = tilt_deg
            #self.msg_if.pub_info("Driving to Pan/Tilt deg position  " + "%.2f" % (pan_deg * self.rpi) + " " + "%.2f" % (tilt_deg * self.rti))
            self.gotoPositionCb(pan_deg, tilt_deg)
            self.publish_status()

    def _gotoPanPositionCb(self, msg):
        #self.msg_if.pub_warn("Got goto pan position msg: " + str(msg))
        pan_deg_adj = self.getPanAdj(msg.data)
        self.gotoPanPosition(pan_deg_adj)

    def gotoPanPosition(self,pan_deg):
        [pan_deg,tilt_deg] = self.getPositionConditioned(pan_deg, self.tilt_goal_deg)
        self.pan_goal_deg = pan_deg
        if self.gotoPanPositionCb is not None:
            #self.msg_if.pub_info("Driving to pan deg" + "%.2f" % (pan_deg * self.rpi))
            self.gotoPanPositionCb(self.pan_goal_deg)
        elif self.gotoPositionCb is not None:
            #self.msg_if.pub_info("Driving to pan/tilt deg " + "%.2f" % pan_deg * self.rpi + " " + "%.2f" % tilt_deg * self.rti)
            self.gotoPositionCb(pan_deg, tilt_deg)
        self.publish_status()
                 

    def _gotoTiltPositionCb(self, msg):
        #self.msg_if.pub_warn("Got goto tilt position msg: " + str(msg))
        tilt_deg_adj = self.getTiltAdj(msg.data)
        #self.msg_if.pub_warn("Got adj goto tilt position msg: " + str(tilt_deg_adj))
        self.gotoTiltPosition(tilt_deg_adj)

    def gotoTiltPosition(self,tilt_deg):
        [pan_deg,tilt_deg] = self.getPositionConditioned(self.pan_goal_deg, tilt_deg)
        self.tilt_goal_deg = tilt_deg
        if self.gotoTiltPositionCb is not None:
            #self.msg_if.pub_info("Driving to tilt deg " + "%.2f" % (tilt_deg * self.rti))
            self.gotoTiltPositionCb(self.tilt_goal_deg)    
        elif self.gotoPositionCb is not None:
            #self.msg_if.pub_info("Driving to pan/tilt deg " + "%.2f" % pan_deg * self.rpi + " " + "%.2f" % tilt_deg * self.rti)
            self.gotoPositionCb(pan_deg, tilt_deg)
        self.publish_status()
    

    def _gotoToPanRatioCb(self, msg):
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            self.msg_if.pub_warn("Invalid pan position ratio " + "%.2f" % ratio)
            return
        self.pan_goal_deg = self.panRatioToDeg(ratio) # Function takes care of reverse conversion
        if self.gotoPanPositionCb is not None:
            #self.msg_if.pub_info("Driving to pan ratio - pan deg:  " + str(self.pan_goal_deg * self.rpi))
            self.msg_if.pub_info("Calling gotoPanPositionCb with deg :" + str(self.pan_goal_deg))
            self.gotoPanPositionCb(self.pan_goal_deg)
        elif self.gotoPositionCb is not None:
            tilt_goal_deg = self.getTiltAdj(self.tilt_goal_deg)
            #self.msg_if.pub_info("Driving to pan ratio - pan/tilt deg  " + "%.2f" % self.pan_goal_deg * self.rpi + " " + "%.2f" % self.tilt_goal_deg * self.rti)
            self.gotoPositionCb(self.pan_goal_deg, tilt_goal_deg)
        self.publish_status()
        

    def _gotoToTiltRatioCb(self, msg):
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            self.msg_if.pub_warn("Invalid tilt position ratio " + "%.2f" % ratio)
            return
        self.tilt_goal_deg = self.tiltRatioToDeg(ratio) # Function takes care of reverse conversion
        if self.gotoTiltPositionCb is not None:
            #self.msg_if.pub_info("Driving to tilt ratio - tilt deg  " + "%.2f" % self.tilt_goal_deg * self.rti)
            self.gotoTiltPositionCb(self.tilt_goal_deg)
        elif self.gotoPositionCb is not None:
            

            self.pan_goal_deg = self.getPanAdj(self.status_msg.pan_now_deg)
            #self.msg_if.pub_info("Driving to tilt ratio - pan/tilt deg  " + "%.2f" % self.pan_goal_deg * self.rpi + " " + "%.2f" % self.tilt_goal_deg * self.rti)
            self.gotoPositionCb(self.pan_goal_deg, self.tilt_goal_deg)
        self.publish_status()
        

    def _jogTimedPanCb(self, msg):
        #self.msg_if.pub_warn("Got job pan msg: " + str(msg))
        if self.movePanCb is not None:
            self.pan_goal_deg = -999
            direction = msg.direction * self.rpi
            time_s = msg.duration_s
            self.movePanCb(direction,  time_s)
            #self.msg_if.pub_info("Jogging pan", log_name_list = self.log_name_list)
        self.publish_status()
        

    def _jogTimedTiltCb(self, msg):
        #self.msg_if.pub_warn("Got job tilt msg: " + str(msg))
        if self.moveTiltCb is not None:
            self.tilt_goal_deg = -999
            direction = msg.direction * self.rti
            time_s = msg.duration_s
            self.moveTiltCb(direction, time_s)
            #self.msg_if.pub_info("Jogging tilt", log_name_list = self.log_name_list)
        self.publish_status()

    def _jogTimedPanSpeedRatioCb(self, msg):
        #self.msg_if.pub_warn("Got job pan msg: " + str(msg))
        if self.movePanSpeedRatioCb is not None:
            self.pan_goal_deg = -999
            direction = msg.direction * self.rpi
            speed_ratio = nepi_utils.check_ratio(msg.speed_ratio)
            time_s = msg.duration_s
            self.movePanSpeedRatioCb(direction, speed_ratio, time_s)
            #self.msg_if.pub_info("Jogging pan", log_name_list = self.log_name_list)
        self.publish_status()
        

    def _jogTimedTiltSpeedRatioCb(self, msg):
        #self.msg_if.pub_warn("Got job tilt msg: " + str(msg))
        if self.moveTiltSpeedRatioCb is not None:
            self.tilt_goal_deg = -999
            direction = msg.direction * self.rti
            speed_ratio = nepi_utils.check_ratio(msg.speed_ratio)
            time_s = msg.duration_s
            self.moveTiltSpeedRatioCb(direction, speed_ratio, time_s)
            #self.msg_if.pub_info("Jogging tilt at ratio ", log_name_list = self.log_name_list)
        self.publish_status()
        

    def _setReversePanEnableCb(self, msg):
        self.reverse_pan_enabled = msg.data
        rpi = 1
        if msg.data == True:
            rpi = -1
        self.rpi = rpi
        if self.node_if is not None:
            self.node_if.set_param('reverse_pan_enabled', self.reverse_pan_enabled)
        self.msg_if.pub_info("Set pan control to reverse=" + str(self.reverse_pan_enabled))
        self.publish_status()


    def _setReverseTiltEnableCb(self, msg):
        self.reverse_tilt_enabled = msg.data
        rti = 1
        if msg.data == True:
            rti = -1
        self.rti = rti
        if self.node_if is not None:
            self.node_if.set_param('reverse_tilt_enabled', self.reverse_tilt_enabled)
        self.msg_if.pub_info("Set tilt control to reverse=" + str(self.reverse_tilt_enabled))
        self.publish_status()
        

    def _setHomePositionHereCb(self, _):
        self.home_pan_deg = self.status_msg.pan_now_deg * self.rpi
        self.home_tilt_deg = self.status_msg.tilt_now_deg * self.rti
        if self.node_if is not None:
            self.node_if.set_param('home_position/pan_deg', self.home_pan_deg)
            self.node_if.set_param('home_position/tilt_deg', self.home_tilt_deg)
        self.msg_if.pub_info("Home positon set to: " + "%.2f" % self.home_pan_deg * self.rpi + " " + "%.2f" % self.home_tilt_deg * self.rti)
        self.publish_status()
        if self.setHomePositionHereCb is not None:
            self.setHomePositionHereCb()
          
    def _calibrateCenterCB(self,msg):
        if self.calibrateCenterCB is not None:
            self.calibrateCenterCB()

    def _setMoveDecimalPlaceCb(self, msg):
        val = msg.data
        if val < 0:
            self.msg_if.pub_warn("Invalid move_decimal_place requested: " + str(val))
            return
        self.move_decimal_place = val
        self.node_if.set_param('move_decimal_place', val)
        self.msg_if.pub_info("Updated move_decimal_place to " + str(val))

    def _setReportDecimalPlaceCb(self, msg):
        val = msg.data
        if val < 0:
            self.msg_if.pub_warn("Invalid report_decimal_place requested: " + str(val))
            return
        self.report_decimal_place = val
        self.node_if.set_param('report_decimal_place', val)
        self.msg_if.pub_info("Updated report_decimal_place to " + str(val))

    # ---- Standard NEPI motor command interface -------------------------------
    # motor_0 = pan, motor_1 = tilt, "all" = both. Direction convention:
    #   1 (clockwise)         -> increasing axis angle -> max soft-limit
    #  -1 (counter-clockwise) -> decreasing axis angle -> min soft-limit
    # Continuous motion (go_direction) is implemented by driving the axis to its
    # soft-limit at the currently set speed, using the existing gotoPanPosition /
    # gotoTiltPosition primitives; motion runs until the limit is reached or a
    # stop_motor command arrives.

    def _resolveMotorAxes(self, motor_name):
        name = motor_name if motor_name is not None else ''
        name = name.strip().lower()
        if name == 'all':
            return ['pan', 'tilt']
        if name == 'motor_0':
            return ['pan']
        if name == 'motor_1':
            return ['tilt']
        self.msg_if.pub_warn("Unknown motor_name in motor command: " + str(motor_name))
        return []

    def _setMotorSpeedCb(self, msg):
        axes = self._resolveMotorAxes(msg.motor_name)
        if len(axes) == 0:
            return
        speed_ratio = nepi_utils.check_ratio(msg.speed_ratio)
        # Use per-axis speed control when the hardware supports it, otherwise
        # fall back to the combined speed ratio (which applies to both axes).
        if self.has_seperate_pan_tilt_speed == True:
            if 'pan' in axes:
                self._setPanSpeedRatio(speed_ratio)
            if 'tilt' in axes:
                self._setTiltSpeedRatio(speed_ratio)
        else:
            self._setSpeedRatio(speed_ratio)

    def _setMotorDirectionCb(self, msg):
        axes = self._resolveMotorAxes(msg.motor_name)
        if len(axes) == 0:
            return
        direction = 1 if msg.direction >= 0 else -1
        if 'pan' in axes:
            self.cmd_dir_pan = direction
        if 'tilt' in axes:
            self.cmd_dir_tilt = direction
        self.msg_if.pub_info("Set motor direction " + str(msg.motor_name) + " to " + str(direction))
        self.publish_status()

    def _goMotorDirectionCb(self, msg):
        axes = self._resolveMotorAxes(msg.motor_name)
        if len(axes) == 0:
            return
        if 'pan' in axes:
            self._goPanDirection(self.cmd_dir_pan)
        if 'tilt' in axes:
            self._goTiltDirection(self.cmd_dir_tilt)

    def _goPanDirection(self, direction):
        target_deg = self.max_pan_softstop_deg if direction >= 0 else self.min_pan_softstop_deg
        self.msg_if.pub_info("Driving pan continuously toward soft-limit deg " + str(target_deg))
        self.gotoPanPosition(target_deg)

    def _goTiltDirection(self, direction):
        target_deg = self.max_tilt_softstop_deg if direction >= 0 else self.min_tilt_softstop_deg
        self.msg_if.pub_info("Driving tilt continuously toward soft-limit deg " + str(target_deg))
        self.gotoTiltPosition(target_deg)

    def _stopMotorCb(self, msg):
        axes = self._resolveMotorAxes(msg.motor_name)
        if len(axes) == 0:
            return
        if 'pan' in axes and 'tilt' in axes:
            self.stopPanTilt('All')
        elif 'pan' in axes:
            self.stopPanTilt('pan')
        elif 'tilt' in axes:
            self.stopPanTilt('tilt')


    ### Info callback
    def info_query_callback(self, _):
        """Service handler that returns the device info report.

        Returns:
            DeviceInfoQueryResponse: Response containing device name, path, node name,
                namespace, serial number, hardware version, software version, and type.
        """
        return self.info_report


    def capabilities_query_callback(self, _):
        """Service handler that returns the PTX capabilities report.

        Returns:
            PTXCapabilitiesQueryResponse: Response describing which PTX capabilities
                are supported, including absolute positioning, timed positioning,
                separate pan/tilt control, position feedback, adjustable speed,
                limit controls, homing, set-home, and calibration.
        """
        return self.caps_report
    
  


    def _publishStatusCb(self,timer):
        self.publish_status()



    def publish_status(self, do_updates = False):
        """Builds and publishes the DevicePTXStatus message to the status topic.

        Reads current pan/tilt position from the driver if absolute positioning is
        available, computes smoothed axis speeds, recalculates ratio and degree fields
        for current and goal positions against soft-stop limits, then publishes the
        assembled DevicePTXStatus message and a NavPosePanTilt message.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.

        Returns:
            float: Elapsed time in seconds spent building and publishing the status.
        """
        #self.msg_if.pub_warn("entering Pub_stat msg", throttle_s = 5.0)
        start_time = nepi_utils.get_time()
        #self.msg_if.pub_info("Entering Publish Status", log_name_list = self.log_name_list)
        if self.has_absolute_positioning == True and self.getPositionCb is not None:
            
            
            cur_time = nepi_utils.get_time()
            last_time = copy.deepcopy(self.pan_last_time)

            self.status_msg.move_decimal_place = self.move_decimal_place
            self.status_msg.report_decimal_place = self.report_decimal_place

            [pan_deg ,tilt_deg] = self.getPositionCb()
            #self.msg_if.pub_warn("Got PT position: " + str(pan_deg) + " : " + str(tilt_deg))
            
            if self.getPositionTimesCb is not None:
                self.position_times = self.getPositionTimesCb()
                #self.msg_if.pub_warn("Got Position Times: " + str(self.position_times))
            else:
                self.position_times = [cur_time,cur_time]
            #self.msg_if.pub_warn("Using PT position times: " + str(self.position_times))
            place = self.report_decimal_place
            pan_deg = nepi_utils.get_sign(pan_deg) * (math.floor( abs(pan_deg * 10**(place)) ) / 10**(place))
            tilt_deg = nepi_utils.get_sign(tilt_deg) * (math.floor( abs(tilt_deg * 10**(place)) ) / 10**(place))
            self.current_position = [pan_deg ,tilt_deg]
            #self.msg_if.pub_warn("Checking for time change: " + str([self.position_times[0],self.pan_time_history[-1]]))
            if self.position_times[0] != self.pan_time_history[-1]:
                self.pan_pos_history.append(pan_deg)
                self.pan_time_history.append(self.position_times[0])
                if len(self.pan_pos_history) > self.SPEED_SMOOTHING_WINDOW:
                    self.pan_pos_history.pop(0)
                    self.pan_time_history.pop(0)
                if len(self.pan_pos_history) >= 2:
                    #self.msg_if.pub_warn("Getting Pan Speed from pos: " + str(self.pan_pos_history))
                    #self.msg_if.pub_warn("Getting Pan Speed from times: " + str(self.pan_time_history))
                    speeds = []
                    for i in range(len(self.pan_pos_history) - 1):
                        delta_time = self.pan_time_history[i+1] - self.pan_time_history[i]
                        speed = (self.pan_pos_history[i+1] - self.pan_pos_history[i]) / delta_time
                        speeds.append(speed)
                    self.speed_pan_dps = sum(speeds) / len(speeds)
                    #self.msg_if.pub_warn("Got Pan Speed: " + str(self.speed_pan_dps))

            if self.position_times[1] != self.tilt_time_history[-1]:
                self.tilt_pos_history.append(tilt_deg)
                self.tilt_time_history.append(self.position_times[1])
                if len(self.tilt_pos_history) > self.SPEED_SMOOTHING_WINDOW:
                    self.tilt_pos_history.pop(0)
                    self.tilt_time_history.pop(0)
                if len(self.tilt_pos_history) >= 2:
                    speeds = []
                    for i in range(len(self.tilt_pos_history) - 1):
                        delta_time = self.tilt_time_history[i+1] - self.tilt_time_history[i]
                        speed = (self.tilt_pos_history[i+1] - self.tilt_pos_history[i]) / delta_time
                        speeds.append(speed)
                    self.speed_tilt_dps = sum(speeds) / len(speeds)

            #self.msg_if.pub_warn("Using PT position: " + str(self.current_position[0]) + " : " + str(self.current_position[1]))

            self.status_msg.speed_pan_dps = abs(self.speed_pan_dps)
            self.status_msg.speed_tilt_dps = abs(self.speed_tilt_dps)


            pan_now_deg_adj = self.getPanAdj(pan_deg)
            self.status_msg.pan_now_deg = pan_now_deg_adj

            if self.pan_goal_deg == -999:
                pan_goal = pan_now_deg_adj
            else:
                pan_goal = self.getPanAdj(self.pan_goal_deg)
            pan_goal_deg_adj = pan_goal
            self.status_msg.pan_goal_deg = pan_goal_deg_adj
            self.status_msg.pan_home_pos_deg = self.getPanAdj(self.home_pan_deg)

            tilt_now_deg_adj = self.getTiltAdj(tilt_deg)
            self.status_msg.tilt_now_deg = tilt_now_deg_adj

            if self.tilt_goal_deg == -999:
                tilt_goal = tilt_now_deg_adj
            else:
                tilt_goal = self.getTiltAdj(self.tilt_goal_deg)
            tilt_goal_deg_adj = tilt_goal
            self.status_msg.tilt_goal_deg = tilt_goal_deg_adj
            self.status_msg.tilt_home_pos_deg = self.getTiltAdj(self.home_tilt_deg)


            [adj_pan_hs_min,adj_pan_hs_max,adj_tilt_hs_min,adj_tilt_hs_max] = self.getLimitsHardstopAdj()
            [adj_pan_ss_min,adj_pan_ss_max,adj_tilt_ss_min,adj_tilt_ss_max] = self.getLimitsSoftstopAdj()

            self.status_msg.pan_min_hardstop_deg = adj_pan_hs_min
            self.status_msg.pan_max_hardstop_deg = adj_pan_hs_max
            self.status_msg.pan_min_softstop_deg = adj_pan_ss_min
            self.status_msg.pan_max_softstop_deg = adj_pan_ss_max

            self.status_msg.tilt_min_hardstop_deg = adj_tilt_hs_min
            self.status_msg.tilt_max_hardstop_deg = adj_tilt_hs_max
            self.status_msg.tilt_min_softstop_deg = adj_tilt_ss_min
            self.status_msg.tilt_max_softstop_deg = adj_tilt_ss_max
            

            axis_info = [pan_now_deg_adj,pan_goal_deg_adj,adj_pan_ss_min,adj_pan_ss_max]
            #self.msg_if.pub_warn("Using Pan now,goal,min,max: " + str(axis_info), log_name_list = self.log_name_list, throttle_s = 2.0)        
            pan_now_ratio_adj =  1 - (pan_now_deg_adj - adj_pan_ss_min) / (adj_pan_ss_max - adj_pan_ss_min) 
            self.status_msg.pan_now_ratio = pan_now_ratio_adj
            pan_goal_ratio_adj =  1 - (pan_goal_deg_adj - adj_pan_ss_min) / (adj_pan_ss_max - adj_pan_ss_min)
            self.status_msg.pan_goal_ratio = pan_goal_ratio_adj

            axis_info = [tilt_now_deg_adj,tilt_goal_deg_adj,adj_tilt_ss_min,adj_tilt_ss_max]
            #self.msg_if.pub_warn("Using Tilt now,goal,min,max: " + str(axis_info), log_name_list = self.log_name_list, throttle_s = 2.0)    
            tilt_now_ratio_adj =  1 - (tilt_now_deg_adj - adj_tilt_ss_min) / (adj_tilt_ss_max - adj_tilt_ss_min) 
            self.status_msg.tilt_now_ratio = tilt_now_ratio_adj
            tilt_goal_ratio_adj =  1 - (tilt_goal_deg_adj - adj_tilt_ss_min) / (adj_tilt_ss_max - adj_tilt_ss_min)
            self.status_msg.tilt_goal_ratio = tilt_goal_ratio_adj

            pan_changed = self.last_position[0] != self.current_position[0]
            tilt_changed = self.last_position[1] != self.current_position[1]

            self.pan_last_time = copy.deepcopy(cur_time)
            self.last_position = copy.deepcopy(self.current_position)

            self.status_msg.is_moving = pan_changed or tilt_changed


        self.status_msg.reverse_pan_enabled = self.reverse_pan_enabled
        self.status_msg.reverse_tilt_enabled = self.reverse_tilt_enabled

        # Re-read the max here rather than trusting the single sample taken in __init__.
        # getSpeedMaxCb can legitimately report a placeholder before the driver's own
        # devices have connected -- ptx_servos_node returns a fallback until its SVX
        # interfaces come up -- and sampling once at construction latched that placeholder
        # permanently. Because supplying getSpeedMaxCb also sets set_max_dps_disabled,
        # nothing downstream could correct it: the reported max was both wrong and locked.
        # Only a positive value is accepted, so a transient None or 0 cannot poison it.
        if self.getSpeedMaxCb is not None:
            speed_max_dps = self.getSpeedMaxCb()
            if speed_max_dps is not None and speed_max_dps > 0:
                self.speed_max_dps = speed_max_dps

        self.status_msg.set_max_dps_disabled = self.set_max_dps_disabled
        self.status_msg.speed_max_dps = self.speed_max_dps
        self.status_msg.speed_ratio = self.speed_ratio
        self.status_msg.has_seperate_pan_tilt_speed = self.has_seperate_pan_tilt_speed
        self.status_msg.speed_pan_ratio = self.speed_pan_ratio
        self.status_msg.speed_tilt_ratio = self.speed_tilt_ratio

        if self.node_if is not None:
            #self.msg_if.pub_warn("Created status msg: " + str(self.status_msg), throttle_s = 5.0)
            #self.msg_if.pub_debug("Publishing Status", log_name_list = self.log_name_list)
            self.node_if.publish_pub('status_pub',self.status_msg)
            pan_tilt_msg = NavPosePanTilt()
            pan_tilt_msg.timestamp = nepi_utils.get_time()
            pan_tilt_msg.pan_deg = self.status_msg.pan_now_deg
            pan_tilt_msg.tilt_deg = self.status_msg.tilt_now_deg
            self.node_if.publish_pub('pan_tilt_pub',pan_tilt_msg)
            # Standard per-motor status on the dedicated motor_status topic
            self.node_if.publish_pub('motor_status_pub', self.get_motors_status_msg())


        pub_time = nepi_utils.get_time() - start_time
        return pub_time

    def get_motors_status_msg(self):
        """Builds the standard MotorsStatus message for this pan/tilt device.

        Each physical motor maps to exactly one MotorStatus: motor_0 = pan,
        motor_1 = tilt. Fields are populated from existing PTX state. motor_dir is
        derived from the reverse-enable flags (-1 when reversed, 1 otherwise).

        Note: PTX has no per-axis enable/disable control, so motor_enable is
        reported True for both axes whenever the interface is active.

        Returns:
            MotorsStatus: Wrapper message with a two-element motors array
                [pan, tilt] published on the dedicated motor_status topic.
        """
        msg = MotorsStatus()
        msg.timestamp = nepi_utils.get_time()
        msg.device_name = self.device_name

        pan_motor = MotorStatus()
        pan_motor.motor_name = "motor_0"
        pan_motor.motor_enable = True
        pan_motor.motor_dir = -1 if self.reverse_pan_enabled else 1
        pan_motor.motor_max_speed = self.speed_max_dps
        pan_motor.motor_speed_ratio = self.speed_pan_ratio
        pan_motor.motor_speed = abs(self.speed_pan_dps)
        pan_motor.motor_position = self.status_msg.pan_now_deg

        tilt_motor = MotorStatus()
        tilt_motor.motor_name = "motor_1"
        tilt_motor.motor_enable = True
        tilt_motor.motor_dir = -1 if self.reverse_tilt_enabled else 1
        tilt_motor.motor_max_speed = self.speed_max_dps
        tilt_motor.motor_speed_ratio = self.speed_tilt_ratio
        tilt_motor.motor_speed = abs(self.speed_tilt_dps)
        tilt_motor.motor_position = self.status_msg.tilt_now_deg

        msg.motors = [pan_motor, tilt_motor]
        return msg

    def getZeroCb(self):
        return 0




    def passFunction(self):
        return 0

