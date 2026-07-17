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
import threading
import subprocess
import numpy as np
import math
import tf
import random
import sys
import cv2
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_nav


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from pygeodesy.ellipsoidalKarney import LatLon

from nepi_interfaces.msg import DeviceRBXInfo, DeviceRBXStatus 
from nepi_interfaces.msg import AxisControls, ErrorBounds
from nepi_interfaces.msg import MotorControl
from nepi_interfaces.msg import GotoPose, GotoPosition, GotoLocation, MotorControl, GotoErrors

from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryResponse, DeviceInfoQueryRequest

from nepi_interfaces.srv import RBXCapabilitiesQuery, RBXCapabilitiesQueryResponse, RBXCapabilitiesQueryRequest

from nepi_interfaces.msg import NavPose, NavPoses
from nepi_interfaces.msg import Transform

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF, Transform3DIF

from nepi_api.data_if import ImageIF
from nepi_api.device_if_npx import NPXDeviceIF
# NavPose is published by the NPXDeviceIF below (driver-supplied getNavPoseCb),
# matching device_if_npx/device_if_idx. The legacy ConnectNavPosesIF was removed
# from connect_data_if, so it is no longer imported here.
#from nepi_api.connect_data_if import ConnectNavPosesIF




EXAMPLE_HEADING_DATA_DICT = {
    'time_heading': nepi_utils.get_time(),
    # Heading should be provided in Degrees True North
    'heading_deg': 120.50,
}

EXAMPLE_POSITION_DATA_DICT = {
    'time_position': nepi_utils.get_time(),
    # Position should be provided in Meters ENU (x,y,z) with x forward, y left, and z up
    'x_m': 1.234,
    'y_m': 1.234,
    'z_m': 1.234,
}

EXAMPLE_ORIENTATION_DATA_DICT = {
    'time_orientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees ENU
    'roll_deg': 30.51,
    'pitch_deg': 30.51,
    'yaw_deg': 30.51,
}

EXAMPLE_LOCATION_DATA_DICT = {
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'lat': 47.080909,
    'long': -120.8787889,
}

EXAMPLE_ALTITUDE_DATA_DICT = {
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters WGS84
    'altitude_m': 12.321,
}

EXAMPLE_ALTITUDE_DATA_DICT = {
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive distance from surface in meters
    'altitude_m': 0
}


class RBXRobotIF:
    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
    STATUS_UPDATE_RATE_HZ = 2
    UPDATE_NAVPOSE_RATE_HZ = 10
    CHECK_SAVE_DATA_RATE_HZ = 40
    
    # Factory Control Values 
    FACTORY_GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
    FACTORY_GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
    FACTORY_GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding
    FACTORY_CMD_TIMEOUT_SEC = 25 # Any action that changes 
    FACTORY_IMAGE_INPUT_TOPIC_NAME = "color_2d_image" # Partial or full ROS namespace string, "" for black image 
    FACTORY_HOME_LOCATION = [47.6540828,-122.3187578,0.0]


    # Define class variables
    ready = False
    status_msg=DeviceRBXStatus()
    info_report = DeviceInfoQueryResponse()
    caps_report = RBXCapabilitiesQueryResponse()

    factory_device_name = ''

    factory_controls = dict()
 

    states = []
    modes = []
    setup_actions = []
    go_actions = []
    data_products_list = ['image']

    node_if = None
    settings_if = None
    save_data_if = None
    transform_if = None
    navpose_if = None
    npx_if = None

    status_msg_pub_interval = float(1)/float(STATUS_UPDATE_RATE_HZ)
    check_save_data_interval_sec = float(1)/CHECK_SAVE_DATA_RATE_HZ
    update_navpose_interval_sec = float(1)/UPDATE_NAVPOSE_RATE_HZ

    ## Initialize Class Variables
    current_heading_deg = 0
    current_orientation_enu_degs = [0,0,0]
    current_orientation_ned_degs = [0,0,0]
    current_position_enu_m = [0,0,0]
    current_position_ned_m = [0,0,0]
    current_location_amsl_geo = [0,0,0]
    current_location_wgs84_geo = [0,0,0]
    current_geoid_height_m = 0
    
    home_location = [0,0,0]
    last_cmd_string = ""

    rbx_image_sub = None
    rbx_state_last = None
    rbx_mode_last = None
    rbx_image_blank = np.zeros((350, 700, 3), dtype = np.uint8) # Empty Black Image
    cv2_img = np.zeros((350, 700, 3), dtype = np.uint8) # Empty Black Image
    rbx_image_source_last = "None"
    init_image_status_overlay = False

    tr_source_ref_description = 'system_center'
    tr_end_ref_description = 'nepi_frame'

    data_source_description = 'control_system'
    data_ref_description = 'control_system'

    navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    navpose_frame = 'nepi_frame'
    sys_navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

    ### IF Initialization
    def __init__(self, device_info, capSettings, 
                 factorySettings, settingUpdateFunction, getSettingsFunction,
                 axisControls,getBatteryPercentFunction,
                 states,getStateIndFunction,setStateIndFunction,
                 modes,getModeIndFunction,setModeIndFunction,
                 checkStopFunction,
                 setup_actions, setSetupActionIndFunction,
                 go_actions, setGoActionIndFunction,
                 data_source_description = 'control_system',
                 data_ref_description = 'control_system',
                 getHomeFunction=None,setHomeFunction=None,
                 manualControlsReadyFunction=None,
                 getMotorControlRatios=None,
                 setMotorControlRatio=None,
                 autonomousControlsReadyFunction=None,
                 goHomeFunction=None, goStopFunction=None, 
                 gotoPoseFunction=None, gotoPositionFunction=None, gotoLocationFunction=None,
                 getNavPoseCb=None,
                 navpose_update_rate = 10,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,'rbx')

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
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)


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
        self.info_report.type = 'RBX'

        self.caps_report.device_name = self.device_name
        self.caps_report.device_path = self.path
        self.caps_report.device_node_name = self.node_name




        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description

        self.states = states
        self.getStateIndFunction = getStateIndFunction
        self.setStateIndFunction = setStateIndFunction

        self.modes = modes
        self.getModeIndFunction = getModeIndFunction
        self.setModeIndFunction = setModeIndFunction
        
        self.checkStopFunction = checkStopFunction

        self.setup_actions = setup_actions
        self.setSetupActionIndFunction = setSetupActionIndFunction

        self.getMotorControlRatios = getMotorControlRatios

        self.getNavPoseCb = getNavPoseCb
        if navpose_update_rate < 1:
           navpose_update_rate = 1
        if navpose_update_rate > 10:
            navpose_update_rate = 10
        self.navpose_update_rate = navpose_update_rate


        # Initialize status message
        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description


        self.status_msg.process_current = "None"
        self.status_msg.process_last = "None"
        self.status_msg.ready = False
        self.status_msg.battery = 0
        errors_msg = GotoErrors()
        errors_msg.x_m = 0
        errors_msg.y_m = 0
        errors_msg.z_m = 0
        errors_msg.heading_deg = 0
        errors_msg.roll_deg = 0
        errors_msg.pitch_deg = 0
        errors_msg.yaw_deg = 0
        self.status_msg.errors_current = errors_msg
        self.status_msg.errors_prev = errors_msg
        self.status_msg.last_error_message = ""

        self.rbx_info=DeviceRBXInfo()
        self.rbx_info.connected = False
        self.rbx_info.serial_num = self.serial_num
        self.rbx_info.hw_version = self.hw_version
        self.rbx_info.sw_version = self.sw_version
        self.rbx_info.standby = False
        self.rbx_info.state = -999
        self.rbx_info.mode = -999
        self.rbx_info.home_lat = -999
        self.rbx_info.home_long = -999
        self.rbx_info.home_alt = -999
        self.rbx_info.state = self.getStateIndFunction()
        self.rbx_info.mode = self.getModeIndFunction()



        # Create and start initializing Capabilities values



        if axisControls == None:
          axis_controls = AxisControls()
          axis_controls.x = False
          axis_controls.y = False
          axis_controls.z = False
          axis_controls.roll = False
          axis_controls.pitch = False
          axis_controls.yaw = False
        self.caps_report.control_support = axisControls
        
        self.caps_report.state_options = states
        self.caps_report.mode_options = modes
        self.caps_report.setup_action_options = setup_actions
        self.caps_report.go_action_options = go_actions
        self.caps_report.data_products = self.data_products_list


        self.getBatteryPercentFunction = getBatteryPercentFunction
        if self.getBatteryPercentFunction is not None:
          self.caps_report.has_battery_feedback = True
        else:
          self.caps_report.has_battery_feedback = False

        self.setMotorControlRatio = setMotorControlRatio
        if self.setMotorControlRatio is None:
            self.caps_report.has_manual_controls = False
        else:
            self.caps_report.has_manual_controls = True
        self.getMotorControlRatios = getMotorControlRatios

        ## Setup Manual Controls
        self.manualControlsReadyFunction = manualControlsReadyFunction
        if self.manualControlsReadyFunction is not None:
          self.status_msg.manual_control_mode_ready = self.manualControlsReadyFunction()
          manual_controls_ready = self.manualControlsReadyFunction()
          if manual_controls_ready:
            if self.setMotorControlRatio is not None:
              for i in range(len(self.getMotorControlRatios())):
                self.setMotorControlRatio(i, 0.0)
        else:
          self.status_msg.manual_control_mode_ready = False

        if self.getMotorControlRatios is not None:
          motor_controls_status_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
          motor_controls_status_msg = self.get_motor_controls_status_msg([])
        self.status_msg.current_motor_control_settings = motor_controls_status_msg

        if self.getMotorControlRatios is not None:
            motor_controls_info_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
            motor_controls_info_msg = self.get_motor_controls_status_msg([])

        # Setup Autonomous Controls
        self.autonomousControlsReadyFunction = autonomousControlsReadyFunction
        if self.autonomousControlsReadyFunction is not None:
          self.status_msg.autonomous_control_mode_ready = self.autonomousControlsReadyFunction()
          self.caps_report.has_autonomous_controls = True
        else:
          self.status_msg.autonomous_control_mode_ready = False
          self.caps_report.has_autonomous_controls = False


        self.go_actions = go_actions
        self.setGoActionIndFunction = setGoActionIndFunction

  
        self.getHomeFunction = getHomeFunction
        self.setHomeFunction  = setHomeFunction
        if self.setHomeFunction is None :
            self.caps_report.has_set_home = False
        else:
            self.caps_report.has_set_home = True
     

        self.goHomeFunction = goHomeFunction
        if self.goHomeFunction is None:
            self.caps_report.has_go_home = False
        else:
            self.caps_report.has_go_home = True


        self.goStopFunction = goStopFunction
        if self.goStopFunction is None:
            self.caps_report.has_go_stop = False
        else:
            self.caps_report.has_go_stop = True


        self.gotoPoseFunction = gotoPoseFunction
        if self.gotoPoseFunction is None:
            self.caps_report.has_goto_pose = False
        else:
            self.caps_report.has_goto_pose = True            

        self.gotoPositionFunction = gotoPositionFunction
        if self.gotoPositionFunction is None:
            self.caps_report.has_goto_position = False
        else:
            self.caps_report.has_goto_position = True            

        self.gotoLocationFunction = gotoLocationFunction
        if self.gotoLocationFunction is None:
            self.caps_report.has_goto_location = False
        else:
            self.caps_report.has_goto_location = True

        self.status_msg.cmd_success = False




        ##################################################
        ### Node Class Setup

        self.msg_if.pub_info("Starting Node IF Initialization", log_name_list = self.log_name_list)
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.namespace
        }

        self.PARAMS_DICT = {
            'cmd_timeout': {
                'namespace': self.namespace,
                'factory_val': self.FACTORY_CMD_TIMEOUT_SEC
            },
            'home_location': {
                'namespace': self.namespace,
                'factory_val': self.FACTORY_HOME_LOCATION
            },
            'max_error_m': {
                'namespace': self.namespace,
                'factory_val': self.FACTORY_GOTO_MAX_ERROR_M
            },
            'max_error_deg': {
                'namespace': self.namespace,
                'factory_val': self.FACTORY_GOTO_MAX_ERROR_DEG
            },
            'stabilized_sec': {
                'namespace': self.namespace,
                'factory_val': self.FACTORY_GOTO_STABILIZED_SEC
            },
            'image_source': {
                'namespace': self.namespace,
                'factory_val': self.FACTORY_IMAGE_INPUT_TOPIC_NAME
            },
            'image_status_overlay': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'navpose_frame': {
                'namespace': self.namespace,
                'factory_val': self.navpose_frame
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
                'srv': RBXCapabilitiesQuery,
                'req': RBXCapabilitiesQueryRequest(),
                'resp': RBXCapabilitiesQueryResponse(),
                'callback': self.capabilities_query_callback
            }
        }

        self.PUBS_DICT = {
            'rbx_info_pub': {
                'namespace': self.namespace,
                'topic': 'info',
                'msg': DeviceRBXInfo,
                'qsize': 1,
                'latch': True
            },
            'status_msg_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DeviceRBXStatus,
                'qsize': 1,
                'latch': True
            },
            'status_msg_str_pub': {
                'namespace': self.namespace,
                'topic': 'status_str',
                'msg': String,
                'qsize': 1,
                'latch': True
            }
        }
     
        self.SUBS_DICT = {
            'set_goto_error_bounds': {
                'namespace': self.namespace,
                'topic': 'set_goto_error_bounds',
                'msg': ErrorBounds,
                'qsize': None,
                'callback': self.setErrorBoundsCb, 
                'callback_args': ()
            },
            'set_state': {
                'namespace': self.namespace,
                'topic': 'set_state',
                'msg': Int32,
                'qsize': None,
                'callback': self.setStateCb, 
                'callback_args': ()
            },

            'set_mode': {
                'namespace': self.namespace,
                'topic': 'set_mode',
                'msg': Int32,
                'qsize': None,
                'callback': self.setModeCb, 
                'callback_args': ()
            },

            'setup_action': {
                'namespace': self.namespace,
                'topic': 'setup_action',
                'msg': Int32,
                'qsize': None,
                'callback': self.setupActionCb,
                'callback_args': ()
            },

            'set_motor_control': {
                'namespace': self.namespace,
                'topic': 'set_motor_control',
                'msg': MotorControl,
                # A "set all motors" action (e.g. the RUI's main slider) fires
                # one message per motor in a near-simultaneous burst. qsize=None
                # maps to a queue_size of 1, which drops all but the newest
                # buffered message whenever the callback (a synchronous MAVROS
                # service round-trip) is still busy on the previous one --
                # observed as only the first and last motor of a burst actually
                # updating. Sized comfortably above CAP_SETTINGS' motor_count
                # max (16) so a full burst is never dropped.
                'qsize': 20,
                'callback': self.setMotorControlCb,
                'callback_args': ()
            },

            'go_action': {
                'namespace': self.namespace,
                'topic': 'go_action',
                'msg': Int32,
                'qsize': None,
                'callback': self.goActionCb,
                'callback_args': ()
            },

            'set_goto_timeout': {
                'namespace': self.namespace,
                'topic': 'set_goto_timeout',
                'msg': UInt32,
                'qsize': None,
                'callback': self.setCmdTimeoutCb,
                'callback_args': ()
            },

            'go_home': {
                'namespace': self.namespace,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': None,
                'callback': self.goHomeCb,
                'callback_args': ()
            },

            'set_home': {
                'namespace': self.namespace,
                'topic': 'set_home',
                'msg': GeoPoint,
                'qsize': None,
                'callback': self.setHomeCb, 
                'callback_args': ()
            },

            'set_home_current': {
                'namespace': self.namespace,
                'topic': 'set_home_current',
                'msg': GotoLocation,
                'qsize': None,
                'callback': self.setHomeCurrentCb, 
                'callback_args': ()
            },

            'goto_location': {
                'namespace': self.namespace,
                'topic': 'goto_location',
                'msg': GotoLocation,
                'qsize': None,
                'callback': self.gotoLocationCb, 
                'callback_args': ()
            },
            'goto_position': {
                'namespace': self.namespace,
                'topic': 'goto_position',
                'msg': GotoPosition,
                'qsize': None,
                'callback': self.gotoPositionCb, 
                'callback_args': ()
            },
            'goto_pose': {
                'namespace': self.namespace,
                'topic': 'goto_pose',
                'msg': GotoPose,
                'qsize': None,
                'callback': self.gotoPoseCb, 
                'callback_args': ()
            },
            'go_stop': {
                'namespace': self.namespace,
                'topic': 'go_stop',
                'msg': Empty,
                'qsize': None,
                'callback': self.goStopCb, 
                'callback_args': ()
            },
            'set_image_topic': {
                'namespace': self.namespace,
                'topic': 'set_image_topic',
                'msg': String,
                'qsize': None,
                'callback': self.setImageTopicCb, 
                'callback_args': ()
            },
            'enable_image_overlay': {
                'namespace': self.namespace,
                'topic': 'enable_image_overlay',
                'msg': Bool,
                'qsize': None,
                'callback': self.enableImageOverlayCb, 
                'callback_args': ()
            },
            'publish_status': {
                'namespace': self.namespace,
                'topic': 'publish_status',
                'msg': Empty,
                'qsize': None,
                'callback': self.publishStatusCb, 
                'callback_args': ()
            },
            'publish_info': {
                'namespace': self.namespace,
                'topic': 'publish_info',
                'msg': Empty,
                'qsize': None,
                'callback': self.publishInfoCb, 
                'callback_args': ()
            },
            'set_process_name': {
                'namespace': self.namespace,
                'topic': 'set_process_name',
                'msg': String,
                'qsize': None,
                'callback': self.setProcessNameCb, 
                'callback_args': ()
            },
            'set_navpose_frame': {
                'namespace': self.namespace,
                'topic': 'set_navpose_frame',
                'msg': String,
                'qsize': 1,
                'callback': self.setNavposeFrameCb, 
                'callback_args': ()
            },
            'sys_navposes_sub': {
                'namespace': self.base_namespace,
                'topic': 'navposes',
                'msg': NavPoses,
                'qsize': 1,
                'callback': self.navposesSysCb, 
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


        ####################################
        ## Initiation Complete
        self.rbx_info.connected = True
        self.status_msg.ready = True 

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(self.status_msg_pub_interval, self.statusPublishCb)
        


        self.publishInfo()
        self.publish_status()

        # Setup 3D Transform IF Class ####################
        # self.msg_if.pub_debug("Starting 3D Transform IF Initialization", log_name_list = self.log_name_list)
        # transform_ns = self.namespace

        # self.transform_if = Transform3DIF(namespace = transform_ns,
        #                 source_ref_description = self.tr_source_ref_description,
        #                 end_ref_description = self.tr_end_ref_description,
        #                 supports_updates = True,
        #                 log_name_list = self.log_name_list,
        #                     msg_if = self.msg_if
        #                 )

        # Setup Data IF Classes ####################
        self.msg_if.pub_info("Starting Image IF Initialization", log_name_list = self.log_name_list)
        self.image_if = ImageIF(namespace = self.node_namespace, log_name = 'image',
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )



        # Setup System IF Classes ####################
        self.msg_if.pub_info("Starting Settings IF Initialization", log_name_list = self.log_name_list)
        settings_ns = self.namespace

        self.SETTINGS_DICT = {
                    'capSettings': capSettings, 
                    'factorySettings': factorySettings,
                    'setSettingFunction': settingUpdateFunction, 
                    'getSettingsFunction': getSettingsFunction
                    
        }

        self.settings_if = SettingsIF(namespace = settings_ns,
                        settings_dict = self.SETTINGS_DICT,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )

        
        # Setup Save Data IF Class ####################
        self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
        self.data_products_list.append('navpose')
        factory_data_rates = {}
        for d in self.data_products_list:
            factory_data_rates[d] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "rbx",
            'add_node_name': True
            }

        sd_namespace = self.namespace
        self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                                factory_rate_dict = factory_data_rates,
                                factory_filename_dict = factory_filename_dict,
                                namespace = sd_namespace,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )

        ####################
        # # Setup NavPose IF Class
        # # NavPose for RBX is published by the NPXDeviceIF created below using the
        # # driver-supplied getNavPoseCb (matching device_if_npx / device_if_idx).
        # # The legacy ConnectNavPosesIF setup is retired (class removed from connect_data_if).
        # self.msg_if.pub_info("Starting NavPose IF Initialization")
        # np_namespace = self.namespace
        # self.navpose_if = ConnectNavPosesIF(namespace = np_namespace,
        #                             save_data_if = self.save_data_if,
        #                         log_name_list = self.log_name_list,
        #                         msg_if = self.msg_if)


        #####################
        # Update Status Message
        nepi_sdk.sleep(1)
        if self.settings_if is not None:
            self.status_msg.settings_topic = self.settings_if.get_namespace()
            self.msg_if.pub_info("Using settings namespace: " + str(self.status_msg.settings_topic))
        if self.save_data_if is not None:
            self.status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic))
        if self.navpose_if is not None:
            self.status_msg.navpose_topic = self.navpose_if.get_namespace()
            self.msg_if.pub_info("Using navpose namespace: " + str(self.status_msg.navpose_topic))

        self.initCb(do_updates = True)



        ##################################
        # Start Node Processes
        #nepi_sdk.start_timer_process(1, self._updaterCb, oneshot = True)
        # NavPose publishing is handled by the NPXDeviceIF below (driver-supplied
        # getNavPoseCb). The legacy navPoseUpdaterCb timer is retired - it drove
        # home-grown navpose methods that referenced undefined attributes.
        

        ##################################
        if self.getNavPoseCb is not None:
            self.msg_if.pub_warn("Starting NPX Device IF Initialization", log_name_list = self.log_name_list)
            self.npx_if = NPXDeviceIF(device_info,
                node_namespace = self.node_namespace,
                data_source_description = self.data_source_description,
                data_ref_description = self.data_ref_description,
                getNavPoseCb = self.getNavPoseCb,
                max_navpose_update_rate = self.navpose_update_rate,
                log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                )

        ####################################
        # The aggregated system 'navposes' (NavPoses) topic that navposesSysCb
        # consumes is not published in this build (the navpose mgr publishes
        # per-frame '.../navposes/<frame>/navpose' instead), so the current_*
        # control navpose was never refreshed and stayed at its [0,0,0] default.
        # That drove goto/takeoff/land/rtl to lat/lon 0. Bridge the device's own
        # navpose (getNavPoseCb, already the preferred source in
        # _updateCurrentNavpose) to the current_* attributes on a timer so the
        # control math uses the same live pose that the NPX navpose publishes.
        if self.getNavPoseCb is not None:
            nepi_sdk.start_timer_process(0.1, self._updateNavposeTimerCb)

        ####################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ####################################

             

    def initCb(self, do_updates = False):
      if self.node_if is not None:
        ########   ADD INIT FROM PARAMS
        pass
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
      if self.transform_if is not None:
          self.transform_if.reset()
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
      if self.transform_if is not None:
          self.transform_if.factory_reset()
      if do_updates == True:
        pass
      self.initCb(do_updates = True)



    def navposesSysCb(self,msg):
        # Build a per-frame navpose dict from the system NavPoses message and
        # cache the entry matching this device's configured navpose frame.
        # (Parse per-element with the working convert_navpose_msg2dict; the
        # aggregate convert_navposes_msg2dict helper is not used here.)
        navposes_dict = {}
        try:
            for np_msg in msg.navposes:
                np_dict = nepi_nav.convert_navpose_msg2dict(np_msg, self.log_name_list)
                if np_dict is not None:
                    navposes_dict[np_dict['navpose_frame']] = np_dict
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert navposes message: " + str(e), throttle_s = 5.0, log_name_list = self.log_name_list)
        navpose_frames = list(navposes_dict.keys())
        if self.navpose_frame in navpose_frames:
            self.sys_navpose_dict = navposes_dict[self.navpose_frame]
        elif len(navpose_frames) > 0:
            self.sys_navpose_dict = navposes_dict[navpose_frames[0]]
        else:
           self.sys_navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        # Bridge the latest navpose to the current_* attributes consumed by the
        # goto/setpoint accuracy calculations.
        self._updateCurrentNavpose()


    def _updateNavposeTimerCb(self, timer):
        # Periodic bridge from the device's own navpose to the current_* control
        # attributes (see setup note). The now-dead navposesSysCb used to be the
        # only caller of _updateCurrentNavpose.
        self._updateCurrentNavpose()

    def _updateCurrentNavpose(self):
        # Prefer the device's own navpose (populated directly from the driver via
        # getNavPoseCb) for control accuracy, falling back to the system navpose
        # cached for this device's frame.
        navpose_dict = None
        if self.getNavPoseCb is not None:
            try:
                navpose_dict = self.getNavPoseCb()
            except Exception as e:
                self.msg_if.pub_warn("getNavPoseCb failed: " + str(e), throttle_s = 5.0, log_name_list = self.log_name_list)
        if navpose_dict is None:
            navpose_dict = self.sys_navpose_dict
        if navpose_dict is None:
            return
        try:
            self.navpose_dict = navpose_dict
            self.current_heading_deg = navpose_dict['heading_deg']
            self.current_geoid_height_m = navpose_dict['geoid_height_meters']
            # Orientation: native ENU vector, plus NED conversion
            self.current_orientation_enu_degs = [navpose_dict['roll_deg'], navpose_dict['pitch_deg'], navpose_dict['yaw_deg']]
            ned_dict = nepi_nav.convert_navpose_enu2ned(copy.deepcopy(navpose_dict))
            self.current_orientation_ned_degs = [ned_dict['roll_deg'], ned_dict['pitch_deg'], ned_dict['yaw_deg']]
            # Position: native ENU vector, plus NED conversion
            self.current_position_enu_m = [navpose_dict['x_m'], navpose_dict['y_m'], navpose_dict['z_m']]
            self.current_position_ned_m = [ned_dict['x_m'], ned_dict['y_m'], ned_dict['z_m']]
            # Location: WGS84 geopoint, plus AMSL conversion
            self.current_location_wgs84_geo = [navpose_dict['latitude'], navpose_dict['longitude'], navpose_dict['altitude_m']]
            amsl_dict = nepi_nav.convert_navpose_wgs842amsl(copy.deepcopy(navpose_dict))
            self.current_location_amsl_geo = [amsl_dict['latitude'], amsl_dict['longitude'], amsl_dict['altitude_m']]
        except Exception as e:
            self.msg_if.pub_warn("Failed to update current navpose: " + str(e), throttle_s = 5.0, log_name_list = self.log_name_list)
            



    

    ##############################
    ### Update image source subscriber
    def imageSubscriberCb(self,img_msg):
        #Convert image from ros to cv2
        cv2_img = nepi_img.rosimg_to_cv2img(img_msg, "bgr8")
        self.cv2_img = cv2_img

    ##############################
    # RBX Settings Topic Callbacks

    # ToDo: Create a custom RBX status message
    ### Callback to set state
    def setStateCb(self,state_msg):
        self.msg_if.pub_info("Received set state message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(state_msg)
        state_val = state_msg.data
        self.setState(state_val)
        

    ### Function to set state
    def setState(self,new_state_ind):
        if new_state_ind < 0 or new_state_ind > (len(self.states)-1):
            self.update_error_msg("No matching rbx state found")
        else:
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.status_msg.process_current = self.states[new_state_ind]
            self.rbx_state_last = self.rbx_info.state
            self.msg_if.pub_info("Waiting for rbx state " + self.states[new_state_ind] + " to set", log_name_list = self.log_name_list)
            self.setStateIndFunction(new_state_ind)
            time.sleep(1)
            self.msg_if.pub_info("Current rbx state is " + self.states[self.getStateIndFunction()])
            self.status_msg.process_last = self.states[new_state_ind]
            self.status_msg.process_current = "None"
            str_val = self.states[new_state_ind]
            self.last_cmd_string = "set_rbx_state(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
        self.publishInfo()
        

    ### Callback to set mode
    def setModeCb(self,mode_msg):
        self.msg_if.pub_info("Received set mode message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(mode_msg)
        mode_val = mode_msg.data
        self.setMode(mode_val)

    ### Function to set mode
    def setMode(self,new_mode_ind):
        if new_mode_ind < 0 or new_mode_ind > (len(self.modes)-1):
            self.update_error_msg("No matching rbx mode found")
        else:
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.status_msg.process_current = self.modes[new_mode_ind]
            self.msg_if.pub_info("Setting rbx mode to : " + self.modes[new_mode_ind])
            self.setModeIndFunction(new_mode_ind)
            self.msg_if.pub_info("Current rbx mode is " + self.modes[self.getModeIndFunction()])
            self.status_msg.process_last = self.modes[new_mode_ind]
            self.status_msg.process_current = "None"
            str_val = self.modes[new_mode_ind]
            self.last_cmd_string = "set_rbx_mode(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
        self.publishInfo()

    ### Callback to execute action
    def setupActionCb(self,action_msg):
        self.msg_if.pub_info("Received setup action message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(action_msg)
        action_ind = action_msg.data
        if self.setSetupActionIndFunction is not None:
            if action_ind < 0 or action_ind > (len(self.setup_actions)-1):
                self.update_error_msg("No matching rbx action found")
            else:
                if self.status_msg.ready is False:
                    self.update_error_msg("Another Command Process is Active")
                    self.update_error_msg("Ignoring this Request")
                else:
                    self.status_msg.process_current = self.setup_actions[action_ind]
                    self.status_msg.ready = False
                    self.rbx_cmd_success_current = False
                    self.msg_if.pub_info("Starting action: " + self.setup_actions[action_ind])
                    success = self.setSetupActionIndFunction(action_ind)
                    self.rbx_cmd_success_current = success
                    if success:
                      self.msg_if.pub_info("Finished action: " + self.setup_actions[action_ind])
                    else:
                      self.msg_if.pub_info("Action: " + self.setup_actions[action_ind] + " Failed to complete", log_name_list = self.log_name_list)
                    self.status_msg.process_last = self.setup_actions[action_ind]
                    self.status_msg.process_current = "None"
                    self.status_msg.cmd_success = self.rbx_cmd_success_current
                    time.sleep(0.5)
                    self.status_msg.ready = True

                    str_val = self.setup_actions[action_ind]
                    self.last_cmd_string = "setup_rbx_action(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
                    self.publishInfo()
        else:
            self.update_error_msg("Ignoring Setup Action command, no Set Action Function")


    ### Callback to start rbx set goto goals process
    def setErrorBoundsCb(self,error_bounds_msg):
        self.msg_if.pub_info("Received set goals message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(error_bounds_msg)
        self.node_if.set_param('max_error_m', error_bounds_msg.max_distance_error_m)
        self.node_if.set_param('max_error_deg', error_bounds_msg.max_rotation_error_deg)
        self.node_if.set_param('stabilized_sec', error_bounds_msg.min_stabilize_time_s)
        self.rbx_info.error_bounds = error_bounds_msg
        self.publishInfo()

    ### Callback to set cmd timeout
    def setCmdTimeoutCb(self,cmd_timeout_msg):
        self.msg_if.pub_info("Received set timeout message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(cmd_timeout_msg)
        self.node_if.set_param('cmd_timeout', cmd_timeout_msg.data)
        self.rbx_info.cmd_timeout = cmd_timeout_msg.data 
        self.publishInfo()


    ### Callback to image topic source
    def setImageTopicCb(self,set_image_topic_msg):
        self.msg_if.pub_info("Received set image topic message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(set_image_topic_msg)
        self.node_if.set_param('image_source', set_image_topic_msg.data)
        self.publishInfo()


    ### Callback to add overlay to image topic source
    def enableImageOverlayCb(self,enable_msg):
        self.msg_if.pub_info("Received enable image overlay message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(enable_msg)
        self.node_if.set_param('image_status_overlay', enable_msg.data)
        self.publishInfo()

    ### Callback to set current process name
    def setProcessNameCb(self,set_process_name_msg):
        self.msg_if.pub_info("Received set process name message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(set_process_name_msg)
        self.status_msg.process_current = (set_process_name_msg.data)

    def setNavposeFrameCb(self,msg):
        self.msg_if.pub_info("Recived set navpose_frame message: " + str(msg))
        self.navpose_frame = msg.data
        self.publish_status(do_updates=False) # Updated inline here 
        self.node_if.set_param('navpose_frame', self.navpose_frame)
 
    

    ##############################
    # RBX Control Topic Callbacks

   ### Callback to set manual motor control ratio
    def setMotorControlCb(self,motor_msg):
        self.msg_if.pub_info("Received set motor control ratio message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(motor_msg)
        if self.setMotorControl is not None:
          self.setMotorControl(motor_msg)

    ### Function to set motor control
    def setMotorControl(self,new_motor_ctrl):
        if self.manualControlsReadyFunction() is True and self.setMotorControlRatio is not None:
            m_ind = new_motor_ctrl.motor_ind
            m_sr = new_motor_ctrl.speed_ratio
            m_len = len(self.getMotorControlRatios())
            if m_ind > (m_len -1):
                self.update_error_msg("New Motor Control Ind " + str(m_ind) + " is out of range")
            elif m_sr < 0 or m_sr > 1:
                self.update_error_msg("New Motor Control Speed Ratio " + str(m_sr) + " is out of range")
            elif self.setMotorControlRatio is not None:
                self.setMotorControlRatio(m_ind,m_sr)
        else:
            self.update_error_msg("Ignoring Set Motor Control msg, Manual Controls not Ready")     


 


    ### Callback to set home
    def setHomeCb(self,geo_msg):
        self.msg_if.pub_info("Received set home message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(geo_msg)
        new_home_loc = [geo_msg.latitude,geo_msg.longitude,geo_msg.altitude]
        for i, loc in enumerate(new_home_loc):
          if loc == -999:
            new_home_loc[i] = self.current_location_wgs84_geo[i]
        if self.setHomeFunction is not None:
            new_home_geo = GeoPoint()
            new_home_geo.latitude = new_home_loc[0]
            new_home_geo.longitude = new_home_loc[1]
            new_home_geo.altitude = new_home_loc[2]
            self.setHomeFunction(new_home_geo)


    ### Callback to set home current
    def setHomeCurrentCb(self,empty_msg):
        self.msg_if.pub_info("Received set home current message", log_name_list = self.log_name_list)
        if self.setHomeFunction is not None:
            self.setHomeFunction(self.current_location_wgs84_geo)

          


    ### Callback to start rbx go home
    def goHomeCb(self,home_msg):
        self.msg_if.pub_info("Received go home message", log_name_list = self.log_name_list)
        if self.goHomeFunction is not None:
            self.status_msg.process_current = "Go Home"
            self.rbx_cmd_success_current = False
            self.status_msg.ready = False
            self.update_prev_errors()
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.rbx_cmd_success_current = self.goHomeFunction()
            self.status_msg.process_last = "Go Home"
            self.status_msg.process_current = "None"
            self.status_msg.cmd_success = self.rbx_cmd_success_current
            time.sleep(0.5)
            self.status_msg.ready = True
            self.last_cmd_string = "go_rbx_home(self,timeout_sec = " + str(self.rbx_info.cmd_timeout)
            self.publishInfo()

    ### Callback to start rbx stop
    def goStopCb(self,stop_msg):
        self.msg_if.pub_info("Received go stop message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(stop_msg)
        time.sleep(1)
        if self.goStopFunction is not None:
            self.status_msg.process_current = "Stop"
            self.rbx_cmd_success_current = False
            self.status_msg.ready = False
            self.update_prev_errors()
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.rbx_cmd_success_current = self.goStopFunction()
            self.status_msg.process_last = "Stop"
            self.status_msg.process_current = "None"
            self.status_msg.cmd_success = self.rbx_cmd_success_current
            time.sleep(0.5)
            self.status_msg.ready = True
            self.last_cmd_string = "go_rbx_stop(self,timeout_sec = " + str(self.rbx_info.cmd_timeout)
            self.publishInfo()

 
  ### Callback to execute action
    def goActionCb(self,action_msg):
        self.msg_if.pub_info("Received go action message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(action_msg)
        action_ind = action_msg.data
        if self.setGoActionIndFunction is not None:
            if action_ind < 0 or action_ind > (len(self.go_actions)-1):
                self.update_error_msg("No matching rbx action found")
            else:
                if self.status_msg.ready is False:
                    self.update_error_msg("Another GoTo Command Process is Active")
                    self.update_error_msg("Ignoring this Request")
                else:
                    self.status_msg.process_current = self.go_actions[action_ind]
                    self.status_msg.ready = False
                    self.rbx_cmd_success_current = False
                    self.msg_if.pub_info("Starting action: " + self.go_actions[action_ind])
                    success = self.setGoActionIndFunction(action_ind)
                    self.rbx_cmd_success_current = success
                    if success:
                      self.msg_if.pub_info("Finished action: " + self.go_actions[action_ind])
                    else:
                      self.msg_if.pub_info("Action: " + self.go_actions[action_ind] + " Failed to complete", log_name_list = self.log_name_list)
                    self.status_msg.process_last = self.go_actions[action_ind]
                    self.status_msg.process_current = "None"
                    self.status_msg.cmd_success = self.rbx_cmd_success_current
                    time.sleep(0.5)
                    self.status_msg.ready = True

                    str_val = self.go_actions[action_ind]
                    self.last_cmd_string = "go_rbx_action(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
                    self.publishInfo()
        else:
            self.update_error_msg("Ignoring Go Action command, no Set Action Function")


    ### Callback to start rbx goto pose process
    def gotoPoseCb(self,pose_cmd_msg):
        self.msg_if.pub_info("Recieved GoTo Pose Message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(pose_cmd_msg)
        time.sleep(1)
        if self.autonomousControlsReadyFunction() is True:
            setpoint_data=[pose_cmd_msg.roll_deg,pose_cmd_msg.pitch_deg,pose_cmd_msg.yaw_deg]
            if self.status_msg.ready is False:
                self.update_error_msg("Ignoring GoTo POSE Request, Another GoTo Command Process is Active")
            else:
                self.status_msg.process_current = "GoTo Pose"
                self.status_msg.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.setpoint_attitude_ned(setpoint_data)
                self.status_msg.process_last = "GoTo Pose"
                self.status_msg.process_current = "None"
                self.status_msg.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.status_msg.ready = True

                str_val = str(setpoint_data)
                self.last_cmd_string = "goto_rbx_pose(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
                self.publishInfo()
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")



    ### Callback to start rbx goto position process
    def gotoPositionCb(self,position_cmd_msg):
        self.msg_if.pub_info("Recieved GoTo Position Command Message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(position_cmd_msg)
        time.sleep(1)
        if self.autonomousControlsReadyFunction() is True:
            setpoint_data=[position_cmd_msg.x_meters,position_cmd_msg.y_meters,position_cmd_msg.z_meters,position_cmd_msg.yaw_deg]
            if self.status_msg.ready is False:
                self.update_error_msg("Ignoring GoTo Position Request, Another GoTo Command Process is Active")
            else:
                self.status_msg.process_current = "GoTo Position"
                self.status_msg.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.setpoint_position_local_body(setpoint_data)
                self.status_msg.process_last = "GoTo Position"
                self.status_msg.process_current = "None"
                self.status_msg.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.status_msg.ready = True
                
                str_val = str(setpoint_data)
                self.last_cmd_string = "goto_rbx_position(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
                self.publishInfo()
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to start rbx goto location subscriber
    def gotoLocationCb(self,location_cmd_msg):
        self.msg_if.pub_info("Recieved GoTo Location Message", log_name_list = self.log_name_list)
        self.msg_if.pub_info(location_cmd_msg)
        if self.autonomousControlsReadyFunction() is True:
            setpoint_data=[location_cmd_msg.lat,location_cmd_msg.long,location_cmd_msg.altitude_meters,location_cmd_msg.yaw_deg]
            if self.status_msg.ready is False:
                self.update_error_msg("Ignoring GoTo Location Request, Another GoTo Command Process is Active")
            else:
                self.status_msg.process_current = "GoTo Location"
                self.status_msg.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.setpoint_location_global_wgs84(setpoint_data)
                self.status_msg.process_last = "GoTo Location"
                self.status_msg.process_current = "None"
                self.status_msg.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.status_msg.ready = True

                str_val = str(setpoint_data)
                self.last_cmd_string = "goto_rbx_location(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
                self.publishInfo()
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")



    def ApplyConfigUpdates(self):
        if self.setHomeFunction is not None:
          home_location = self.node_if.get_param('home_location')
          geo_home = GeoPoint()
          geo_home.latitude = home_location[0]
          geo_home.longitude = home_location[1]
          geo_home.altitude = home_location[2]
          self.setHomeFunction(geo_home)

  
    def setCurrentAsDefault(self):
        if self.settings_if is not None:
          self.settings_if.initialize_settings(do_updates = False)
        pass # We only use the param server, no member variables to apply to param server
   
    ### Info callback
    def info_query_callback(self, _):
        """Handles a DeviceInfoQuery service request.

        Args:
            _: Unused service request object.

        Returns:
            DeviceInfoQueryResponse: The pre-populated device info report
                containing name, path, node, serial number, and version
                strings.
        """
        return self.info_report

    def capabilities_query_callback(self, _):
        """Handles an RBXCapabilitiesQuery service request.

        Args:
            _: Unused service request object.

        Returns:
            RBXCapabilitiesQueryResponse: The pre-populated capabilities
                report describing supported control axes, states, modes,
                actions, and data products.
        """
        return self.caps_report
    

           

  # RBX Status Topic Publishers

    def publishInfoCb(self, msg):
        self.publishInfo()
    
    
    def publishInfo(self):
        self.rbx_info.device_name = self.device_name
        error_bounds = ErrorBounds()
        error_bounds.max_distance_error_m = self.node_if.get_param('max_error_m')
        error_bounds.max_rotation_error_deg = self.node_if.get_param('max_error_deg')
        error_bounds.min_stabilize_time_s = self.node_if.get_param('stabilized_sec')
        self.rbx_info.error_bounds = error_bounds
        self.rbx_info.cmd_timeout = self.node_if.get_param('cmd_timeout')
        self.rbx_info.image_status_overlay = self.node_if.get_param('image_status_overlay') 
        self.rbx_info.state = self.getStateIndFunction()
        self.rbx_info.mode = self.getModeIndFunction()
        if self.getHomeFunction is not None:
          home_geo = self.getHomeFunction()
          home_location = [home_geo.latitude,home_geo.longitude,home_geo.altitude]   
        else: # No Home Information
          home_location = [-999,-999,-999]    
        self.rbx_info.home_lat = home_location[0]
        self.rbx_info.home_long = home_location[1]
        self.rbx_info.home_alt = home_location[2]


        if not nepi_sdk.is_shutdown():
            #self.msg_if.pub_info(self.rbx_info)
            self.node_if.publish_pub('rbx_info_pub',self.rbx_info)


 

    #######################
    # RBX IF Methods

    ### Function to set and check setpoint attitude NED command
    ###################################################
    # Input is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
    # Converted to ENU befor sending message
    ###################################################
    def setpoint_attitude_ned(self,setpoint_attitude):
      """Commands the robot to a target attitude in the NED reference frame.

      Converts the NED input to ENU before sending a pose setpoint command,
      then blocks until the attitude goal is reached within error bounds or
      the command times out.

      Args:
          setpoint_attitude (list): Target attitude as
              [ROLL_NED_DEG, PITCH_NED_DEG, YAW_NED_DEG]. Use -999 for any
              axis to hold the current value.

      Returns:
          bool: True if the attitude setpoint was reached within bounds and
              timeout, False otherwise.
      """
      # setpoint_attitude is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
      # Use value -999 to use current value
      cmd_success = True
      timeout_sec = self.rbx_info.cmd_timeout
      self.update_prev_errors()
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.msg_if.pub_info("", log_name_list = self.log_name_list)
      self.msg_if.pub_info("************************", log_name_list = self.log_name_list)
      self.msg_if.pub_info("Starting Setpoint Attitude Process", log_name_list = self.log_name_list)
      ##############################################
      # Capture Current NavPoses Data
      ##############################################
      start_orientation_ned_degs=list(self.current_orientation_ned_degs)
      self.msg_if.pub_info("Attitude Current NED Roll, Pitch, Yaw in Degrees", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % start_orientation_ned_degs[0],"%.2f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
      ##############################################
      # Condition Inputs
      ##############################################
      input_attitude_ned_degs = list(setpoint_attitude)
      #self.msg_if.pub_info("Attitude Input NED Roll, Pitch, Yaw in Degrees", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(["%.2f" % input_attitude_ned_degs[0],"%.2f" % input_attitude_ned_degs[1],"%.2f" % input_attitude_ned_degs[2]])
      # Set new attitude in degs NED
      new_attitude_ned_degs=list(start_orientation_ned_degs) # Initialize with start values
      for ind in range(3): # Overwrite current with new if set and valid
        if setpoint_attitude[ind] != -999:
          new_attitude_ned_degs[ind]=setpoint_attitude[ind]
        # Condition to +-180 deg
        if new_attitude_ned_degs[ind] > 180:
          new_attitude_ned_degs[ind] = new_attitude_ned_degs[ind] - 360
      #self.msg_if.pub_info("Attitude Input Conditioned NED Roll, Pitch, Yaw in Degrees", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
      ##############################################
      # Convert NED attitude to Pose
      ##############################################
      # Convert to ROS ENU attitude degs and create ENU quaternion setpoint attitude goal
      yaw_enu_deg = nepi_nav.convert_yaw_ned2enu(new_attitude_ned_degs[2])
      new_attitude_enu_degs = [new_attitude_ned_degs[0],new_attitude_ned_degs[1],yaw_enu_deg]
      self.msg_if.pub_info("Attitude Goal ENU Roll, Pitch, Yaw in Degrees", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % new_attitude_enu_degs[0],"%.2f" % new_attitude_enu_degs[1],"%.2f" % new_attitude_enu_degs[2]])
      ##############################################
      ## Send Setpoint Message and Check for Success
      ##############################################
      self.msg_if.pub_info("Sending Setpoint Attitude Command", log_name_list = self.log_name_list)
      self.gotoPoseFunction(new_attitude_enu_degs)
      self.msg_if.pub_info("Waiting for Attitude Setpoint to complete", log_name_list = self.log_name_list)
      setpoint_attitude_reached = False
      stabilize_timer=0
      timeout_timer = 0 # Initialize timeout timer
      attitude_errors = [] # Initialize running list of errors
      time2sleep = 0.1
      while setpoint_attitude_reached is False and not nepi_sdk.is_shutdown():  # Wait for setpoint goal to be set
        if self.checkStopFunction() is True:
            self.msg_if.pub_info("Setpoint Attitude received Stop Command", log_name_list = self.log_name_list)
            # Hold the current attitude on stop (same source used below for error calc).
            new_attitude_ned_degs = list(self.current_orientation_ned_degs)
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment message timer
        timeout_timer = timeout_timer+time2sleep
        # Calculate setpoint attitude errors
        cur_attitude_ned_degs = [self.current_orientation_ned_degs[0],self.current_orientation_ned_degs[1],self.current_orientation_ned_degs[2]]
        attitude_errors_degs = np.array(new_attitude_ned_degs) - np.array(cur_attitude_ned_degs)
        for ind in range(3):
          if input_attitude_ned_degs[ind] == -999.0: # Ignore error check if set to current
            attitude_errors_degs[ind]=0.0
        max_attutude_error_deg = max(abs(attitude_errors_degs))
        # Check for setpoint position local point goal
        if  setpoint_attitude_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
            max_attitude_errors = max(attitude_errors) # Get max from error window
            attitude_errors = [max_attutude_error_deg] # reset running list of errors
            if max_attitude_errors < self.rbx_info.error_bounds.max_rotation_error_deg:
              self.msg_if.pub_info("Attitude Setpoint Reached", log_name_list = self.log_name_list)
              setpoint_attitude_reached = True
          else:
            attitude_errors.append(max_attutude_error_deg) # append last
        # Reset timer if past
        if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
          stabilize_timer=0 # Reset timer
        self.update_current_errors( [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]]  )
      if cmd_success:
        self.msg_if.pub_info("************************", log_name_list = self.log_name_list)
        self.msg_if.pub_info("Setpoint Reached", log_name_list = self.log_name_list)
      self.update_current_errors( [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]]  )
      return cmd_success
      



    ### Function to set and check setpoint position local body command
    ###################################################
    # Input is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
    # Converted to Local ENU Frame before sending
    # Local Body Position Setpoint Function use these body relative x,y,z,yaw conventions
    # x+ axis is forward
    # y+ axis is left
    # z+ axis is up
    # Only yaw orientation updated
    # yaw+ counter clockwise from x axis (0 degrees faces x+ and rotates using right hand rule around z+ axis down)
    #####################################################
    def setpoint_position_local_body(self,setpoint_position):
      """Commands the robot to a target position in the local body frame.

      Converts the body-relative input position and yaw to the ENU frame,
      sends a position setpoint, then blocks until the position and yaw goals
      are reached within error bounds or the command times out.

      Body frame conventions: x+ forward, y+ left, z+ up, yaw+ counter-clockwise
      from x axis.

      Args:
          setpoint_position (list): Target position and yaw as
              [X_BODY_M, Y_BODY_M, Z_BODY_M, YAW_BODY_DEG]. Use -999 for any
              element to hold the current value.

      Returns:
          bool: True if both position and yaw setpoints were reached within
              bounds and timeout, False otherwise.
      """
      # setpoint_position is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
      # use value 0 for no change
      cmd_success = True
      timeout_sec = self.rbx_info.cmd_timeout
      self.update_prev_errors()
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.msg_if.pub_info("************************", log_name_list = self.log_name_list)
      self.msg_if.pub_info("Starting Setpoint Position Local Process", log_name_list = self.log_name_list)
      ##############################################
      # Capture Current NavPose Data
      ##############################################
      start_geopoint_wgs84 = list(self.current_location_wgs84_geo)
      #self.msg_if.pub_info("Start Location WSG84 geopoint", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(" Lat, Long, Alt", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(["%.2f" % start_geopoint_wgs84[0],"%.2f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
      start_position_enu_m = list(self.current_position_enu_m)
      self.msg_if.pub_info("Start Position ENU degs", log_name_list = self.log_name_list)
      self.msg_if.pub_info(" X, Y, Z", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % start_position_enu_m[0],"%.2f" % start_position_enu_m[1],"%.2f" % start_position_enu_m[2]])   
      start_orientation_enu_degs=list(self.current_orientation_enu_degs)
      self.msg_if.pub_info("Start Orientation ENU degs", log_name_list = self.log_name_list)
      self.msg_if.pub_info(" Roll, Pitch, Yaw", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % start_orientation_enu_degs[0],"%.2f" % start_orientation_enu_degs[1],"%.2f" % start_orientation_enu_degs[2]])

      start_yaw_enu_deg = start_orientation_enu_degs[2]
      self.msg_if.pub_info("Start Yaw ENU degs", log_name_list = self.log_name_list)
      self.msg_if.pub_info(start_yaw_enu_deg) 
      start_heading_deg=self.current_heading_deg
      #self.msg_if.pub_info("Start Heading degs", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(start_heading_deg)   
      ##############################################
      # Condition Body Input Data
      ##############################################
      # Condition Point Input
      input_point_body_m=setpoint_position[0:3]
      self.msg_if.pub_info("Input Postion Body  X, Y, Z in Meters", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % input_point_body_m[0],"%.2f" % input_point_body_m[1],"%.2f" % input_point_body_m[2]])
      new_point_body_m=list(input_point_body_m) # No conditioning required
      #self.msg_if.pub_info("Point Conditioned Body Meters", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(" X, Y, Z", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(["%.2f" % new_point_body_m[0],"%.2f" % new_point_body_m[1],"%.2f" % new_point_body_m[2]])
      # Condition Orienation Input
      input_yaw_body_deg = setpoint_position[3]
      self.msg_if.pub_info("Yaw Input Body Degrees", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % input_yaw_body_deg])   
      ##############################################
      # Convert Body Data to ENU Data
      ##############################################
      # Set new yaw orientation in ENU degrees
      offset_enu_m = nepi_nav.convert_point_body2enu(new_point_body_m,start_yaw_enu_deg)
      self.msg_if.pub_info("Point Goal Offsets ENU Meters", log_name_list = self.log_name_list)
      self.msg_if.pub_info(" X, Y, Z", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % offset_enu_m[0],"%.2f" % offset_enu_m[1],"%.2f" % offset_enu_m[2]])
      new_x_enu_m = start_position_enu_m[0] + offset_enu_m[0]
      new_y_enu_m = start_position_enu_m[1] + offset_enu_m[1]
      new_z_enu_m = start_position_enu_m[2] + offset_enu_m[2]
      new_position_enu_m = [new_x_enu_m,new_y_enu_m,new_z_enu_m]
      self.msg_if.pub_info("Point Goal ENU X, Y, Z in Meters", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % new_position_enu_m[0],"%.2f" % new_position_enu_m[1],"%.2f" % new_position_enu_m[2]])

      new_yaw_enu_deg = start_yaw_enu_deg + input_yaw_body_deg
        # Condition to +-180 deg
      if new_yaw_enu_deg > 180:
        new_yaw_enu_deg = new_yaw_enu_deg - 360
      elif new_yaw_enu_deg < -180:
        new_yaw_enu_deg = 360 + new_yaw_enu_deg
      self.msg_if.pub_info("Yaw Goal ENU Degrees", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % new_yaw_enu_deg])
      ##############################################
      # Create Point and Pose Data
      ##############################################
      # New Point ENU in meters
      new_point_enu_m=Point()
      new_point_enu_m.x = offset_enu_m[0]
      new_point_enu_m.y = offset_enu_m[1]
      new_point_enu_m.z = offset_enu_m[2]
      self.msg_if.pub_info("Position Goal ENU X, Y, Z in Meters", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])

      new_orientation_enu_deg = [start_orientation_enu_degs[0],start_orientation_enu_degs[1],new_yaw_enu_deg]
      self.msg_if.pub_info("Orienation Goal ENU  Roll, Pitch, Yaw in Degrees", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % new_orientation_enu_deg[0],"%.2f" % new_orientation_enu_deg[1],"%.2f" % new_orientation_enu_deg[2]])
      ##############################################
      ## Send Message and Check for Setpoint Success
      ##############################################
      self.msg_if.pub_info("Sending Setpoint Position Local Command", log_name_list = self.log_name_list)
      self.gotoPositionFunction(new_point_enu_m,new_orientation_enu_deg)
      self.msg_if.pub_info("Waiting for Position Setpoint to complete", log_name_list = self.log_name_list)
      setpoint_position_local_point_reached = False
      setpoint_position_local_yaw_reached = False
      stabilize_timer=0
      point_errors = [] # Initialize running list of errors
      yaw_errors = [] # Initialize running list of errors
      timeout_timer = 0 # Initialize timeout timer
      time2sleep = 0.1
      while setpoint_position_local_point_reached is False or setpoint_position_local_yaw_reached is False and not nepi_sdk.is_shutdown():  # Wait for setpoint goal to be set
        if self.checkStopFunction() is True:
            self.msg_if.pub_info("Setpoint Position received Stop Command", log_name_list = self.log_name_list)
            new_position_enu_m = copy.deepcopy(self.current_position_enu_m)
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment message timer
        timeout_timer = timeout_timer+time2sleep
        # Calculate setpoint position enu errors    
        point_body_errors_m = [0,0,0] # initialize for later
        point_enu_errors_m = np.array(self.current_position_enu_m) - np.array(new_position_enu_m)
        for ind in range(3):
          if input_point_body_m == -999: # Ignore error check if set to current
            point_enu_errors_m[ind] = 0
        max_point_enu_errors_m = np.max(np.abs(point_enu_errors_m))
        # Calculate setpoint yaw enu error
        if input_yaw_body_deg == -999: # Ignore error check if set to current
          setpoint_position_local_yaw_reached = True
          max_yaw_enu_error_deg = 0
        else:
          cur_yaw_enu_deg = self.current_orientation_enu_degs[2]
          yaw_enu_error_deg =  cur_yaw_enu_deg - new_yaw_enu_deg
          max_yaw_enu_error_deg = abs(yaw_enu_error_deg)
        # Check for setpoint position local point goal
        if  setpoint_position_local_point_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
            max_point_errors = max(point_errors) # Get max from error window
            point_errors = [max_point_enu_errors_m] # reset running list of errors
            if max_point_errors < self.rbx_info.error_bounds.max_distance_error_m:
              self.msg_if.pub_info("Position Setpoint Reached", log_name_list = self.log_name_list)
              setpoint_position_local_point_reached = True
          else:
            point_errors.append(max_point_enu_errors_m) # append last
        # Check for setpoint position yaw point goal
        if  setpoint_position_local_yaw_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
            max_yaw_errors = max(yaw_errors) # Get max from error window
            yaw_errors = [max_yaw_enu_error_deg] # reset running list of errors
            if max_yaw_errors < self.rbx_info.error_bounds.max_rotation_error_deg:
              self.msg_if.pub_info("Yaw Setpoint Reached", log_name_list = self.log_name_list)
              setpoint_position_local_yaw_reached = True
          else:
            yaw_errors.append(max_yaw_enu_error_deg) # append last
        # Reset timer if past
        if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
          stabilize_timer=0 # Reset timer
        point_body_errors_m = nepi_nav.convert_point_enu2body(point_enu_errors_m,start_yaw_enu_deg)
        self.update_current_errors(  [point_body_errors_m[1],point_body_errors_m[0],point_body_errors_m[2],0,0,0,max_yaw_enu_error_deg] )
      if cmd_success:
        self.msg_if.pub_info("Setpoint Reached", log_name_list = self.log_name_list)
        self.msg_if.pub_info("************************", log_name_list = self.log_name_list)
      point_body_errors_m = nepi_nav.convert_point_enu2body(point_enu_errors_m,start_yaw_enu_deg)
      self.update_current_errors(  [point_body_errors_m[1],point_body_errors_m[0],point_body_errors_m[2],0,0,0,max_yaw_enu_error_deg] )
      return cmd_success



    ### Function to set and check setpoint location global geopoint and yaw command
    ###################################################
    # Input is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
    # Converted to AMSL Altitude and ENU Yaw berore sending
    # Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
    # Yaw is specified in NED frame degrees 0-360 or +-180 
    #####################################################
    def setpoint_location_global_wgs84(self,setpoint_location):
      """Commands the robot to a global WGS84 geographic position.

      Converts WGS84 altitude to AMSL, converts NED yaw to ENU, sends a
      location setpoint, then blocks until the geopoint and yaw goals are
      reached within error bounds or the command times out.

      Args:
          setpoint_location (list): Target location as
              [LAT, LONG, ALT_WGS84_M, YAW_NED_DEG]. Latitude and longitude
              in decimal degrees, altitude in meters above WGS84 ellipsoid.
              Use -999 for any element to hold the current value.

      Returns:
          bool: True if both location and yaw setpoints were reached within
              bounds and timeout, False otherwise.
      """
      # setpoint_location is [LAT, LONG, ALT_WGS84, YEW_NED_DEGREES 0-360 or +-180]
      # Use value -999 to use current value
      cmd_success = True
      timeout_sec = self.rbx_info.cmd_timeout
      self.update_prev_errors()
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.msg_if.pub_info("************************", log_name_list = self.log_name_list)
      self.msg_if.pub_info("Starting Setpoint Location Global Process", log_name_list = self.log_name_list)
      ##############################################
      # Capture Current NavPose Data
      ##############################################
      start_geopoint_wgs84 = list(self.current_location_wgs84_geo)  
      #self.msg_if.pub_info("Start Location WSG84 geopoint", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(" Lat, Long, Alt", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(["%.6f" % start_geopoint_wgs84[0],"%.6f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
      start_orientation_ned_degs=list(self.current_orientation_ned_degs)
      #self.msg_if.pub_info("Start Orientation NED degs", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(" Roll, Pitch, Yaw", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(["%.6f" % start_orientation_ned_degs[0],"%.6f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
      start_yaw_ned_deg = start_orientation_ned_degs[2]
      if start_yaw_ned_deg < 0:
        start_yaw_ned_deg = start_yaw_ned_deg + 360
      #self.msg_if.pub_info("Start Yaw NED degs 0-360", log_name_list = self.log_name_list)
      self.msg_if.pub_info(start_yaw_ned_deg) 
      start_heading_deg=self.current_heading_deg
      self.msg_if.pub_info("Start Heading degs", log_name_list = self.log_name_list)
      self.msg_if.pub_info(start_heading_deg)
      start_geoid_height_m = self.current_geoid_height_m
      ##############################################
      # Condition NED Input Data
      ##############################################
      # Condition Location Input
      input_geopoint_wgs84 = list(setpoint_location[0:3])
      #self.msg_if.pub_info("Location Input Global Geo", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(" Lat, Long, Alt_WGS84", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(["%.8f" % input_geopoint_wgs84[0],"%.8f" % input_geopoint_wgs84[1],"%.2f" % input_geopoint_wgs84[2]])
      new_geopoint_wgs84=list(start_geopoint_wgs84) # Initialize with start
      for ind in range(3): # Overwrite current with new if set and valid
        if input_geopoint_wgs84[ind] != -999:
          new_geopoint_wgs84[ind]=input_geopoint_wgs84[ind]
      #self.msg_if.pub_info("Location Input Conditioned Global Geo", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(" Lat, Long, Alt_WGS84", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(["%.8f" % new_geopoint_wgs84[0],"%.8f" % new_geopoint_wgs84[1],"%.2f" % new_geopoint_wgs84[2]])
      # Condition Yaw Input
      input_yaw_ned_deg = setpoint_location[3]
      #self.msg_if.pub_info("Yaw Input NED Degrees", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(["%.2f" % input_yaw_ned_deg])
      new_yaw_ned_deg = start_yaw_ned_deg # Initialize to current
      if input_yaw_ned_deg != -999: # Replace if not -999
        new_yaw_ned_deg = input_yaw_ned_deg
      # Condition to 0-360 degs
      if new_yaw_ned_deg < 0:
        new_yaw_ned_deg = new_yaw_ned_deg + 360
      #self.msg_if.pub_info("Yaw Input Conditioned NED Degrees 0-360", log_name_list = self.log_name_list)
      #self.msg_if.pub_info(["%.2f" % new_yaw_ned_deg])      
      ##############################################
      # Create Global AMSL Location and NED Orienation Setpoint Values
      ##############################################
      # New Global location ENU in meters
      new_geopoint_amsl=GeoPoint()
      new_geopoint_amsl.latitude = new_geopoint_wgs84[0]
      new_geopoint_amsl.longitude = new_geopoint_wgs84[1]
      new_geopoint_amsl.altitude = new_geopoint_wgs84[2] + start_geoid_height_m
      self.msg_if.pub_info("Location Goal Lat, Long, Alt_AMSL", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.8f" % new_geopoint_amsl.latitude,"%.8f" % new_geopoint_amsl.longitude,"%.2f" % new_geopoint_amsl.altitude])
      # New Local Orienation NED in degs  
      new_orientation_ned_deg = [start_orientation_ned_degs[0],start_orientation_ned_degs[1],new_yaw_ned_deg]
      self.msg_if.pub_info("Orienation Goal NED  Roll, Pitch, Yaw in Degrees", log_name_list = self.log_name_list)
      self.msg_if.pub_info(["%.2f" % new_orientation_ned_deg[0],"%.2f" % new_orientation_ned_deg[1],"%.2f" % new_orientation_ned_deg[2]])
      ##############################################
      ## Send Message and Check for Setpoint Success
      ##############################################
      self.msg_if.pub_info("Sending MAVLINK Setpoint Position Local Command", log_name_list = self.log_name_list)
      self.gotoLocationFunction(new_geopoint_amsl,new_orientation_ned_deg)
      self.msg_if.pub_info(" checking for Setpoint Reached", log_name_list = self.log_name_list)
      setpoint_location_global_geopoint_reached = False
      setpoint_location_global_yaw_reached = False 
      self.msg_if.pub_info("Waiting for Position Local Setpoint to complete", log_name_list = self.log_name_list)
      stabilize_timer=0
      geopoint_errors = [] # Initialize running list of errors
      yaw_errors = [] # Initialize running list of errors
      timeout_timer = 0 # Initialize timeout timer
      time2sleep = 0.1
      while (setpoint_location_global_geopoint_reached is False or setpoint_location_global_yaw_reached is False) and not nepi_sdk.is_shutdown(): # Wait for setpoint goal to be set
        if self.checkStopFunction() is True:
          self.msg_if.pub_info("Setpoint Location received Stop Command", log_name_list = self.log_name_list)
          new_geopoint_wgs84 = copy.deepcopy(self.current_location_wgs84_geo)
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment self.update_error_msg message timer
        timeout_timer = timeout_timer+time2sleep        
        # Calculate setpoint position and yaw errors
        geopoint_errors_geo = np.array(self.current_location_wgs84_geo) - np.array(new_geopoint_wgs84)
        geopoint_errors_m = [geopoint_errors_geo[0]*111139,geopoint_errors_geo[1]*111139,geopoint_errors_geo[2]]
        for ind in range(3):  # Ignore error check if set to current
          if input_geopoint_wgs84[ind] == -999.0:
            geopoint_errors_m[ind] = 0
        max_geopoint_error_m = np.max(np.abs(geopoint_errors_m))
        if input_yaw_ned_deg == -999: # Ignore error check if set to current
          setpoint_location_global_yaw_reached = True
          max_yaw_ned_error_deg = 0
        else:
          cur_yaw_ned_deg = self.current_orientation_ned_degs[2]
          if cur_yaw_ned_deg < 0:
            cur_yaw_ned_deg = cur_yaw_ned_deg + 360
          yaw_ned_error_deg =  cur_yaw_ned_deg - new_yaw_ned_deg
          max_yaw_ned_error_deg = abs(yaw_ned_error_deg)
        # Check for setpoint position global goal
        if  setpoint_location_global_geopoint_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
            max_geopoint_errors = max(geopoint_errors) # Get max from error window
            geopoint_errors = [max_geopoint_error_m] # reset running list of errors
            if max_geopoint_errors < self.rbx_info.error_bounds.max_distance_error_m:
              self.msg_if.pub_info("Location Setpoint Reached", log_name_list = self.log_name_list)
              setpoint_location_global_geopoint_reached = True
          else:
            geopoint_errors.append(max_geopoint_error_m) # append last
        # Check for setpoint position yaw goal
        if  setpoint_location_global_yaw_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
            max_yaw_errors = max(yaw_errors) # Get max from error window
            yaw_errors = [max_yaw_ned_error_deg] # reset running list of errors
            if max_yaw_errors < self.rbx_info.error_bounds.max_rotation_error_deg:
              self.msg_if.pub_info("Yaw Setpoint Reached", log_name_list = self.log_name_list)
              setpoint_location_global_yaw_reached = True
          else:
            yaw_errors.append(max_yaw_ned_error_deg) # append last
        # Reset timer if past
        if stabilize_timer > 1:
          stabilize_timer=0 # Reset timer
        self.update_current_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg] )
      if cmd_success:
        self.msg_if.pub_info("Setpoint Reached", log_name_list = self.log_name_list)
        self.msg_if.pub_info("************************", log_name_list = self.log_name_list)
      self.update_current_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg] )
      return cmd_success

    #######################
    # Class Utility Functions
    
    ### Function for updating current goto error values
    def update_current_errors(self,error_list):
      """Updates the current goto error values in the status message.

      Expects exactly seven elements corresponding to x, y, z translation
      errors in meters and heading, roll, pitch, yaw rotation errors in
      degrees. Does nothing if the list length is not seven.

      Args:
          error_list (list): Error values in the order
              [x_m, y_m, z_m, heading_deg, roll_deg, pitch_deg, yaw_deg].
      """
      if len(error_list) == 7:
        errors_msg = GotoErrors()
        errors_msg.x_m = error_list[0]
        errors_msg.y_m = error_list[1]
        errors_msg.z_m = error_list[2]
        errors_msg.heading_deg = error_list[3]
        errors_msg.roll_deg = error_list[4]
        errors_msg.pitch_deg = error_list[5]
        errors_msg.yaw_deg = error_list[6]

        self.status_msg.errors_current = errors_msg
      else:
        self.msg_if.pub_info("Skipping current error update. Error list to short", log_name_list = self.log_name_list)

    ### Function for updating last goto error values
    def update_prev_errors(self):
        """Copies the current goto error values into the previous error slot.

        Called before a new command is issued so the last command's errors
        are preserved in the status message for comparison.
        """
        errors_msg = GotoErrors()
        errors_msg.x_m = self.status_msg.errors_current.x_m
        errors_msg.y_m = self.status_msg.errors_current.y_m
        errors_msg.z_m = self.status_msg.errors_current.z_m
        errors_msg.heading_deg = self.status_msg.errors_current.heading_deg
        errors_msg.roll_deg = self.status_msg.errors_current.roll_deg
        errors_msg.pitch_deg = self.status_msg.errors_current.pitch_deg
        errors_msg.yaw_deg = self.status_msg.errors_current.yaw_deg
        self.status_msg.errors_prev = errors_msg


    def update_error_msg(self,error_msg):
      """Logs an error and stores it in the status message.

      Args:
          error_msg (str): The error description to log and persist.
      """
      self.msg_if.pub_info(error_msg)
      self.status_msg.last_error_message = error_msg

    def get_motor_controls_status_msg(self,motor_controls):
      """Builds a list of MotorControl messages from a speed-ratio list.

      Args:
          motor_controls (list): Ordered list of speed ratios, one per motor.

      Returns:
          list: A list of MotorControl messages with motor_ind and speed_ratio
              populated for each motor.
      """
      mcs = []
      for i in range(len(motor_controls)):
        mc = MotorControl()
        mc.motor_ind = i
        mc.speed_ratio = motor_controls[i]
        mcs.append(mc)
      return mcs

    def get_3d_transform(self):
        """Returns the current 3D transformation data.

        Returns:
            Transform: The current 3D transform, or the zero transform if no
                transform interface is configured.
        """
        transform = nepi_nav.ZERO_TRANSFORM
        # if self.transform_if is not None:
        #     transform = self.transform_if.get_3d_transform()
        return transform
       
   ### Callback for rbx status publisher
    def statusPublishCb(self,timer):
        self.publish_status()

    def publishStatusCb(self, msg):
        self.publish_status()

    def publish_status(self):
        """Assembles and publishes the RBX device status and image.

        Updates the status message with current battery, control readiness,
        error values, and motor state, then publishes it to the status and
        status_str topics. Also overlays status text on the current image
        if the overlay flag is set, publishes the image, and saves it via
        the save data interface.
        """
        # Do not publish until all IFs (node_if, image_if, save_data_if) are up.
        # publish_status is invoked during NodeClassIF construction (config reset
        # callback) and again before the data IFs are created; self.ready is only
        # set True at the end of __init__, so this guard makes those calls no-op.
        if self.ready == False:
            return
        self.status_msg.device_name = self.device_name
        if self.getBatteryPercentFunction is not None:
          self.rbx_battery = self.getBatteryPercentFunction()
        else:
          self.rbx_battery = -999

        self.status_msg.last_cmd_string = self.last_cmd_string

        ## Update Control Info
        if self.manualControlsReadyFunction is not None:
          self.status_msg.manual_control_mode_ready = self.manualControlsReadyFunction()
        else:
          self.status_msg.manual_control_mode_ready = False

        if self.getMotorControlRatios is not None:
            motor_controls_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
            motor_controls_msg = self.get_motor_controls_status_msg([])
        self.status_msg.current_motor_control_settings = motor_controls_msg


        if self.autonomousControlsReadyFunction is not None:
          self.status_msg.autonomous_control_mode_ready = self.autonomousControlsReadyFunction()
        else:
          self.status_msg.autonomous_control_mode_ready = False

         # Create Status Info Text List
        status_str_msg = []
        if self.status_msg.battery < 0.1:
            battery_string = "No Reading"
        else:
            battery_string = '%.2f' % self.status_msg.battery

        state_ind = self.getStateIndFunction()
        if len(self.states) > 0 and state_ind >= 0 and state_ind <= (len(self.states)):
              state_name = self.states[state_ind]
        else:
              state_name = "Not Set"
        status_str_msg.append("State Current: " + state_name)

        	
        mode_ind = self.getModeIndFunction()
        if len(self.modes) > 0 and mode_ind >= 0 and mode_ind < (len(self.modes) ):
              mode_name = self.modes[mode_ind]
        else:
              mode_name = "Not Set"
        status_str_msg.append("Mode Current: " + mode_name)
        
        if self.rbx_mode_last is not None:
          status_str_msg.append("Mode Last: " + self.modes[self.rbx_mode_last])
        else:
          status_str_msg.append("Mode Last: None")
        status_str_msg.append("Ready: " + str(self.status_msg.ready))
        status_str_msg.append("")
        status_str_msg.append("Current Process: " + self.status_msg.process_current)
        status_str_msg.append(" XYZ Errors Meters: ")
        status_str_msg.append(" " + '%.2f' % self.status_msg.errors_current.x_m + "  " + '%.2f' % self.status_msg.errors_current.y_m + "  " + '%.2f' % self.status_msg.errors_current.z_m)
        status_str_msg.append(" RPY Errors Degrees: ")
        status_str_msg.append(" " + '%.2f' % self.status_msg.errors_current.roll_deg + "  " + '%.2f' % self.status_msg.errors_current.pitch_deg + "  " + '%.2f' % self.status_msg.errors_current.yaw_deg)
        status_str_msg.append("")
        status_str_msg.append("Last Process: " + self.status_msg.process_last)
        status_str_msg.append(" Success: " + str(self.status_msg.cmd_success))
        status_str_msg.append(" XYZ Errors Meters: ")
        status_str_msg.append(" " + '%.2f' % self.status_msg.errors_prev.x_m + "  " + '%.2f' % self.status_msg.errors_prev.y_m + "  " + '%.2f' % self.status_msg.errors_prev.z_m)
        status_str_msg.append(" RPY Errors Degrees: ")
        status_str_msg.append(" " + '%.2f' % self.status_msg.errors_prev.roll_deg + "  " + '%.2f' % self.status_msg.errors_prev.pitch_deg + "  " + '%.2f' % self.status_msg.errors_prev.yaw_deg)
        status_str_msg.append("")

   

        self.status_str_msg = status_str_msg
        if not nepi_sdk.is_shutdown():
            self.status_msg.data_ref_description = self.data_ref_description
            self.node_if.publish_pub('status_msg_pub', self.status_msg)
            self.node_if.publish_pub('status_msg_str_pub', str(status_str_msg))

        # Create ROS Image message
        cv2_img = copy.deepcopy(self.cv2_img) # Initialize status image
        # Overlay status info on image
        if self.rbx_info.image_status_overlay:
            box_x = 10
            box_y = 10
            box_w = 350
            box_h = 450
            # Add status box overlay
            cv2.rectangle(cv2_img, (box_x, box_y), (box_w, box_h), (255, 255, 255), -1)
            # Overlay Status Text List
            x=box_x+10 
            y=box_y+20
            status_str_msg = self.status_str_msg
            for text in status_str_msg:
                self.statusTextOverlay(cv2_img,text,x, y)
                y = y + 20
        # Publish new image to ros
        if not nepi_sdk.is_shutdown():
            self.image_if.publish_cv2_img(cv2_img)
            # You can view the enhanced_2D_image topic at 
            # //192.168.179.103:9091/ in a connected web browser
        timestamp = nepi_utils.get_time()
        self.save_data_if.save('image',cv2_img,timestamp = timestamp)

        ## Update image source topic and subscriber if changed from last time.
        image_source = self.node_if.get_param('image_source')
        image_topic = nepi_sdk.find_topic(image_source)
        if image_topic != "":
          if image_topic != self.rbx_image_source_last:
              if self.rbx_image_sub != None:
                  try:
                    self.msg_if.pub_info("Unsubscribing from image source: " + image_topic)
                    self.rbx_image_sub.unregister()
                    self.rbx_image_sub = None
                    time.sleep(1)
                  except Exception as e:
                    self.msg_if.pub_info(e)
          if self.rbx_image_sub == None:
            self.msg_if.pub_info("Subscribing to image topic: " + image_topic)
            self.rbx_image_sub = nepi_sdk.create_subscriber(image_topic, Image, self.imageSubscriberCb, queue_size = 1)
            self.node_if.set_param('image_source', image_topic)
        else:
              image_topic = "None"
        if image_topic == "None":
              self.cv2_img = self.rbx_image_blank # Set to blank image if source topic is cleared.
        # Set info.image_source value
        self.rbx_info.image_source = image_topic
        self.rbx_image_source_last = image_topic
 
 
        
    ## Status Text Overlay Function
    def statusTextOverlay(self,cv_image,status_text,x,y):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (x,y)
        fontScale              = 0.5
        fontColor              = (0, 0, 0)
        thickness              = 1
        lineType               = 1
        cv2.putText(cv_image,status_text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)


    
