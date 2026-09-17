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
import threading
import subprocess
import numpy as np

import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
# from nepi_sdk import nepi_system
# from nepi_sdk import nepi_pc
# from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav
# from nepi_sdk import nepi_devices

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image, PointCloud2

from nepi_interfaces.msg import DeviceIDXStatus, RangeWindow

from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryResponse, DeviceInfoQueryRequest

from nepi_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryRequest, IDXCapabilitiesQueryResponse
from nepi_interfaces.msg import ImageStatus, PointcloudStatus, NavPose


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SettingsIF, SaveDataIF, Transform3DIF
from nepi_api.device_if_npx import NPXDeviceIF

from nepi_api.data_if import ColorImageIF, DepthMapIF, PointcloudIF, NavPoseIF
#from nepi_api.connect_data_if import ConnectNavPosesIF







SUPPORTED_DATA_PRODUCTS = ['color_image','bw_image',
                            'intensity_map','depth_map','pointcloud']

PERSPECTIVE_OPTIONS = ['pov','top']

#Factory Control Values 
DEFAULT_CONTROLS_DICT = dict( controls_enable = True,
    auto_adjust_ebabled = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    resolution_ratio = 1.0, 
    max_framerate = 10, 
    start_range_ratio = 0.0,
    stop_range_ratio = 1.0,
    min_range_m = 0.0,
    max_range_m = 1.0,
    width_deg = 110,
    height_deg = 70,    
    zoom_ratio = 0.5, 
    rotate_ratio = 0.5,
    )




class IDXDeviceIF:
    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
    UPDATE_NAVPOSE_RATE_HZ = 10

    DEFUALT_IMG_WIDTH_DEG = 100
    DEFUALT_IMG_HEIGHT_DEG = 70

    # Define class variables
    namespace = '~'
    ready = False

    status_msg = DeviceIDXStatus()
    info_report = DeviceInfoQueryResponse()
    caps_report = IDXCapabilitiesQueryResponse()

    node_if = None
    settings_if = None
    navpose_if = None
    save_data_if = None

    color_image_if = None
    depthmap_if = None
    pointcloud_if = None

    # Per-camera 3D mount transform (metadata: where the camera is located/oriented)
    transform_if = None
    transform_topic = ''
    npx_navpose_topic = ''

    # Transformed-navpose publisher (reference frame navpose + mount transform applied)
    idx_navpose_if = None
    ref_navpose_sub = None
    ref_navpose_frame = None

    namespace_npx = ''
    npx_if = None
    device_info_dict = dict()
    navpose_update_rate = 10
    max_navpose_update_rate = 10



    device_name = ''

    factory_controls_dict = copy.deepcopy(DEFAULT_CONTROLS_DICT)

    auto_adjust_controls = []
    auto_adjust_ebabled = False
    brightness_ratio = 0.5
    contrast_ratio = 0.5
    threshold_ratio = 0.0
    resolution_ratio = 1.0  
    max_framerate = 10
    start_range_ratio = 0.0
    stop_range_ratio = 1.0

    min_range_m = 0.0
    max_range_m = 1.0

    data_products_list = []

    data_products_dict = dict()

    update_navpose_interval_sec = float(1)/UPDATE_NAVPOSE_RATE_HZ
    last_gps_timestamp = None
    last_odom_timestamp = None
    last_heading_timestamp = None

   
    rtsp_url = None

    width_px = 1
    height_px = 1

    perspective = 'pov'

    width_deg = DEFUALT_IMG_WIDTH_DEG
    height_deg = DEFUALT_IMG_HEIGHT_DEG
    aspect_ratio_deg = -999

    

    npx_if = None
    
    fps_queue = dict()
    current_fps = dict()
    last_data_time = dict()

    image_thread = None
    depth_map_thread = None
    pointcloud_thread = None

    data_source_description = 'imaging_sensor'
    data_ref_description = 'sensor'

    device_disabled = False

    start_time = 0

    #######################
    ### IF Initialization
    def __init__(self, device_info, 
                 getSettingsFunction=None, setSettingFunction=None, 
                 factoryControls = None, 
                 data_source_description = 'imaging_sensor',
                 data_ref_description = 'sensor',
                 getFOV=None, perspective = 'pov',
                 get_rtsp_url = None,
                 setResolutionRatio=None, setMaxFramerate=None,
                 setContrastRatio=None, setBrightnessRatio=None, 
                 setThresholdingRatio=None, setRangeRatio=None, 
                 setAutoAdjustRatio=None, autoAdjustControls=[],
                 getFramerate=None,
                 getColorImage=None, stopColorImageAcquisition=None, 
                 getDepthMap=None, stopDepthMapAcquisition=None, 
                 getPointcloud=None, stopPointcloudAcquisition=None, 
                 getNavPoseCb = None,
                 navpose_update_rate = 10,
                 data_products =  [],
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,'idx')

        self.start_time = nepi_utils.get_time()
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
        self.msg_if.pub_info("Starting IDX IF Initialization Processes", log_name_list = self.log_name_list)
        self.msg_if.pub_info("Using Namespace: " + str(self.namespace), log_name_list = self.log_name_list)

        ############################# 
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
        self.info_report.type = 'IDX'

        self.caps_report.device_name = self.device_name
        self.caps_report.device_path = self.path
        self.caps_report.device_node_name = self.node_name

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description

        self.msg_if.pub_warn("Got driver supported data products: " + str(data_products))
        #self.msg_if.pub_warn("Supported data products: " + str(SUPPORTED_DATA_PRODUCTS))
        data_products_list = []
        for data_product in data_products:
            #self.msg_if.pub_warn("Checking data product: " + str(data_product))
            if data_product in SUPPORTED_DATA_PRODUCTS:
                data_products_list.append(data_product)
        self.data_products_list = data_products_list
        self.msg_if.pub_warn("Enabled data products: " + str(self.data_products_list))
        # Create the CV bridge. Do this early so it can be used in the threading run() methods below 
        # TODO: Need one per image output type for thread safety?


        # Create and update factory controls dictionary
        self.msg_if.pub_warn("Got Factory Controls: " + str(factoryControls))
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]
        
        self.msg_if.pub_warn("Using Factory Controls: " + str(self.factory_controls_dict))
        self.auto_adjust_ebabled = self.factory_controls_dict['auto_adjust_ebabled']
        self.brightness_ratio = self.factory_controls_dict['brightness_ratio']
        self.contrast_ratio = self.factory_controls_dict['contrast_ratio']
        self.threshold_ratio = self.factory_controls_dict['threshold_ratio']
        self.resolution_ratio = self.factory_controls_dict['resolution_ratio']
        self.max_framerate = self.factory_controls_dict['max_framerate']


        self.min_range_m = self.factory_controls_dict['min_range_m']
        self.max_range_m = self.factory_controls_dict['max_range_m']
        self.msg_if.pub_warn("Set Min Max Ranges: " + str([self.min_range_m,self.max_range_m]))
        self.width_deg = self.factory_controls_dict['width_deg']
        self.height_deg = self.factory_controls_dict['height_deg']

        self.start_range_ratio = self.factory_controls_dict['start_range_ratio']
        self.stop_range_ratio = self.factory_controls_dict['stop_range_ratio']

        
        # Set up standard IDX parameters with ROS param and subscriptions
        # Defer actually setting these on the camera via the parent callbacks... the parent may need to do some 
        # additional setup/calculation first. Parent can then get these all applied by calling ApplyConfigUpdates()

        self.getFOV = getFOV
        if self.getFOV is not None:
            try:
                [self.width_deg,self.height_deg] = self.getFOV()
            except:
                self.getFOV = None
        self.perspective = perspective

        self.get_rtsp_url = get_rtsp_url

        self.getNavPoseCb = getNavPoseCb
        if navpose_update_rate is not None:
            self.max_navpose_update_rate  = copy.deepcopy(navpose_update_rate) 
            if navpose_update_rate < 1:
                navpose_update_rate = 1
            if navpose_update_rate > 10:
                navpose_update_rate = 10
            self.navpose_update_rate = navpose_update_rate
            

        ## Set None Capabilities Variables

        self.setMaxFramerate = setMaxFramerate
        if self.setMaxFramerate is not None:
            self.caps_report.has_framerate = True
        
        self.getFramerate = getFramerate

        self.setRangeRatio = setRangeRatio
        if self.setRangeRatio is not None:
            self.caps_report.has_range = True


        self.setAutoAdjustRatio = setAutoAdjustRatio
        if self.setAutoAdjustRatio is not None:
            self.caps_report.has_auto_adjust = True
        self.auto_adjust_controls = autoAdjustControls

        self.setBrightnessRatio = setBrightnessRatio
        if self.setBrightnessRatio is not None:
            self.caps_report.has_brightness = True

        self.setContrastRatio = setContrastRatio
        if self.setContrastRatio is not None:
            self.caps_report.has_contrast = True

        self.setThresholdingRatio = setThresholdingRatio       
        if self.setThresholdingRatio is not None:
            self.caps_report.has_threshold = True

        self.setResolutionRatio = setResolutionRatio
        if self.setResolutionRatio is not None:
            self.caps_report.has_resolution = True


     

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
            'width_deg': {
                'namespace': self.namespace,
                'factory_val': self.width_deg
            },
            'height_deg': {
                'namespace': self.namespace,
                'factory_val': self.height_deg
            },
            'aspect_ratio_deg': {
                'namespace': self.namespace,
                'factory_val': self.aspect_ratio_deg
            },
            'auto_adjust_ebabled': {
                'namespace': self.namespace,
                'factory_val': self.auto_adjust_ebabled
            },
            'brightness_ratio': {
                'namespace': self.namespace,
                'factory_val': self.brightness_ratio
            },
            'contrast_ratio': {
                'namespace': self.namespace,
                'factory_val': self.contrast_ratio
            },
            'threshold_ratio': {
                'namespace': self.namespace,
                'factory_val': self.threshold_ratio
            },
            'resolution_ratio': {
                'namespace': self.namespace,
                'factory_val': self.resolution_ratio
            },
            'max_framerate': {
                'namespace': self.namespace,
                'factory_val': self.max_framerate
            },
            'start_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.start_range_ratio
            },
            'stop_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.stop_range_ratio
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
                'srv': IDXCapabilitiesQuery,
                'req': IDXCapabilitiesQueryRequest(),
                'resp': IDXCapabilitiesQueryResponse(),
                'callback': self.capabilities_query_callback
            }
        }


        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DeviceIDXStatus,
                'qsize': 1,
                'latch': True
            }
        }
                            

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'disable': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'disable',
                'qsize': 5,
                'callback': self._disableCb, 
                'callback_args': ()
            }, 
            'set_width_deg': {
                'namespace': self.namespace,
                'topic': 'set_width_deg',
                'msg': Int32,
                'qsize': 1,
                'callback': self.setWidthDegCb, 
                'callback_args': ()
            },
            'set_height_deg': {
                'namespace': self.namespace,
                'topic': 'set_height_deg',
                'msg': Int32,
                'qsize': 1,
                'callback': self.setHeightDegCb, 
                'callback_args': ()
            },
            'set_auto_adjust': {
                'namespace': self.namespace,
                'topic': 'set_auto_adjust_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setAutoAdjustEnableCb, 
                'callback_args': ()
            },
            'set_brightness': {
                'namespace': self.namespace,
                'topic': 'set_brightness_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setBrightnessRatioCb, 
                'callback_args': ()
            },
            'set_contrast': {
                'namespace': self.namespace,
                'topic': 'set_contrast_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setContrastRatioCb, 
                'callback_args': ()
            },
            'set_threshold': {
                'namespace': self.namespace,
                'topic': 'set_threshold_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setThresholdingRatioCb, 
                'callback_args': ()
            },
            'set_resolution_ratio': {
                'namespace': self.namespace,
                'topic': 'set_resolution_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setResolutionRatioCb, 
                'callback_args': ()
            },
            'set_max_framerate': {
                'namespace': self.namespace,
                'topic': 'set_max_framerate',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setMaxFramerateCb, 
                'callback_args': ()
            },
            'set_range_window': {
                'namespace': self.namespace,
                'topic': 'set_range_window',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self.setRangeRatioCb, 
                'callback_args': ()
            },
            'reset_controls': {
                'namespace': self.namespace,
                'topic': 'reset_controls',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetControlsCb, 
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

        nepi_sdk.sleep(2)


        start_delay = nepi_utils.get_time() - self.start_time
        self.msg_if.pub_warn("", log_name_list = self.log_name_list)
        self.msg_if.pub_warn("##########################", log_name_list = self.log_name_list)
        self.msg_if.pub_warn("Settings IF Initialization Complete: : " + str(start_delay), log_name_list = self.log_name_list)
        self.msg_if.pub_warn("##########################", log_name_list = self.log_name_list)
        self.msg_if.pub_warn("", log_name_list = self.log_name_list)

        ##############################
        # Update vals from param server
        self.initCb(do_updates = True)
        # self.publish_status()


        # Start the data producers
        if (getColorImage is not None and 'color_image' in self.data_products_list):
            self.getColorImage = getColorImage
            self.stopColorImageAcquisition = stopColorImageAcquisition
            data_product = 'color_image'

            start_data_function = self.getColorImage
            stop_data_function = self.stopColorImageAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread", log_name_list = self.log_name_list)
            self.image_thread = threading.Thread(target=self.runImageThread)
            self.image_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.caps_report.has_color_image = True
        else:
            self.caps_report.has_color_image = False
        
        self.caps_report.has_depth_map = False
        self.caps_report.has_pointcloud = False

        if (getDepthMap is not None and 'depth_map' in self.data_products_list):
            self.getDepthMap = getDepthMap
            self.stopDepthMapAcquisition = stopDepthMapAcquisition
            data_product = 'depth_map'
            
            start_data_function = self.getDepthMap
            stop_data_function = self.stopDepthMapAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread", log_name_list = self.log_name_list)
            self.depth_map_thread = threading.Thread(target=self.runDepthMapThread)
            self.depth_map_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.caps_report.has_depth_map = True
        else:
            self.caps_report.has_depth_map = False

        if (getPointcloud is not None and 'pointcloud' in self.data_products_list):
            self.getPointcloud = getPointcloud
            self.stopPointcloudAcquisition = stopPointcloudAcquisition
            data_product = 'pointcloud'

            start_data_function = self.getPointcloud
            stop_data_function = self.stopPointcloudAcquisition
            data_msg = PointCloud2
            data_status_msg = PointcloudStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread", log_name_list = self.log_name_list)
            self.pointcloud_thread = threading.Thread(target=self.runPointcloudThread)
            self.pointcloud_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.caps_report.has_pointcloud = True
        else:
            self.caps_report.has_pointcloud = False



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


        ##################################
        # Setup Save Data IF Class ####################
        self.msg_if.pub_debug("Starting Save Data IF Initialization", log_name_list = self.log_name_list)     

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "",
            'add_node_name': True
            }

        sd_namespace = self.node_namespace
        self.save_data_if = SaveDataIF(
                                factory_filename_dict = factory_filename_dict,
                                namespace = sd_namespace,
                                log_name_list = self.log_name_list,
                                msg_if = self.msg_if,
                                node_if = self.node_if
                                )


        
        ####################
        # # Setup NavPose IF Class
        # self.msg_if.pub_info("Starting NavPose IF Initialization")
        # np_namespace = self.node_namespace
        # self.navpose_if = ConnectNavPosesIF(namespace = np_namespace,  
        #                             save_data_if = self.save_data_if,
        #                         log_name_list = self.log_name_list,
        #                         msg_if = self.msg_if,
        #                        node_if = self.node_if)
        

        #####################
        # Update Status Message
        nepi_sdk.sleep(1)
        if self.settings_if is not None:
            self.status_msg.settings_topic = self.settings_if.get_namespace()
            self.msg_if.pub_info("Using settings namespace: " + str(self.status_msg.settings_topic))
        if self.save_data_if is not None:
            ready = self.save_data_if.wait_for_ready()
            self.status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic))
        if self.navpose_if is not None:
            self.status_msg.navpose_topic = self.navpose_if.get_namespace()
            self.msg_if.pub_info("Using navpose namespace: " + str(self.status_msg.navpose_topic))

        # The camera's navpose is published by the NPX device at <node>/npx/navpose.
        # Compute it deterministically so the image data products can advertise it in
        # ImageStatus.navpose_topic (the NPX device is created after the data threads start).
        self.npx_navpose_topic = ''
        if self.getNavPoseCb is not None:
            self.npx_navpose_topic = nepi_sdk.create_namespace(
                nepi_sdk.create_namespace(self.node_namespace, 'npx'), 'navpose')

        # Per-camera 3D mount transform: where this camera is located/oriented, as
        # metadata for downstream consumers. Stored, published and persisted here; it is
        # not applied to any navpose stream. The reference frame the transform is defined
        # relative to is carried as its source_ref_description. Created before the data
        # threads so the image data products can advertise its namespace in ImageStatus.
        self.transform_if = Transform3DIF(namespace = self.namespace,
                                source_ref_description = 'base_frame',
                                end_ref_description = self.node_name,
                                get_3d_transform_function = None,
                                log_name_list = self.log_name_list,
                                msg_if = self.msg_if,
                                node_if = self.node_if
                                )
        self.transform_topic = self.transform_if.get_namespace()
        self.msg_if.pub_info("Using transform namespace: " + str(self.transform_topic))

        # Transformed-navpose publisher: subscribes to the selected reference frame's
        # navpose, applies this camera's mount transform, and republishes the result at
        # <node>/idx/navpose so the UI can show the camera's pose with the transform applied.
        # Only the aggregate <node>/idx/navpose topic is published; the per-component
        # sub-topics (location/heading/orientation/position/altitude/depth/pan_tilt) are
        # disabled so the navpose_mgr does not discover this camera as a selectable navpose
        # source (it scans the ROS graph for those component message types).
        self.idx_navpose_if = NavPoseIF(namespace = self.namespace,
                                data_source_description = self.data_source_description,
                                data_ref_description = self.data_ref_description,
                                pub_navpose = True,
                                pub_location = False,
                                pub_heading = False,
                                pub_orientation = False,
                                pub_position = False,
                                pub_altitude = False,
                                pub_depth = False,
                                pub_pan_tilt = False,
                                save_data_if = None,
                                save_data_enabled = False,
                                transform_namespace = self.transform_topic,
                                log_name = 'navpose',
                                log_name_list = self.log_name_list,
                            msg_if = self.msg_if)
                            # msg_if = self.msg_if,
                            # node_if = self.node_if
                            # )
        # Manages the reference-frame subscription and republishes on each navpose
        nepi_sdk.start_timer_process(2.0, self._updateTransformedNavPoseCb, oneshot = True)

        ##################################
        # Start Node Processes
        #nepi_sdk.start_timer_process(1, self._updaterCb, oneshot = True)


        ############################
        # Start Data Get Threads


        for data_product in self.data_products_list:
            self.last_data_time[data_product] = nepi_utils.get_time()
            self.current_fps[data_product] = 0
            self.fps_queue[data_product] = [0 for _ in range(100)]

        # Launch the acquisition and saving threads
        if self.image_thread is not None:
            self.image_thread.start()

        if self.depth_map_thread is not None:
            self.depth_map_thread.start()

        if self.pointcloud_thread is not None:
            self.pointcloud_thread.start()

        nepi_sdk.sleep(1)

        nepi_sdk.start_timer_process(1, self.publishStatusCb)

        # if self.getNavPoseCb is not None:
        #     ###############################
        #     # Create a NPX Device IF
        #     self.msg_if.pub_warn("Starting NPX Class Initialization")

        #     if self.getNavPoseCb is not None:
        #         self.msg_if.pub_warn("Starting NPX Device IF Initialization")
        #         self.npx_if = NPXDeviceIF(device_info = self.device_info_dict,
        #             node_namespace = self.node_namespace,
        #             data_source_description = self.data_source_description,
        #             data_ref_description = self.data_ref_description,
        #             getNavPoseCb = self.getNavPoseCb,
        #             max_navpose_update_rate = self.navpose_update_rate,
        #                     msg_if = self.msg_if)
        #                     # msg_if = self.msg_if,
        #                     # node_if = self.node_if
        #                     # )


    
        ####################################
        self.ready = True
        start_delay = nepi_utils.get_time() - self.start_time
        self.msg_if.pub_warn("", log_name_list = self.log_name_list)
        self.msg_if.pub_warn("##########################", log_name_list = self.log_name_list)
        self.msg_if.pub_warn("IF Initialization Complete: " + str(start_delay), log_name_list = self.log_name_list)
        self.msg_if.pub_warn("##########################", log_name_list = self.log_name_list)
        self.msg_if.pub_warn("", log_name_list = self.log_name_list)
        ####################################

        



    ###############################
    # Class Methods


    def get_ready_state(self):
        """Returns whether the IDX device interface has finished initializing.

        Returns:
            bool: True if the interface is fully initialized and ready, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Blocks until the device interface is ready or the timeout expires.

        Polls the ready flag at 0.1-second intervals. Logs connection status
        on completion or timeout.

        Args:
            timeout (float, optional): Maximum number of seconds to wait.
                Defaults to float('inf') (wait indefinitely).

        Returns:
            bool: True if the interface became ready within the timeout,
                False if the timeout elapsed before ready was set.
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


    def initConfig(self):
        self.initCb(do_updates = True)


    def initCb(self,do_updates = False):
      if self.node_if is not None:
            self.width_deg = self.node_if.get_param('width_deg')
            self.height_deg = self.node_if.get_param('height_deg')  
            aspect_ratio_deg = self.node_if.get_param('aspect_ratio_deg')
            if aspect_ratio_deg is not None:
                self.aspect_ratio_deg = round(aspect_ratio_deg,2)
            self.msg_if.pub_warn("Init degs w,h,ar:: " + str([self.width_deg,self.height_deg,self.aspect_ratio_deg]))

            self.resolution_ratio = self.node_if.get_param('resolution_ratio')
            max_framerate = self.node_if.get_param('max_framerate') 
            if max_framerate is not None:
                self.max_framerate = max_framerate
            self.auto_adjust_ebabled = self.node_if.get_param('auto_adjust_ebabled') 
            self.brightness_ratio = self.node_if.get_param('brightness_ratio')
            self.contrast_ratio = self.node_if.get_param('contrast_ratio')        
            self.threshold_ratio = self.node_if.get_param('threshold_ratio')  
            self.msg_if.pub_warn("Starting range ratios:: " + str([self.start_range_ratio,self.stop_range_ratio]))
            start_range_ratio = self.node_if.get_param('start_range_ratio')
            stop_range_ratio = self.node_if.get_param('stop_range_ratio')
            if start_range_ratio is not None and stop_range_ratio is not None:
                self.start_range_ratio = start_range_ratio
                self.stop_range_ratio = stop_range_ratio
            else:
                self.start_range_ratio = 0
                self.stop_range_ratio = 1
                self.node_if.set_param('start_range_ratio',self.start_range_ratio)
                stop_range_ratio = self.node_if.set_param('stop_range_ratio',self.stop_range_ratio)
            self.msg_if.pub_warn("Updated range ratios:: " + str([self.start_range_ratio,self.stop_range_ratio]))


            self.pt_mounted = self.node_if.get_param('pt_mounted')
            self.pt_topic = self.node_if.get_param('pt_topic')
            self.node_if.save_config()

      if do_updates == True and self.node_if is not None:
        pass        
      self.publish_status()

    def resetCb(self,do_updates = True):
      self.msg_if.pub_warn("Resetting Configs")
      if self.node_if is not None:
        self.node_if.reset_params()
        if self.getFOV is not None:
            try:
                [self.width_deg,self.height_deg] = self.getFOV()
                self.aspect_ratio_deg = -999
                self.msg_if.pub_warn("Updated degs w,h,ar:: " + str([self.width_deg,self.height_deg,self.aspect_ratio_deg]))
                if self.node_if is not None:
                    self.node_if.set_param('width_deg',self.width_deg)
                    self.node_if.set_param('height_deg',self.height_deg) 
                    self.node_if.set_param('aspect_ratio_deg',-999)
            except:
                pass
      if self.save_data_if is not None:
          self.save_data_if.reset()

      if self.settings_if is not None:
          self.settings_if.reset()

      if self.color_image_if is not None:
          self.color_image_if.reset()

      if self.depthmap_if is not None:
          self.depth_map_if.reset()

      if self.pointcloud_if is not None:
          self.pointcloud_if.reset()

      if self.navpose_if is not None:
          self.navpose_if.reset()

      if do_updates == True:
        pass
      self.initCb(do_updates = True)

    def factoryResetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.factory_reset_params()
        if self.getFOV is not None:
            try:
                [self.width_deg,self.height_deg] = self.getFOV()
                self.aspect_ratio_deg = -999
                self.msg_if.pub_warn("Updated degs w,h,ar:: " + str([self.width_deg,self.height_deg,self.aspect_ratio_deg]))
                if self.node_if is not None:
                    self.node_if.set_param('width_deg',self.width_deg)
                    self.node_if.set_param('height_deg',self.height_deg) 
                    self.node_if.set_param('aspect_ratio_deg',-999)
                    
            except:
                pass
      if self.save_data_if is not None:
          self.save_data_if.factory_reset()

      if self.settings_if is not None:
          self.settings_if.factory_reset()

      if self.color_image_if is not None:
          self.color_image_if.factory_reset()

      if self.depthmap_if is not None:
          self.depth_map_if.factory_reset()

      if self.pointcloud_if is not None:
          self.pointcloud_if.factory_reset()

      if self.navpose_if is not None:
          self.navpose_if.factory_reset()
      if do_updates == True:
        pass
      self.initCb(do_updates = True)



    # def ApplyConfigUpdates(self):
    #     self.msg_if.pub_warn("Apply Auto Updates from current values")
    #     if (self.setAutoAdjustRatio is not None):
    #         self.setAutoAdjustRatio(self.auto_adjust_ebabled)
    #     if (self.setBrightnessRatio is not None):
    #         self.setBrightnessRatio(self.brightness_ratio)
    #     if (self.setContrastRatio is not None):
    #         self.setContrastRatio(self.contrast_ratio)
    #     if (self.setThresholdingRatio is not None):
    #         self.setThresholdingRatio(self.threshold_ratio)
    #     if (self.setResolutionRatio is not None):
    #         self.setResolutionRatio(self.resolution_ratio)
    #     if (self.setMaxFramerate is not None and self.max_framerate is not None):
    #         self.msg_if.pub_warn("Apply Config Framerate: " + str(self.max_framerate))
    #         self.setMaxFramerate(self.max_framerate)
    #     if (self.setRangeRatio is not None):
    #         #self.msg_if.pub_warn("Applying range ratios:: " + str([self.start_range_ratio,self.stop_range_ratio]))
    #         self.setRangeRatio(self.start_range_ratio, self.stop_range_ratio)




    def addDataProduct2Dict(self,data_product, start_data_function,stop_data_function,data_msg,data_status_msg):
        success = False
        data_product = data_product
        namespace = os.path.join(self.base_namespace,self.node_name,'idx')
        dp_dict = dict()
        dp_dict['data_product'] = data_product
        dp_dict['image_topic'] = 'None'
        

        dp_dict['get_data'] = start_data_function
        dp_dict['stop_data'] = stop_data_function

        self.data_products_dict[data_product] = dp_dict

        # do wait here for all
        success = True
        return success

    def _disableCb(self, msg):
        #self.msg_if.pub_info("Recieved Disable Update: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data
        self.disable(enabled)

    def disable(self, enabled):
        """Enable or disable data for all registered data products.

        Args:
            enabled (bool): True to enable data, False to disable.
        """
        self.msg_if.pub_warn("Setting Saving Disabled to: " + str(enabled))  
        self.device_disabled = enabled
        self.publish_status()   



    def setWidthDegCb(self, msg):
        #self.msg_if.pub_info("Recived Width Deg update message: " + str(msg))
        width_deg = msg.data
        if width_deg > 10:
            self.aspect_ratio_deg = round(self.width_px / self.height_px, 2)
            self.width_deg = width_deg     
            self.msg_if.pub_warn("Setting degs w,h,ar:: " + str([self.width_deg,self.height_deg,self.aspect_ratio_deg]))
            self.publish_status(do_updates=False) # Updated inline here   
            if self.node_if is not None:
                self.node_if.set_param('width_deg', width_deg)
                self.node_if.set_param('aspect_ratio_deg',self.aspect_ratio_deg)                


 
    def setHeightDegCb(self, msg):
        #self.msg_if.pub_info("Recived Height Deg update message: " + str(msg))
        height_deg = msg.data
        if height_deg > 10:
            self.aspect_ratio_deg = round(self.width_px / self.height_px, 2)
            self.height_deg = height_deg   
            self.msg_if.pub_warn("Setting degs w,h,ar:: " + str([self.width_deg,self.height_deg,self.aspect_ratio_deg, ]))
            self.publish_status(do_updates=False) # Updated inline here   
            if self.node_if is not None:
                self.node_if.set_param('height_deg', height_deg)
                self.node_if.set_param('aspect_ratio_deg',self.aspect_ratio_deg)
            
    def setAutoAdjustEnableCb(self, msg):
        #self.msg_if.pub_info("Recived Auto Adjust update message: " + str(msg))
        enabled = msg.data
        if self.setAutoAdjustRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setAutoAdjustRatio(enabled)

        if enabled:
            self.msg_if.pub_info("Enabling Auto Adjust", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Disabling IDX Auto Adjust", log_name_list = self.log_name_list)

        self.auto_adjust_ebabled = enabled       
        self.publish_status(do_updates=False) # Updated inline here
        if self.node_if is not None:
            self.node_if.set_param('auto_adjust_ebabled', enabled)




    def setBrightnessRatioCb(self, msg):
        #self.msg_if.pub_info("Recived Brightness update message: " + str(msg))
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.brightness_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        if self.setBrightnessRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setBrightnessRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param('brightness_ratio', ratio)


    def setContrastRatioCb(self, msg):
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.contrast_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        if self.setContrastRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setContrastRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param('contrast_ratio', ratio)
        


    def setThresholdingRatioCb(self, msg):
        self.msg_if.pub_info("Received Threshold update message: " + str(msg))
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.threshold_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        if self.setThresholdingRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setThresholdingRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param('threshold_ratio', ratio)
        

    def setResolutionRatioCb(self, msg):
        #self.msg_if.pub_info("Recived Resolution update message: " + str(msg))
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.resolution_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        if self.setResolutionRatio is not None:
            status, err_str = self.setResolutionRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param('resolution_ratio', ratio)
        


        
    def setMaxFramerateCb(self, msg):
        #self.msg_if.pub_info("Recived Max Framerate update message: " + str(msg))
        rate = msg.data
 
        if rate < 1:
            rate = 1
        if rate > 100:
            rate = 100

        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        self.max_framerate = rate
        self.publish_status(do_updates=False) # Updated inline here

        if self.setMaxFramerate is not None:
            #self.msg_if.pub_warn("Sending update framerate ratio to driver: ")
            status, err_str = self.setMaxFramerate(rate)
            #self.msg_if.pub_warn("Recived Framerate update: " + str(status))


        for data_product in self.data_products_list:
            self.fps_queue[data_product] = [0 for _ in range(100)]

        if self.node_if is not None:
            self.node_if.set_param('max_framerate', rate)


 
    def setRangeRatioCb(self, msg):
        #self.msg_if.pub_info("Recived Range update message: " + str(msg))
        #self.msg_if.pub_info("Recived update message: " + str(msg))
        new_start_range_ratio = msg.start_range
        new_stop_range_ratio = msg.stop_range
        if (new_start_range_ratio < 0 or new_stop_range_ratio > 1 or new_stop_range_ratio < new_start_range_ratio):
            self.msg_if.pub_error("Range values out of bounds", log_name_list = self.log_name_list)
            self.publish_status(do_updates=False) # No change
            return
        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            if self.setRangeRatio is not None:
                status, err_str = self.setRangeRatio(new_start_range_ratio,new_stop_range_ratio)

        self.start_range_ratio = new_start_range_ratio
        self.stop_range_ratio = new_stop_range_ratio
        self.publish_status(do_updates=False) # Updated inline here  
        if self.node_if is not None:
            self.node_if.set_param('start_range_ratio', new_start_range_ratio)
            self.node_if.set_param('stop_range_ratio', new_stop_range_ratio)
     
  

    def resetControlsCb(self, msg):
        #self.msg_if.pub_info("Recived reset controls message: " + str(msg))
        self.node_if.reset_param('controls_enable')
        self.node_if.reset_param('auto_adjust_ebabled')       
        self.node_if.reset_param('brightness_ratio')
        self.node_if.reset_param('contrast_ratio')        
        self.node_if.reset_param('threshold_ratio')
        self.node_if.reset_param('resolution_ratio')   
        self.node_if.reset_param('max_framerate')
        self.node_if.reset_param('start_range_ratio')
        self.node_if.reset_param('stop_range_ratio')
        self.ApplyConfigUpdates()



  
    def update_fps(self, data_product):
        """Calculates and updates FPS statistics for a data product using a rolling window.

        Records the current timestamp, computes instantaneous FPS from the
        interval since the last frame, and maintains a 100-sample rolling
        queue for averaging. Publishes a status update when the averaged FPS
        changes by more than 1 Hz or is still settling.

        Args:
            data_product (str): Key identifying the data product whose FPS
                should be updated (e.g. ``'color_image'``, ``'depth_map'``).
        """
        last_data_time = copy.deepcopy(self.last_data_time[data_product])
        self.last_data_time[data_product] = nepi_utils.get_time()
        last_fps = copy.deepcopy(self.current_fps[data_product])
        if last_data_time is not None:
            f_time = (self.last_data_time[data_product] - last_data_time)
            current_fps = float(1) / f_time
            #self.msg_if.pub_warn("Got " + data_product + " time and fps: " + str(round(f_time,3)) + " : " + str(round(current_fps,2)), throttle_s = 2)
            self.fps_queue[data_product].pop(0)
            self.fps_queue[data_product].append(current_fps)
            self.msg_if.pub_debug("fps queue " + str(self.fps_queue[data_product]), throttle_s = 2)
            fps_queue = [x for x in self.fps_queue[data_product] if x != 0]
            if len(fps_queue) > 1:
                self.current_fps[data_product] = round(sum(fps_queue)/len(fps_queue),0)
            fps_changed = abs(self.current_fps[data_product] - last_fps) > 1
            fps_changing = 0 in self.fps_queue[data_product]
            if fps_changed or fps_changing:
                self.publish_status()
  

    # Image from img_get_function can be CV2 or ROS image.  Will be converted as needed in the thread
    def image_thread_proccess(self, data_product):
        """Continuously acquires, processes, and publishes image data for a given data product.

        Waits for the save-data interface to be ready, then enters a loop that
        checks whether any subscribers need data. When data is needed, calls the
        registered acquisition function and publishes the resulting CV2 image via
        the appropriate data interface (e.g. ``ColorImageIF``). Stops acquisition
        and resets FPS counters when no subscribers are active.

        Args:
            data_product (str): Key identifying the image data product to run
                (e.g. ``'color_image'``). Must be present in
                ``self.data_products_dict``.
        """
        cv2_img = None

        if data_product not in self.data_products_dict.keys():
            self.msg_if.pub_warn("Can't start data product acquisition " + data_product + " , not in data product dict", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_warn("Starting " + data_product + " acquisition", log_name_list = self.log_name_list)
            acquiring = False

            self.msg_if.pub_debug("Waiting for save_data_if: " + data_product)
            while (not nepi_sdk.is_shutdown() and self.save_data_if is None):
                nepi_sdk.sleep(1)

            dp_dict = self.data_products_dict[data_product]
            dp_get_data = dp_dict['get_data']
            dp_stop_data = dp_dict['stop_data']

            #img_pub = nepi_sdk.create_publisher(pub_namespace, Image, queue_size = 10)
            if data_product == 'color_image':
                self.msg_if.pub_warn("Creating ColorImageIF for data product: " + data_product)
                dp_namespace = self.namespace
                self.color_image_if = ColorImageIF(namespace = dp_namespace,
                            data_source_description = self.data_source_description,
                            data_ref_description = self.data_ref_description,
                            perspective = self.perspective,
                            # Use the transformed-navpose IF so image overlays show this
                            # camera's pose (with its mount transform applied), the same
                            # data published at <node>/idx/navpose. self.navpose_if is None
                            # in this node, which would leave the overlay showing all zeros.
                            navpose_if = self.idx_navpose_if,
                            navpose_namespace = self.npx_navpose_topic,
                            transform_namespace = self.transform_topic,
                            save_data_if = self.save_data_if,
                            log_name = data_product,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if,
                            node_if = self.node_if
                            )
                ready = self.color_image_if.wait_for_ready()
                self.data_products_dict[data_product]['image_topic'] = self.color_image_if.get_namespace()

            if self.color_image_if is None:
                self.msg_if.pub_debug("Failed to create data IF class for: " + data_product + " ** Ending thread")
                return

            # Get Data Product Dict and Data_IF

            self.msg_if.pub_warn("Starting data capture thread for data product: " + data_product)
            while (not nepi_sdk.is_shutdown()):
                # Get data if requried
                get_data = self.color_image_if.needs_data_check()
                if get_data == True and self.device_disabled == False:
                    acquiring = True
                    status, msg, cv2_img, timestamp, encoding = dp_get_data()

                    if (status is False or cv2_img is None):
                        #self.msg_if.pub_debug("No Data Recieved: " + data_product, throttle_s = 5.0)
                        pass
                    else:
                        #self.msg_if.pub_debug("Got Data: " + data_product, throttle_s = 5.0)
                        
                        # Get Image Info and Pub Status if Changed
                        cur_width = self.width_px
                        cur_height = self.height_px
                        cv2_shape = cv2_img.shape
                        self.width_px = cv2_shape[1] 
                        self.height_px = cv2_shape[0]             

                        if self.aspect_ratio_deg is None:
                            self.aspect_ratio_deg = -999
                        if self.aspect_ratio_deg == -999 or self.aspect_ratio_deg < 0.5 or self.aspect_ratio_deg > 2.5:
                            self.aspect_ratio_deg = round(self.width_px / self.height_px, 2)
                            if self.node_if is not None:
                                self.node_if.set_param('aspect_ratio_deg',self.aspect_ratio_deg)

                        cur_aspect_ratio = round(self.width_px / self.height_px, 2)
                        
                        width_deg = self.width_deg / self.aspect_ratio_deg * cur_aspect_ratio
                        height_deg = self.height_deg
                        #self.msg_if.pub_warn("Image Pub degs w,cw,ch,h,ar,cr:: " + str([self.width_deg,width_deg,self.height_deg,height_deg,self.aspect_ratio_deg,cur_aspect_ratio]), throttle_s = 5)
                        # Now process and publish image
                        cv2_img = self.color_image_if.publish_cv2_img(cv2_img, encoding = encoding,
                                                        timestamp = timestamp,
                                                        width_deg = width_deg,
                                                        height_deg = height_deg
                                                        )


                        self.update_fps(data_product)

                        #self.msg_if.pub_debug("Got cv2_img size: " + str(self.width_px) + ":" + str(self.height_px), log_name_list = self.log_name_list, throttle_s = 5.0)
                        if cur_width != self.width_px or cur_height != self.height_px:
                            self.publish_status()

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition", log_name_list = self.log_name_list)
                        dp_stop_data()
                    acquiring = False
                    self.current_fps[data_product] = 0.0
                    self.fps_queue[data_product] = [0,0,0,0,0,0,0,0,0,0]
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_sdk.sleep(0.25)
                #self.msg_if.pub_debug("Ending with avg fps: " + str(self.current_fps[data_product]), log_name_list = self.log_name_list, throttle_s = 5.0)  
                nepi_sdk.sleep(0.01) # Yield


    def depth_map_thread_proccess(self, data_product):
        """Continuously acquires, processes, and publishes depth map data for a given data product.

        Waits for the save-data interface to be ready, then enters a loop that
        checks whether any subscribers need data. When data is needed, calls the
        registered acquisition function and publishes the resulting numpy depth
        map via ``DepthMapIF``, applying the configured range-window ratios to
        derive effective min/max range values. Stops acquisition and resets FPS
        counters when no subscribers are active.

        Args:
            data_product (str): Key identifying the depth map data product to run
                (e.g. ``'depth_map'``). Must be present in
                ``self.data_products_dict``.
        """
        np_depth_map = None

        if data_product not in self.data_products_dict.keys():
            self.msg_if.pub_warn("Can't start data product acquisition " + data_product + " , not in data product dict", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_warn("Starting " + data_product + " acquisition", log_name_list = self.log_name_list)
            acquiring = False

            self.msg_if.pub_debug("Waiting for save_data_if: " + data_product)
            while (not nepi_sdk.is_shutdown() and self.save_data_if is None):
                nepi_sdk.sleep(1)

            dp_dict = self.data_products_dict[data_product]
            dp_get_data = dp_dict['get_data']
            dp_stop_data = dp_dict['stop_data']


            dp_namespace = self.namespace
            self.depthmap_if = DepthMapIF(namespace = dp_namespace,
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        pub_image = True,
                        init_overlay_text_list = [],
                        # Transformed-navpose IF (see ColorImageIF above): drives image
                        # overlays with this camera's pose; self.navpose_if is None here.
                        navpose_if = self.idx_navpose_if,
                        save_data_if = self.save_data_if,
                        log_name = data_product,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if,
                            node_if = self.node_if
                            )
            ready = self.depthmap_if.wait_for_ready()
            self.data_products_dict[data_product]['image_topic'] = self.depthmap_if.get_namespace()

            if self.depthmap_if is None:
                self.msg_if.pub_debug("Failed to create data IF class for: " + data_product + " ** Ending thread")
                return

            # Get Data Product Dict and Data_IF

            self.msg_if.pub_warn("Starting data capture thread for data product: " + data_product)
            while (not nepi_sdk.is_shutdown()):
                # Get data if requried
                get_data = self.depthmap_if.needs_data_check()
                if get_data == True and self.device_disabled == False:
                    #self.msg_if.pub_warn("Got Depth Map Needs Data", log_name_list = self.log_name_list)

                    acquiring = True

                    status, msg, np_depth_map, timestamp, encoding = dp_get_data()
                    #print('IDX Min Max Depths: ' + str([np.nanmin(np_depth_map),np.nanmax(np_depth_map)]) )
                    if (status is False or np_depth_map is None):
                        #self.msg_if.pub_warn("No Data Recieved: " + data_product, throttle_s = 5.0)
                        pass
                    else:                       
                        # Get Image Info and Pub Status if Changed
                        cur_width = self.width_px
                        cur_height = self.height_px
                        cv2_shape = np_depth_map.shape
                        self.width_px = cv2_shape[1] 
                        self.height_px = cv2_shape[0] 

                        # if self.aspect_ratio_deg == -999 or self.aspect_ratio_deg < 0.5 or self.aspect_ratio_deg > 2.5:
                        #     self.aspect_ratio_deg  = round(self.width_px / self.height_px, 2)
                        #     if self.node_if is not None:
                        #         self.node_if.set_param('aspect_ratio_deg',self.aspect_ratio_deg)
                        cur_aspect_ratio = self.width_px / self.height_px
                        if self.aspect_ratio_deg is not None:
                            self.aspect_ratio_deg = cur_aspect_ratio
                        width_deg = self.width_deg / self.aspect_ratio_deg * cur_aspect_ratio
                        height_deg = self.height_deg

                        #Publish Ros Image
                        #self.msg_if.pub_warn("Got Min Max Ranges: " + str([self.min_range_m,self.max_range_m]))
                        range_m = self.max_range_m - self.min_range_m
                        min_range_m = self.min_range_m + self.start_range_ratio * range_m
                        max_range_m = self.max_range_m - (1-self.stop_range_ratio) * range_m
                        #self.msg_if.pub_warn("Using Min Max Ranges: " + str([self.min_range_m,self.max_range_m]))
                        np_depth_map = self.depthmap_if.publish_np_depth_map(np_depth_map,
                                                encoding = encoding,
                                                width_deg = width_deg,
                                                height_deg = height_deg,
                                                min_range_m = min_range_m,
                                                max_range_m = max_range_m,
                                                timestamp = timestamp
                                                )

                        self.update_fps(data_product)
                        if cur_width != self.width_px or cur_height != self.height_px:
                            self.publish_status()

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition", log_name_list = self.log_name_list)
                        dp_stop_data()
                    acquiring = False
                    self.current_fps[data_product] = 0.0
                    self.fps_queue[data_product] = [0 for _ in range(100)]
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_sdk.sleep(0.25)
                #self.msg_if.pub_debug("Ending with avg fps: " + str(self.current_fps[data_product]), log_name_list = self.log_name_list, throttle_s = 5.0)    
                nepi_sdk.sleep(0.01) # Yield


              

   
    # Pointcloud from pointcloud_get_function can be open3D or ROS pointcloud.  Will be converted as needed in the thread
    def pointcloud_thread_proccess(self, data_product):
        """Continuously acquires, processes, and publishes pointcloud data for a given data product.

        Waits for the save-data interface to be ready, then enters a loop that
        checks whether any subscribers need data. When data is needed, calls the
        registered acquisition function and publishes the resulting Open3D or ROS
        pointcloud via ``PointcloudIF``, applying the configured range-window
        ratios to derive effective min/max range values. Stops acquisition when
        no subscribers are active.

        Args:
            data_product (str): Key identifying the pointcloud data product to run
                (e.g. ``'pointcloud'``). Must be present in
                ``self.data_products_dict``.
        """
        o3d_pc = None

        if data_product not in self.data_products_dict.keys():
            self.msg_if.pub_warn("Can't start data product acquisition " + data_product + " , not in data product dict", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_warn("Starting " + data_product + " acquisition", log_name_list = self.log_name_list)
            acquiring = False

            self.msg_if.pub_debug("Waiting for save_data_if: " + data_product)
            while (not nepi_sdk.is_shutdown() and self.save_data_if is None):
                nepi_sdk.sleep(1)


            dp_dict = self.data_products_dict[data_product]
            dp_get_data = dp_dict['get_data']
            dp_stop_data = dp_dict['stop_data']

            #img_pub = nepi_sdk.create_publisher(pub_namespace, Image, queue_size = 10)
            dp_namespace = self.namespace
            self.pointcloud_if = PointcloudIF(namespace = dp_namespace,
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        pub_image = True,
                        init_overlay_text_list = [],
                        # Transformed-navpose IF (see ColorImageIF above): drives image
                        # overlays with this camera's pose; self.navpose_if is None here.
                        navpose_if = self.idx_navpose_if,
                        save_data_if = self.save_data_if,
                        log_name = data_product,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if,
                            node_if = self.node_if
                            )
            ready = self.pointcloud_if.wait_for_ready()

            if self.pointcloud_if is None:
                self.msg_if.pub_debug("Failed to create data IF class for: " + data_product + " ** Ending thread")
                return

        
            # Get Data Product Dict and Data_IF

            self.msg_if.pub_warn("Starting data capture thread for data product: " + data_product)
            while (not nepi_sdk.is_shutdown()):
                # Get data if requried
                get_data = self.pointcloud_if.needs_data_check()
                if get_data == True and self.device_disabled == False:
                    acquiring = True
                    status, msg, o3d_pc, timestamp, pc_frame = dp_get_data()
                    if (status is False or o3d_pc is None):
                        #self.msg_if.pub_warn("No Data Recieved: " + data_product, throttle_s = 5.0)
                        pass
                    else:
                        range_m = self.max_range_m - self.min_range_m
                        min_range_m = self.min_range_m + self.start_range_ratio * range_m
                        max_range_m = self.max_range_m - (1-self.stop_range_ratio) * range_m
                        o3d_pc = self.pointcloud_if.publish_o3d_pc(o3d_pc,
                                                width_deg = self.width_deg,
                                                height_deg = self.height_deg,
                                                min_range_m = min_range_m,
                                                max_range_m = max_range_m,
                                                timestamp = timestamp
                                                )

                        self.update_fps(data_product)

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition", log_name_list = self.log_name_list)
                        dp_stop_data()
                    acquiring = False
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_sdk.sleep(0.25)
                nepi_sdk.sleep(0.01) # Yield
                



    def runImageThread(self):
        self.image_thread_proccess('color_image')

    def runDepthMapThread(self):
        self.depth_map_thread_proccess('depth_map')
        #pass

    def runPointcloudThread(self):
        self.pointcloud_thread_proccess('pointcloud')

 
    ### Info callback
    def info_query_callback(self, _):
        """Handles a ROS service request for device information.

        Returns the pre-populated ``DeviceInfoQueryResponse`` containing
        device name, path, node name, namespace, serial number, hardware
        version, software version, and device type.

        Args:
            _ (DeviceInfoQueryRequest): Unused service request object.

        Returns:
            DeviceInfoQueryResponse: The device information report.
        """
        return self.info_report

    def capabilities_query_callback(self, _):
        """Handles a ROS service request for device capabilities.

        Refreshes the data-products list in the capabilities report before
        returning it. The report reflects which optional capabilities
        (framerate, range, auto-adjust, brightness, contrast, threshold,
        resolution, color image, depth map, pointcloud) were provided at
        construction time.

        Args:
            _ (IDXCapabilitiesQueryRequest): Unused service request object.

        Returns:
            IDXCapabilitiesQueryResponse: The device capabilities report.
        """
        self.caps_report.data_products = self.data_products_list
        return self.caps_report

    # Function to update and publish status message
    def publishStatusCb(self,timer):
        self.publish_status()


    def _updateTransformedNavPoseCb(self, timer):
        # Keep the reference-frame navpose subscription in sync with the selected frame
        # (the transform's source_ref_description), re-subscribing when it changes.
        frame = ''
        if self.transform_if is not None:
            frame = self.transform_if.get_source_description()
        if frame != self.ref_navpose_frame:
            if self.ref_navpose_sub is not None:
                try:
                    self.ref_navpose_sub.unregister()
                except Exception:
                    pass
                self.ref_navpose_sub = None
            self.ref_navpose_frame = frame
            if frame is not None and frame != '' and frame != 'None':
                ref_topic = nepi_sdk.create_namespace(
                    nepi_sdk.create_namespace(
                        nepi_sdk.create_namespace(self.base_namespace, 'navposes'), frame), 'navpose')
                self.ref_navpose_sub = nepi_sdk.create_subscriber(ref_topic, NavPose, self._refNavPoseCb)
                self.msg_if.pub_info("Transformed navpose using reference frame topic: " + str(ref_topic), log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._updateTransformedNavPoseCb, oneshot = True)


    def _refNavPoseCb(self, msg):
        if self.idx_navpose_if is None:
            return
        try:
            np_dict = nepi_nav.convert_navpose_msg2dict(msg)
            if self.transform_if is not None:
                tf_dict = self.transform_if.get_3d_transform_dict()
                # nepi_nav.transform_navpose_dict's position/altitude branches reference keys
                # that don't exist (x_deg / altitude_m) and raise; skip them (we set the
                # camera's local position ourselves) so only location/orientation/heading
                # get transformed here.
                base_altitude_m = np_dict.get('altitude_m', 0.0)
                base_has_altitude = np_dict.get('has_altitude', False)
                np_dict['has_position'] = False
                np_dict['has_altitude'] = False
                np_dict = nepi_nav.transform_navpose_dict(np_dict, tf_dict)
                # The camera's LOCAL POSITION is its offset from the reference frame origin,
                # i.e. the mount translation. Expose it in the navpose X/Y/Z position fields.
                np_dict['has_position'] = True
                np_dict['time_position'] = nepi_utils.get_time()
                np_dict['x_m'] = tf_dict['x_m']
                np_dict['y_m'] = tf_dict['y_m']
                np_dict['z_m'] = tf_dict['z_m']
                # Restore altitude passthrough (branch skipped above)
                np_dict['has_altitude'] = base_has_altitude
                np_dict['altitude_m'] = base_altitude_m
            self.idx_navpose_if.publish_navpose(np_dict, transform = None)
        except Exception as e:
            self.msg_if.pub_warn("Failed to publish transformed navpose: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)


    def publish_status(self, do_updates=True):
        """Builds and publishes the DeviceIDXStatus message on the status topic.

        Populates all fields of the status message from current instance state,
        including device identity, FOV angles, resolution, framerate, image
        control ratios, range-window settings, and per-data-product FPS. When
        ``do_updates`` is True, also queries the RTSP URL callback (if
        registered) and includes the result in the message.

        Args:
            do_updates (bool, optional): When True, perform live queries (e.g.
                RTSP URL) before publishing. Set to False when the caller has
                already updated state inline and a lightweight publish is
                sufficient. Defaults to True.
        """
        self.status_msg.device_name = self.device_name

        self.status_msg.device_disabled = self.device_disabled

        # aspect_ratio_deg = self.width_px / self.height_px
        # if self.aspect_ratio_deg == -999 or self.aspect_ratio_deg < 0.5 or self.aspect_ratio_deg > 2.5:
        #     self.aspect_ratio_deg  = round(self.width_px / self.height_px, 2)
        
        cur_aspect_ratio = self.width_px / self.height_px
        if cur_aspect_ratio < 0.5 or cur_aspect_ratio > 2.5:
            cur_aspect_ratio = 1        

        width_deg = self.width_deg / self.aspect_ratio_deg * cur_aspect_ratio
        height_deg = self.height_deg

        self.status_msg.width_deg = int(width_deg)
        self.status_msg.height_deg = int(height_deg)
        self.status_msg.aspect_ratio_deg = round(cur_aspect_ratio,2)

        self.status_msg.perspective = self.perspective
        
        self.status_msg.resolution_ratio = self.resolution_ratio
        res_str = str(self.width_px) + ":" + str(self.height_px)
        self.status_msg.resolution_current = res_str

        self.status_msg.max_framerate = self.max_framerate

        data_products = self.data_products_list
        framerates = []
        for dp in data_products:
            if dp in self.current_fps.keys():
                framerates.append(self.current_fps[dp])
            else:
                framerates.append(0.0)
        self.status_msg.data_products = data_products
        self.status_msg.framerates = framerates


        self.status_msg.auto_adjust_enabled = self.auto_adjust_ebabled
        self.status_msg.auto_adjust_controls = self.auto_adjust_controls
        self.status_msg.contrast_ratio = self.contrast_ratio
        self.status_msg.brightness_ratio = self.brightness_ratio
        self.status_msg.threshold_ratio = self.threshold_ratio
        
        self.status_msg.range_window_ratios.start_range = self.start_range_ratio
        self.status_msg.range_window_ratios.stop_range =  self.stop_range_ratio

        self.status_msg.min_range_m = self.min_range_m
        self.status_msg.max_range_m = self.max_range_m

        delta_range = self.max_range_m - self.min_range_m
        self.status_msg.min_range_m_adj = self.min_range_m + delta_range * self.start_range_ratio
        self.status_msg.max_range_m_adj = self.min_range_m + delta_range * self.stop_range_ratio


        if do_updates == True:
           
            rtsp_url = ""
            rtsp_username = ""
            rtsp_password = ""
            if self.get_rtsp_url is not None:
                [rtsp_url,rtsp_username,rtsp_password] = self.get_rtsp_url()
                if rtsp_url is None:
                    rtsp_url = ""
            self.status_msg.rtsp_url = rtsp_url
            self.status_msg.rtsp_username = rtsp_username
            self.status_msg.rtsp_password = rtsp_password

        self.msg_if.pub_debug("Created status msg: " + str(self.status_msg), throttle_s = 5.0)
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub',self.status_msg)
    
