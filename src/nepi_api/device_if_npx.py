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
import numpy as np
import math
import time 
import copy
import sys
import tf
import yaml

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from geographic_msgs.msg import GeoPoint


from geometry_msgs.msg import Vector3

from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryResponse, DeviceInfoQueryRequest

from nepi_interfaces.msg import DeviceNPXStatus
from nepi_interfaces.srv import  NPXCapabilitiesQuery, NPXCapabilitiesQueryRequest, NPXCapabilitiesQueryResponse


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_nav

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF, Transform3DIF

from nepi_api.data_if import NavPoseIF


#########################################
# Node Class
#########################################


class NPXDeviceIF:

  MIN_PUB_RATE = 1.0
  NAVPOSE_FRAME_ID_OPTIONS = ['sensor_frame','custom_frame']
  NAVPOSE_NAV_FRAME_OPTIONS = ['ENU','NED','UKNOWN']
  NAVPOSE_ALT_FRAME_OPTIONS = ['WGS84','AMSL','AGL','MSL','HAE','BAROMETER','UKNOWN']
  NAVPOSE_DEPTH_FRAME_OPTIONS = ['MSL','TOC','DF','KB','DEPTH','UKNOWN']

  DEFAULT_UPDATE_RATE = 30
  # [min, max] Hz clamp range for the update rate. Mirrors the caps report range
  # [MIN_PUB_RATE, DEFAULT_UPDATE_RATE]; used by _setUpdateMaxRateCb.
  NAVPOSE_UPDATE_RATE_OPTIONS = [MIN_PUB_RATE, DEFAULT_UPDATE_RATE]
  DEFAULT_3D_FRAME = 'sensor_frame'
  DEFAULT_NAV_FRAME = 'ENU'
  DEFAULT_ALT_FRAME = 'WGS84'
  DEFAULT_DEPTH_FRAME = 'DEPTH'


  data_products_list = ['navpose']

  node_if = None
  settings_if = None
  save_data_if = None
  navpose_if = None

  status_msg = DeviceNPXStatus()
  info_report = DeviceInfoQueryResponse()
  caps_report = NPXCapabilitiesQueryResponse()
  
  update_rate = DEFAULT_UPDATE_RATE
  navpose_frame = DEFAULT_3D_FRAME
  frame_nav = DEFAULT_NAV_FRAME
  frame_altitude = DEFAULT_ALT_FRAME
  frame_depth = DEFAULT_DEPTH_FRAME


  has_location = False  
  has_heading = False
  has_orientation = False
  has_position = False
  has_altitude = False
  has_depth = False
  has_pan_tilt = False
  supports_updates = True

  set_location_source = False
  set_heading_source = False
  set_orientation_source = False  
  set_position_source = False
  set_altitude_source = False
  set_depth_source = False
  set_pan_tilt_source = False

  was_location_source = False
  was_heading_source = False
  was_orientation_source = False  
  was_position_source = False
  was_altitude_source = False
  was_depth_source = False
  was_pan_tilt_source = False

  navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

  navpose_if = None  

  settings_if = None

  has_updated = False

  pub_subs = False

  navpose_mgr_if = None

  # User-settable per-device 3D mount transform applied to each published navpose
  transform_if = None

  data_source_description = 'navpose_sensor'
  data_ref_description = 'data_reference'
  navpose_frame = 'None'

  getNavPoseCb = None
  supports_updates = True

  # True once the has_* flags have been derived from the telemetry source (or once
  # it is known there is no source to derive them from). The NavPose IF is created
  # at the moment this becomes True, so its publisher set and the reported flags are
  # always decided together, from the same data.
  navpose_caps_known = False

  #######################
  ### IF Initialization
  def __init__(self, 
                device_info,
                node_namespace = None,
                getSettingsFunction=None, setSettingFunction=None, 
                data_source_description = 'navpose_sensor',
                data_ref_description = 'data_reference',
                navpose_frame = 'sensor_frame', frame_nav = 'ENU',
                frame_altitude = 'WGS84', frame_depth = 'DEPTH',
                getNavPoseCb = None,
                max_navpose_update_rate = DEFAULT_UPDATE_RATE,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        if node_namespace is not None:
            self.namespace = nepi_sdk.create_namespace(node_namespace,'npx')
        else:
            self.namespace = nepi_sdk.create_namespace(self.node_namespace,'npx')
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
        self.msg_if.pub_info("Starting NPX Device IF Initialization Processes", log_name_list = self.log_name_list)  



        ##############################
        # Initialize Class 
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
        self.info_report.type = 'NPX'

        self.caps_report.device_name = self.device_name
        self.caps_report.device_path = self.path
        self.caps_report.device_node_name = self.node_name

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description
        self.tr_source_ref_description = data_ref_description
        

        if navpose_frame is not None:
          self.navpose_frame = navpose_frame

        if frame_nav is not None:
          self.frame_nav = frame_nav

        if frame_altitude is not None:
          self.frame_altitude = frame_altitude

        if frame_depth is not None:
          self.frame_depth = frame_depth
       

        # Probe the telemetry source once for its capability flags. The callback is
        # retained either way: a source that is not connected yet is not a broken
        # source, and disabling it here is what made a device whose bridge or link
        # comes up later report has_position = False for the life of the node.
        self.getNavPoseCb = getNavPoseCb
        if getNavPoseCb is not None:
            navpose_dict = None
            try:
                navpose_dict = getNavPoseCb()
            except Exception as e:
                self.msg_if.pub_warn("getNavPoseCb failed during initialization: " + str(e), log_name_list = self.log_name_list)

            if navpose_dict is None:
                # Source not up yet. Defer the NavPose IF to _updateNavPoseDictCb so
                # its capability flags get derived from real telemetry rather than
                # frozen at all-False here. See createNavPoseIf below.
                self.msg_if.pub_info("NavPose source not ready during initialization, deferring NavPose IF until first telemetry", log_name_list = self.log_name_list)
            else:
                self.deriveNavPoseCaps(navpose_dict)
        else:
            # No telemetry callback at all. Nothing to learn later, so the flags are
            # already final and the NavPose IF is created inline as it always was.
            self.navpose_caps_known = True


        # Create capabilities report
        self.caps_report.has_heading = self.has_heading
        self.caps_report.has_position = self.has_position
        self.caps_report.has_orientation = self.has_orientation
        self.caps_report.has_location = self.has_location      
        self.caps_report.has_altitude = self.has_altitude
        self.caps_report.has_depth =  self.has_depth
        self.caps_report.has_pan_tilt = self.has_pan_tilt


        self.caps_report.supports_updates = self.supports_updates

        self.caps_report.pub_rate_min_max = [self.MIN_PUB_RATE,max_navpose_update_rate]

        self.caps_report.data_products =  self.data_products_list


        # Initialize navpose_dict
        self.navpose_dict['navpose_frame'] = self.navpose_frame
        self.navpose_dict['frame_nav'] = self.frame_nav
        self.navpose_dict['frame_altitude'] = self.frame_altitude
        self.navpose_dict['frame_depth'] = self.frame_depth

        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description


        self.status_msg.has_location = self.has_location
        self.status_msg.has_heading = self.has_heading
        self.status_msg.has_orientation = self.has_orientation
        self.status_msg.has_position = self.has_position
        self.status_msg.has_altitude = self.has_altitude
        self.status_msg.has_depth = self.has_depth
        self.status_msg.has_pan_tilt = self.has_pan_tilt

        self.status_msg.navpose_frame = self.navpose_frame

        self.status_msg.source_frame_nav = self.frame_nav
        self.status_msg.source_frame_altitude = self.frame_altitude
        self.status_msg.source_frame_depth = self.frame_depth

   

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
            'update_rate': {
                'namespace': self.namespace,
                'factory_val': self.update_rate
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
            'navpose_capabilities_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': NPXCapabilitiesQuery,
                'req': NPXCapabilitiesQueryRequest(),
                'resp': NPXCapabilitiesQueryResponse(),
                'callback': self.capabilities_query_callback
            }
        }

        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DeviceNPXStatus,
                'qsize': 1,
                'latch': True
            }
        }


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_max_update_rate': {
                'namespace': self.namespace,
                'topic': 'set_max_update_rate',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setUpdateMaxRateCb,
                'callback_args': ()
            },
            'set_navpose_frame': {
                'namespace': self.namespace,
                'topic': 'set_navpose_frame',
                'msg': String,
                'qsize': 1,
                'callback': self._setNavPoseFrameCb,
                'callback_args': ()
            },
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

        
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

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




        # Setup Save Data IF Class ####################
        self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
        factory_data_rates = {}
        for d in self.data_products_list:
            factory_data_rates[d] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
        if 'navpose' in self.data_products_list:
            factory_data_rates['navpse'] = [1.0, 0.0, 100] 

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "npx",
            'add_node_name': True
            }

        sd_namespace = self.node_namespace
        self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                            factory_rate_dict = factory_data_rates,
                            factory_filename_dict = factory_filename_dict,
                            namespace = sd_namespace,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if,
                            node_if = self.node_if
                            )


        # Create the per-device 3D mount transform (where the camera is located/oriented).
        # User-settable and persisted; applied to each published navpose below.
        self.transform_if = Transform3DIF(namespace = self.namespace,
                            source_ref_description = '',
                            end_ref_description = self.node_name,
                            get_3d_transform_function = None,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if,
                            node_if = self.node_if
                            )

        # Create a NavPose IF, but only if the capability flags are actually known.
        # NavPoseIF decides its publisher set from these flags once, at construction,
        # and has no way to add a component later, so creating it before the telemetry
        # source has reported would lock in an all-False publisher set permanently.
        # When the source is not up yet, _updateNavPoseDictCb creates it on the first
        # successful read instead.
        if self.navpose_caps_known == True:
            self.createNavPoseIf()


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

        nepi_sdk.start_timer_process(0.1, self._updateNavPoseDictCb, oneshot = True)
        
        ###############################
        # Finish Initialization

        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)



  ###############################
  # Class Methods

  def check_ready(self):
      """Returns the current ready state of the NPX device interface.

      Returns:
          bool: True if the interface has completed initialization, False otherwise.
      """
      return self.ready

  def wait_for_ready(self, timeout = float('inf') ):
      """Blocks until the NPX device interface is ready or the timeout expires.

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


  def get_navpose_dict(self):
    """Returns the current navigation pose dictionary.

    The dictionary is updated periodically by the internal timer that calls the
    driver-supplied getNavPoseCb. It follows the nepi_nav.BLANK_NAVPOSE_DICT
    structure and includes has_* capability flags alongside the data fields.

    Returns:
        dict: The most recently published navpose data dictionary.
    """
    return self.navpose_dict

    ###############################
    # Class Private Methods
    ###############################


  def deriveNavPoseCaps(self,navpose_dict):
    # Derive the has_* capability flags from a telemetry dict and mirror them into
    # the caps report and the status message. Called exactly once per node life --
    # either from __init__ when the source answers immediately, or from
    # _updateNavPoseDictCb on the first successful read when it does not. The flags
    # cannot move after that, because NavPoseIF is created from them in the same
    # breath and its publisher set is fixed at its construction.
    self.has_location = navpose_dict['has_location']
    self.has_heading = navpose_dict['has_heading']
    self.has_orientation = navpose_dict['has_orientation']
    self.has_position = navpose_dict['has_position']
    self.has_altitude = navpose_dict['has_altitude']
    self.has_depth = navpose_dict['has_depth']
    self.has_pan_tilt = navpose_dict['has_pan_tilt']

    self.caps_report.has_location = self.has_location
    self.caps_report.has_heading = self.has_heading
    self.caps_report.has_orientation = self.has_orientation
    self.caps_report.has_position = self.has_position
    self.caps_report.has_altitude = self.has_altitude
    self.caps_report.has_depth = self.has_depth
    self.caps_report.has_pan_tilt = self.has_pan_tilt

    self.status_msg.has_location = self.has_location
    self.status_msg.has_heading = self.has_heading
    self.status_msg.has_orientation = self.has_orientation
    self.status_msg.has_position = self.has_position
    self.status_msg.has_altitude = self.has_altitude
    self.status_msg.has_depth = self.has_depth
    self.status_msg.has_pan_tilt = self.has_pan_tilt

    self.navpose_caps_known = True


  def createNavPoseIf(self):
    # Create the NavPose IF from the current has_* flags. Split out of __init__ so
    # the deferred path uses the same construction, with the same namespace and the
    # same arguments, as the immediate one.
    np_namespace = self.namespace
    self.navpose_if = NavPoseIF(namespace = np_namespace,
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        pub_navpose = True,
                        pub_location = self.has_location,
                        pub_heading = self.has_heading,
                        pub_orientation =  self.has_orientation,
                        pub_position = self.has_position,
                        pub_altitude = self.has_altitude,
                        pub_depth = self.has_depth,
                        pub_pan_tilt = self.has_pan_tilt,
                        save_data_if = self.save_data_if,
                        transform_namespace = self.transform_if.get_namespace(),
                        log_name = 'navpose',
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if)
    self.status_msg.navpose_topic = self.navpose_if.get_namespace()


  def _setUpdateMaxRateCb(self,msg):
    rate = msg.data
    min = self.NAVPOSE_UPDATE_RATE_OPTIONS[0]
    max = self.NAVPOSE_UPDATE_RATE_OPTIONS[1]
    if rate < min:
      rate = min
    if rate > max:
      rate = max
    self.update_rate = rate
    self.publish_status()
    if self.node_if is not None:
        self.node_if.set_param('update_rate',rate)


  def _setNavPoseFrameCb(self,msg):
    frame = msg.data
    if frame in nepi_nav.NAVPOSE_3D_FRAME_OPTIONS:
      self.navpose_frame = frame
      if self.node_if is not None:
        self.node_if.set_param('navpose_frame',frame)
      self.publish_status()
    else:
      self.msg_if.pub_warn("Ignoring invalid navpose_frame: " + str(frame) + " not in " + str(nepi_nav.NAVPOSE_3D_FRAME_OPTIONS), log_name_list = self.log_name_list)


  def publishStatus(self, do_updates=True):
      self._publishStatusCb(None)

  def initConfig(self):
      self.initCb()

  def initCb(self,do_updates = False):
      #self.msg_if.pub_info("Setting init values to param values", log_name_list = self.log_name_list)
      if self.node_if is not None:
        self.update_rate = self.node_if.get_param('update_rate')
        self.navpose_frame = self.node_if.get_param('navpose_frame')
      if do_updates == True:
        pass
      self.publish_status()

  def resetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.reset_params()
      if self.save_data_if is not None:
          self.save_data_if.reset()
      if self.settings_if is not None:
          self.settings_if.reset()
      if self.navpose_if is not None:
          self.navpose_if.reset()
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
      if self.navpose_if is not None:
          self.navpose_if.factory_reset()
      if do_updates == True:
        pass
      self.initCb(do_updates = True)

  ### Info callback
  def info_query_callback(self, _):
    """Service handler that returns the device info report.

    Returns:
        DeviceInfoQueryResponse: Response containing device name, path, node name,
            namespace, serial number, hardware version, software version, and type.
    """
    return self.info_report

  def capabilities_query_callback(self, _):
    """Service handler that returns the NPX capabilities report.

    Returns:
        NPXCapabilitiesQueryResponse: Response describing which navpose data types
            this device provides, including location, heading, orientation, position,
            altitude, depth, and pan/tilt, as well as supported update rate range
            and available data products.
    """
    return self.caps_report


  def _updateNavPoseDictCb(self,timer):
    navpose_dict = None
    if self.getNavPoseCb is not None:
        try:
            navpose_dict = self.getNavPoseCb()
        except Exception as e:
            self.msg_if.pub_warn("getNavPoseCb failed: " + str(e), throttle_s = 5.0, log_name_list = self.log_name_list)

    if self.navpose_caps_known == False:
        # The telemetry source was not up when this IF was constructed, so the
        # capability flags were never derived and the NavPose IF was not created.
        # Do both now, off the first read that returns something.
        if navpose_dict is None:
            self.msg_if.pub_warn("Waiting on NavPose source before creating NavPose IF", throttle_s = 10.0, log_name_list = self.log_name_list)
            nepi_sdk.start_timer_process(1.0, self._updateNavPoseDictCb, oneshot = True)
            return
        try:
            self.deriveNavPoseCaps(navpose_dict)
            self.createNavPoseIf()
        except Exception as e:
            # A source mid-handshake can return a partial dict. Stay unresolved and
            # retry on the next read rather than locking in half-derived flags or
            # letting the exception kill this timer loop for good.
            self.navpose_caps_known = False
            self.msg_if.pub_warn("Failed to create NavPose IF from telemetry, will retry: " + str(e), throttle_s = 10.0, log_name_list = self.log_name_list)
            nepi_sdk.start_timer_process(1.0, self._updateNavPoseDictCb, oneshot = True)
            return
        self.msg_if.pub_info("Created NavPose IF from first telemetry at " + str(self.status_msg.navpose_topic) + \
            " with has_location: " + str(self.has_location) + " has_heading: " + str(self.has_heading) + \
            " has_orientation: " + str(self.has_orientation) + " has_position: " + str(self.has_position) + \
            " has_altitude: " + str(self.has_altitude) + " has_depth: " + str(self.has_depth) + \
            " has_pan_tilt: " + str(self.has_pan_tilt), log_name_list = self.log_name_list)
        self.publish_status()

    if navpose_dict is None:
        navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    # publish navpose data

    if self.navpose_if is not None:
        # Stamp the operator-selected 3D frame, then apply the per-device mount transform
        navpose_dict['navpose_frame'] = self.navpose_frame
        transform_dict = None
        if self.transform_if is not None:
            transform_dict = self.transform_if.get_3d_transform_dict()
        self.navpose_dict = self.navpose_if.publish_navpose(navpose_dict, transform = transform_dict)

    # # save data if needed
    # timestamp = nepi_utils.get_time()
    # self.save_data_if.save('navpose',self.navpose_dict,timestamp)

    # set up next loop
    delay = float(1) / self.update_rate
    nepi_sdk.start_timer_process(delay, self._updateNavPoseDictCb, oneshot = True)



  def _publishStatusCb(self,timer):
    self.publish_status()

  def publish_status(self):
      """Builds and publishes the DeviceNPXStatus message to the status topic.

      Updates the device name and current update rate on the status message, then
      publishes it on the latched status publisher if the node interface is available.
      """
      self.status_msg.device_name = self.device_name
      self.status_msg.update_rate = self.update_rate

      if self.node_if is not None:
        self.node_if.publish_pub('status_pub', self.status_msg)

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    """Performs cleanup actions when the ROS node is shutting down.

    Logs a shutdown message via the message interface. Called by the node shutdown
    hook to allow orderly teardown of the device interface.
    """
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions", log_name_list = self.log_name_list)



