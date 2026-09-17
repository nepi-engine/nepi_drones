#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import sys
import threading
import cv2
import open3d as o3d
import numpy as np
import math
import copy



pyzed_folder = '/home/nepi/.local/lib/python3.8/site-packages'
sys.path.insert(0, pyzed_folder)
import pyzed.sl as sl

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
# nepi_sdk import nepi_img
# from nepi_sdk import nepi_pc
#from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_controls


from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'IDX_ZED' # Use in display menus
FILE_TYPE = 'NODE'

TEST_DRV_DICT = {
'type': 'IDX',
'group_id': 'None',
'usr_cfg_path': '/mnt/nepi_storage/user_cfg',
'NODE_DICT': {
    'file_name': 'idx_zed_node.py',
    'class_name': 'ZedCamNode',
},
'DRIVER_DICT': {
    'file_name': 'idx_zed_driver.py' ,
    'class_name':  'ZedCamDriver'
},
'DEVICE_DICT': {'zed_type': 'zed2','resolution': 'VGA'},
}



class ZedCamNode(object):
    CHECK_INTERVAL_S = 3.0
    CAL_SRC_PATH = "/usr/local/zed/settings"
    CAL_BACKUP_PATH = "/mnt/nepi_storage/user_cfg/cals"



    #CAP_SETTINGS = nepi_controls.NONE_CAP_SETTINGS
    CAP_SETTINGS = dict(
      #pub_frame_rate = {"type":"Float","name":"pub_frame_rate","options":["0.1","15"]},
      #depth_confidence = {"type":"Int","name":"depth_confidence","options":["0","100"]},
      #depth_texture_conf = {"type":"Int","name":"depth_texture_conf","options":["0","100"]},
      pointcloud_max_rate = {"type":"Int","name":"pointcloud_max_rate","options":["1","30"]},
      pointcloud_rez_ratio = {"type":"Float","name":"pointcloud_rez_ratio","options":["0","1"]},
      brightness = {"type":"Int","name":"brightness","options":["0","8"]},
      contrast ={"type":"Int","name":"contrast","options":["0","8"]},
      hue = {"type":"Int","name":"hue","options":["0","11"]},
      saturation ={"type":"Int","name":"saturation","options":["0","8"]},
      sharpness ={"type":"Int","name":"sharpness","options":["0","8"]},
      gamma ={"type":"Int","name":"gamma","options":["1","9"]},
      auto_exposure_gain = {"type":"Toggle","name":"auto_exposure_gain"},
      gain = {"type":"Int","name":"gain","options":["0","100"]},
      exposure = {"type":"Int","name":"exposure","options":["0","100"]},
      auto_whitebalance = {"type":"Toggle","name":"auto_whitebalance"},
      whitebalance_temperature = {"type":"Int","name":"whitebalance_temperature","options":["2800","6500"]}
    )


    CAP_ZED_DICT = dict(
      brightness = sl.VIDEO_SETTINGS.BRIGHTNESS,
      contrast = sl.VIDEO_SETTINGS.CONTRAST,
      hue = sl.VIDEO_SETTINGS.HUE,
      saturation = sl.VIDEO_SETTINGS.SATURATION,
      sharpness = sl.VIDEO_SETTINGS.SHARPNESS,
      gamma = sl.VIDEO_SETTINGS.GAMMA,
      gain = sl.VIDEO_SETTINGS.GAIN,
      exposure = sl.VIDEO_SETTINGS.EXPOSURE,
      auto_exposure_gain = sl.VIDEO_SETTINGS.AEC_AGC_ROI,
      whitebalance_temperature = sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE,
      auto_whitebalance = sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO


    )

    FACTORY_SETTINGS_OVERRIDES = dict( )
    
    #Factory Control Values 
    FACTORY_CONTROLS = dict( 
    start_range_ratio = 0.0, 
    stop_range_ratio = 1.0,
    min_range_m = 0.0,
    max_range_m = 20.0,
    width_deg = 110,
    height_deg = 70,
    frame_id = 'sensor_frame' 
    )

    data_source_description = 'stereo_camera'
    data_ref_description = 'left_camera_lense'

    ZED_MIN_RANGE_M_OVERRIDES = { 'zed': .2, 'zedm': .15, 'zed2': .2, 'zedx': .2} 
    ZED_MAX_RANGE_M_OVERRIDES = { 'zed':  15, 'zedm': 15, 'zed2': 20, 'zedx': 15} 
    ZED_WIDTH_DEG_OVERRIDES = { 'zed': 110, 'zedm': 110, 'zed2': 110, 'zedx': 110} 
    ZED_HEIGHT_DEG_OVERRIDES = { 'zed':  70, 'zedm': 70, 'zed2': 70, 'zedx': 80} 

    zed = None
    zed_type = 'zed'
    runtime_parameters = None
    zed_pose = None

    # Shared single-grab state. All three data-product threads (color/depth/
    # pointcloud) drive the SAME camera handle. Previously each thread called
    # self.zed.grab() independently, which (a) contended on the non-thread-safe
    # handle and (b) consumed a separate frame per product, so the collective
    # throughput was capped near cam_fps/num_products and the slowest product
    # (pointcloud) was starved to ~1 Hz. _grabSharedFrame() performs at most one
    # grab per frame period under zed_grab_lock; all products then retrieve from
    # that same frame, so each can run up to the camera framerate.
    zed_grab_lock = threading.Lock()
    last_grab_time = None
    last_grab_status = False
    grab_interval = None  # seconds between shared grabs; set to 1/framerate in __init__

    # Create shared class variables and thread locks
    
    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")
    
    color_img_acquire = False
    color_img_msg = None
    color_img_last_stamp = None
    color_img_lock = threading.Lock()
    depth_map_acquire = False
    depth_map_msg = None
    depth_map_last_stamp = None
    depth_map_lock = threading.Lock() 
    depth_img_acquire = False   
    depth_img_msg = None
    depth_img_last_stamp = None
    depth_img_lock = threading.Lock() 
    pc_acquire = False   
    pc_msg = None
    pc_last_stamp = None
    pc_lock = threading.Lock()
    pc_img_acquire = False
    pc_img_msg = None
    pc_img_last_stamp = None
    pc_img_lock = threading.Lock()

    gps_msg = None
    odom_msg = None
    heading_msg = None

    # Rendering Initialization Values
    render_img_width = 1280
    render_img_height = 720
    render_background = [0, 0, 0, 0] # background color rgba
    render_fov = 60 # camera field of view in degrees
    render_center = [3, 0, 0]  # look_at target
    render_eye = [-5, -2.5, 0]  # camera position
    render_up = [0, 0, 1]  # camera orientation


    img_renderer = None
    img_renderer_mtl = None
    
    idx_if = None

    current_fps = 100
    cl_img_last_time = None
    dm_data_last_time = None
    di_img_last_time = None
    pc_img_last_time = None
    pc_last_time = None

    zed_type = 'zed'
    resolution = 'VGA'
    resolution_wh = [640,480]
    framerate = 15
    data_products = []

    max_framerate = 100
    # Pointcloud rate cap. Previously hard-defaulted to 1 Hz, which throttled the
    # pointcloud ~15x below the camera fps even though depth (same grab+retrieve
    # path) ran at full rate. Overridden to the camera framerate in __init__ so the
    # pointcloud defaults to the same full-speed behavior as color/depth. grab()
    # blocks at the camera fps, so this can never over-grab the hardware.
    max_pointcloud_framerate = 30

    # Pointcloud-only spatial decimation. Applied to the shared XYZRGBA grab
    # in getPointcloud() BEFORE the Open3D conversion, so color image and
    # depth map keep the full camera capture resolution untouched — only the
    # pointcloud gets sparser. Vector3dVector's per-point construction cost
    # scales with point count, so striding by N cuts that cost by ~N^2.
    # Derived from pointcloud_rez_ratio via setPointRezRatio() - this class
    # default is only a placeholder until __init__ calls it.
    pointcloud_rez_ratio = 0.5

    init_settings_dict = dict()
    settings_dict = dict()

    navpose_enabled = False
    navpose_pub_rate = 10
    navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
 
    nav_published = False

    print_pc_stats = True

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
            #self.msg_if.pub_warn("Nex_Dict: " + str(self.drv_dict))
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.zed_type = self.drv_dict['DEVICE_DICT']['zed_type']
            self.resolution = self.drv_dict['DEVICE_DICT']['resolution']
            self.framerate = self.drv_dict['DEVICE_DICT']['framerate']
            self.data_products = self.drv_dict['DEVICE_DICT']['data_products']
            self.navpose_enabled = self.drv_dict['DEVICE_DICT']['navpose_enabled']
            self.navpose_pub_rate = self.drv_dict['DEVICE_DICT']['navpose_pub_rate']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        ################################################
        # Try to restore camera calibration files from
        [success,files_copied,files_not_copied] = nepi_utils.copy_files_from_folder(self.CAL_BACKUP_PATH,self.CAL_SRC_PATH)
        if success:
          if len(files_copied) > 0:
            strList = str(files_copied)
            self.msg_if.pub_info("Restored zed cal files: " + strList)
        else:
          self.msg_if.pub_info("Failed to restore zed cal files")

    


        # Create a Camera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.sensors_required = True 
        res_dict = {
            'HD2K': sl.RESOLUTION.HD2K,
            'HD1080': sl.RESOLUTION.HD1080,
            'HD720': sl.RESOLUTION.HD720,
            'VGA': sl.RESOLUTION.VGA
        }

        res_wh_dict = {
            'HD2K': [2048,1080],
            'HD1080': [1920,1080],
            'HD720': [1280,720],
            'VGA': [640,480]
        }



        if self.resolution not in res_dict.keys():
           self.resolution = 'VGA'

        resolution = res_dict[self.resolution]
        self.resolution_wh = res_wh_dict[self.resolution]

        init_params.camera_resolution = resolution #sl.RESOLUTION.AUTO # Use HD720 opr HD1200 video mode, depending on camera type.
        init_params.camera_fps = self.framerate #30  # Set fps at 30
        # Use the ROS coordinate system (X forward, Y left, Z up). NEPI's pointcloud
        # render camera + rotate/tilt/zoom controls are built for X-fwd/Z-up; Y-up
        # here rendered the cloud side-on and rolled 90 deg.
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.coordinate_units = sl.UNIT.METER  # NEPI range/clip/render pipeline is meters-based



        # Open the camera. On a fresh hot-plug the ZED SDK often reports
        # CAMERA NOT DETECTED because the USB device is not fully enumerated yet,
        # so retry a few times with a short settle delay before giving up.
        OPEN_MAX_TRIES = 5
        OPEN_RETRY_DELAY_SEC = 2.0
        err = None
        for attempt in range(OPEN_MAX_TRIES):
            err = self.zed.open(init_params)
            if err == sl.ERROR_CODE.SUCCESS:
                break
            self.msg_if.pub_warn("Camera Open (attempt " + str(attempt + 1) + "/" +
                                 str(OPEN_MAX_TRIES) + ") : " + str(err))
            # A failed open() leaves the Camera object holding partial state, and
            # calling open() on it again returns INVALID FUNCTION CALL regardless of
            # what is actually wrong with the hardware. That masks the real error
            # from attempt 1 and makes every later retry meaningless, so reset the
            # object before retrying. close() is safe on a camera that never opened.
            self.zed.close()
            if nepi_sdk.is_shutdown():
                return
            nepi_sdk.sleep(OPEN_RETRY_DELAY_SEC)
        if err != sl.ERROR_CODE.SUCCESS:
            self.msg_if.pub_warn("Camera Open failed after " + str(OPEN_MAX_TRIES) + " attempts : " + str(err))
            # Cleanly exit so drivers_mgr can re-discover and relaunch the node
            # once the camera is available. Without the return the node would fall
            # through into a half-open state and flood the log with grab errors.
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because Not Able to Connect to Zed Camera")
            return

        self.msg_if.pub_info("Zed Camera Connected!")

        self.tracking_parameters = sl.PositionalTrackingParameters()
        err = self.zed.enable_positional_tracking(self.tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            self.msg_if.pub_warn("Positional Tracking enable failed : " + str(err))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because Not Able to Connect to Zed Camera")
            return

        self.runtime_parameters = sl.RuntimeParameters()
        self.zed_pose = sl.Pose()


        #############################
        if self.navpose_enabled == True:
            getNavPoseCb = self.getNavPoseDict
        else:
            getNavPoseCb = None

       # Initialize controls
        self.factory_controls = self.FACTORY_CONTROLS
        # Apply OVERRIDES
        if self.zed_type in self.ZED_MIN_RANGE_M_OVERRIDES:
          self.factory_controls['min_range_m'] = self.ZED_MIN_RANGE_M_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_MAX_RANGE_M_OVERRIDES:
          self.factory_controls['max_range_m'] = self.ZED_MAX_RANGE_M_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_WIDTH_DEG_OVERRIDES:
          self.factory_controls['width_deg'] = self.ZED_WIDTH_DEG_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_HEIGHT_DEG_OVERRIDES:
          self.factory_controls['height_deg'] = self.ZED_HEIGHT_DEG_OVERRIDES[self.zed_type]
        
        self.current_controls = self.factory_controls # Updateded during initialization
        self.current_fps = self.framerate # Should be updateded when settings read

        # Default the pointcloud rate cap to the camera framerate so the pointcloud
        # runs as fast as the hardware/pipeline allow (matching color/depth), instead
        # of the old fixed 1 Hz. Set before initSettingsDict() below so the
        # pointcloud_max_rate factory/default value reflects the full camera rate. Users
        # can still throttle it at runtime via the pointcloud_max_rate setting.
        self.max_pointcloud_framerate = self.framerate


        # Shared-grab pacing: one camera grab per frame period, reused by every
        # enabled data product (see _grabSharedFrame / getColorImage etc.).
        try:
            self.grab_interval = 1.0 / float(self.framerate)
        except (ZeroDivisionError, TypeError, ValueError):
            self.grab_interval = 0.0

        # Initialize settings
        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()
          


        # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
        self.msg_if.pub_info("Launching NEPI IDX () interface...")
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.idx_if = IDXDeviceIF(device_info = self.device_info_dict,
                                    data_products =  self.data_products,
                                    data_source_description = self.data_source_description,
                                    data_ref_description = self.data_ref_description,
                                    getSettingsFunction=self.getSettingsFunction,
                                    setSettingFunction=self.setSettingFunction,
                                    factoryControls = self.factory_controls,
                                    setMaxFramerate =self.setMaxFramerate, 
                                    getFramerate = self.getFramerate,
                                    setRangeRatio = self.setRangeRatio,
                                    getColorImage = self.getColorImage, 
                                    stopColorImageAcquisition = self.stopColorImage,
                                    getDepthMap = self.getDepthMap, 
                                    stopDepthMapAcquisition = self.stopDepthMap,
                                    getPointcloud = self.getPointcloud, 
                                    stopPointcloudAcquisition = self.stopPointcloud,                                  
                                    getNavPoseCb = getNavPoseCb, 
                                    navpose_update_rate = self.navpose_pub_rate
                                    )
        self.msg_if.pub_info("... IDX interface running")

        # Update available IDX callbacks based on capabilities that the driver reports
        self.logDeviceInfo()

        # Now that all camera start-up stuff is processed, we can update the camera from the parameters that have been established

        # Try to backup camera calibration files
        [success,files_copied,files_not_copied] = nepi_utils.copy_files_from_folder(self.CAL_SRC_PATH,self.CAL_BACKUP_PATH)
        if success:
          if len(files_copied) > 0:
            strList = str(files_copied)
            self.msg_if.pub_info("Backed up zed cal files: " + strList)
        else:
          self.msg_if.pub_info("Failed to back up up zed cal files")


        ##########################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Now start zed node check process
        self.attempts = 0
        #nepi_sdk.start_timer_process((1), self.checkZedStatusCb)
        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()

    
    
    
    def checkZedStatusCb(self,timer):
        timestamp = None
        try:
          timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
        except:
          pass
        
        if timestamp is None:
          self.msg_if.pub_info("Failed to get timestamp from zed camera")
          nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because Zed Node not running")

      



    #**********************
    # Sensor setting functions

    def initSettingsDict(self):
      init_settings_dict = dict()
      for setting_name in self.CAP_SETTINGS.keys():
        cap_setting = self.CAP_SETTINGS[setting_name]
        setting_dict = dict()
        setting_dict['type'] = cap_setting['type']
        # The retired cap-settings form carried an Int/Float control's min and
        # max in an 'options' pair. The controls contract calls that 'bounds'.
        if 'options' in cap_setting.keys():
          try:
            if cap_setting['type'] == 'Int':
              setting_dict['bounds'] = [int(cap_setting['options'][0]),int(cap_setting['options'][1])]
            elif cap_setting['type'] == 'Float':
              setting_dict['bounds'] = [float(cap_setting['options'][0]),float(cap_setting['options'][1])]
            else:
              setting_dict['options'] = [str(option) for option in cap_setting['options']]
          except Exception as e:
            self.msg_if.pub_warn("Invalid bounds for setting: " + setting_name + " : " + str(e))

        value = self.readSettingValue(setting_name)
        if value is None:
          continue
        setting_dict['default'] = value
        init_settings_dict[setting_name] = setting_dict

      # Apply factory setting overrides
      for setting_name in self.FACTORY_SETTINGS_OVERRIDES.keys():
        if setting_name in init_settings_dict.keys():
          init_settings_dict[setting_name]['default'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]

      self.init_settings_dict = init_settings_dict
      settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
      settings_dict_values = nepi_controls.get_controls_values_dict(settings_dict)
      self.msg_if.pub_info("Initialized Settings: " + str(settings_dict_values))
      return settings_dict


    def refreshSettingsDict(self):
      settings_dict = copy.deepcopy(self.settings_dict)
      for setting_name in settings_dict.keys():
        value = self.readSettingValue(setting_name)
        if value is None:
          continue
        settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, value)
      return settings_dict


    def readSettingValue(self, setting_name):
      # Returns the device's live value for one setting, already typed for the
      # control, or None if the device could not report it.
      setting_type = self.CAP_SETTINGS[setting_name]['type'] if setting_name in self.CAP_SETTINGS.keys() else 'String'
      value = None
      if setting_name in self.CAP_ZED_DICT.keys():
        zed_name = setting_name.upper()
        zed_setting = self.CAP_ZED_DICT[setting_name]
        try:
          value = self.zed.get_camera_settings(zed_setting)[1]
          if zed_name == "WHITEBALANCE_AUTO" or zed_name == "AEC_AGC_ROI":
            value = (value == 1)
        except Exception as e:
          self.msg_if.pub_warn("Failed to get setting: " + str(zed_name) + " : " + str(e))
          return None
      elif setting_name == "pointcloud_max_rate":
        value = self.max_pointcloud_framerate
      elif setting_name == "pointcloud_rez_ratio":
        value = self.pointcloud_rez_ratio
      else:
        return None

      try:
        if setting_type == 'Int':
          value = int(value)
        elif setting_type == 'Float':
          value = float(value)
        elif setting_type == 'Toggle':
          value = (value == True)
      except Exception as e:
        self.msg_if.pub_warn("Failed to convert setting: " + setting_name + " : " + str(e))
        return None
      return value


    def getSettingsFunction(self):
      return self.settings_dict


    def setSettingFunction(self,setting_name, setting_value):
      setting_str = setting_name + ":" + str(setting_value)
      success = False
      msg = ""
      if setting_name not in self.settings_dict.keys():
        msg = (self.node_name + " Setting name " + setting_str + " is not supported")
        return False, msg, self.settings_dict

      data = setting_value
      if setting_name in self.CAP_ZED_DICT.keys():
        zed_name = setting_name.upper()
        zed_setting = self.CAP_ZED_DICT[setting_name]
        if zed_name == "WHITEBALANCE_AUTO" or zed_name == "AEC_AGC_ROI":
          if data == True:
            data = 1
          elif data == False:
            data = 0
        try:
          self.zed.set_camera_settings(zed_setting, data)
          success = True
          msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)
        except Exception as e:
          msg = "Failed to set setting: " + str(zed_name) + " : " + str(e)
          self.msg_if.pub_warn(msg)
      elif setting_name == "pointcloud_max_rate":
        success, msg = self.setPointcloudFramerate(setting_value)
      elif setting_name == "pointcloud_rez_ratio":
        success, msg = self.setPointRezRatio(setting_value)
      else:
        msg = (self.node_name  + " Setting name " + setting_str + " is not supported")

      self.settings_dict = self.refreshSettingsDict()
      return success, msg, self.settings_dict

    #**********************
    # Zed camera data callbacks


    def getNavPoseDict(self):
        
        success = False
        navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        navpose_dict['has_orientation'] = True
        navpose_dict['time_oreantation'] = nepi_utils.get_time()

        rpy = self.getOrientation()
        if rpy is not None:
              
            navpose_dict['roll_deg'] = rpy[0]
            navpose_dict['pitch_deg'] = rpy[1]
            navpose_dict['yaw_deg'] = rpy[2]



            # Relative Position Meters in selected 3d frame (x,y,z) with x forward, y right/left, and z up/down
            xyz = self.getPosition()
            if xyz is not None:
              navpose_dict['x_m'] = xyz[0] * 1000
              navpose_dict['y_m'] = xyz[1] * 1000
              navpose_dict['z_m'] = xyz[2] * 1000
              navpose_dict['has_position'] = True
              navpose_dict['time_position'] = nepi_utils.get_time()

              success = True

              # if self.nav_published == False:
              #    self.msg_if.pub_warn("nav navpose_dict befor: " + str(navpose_dict))

              #navpose_dict = nepi_nav.convert_navpose_ned2edu(navpose_dict)
              # self.msg_if.pub_warn("roll: " + str(navpose_dict['roll_deg']) + " | pitch: " + str(navpose_dict['pitch_deg']) + " | yaw: " + str(navpose_dict['yaw_deg']))


              if self.nav_published == False:
                self.nav_published = True
                self.msg_if.pub_warn("nav navpose_dict after: " + str(navpose_dict))

        if success == False:
           navpose_dict = None
        return navpose_dict
    
    def getOrientation(self):
        rpy = None
        # return rpy
        orientation_pose = sl.Pose()
        # Route pose grabs through the shared lock so the navpose path does not
        # contend with the color/depth/pointcloud threads on the single ZED handle.
        with self.zed_grab_lock:
            if self._grabSharedFrame():
              # Get camera pose
              self.zed.get_position(orientation_pose, sl.REFERENCE_FRAME.WORLD)

              # Get orientation as a quaternion
              orientation = orientation_pose.get_orientation()
              # You can use the quaternion values (orientation.get()[0], etc.)
              
              # Get orientation directly as Euler angles (roll, pitch, yaw)
              # Angles are in radians by default. Use get_roll_pitch_yaw() to get them in a list/vector.
              roll_pitch_yaw = orientation_pose.get_euler_angles() 
              roll_deg = round(math.degrees(roll_pitch_yaw[2])*-1, 3)
              pitch_deg = round(math.degrees(roll_pitch_yaw[0])*-1)
              yaw_deg = round(math.degrees(roll_pitch_yaw[1])*-1)

              rpy = [roll_deg, pitch_deg, yaw_deg]
              #self.msg_if.pub_warn("roll deg: " + str(roll_deg) + " | pitch deg: " + str(pitch_deg) + " | yaw deg: " + str(yaw_deg))
        return rpy
    
    def getPosition(self):
      position = None
      position_pose = sl.Pose()

      # Route pose grabs through the shared lock (see getColorImage) so the
      # navpose path does not contend with the data-product threads.
      with self.zed_grab_lock:
        if self._grabSharedFrame():
          # Get the pose of the left eye of the camera with reference to the world frame
          # Get the pose of the camera relative to the world frame
          state = self.zed.get_position(position_pose, sl.REFERENCE_FRAME.WORLD)
          # Display translation and timestamp
          py_translation = sl.Translation()
          tx = round(position_pose.get_translation(py_translation).get()[0], 3)
          ty = round(position_pose.get_translation(py_translation).get()[1], 3)
          tz = round(position_pose.get_translation(py_translation).get()[2], 3)

          position = [tx, ty, tz]
      return position


    #**********************
    # IDX driver functions

    def logDeviceInfo(self):
        device_info_str = self.node_name + " info:\n"
        self.msg_if.pub_info(device_info_str)
        self.msg_if.pub_info(str(self.device_info_dict))

     
    def setMaxFramerate(self, rate):
        if rate is None:
            return False, 'Got None Max Framerate'
        if rate < 1:
            rate = 1
        if rate > 100:
            rate = 100
        self.max_framerate = rate
        #print('Set FR Mode: ' +  str(self.current_controls["max_framerate"]))
        status = True
        err_str = ""
        return status, err_str
    
    def setPointcloudFramerate(self, rate):
        if rate is None:
            return False, 'Got None Max Framerate'
        if rate < 1:
            rate = 1
        # Allow up to 30 Hz (the max selectable camera framerate in idx_zed_params.yaml).
        # The old ceiling of 10 held the pointcloud below the camera fps. grab() blocks
        # at the actual camera fps, so a value above the running fps simply tracks it.
        if rate > 30:
            rate = 30
        self.max_pointcloud_framerate = rate
        status = True
        err_str = ""
        self.print_pc_stats = True
        return status, err_str

      
    def setPointRezRatio(self, ratio):
        if ratio is None:
            return False, 'Got None Pointcloud Rez Ratio'
        ratio = nepi_utils.check_ratio(ratio)
        
        self.pointcloud_rez_ratio = ratio

        status = True
        err_str = ""
        self.print_pc_stats = True
        return status, err_str

    def setRangeRatio(self, min_ratio, max_ratio):
        if min_ratio > 1:
            min_ratio = 1
        elif min_ratio < 0:
            min_ratio = 0
        self.current_controls["start_range_ratio"] = min_ratio
        if max_ratio > 1:
            max_ratio = 1
        elif max_ratio < 0:
            max_ratio = 0
        if min_ratio < max_ratio:
          self.current_controls["stop_range_ratio"] = max_ratio
          status = True
          err_str = ""
        else:
          status = False
          err_str = "Invalid Range Window"
        self.print_pc_stats = True
        return status, err_str


 
    def getFramerate(self):
       return int(self.framerate)


    def _grabSharedFrame(self):
        # Perform at most one camera grab per frame period, shared across the
        # color/depth/pointcloud product threads. MUST be called while holding
        # self.zed_grab_lock. Whichever product thread runs first in a given
        # frame period does the grab; the others reuse that same frame via their
        # own retrieve_* calls, so no product is starved by the others each
        # grabbing (and consuming) a separate frame from the single 15 fps stream.
        now = nepi_utils.get_time()
        interval = self.grab_interval if self.grab_interval is not None else 0.0
        if self.last_grab_time is None or (now - self.last_grab_time) >= interval:
            self.last_grab_status = (self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS)
            if self.last_grab_status:
                self.last_grab_time = now
        return self.last_grab_status


    # Good base class candidate - Shared with ONVIF
    def getColorImage(self):
      status = False
      msg = ""
      cv2_img = None
      timestamp = None
      encoding = 'bgr8'
      # Check for control framerate adjustment
      last_time = self.cl_img_last_time
      current_time = nepi_utils.get_time()
      
      need_data = False
      if last_time != None and self.idx_if is not None:
        fr_delay = float(1) / self.max_framerate
        timer = current_time - last_time
        if timer > fr_delay:
          need_data = True
      else:
        need_data = True

      #need_data = True
      # Get and Process Data if Needed
      if need_data == False:
        return False, "Waiting for Timer", None, None, None  # Return None data
      else:
        # Retrieve the left view from the shared grab (see _grabSharedFrame).
        # Hold the lock only for the camera access; do the color conversion
        # afterwards so other product threads aren't blocked.
        raw_img = None
        with self.zed_grab_lock:
            if self._grabSharedFrame():
                zed_img = sl.Mat()
                self.zed.retrieve_image(zed_img, sl.VIEW.LEFT)
                raw_img = zed_img.get_data()
                timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
                self.cl_img_last_time = nepi_utils.get_time()
        if raw_img is not None:
            status = True
            cv2_img = cv2.cvtColor(raw_img, cv2.COLOR_BGRA2BGR)
        return status, msg, cv2_img, timestamp, encoding

      
    # Good base class candidate - Shared with ONVIF
    def stopColorImage(self):
        self.stop_color_img = True
        # self.color_img_lock.acquire()
        # self.color_img_sub.unregister()
        # self.color_img_sub = None
        # self.color_img_msg = None
        # self.color_img_lock.release()
        self.msg_if.pub_warn("Stoped Color Image Acquire")
        ret = True
        msg = "Success"
        return ret,msg
    
    def getDepthMap(self):
      status = False
      msg = ""
      np_depth_map = None
      encoding = '32FC1'
      # Check for control framerate adjustment
      last_time = self.dm_data_last_time
      current_time = nepi_utils.get_time()
      
      need_data = False
      if last_time != None and self.idx_if is not None:
        fr_delay = float(1) / self.max_framerate
        timer = current_time - last_time
        if timer > fr_delay:
          need_data = True
      else:
        need_data = True

      #need_data = True
      # Get and Process Data if Needed
      if need_data == False:
        return False, "Waiting for Timer", None, None, None  # Return None data
      else:
        # Retrieve the DEPTH measure from the shared grab (see _grabSharedFrame).
        # Hold the lock only for the camera access; do the numpy conversion
        # afterwards so other product threads aren't blocked.
        zed_depth_map = None
        with self.zed_grab_lock:
            if self._grabSharedFrame():
                zed_depth = sl.Mat()
                self.zed.retrieve_measure(zed_depth, sl.MEASURE.DEPTH, sl.MEM.CPU)
                zed_depth_map = zed_depth.get_data()
                timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
                self.dm_data_last_time = nepi_utils.get_time()
        if zed_depth_map is not None:
            status = True
            zed_depth_map[np.isinf(zed_depth_map)] = np.nan
            #print('Zed Depth Map Min Max: ' + str([np.nanmin(zed_depth_map),np.nanmax(zed_depth_map)]) )
            # The ZED is configured in METER units for the pointcloud render pipeline
            # (see init_params.coordinate_units above), so this DEPTH measure comes back
            # in meters. The NEPI depth_map data product and its colorizer expect
            # millimeters (nepi_img.npDepthMap_to_cv2ColorImg scales the range bounds by
            # 1e3), so convert meters -> millimeters here. Without this, every depth value
            # falls below min_range_m*1e3 and the depth map renders as a flat single color.
            np_depth_map = (np.array(zed_depth_map, dtype=np.float32)) * 1000.0 # meters -> mm; replaces nan values
        return status, msg, np_depth_map, timestamp, encoding


    def stopDepthMap(self):
        self.stop_depth_map = True

        # self.depth_map_lock.acquire()

        # self.depth_map_sub.unregister()
        # self.depth_map_sub = None

        # self.depth_map_msg = None
        # self.depth_map_lock.release()
        self.msg_if.pub_warn("Stopped Depthmap Acquire")
        ret = True
        msg = "Success"
        return ret,msg

    def getPointcloud(self): 
        start_time = nepi_utils.get_time()
        acquire_dict = dict()
        status = False
        msg = ""    
        frame_id = "sensor frame"
        o3d_pc = None
        timestamp = None
        # Check for control framerate adjustment
        last_time = self.pc_last_time
        current_time = nepi_utils.get_time()
        
        need_data = False
        if last_time != None and self.idx_if is not None:
          max_framerate = min(self.max_framerate, self.max_pointcloud_framerate)
          fr_delay = float(1) / max_framerate
          timer = current_time - last_time
          if timer > fr_delay:
            need_data = True
        else:
          need_data = True

        #need_data = True
        # Get and Process Data if Needed
        if need_data == False:
          return False, "Waiting for Timer", None, None, None  # Return None data
        else:
          # Retrieve the XYZRGBA measure from the shared grab (see
          # _grabSharedFrame), holding the lock ONLY for the camera access. The
          # Open3D construction below is by far the most expensive per-frame work
          # of the three products, so it runs OUTSIDE the lock — otherwise it
          # would block the color/depth threads' grabs and drag every product
          # down to the pointcloud's rate.
          zed_pc = None
          with self.zed_grab_lock:
              acquire_dict['start'] = round(nepi_utils.get_time() - start_time, 3)
              if self._grabSharedFrame():
                  # Retrieve the point cloud into this call's own Mat (the local
                  # stays alive for the whole method, so its buffer remains valid
                  # after the lock is released even as other threads grab again).
                  res_scale =  0.1 + 0.9 * self.pointcloud_rez_ratio
                  custom_res = sl.Resolution(int(self.resolution_wh[0] * res_scale), int(self.resolution_wh[1] * res_scale)) 
                  self.point_cloud = sl.Mat(custom_res.width,custom_res.height)
                  acquire_dict['create'] = round(nepi_utils.get_time() - start_time, 3)

                  self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, custom_res)
                  acquire_dict['retrieve'] = round(nepi_utils.get_time() - start_time, 3)

                  # Get the point cloud data as a numpy array
                  zed_pc = self.point_cloud.get_data()
                  timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
                  self.pc_last_time = nepi_utils.get_time()
                  acquire_dict['grab'] = round(nepi_utils.get_time() - start_time, 3)
          if zed_pc is not None:
              status = True
              # Decimate spatially before the Open3D conversion (see
              # pointcloud product reads zed_pc, so color image and depth map
              # are unaffected.
              # Derive the integer stride getPointcloud() applies to the shared XYZRGBA
              # grab from this ratio: ratio 1.0 -> stride 1 (full res), 0.5 -> stride 2,
              # 0.3 -> stride 3, etc. (approximate fraction of full resolution kept per axis).
              stride = max(1, round(1.0 / (0.1 + self.pointcloud_rez_ratio * 0.9)))
              if stride > 1:
                  zed_pc = zed_pc[::stride, ::stride, :]
              acquire_dict['reduce'] = round(nepi_utils.get_time() - start_time, 3)

              # Extract the XYZ data
              xyz_data = zed_pc[:, :, :3].reshape(-1,3)
              acquire_dict['reshape'] = round(nepi_utils.get_time() - start_time, 3)

              # Create an Open3D point cloud
              o3d_pc = o3d.geometry.PointCloud()
              o3d_pc.points = o3d.utility.Vector3dVector(xyz_data)
              acquire_dict['convert'] = round(nepi_utils.get_time() - start_time, 3)

              # Unpack the packed-float RGBA 4th channel into per-point RGB colors.
              # ZED stores color as a float32 whose bytes are R,G,B,A (little-endian);
              # reinterpret the bits as uint32 and split out the channels. If red and
              # blue look swapped in the rendered image, swap the r and b lines below
              # (packed byte order can vary by ZED SDK build).
              rgba_u32 = np.ascontiguousarray(zed_pc[:, :, 3]).reshape(-1).view(np.uint32)
              r = (rgba_u32 & 0x000000FF).astype(np.float64)
              g = ((rgba_u32 & 0x0000FF00) >> 8).astype(np.float64)
              b = ((rgba_u32 & 0x00FF0000) >> 16).astype(np.float64)
              o3d_pc.colors = o3d.utility.Vector3dVector(np.stack([r, g, b], axis=1) / 255.0)
              acquire_dict['color'] = round(nepi_utils.get_time() - start_time, 3)

              if self.print_pc_stats == True:
                 self.msg_if.pub_warn("Pointcloud Acquire times: " + str(acquire_dict), throttle_s = 5)
              self.print_pc_stats = False

          return status, msg, o3d_pc, timestamp, frame_id


    
    def stopPointcloud(self):
      self.stop_pointcloud = True

      # ZED grabs the pointcloud directly (no ROS subscriber), so there is no
      # pc_sub/pc_msg to tear down here — mirror stopColorImage/stopDepthMap.
      # self.pc_lock.acquire()
      # self.pc_sub.unregister()
      # self.pc_sub = None
      # self.pc_msg = None
      # self.pc_lock.release()
      self.msg_if.pub_warn("Stopped Pointcloud Acquire")
      ret = True
      msg = "Success"
      return ret,msg


    def cleanup_actions(self):
      self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
      try:
        self.zed.close()
        self.msg_if.pub_warn("Closed zed sdk connection")
      except Exception as e:
        self.msg_if.pub_warn("Failed to close zed sdk connection" + str(e))
        
if __name__ == '__main__':
    node = ZedCamNode()
