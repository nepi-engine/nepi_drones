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


import sys
import copy
import time
import math
import threading
import cv2

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_img
from nepi_sdk import nepi_controls

from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'IDX_V4L2' # Use in display menus
FILE_TYPE = 'NODE'

TEST_DRV_DICT = {
'type': 'IDX',
'group_id': 'None',
'path': '/opt/nepi/nepi_engine/lib/nepi_drivers',
'NODE_DICT': {
    'file_name': 'idx_v4l2_node.py',
    'class_name': 'V4l2CamNode',
},
'DRIVER_DICT': {
    'file_name': 'idx_v4l2_driver.py' ,
    'class_name':  'V4l2CamDriver'
},
'DEVICE_DICT': {'device_path': '/dev/video0'},
}

class V4l2CamNode:

    FACTORY_SETTINGS_OVERRIDES = dict( white_balance_temperature_auto = "True",
                                    focus_auto = "True" )
                                    
    DEFAULT_DEVICE_PATH = '/dev/video0'

    MJPG_CAMS = ['explorehd','miramar','boson']


    #Factory Control Values 
    FACTORY_CONTROLS = dict( 
    width_deg = 90,
    weight_deg = 60,    
    frame_id = 'sensor_frame' 
    )

 
    DEFAULT_CURRENT_FPS = 20 # Will be update later with actual

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")


    # Create threading locks, controls, and status
    img_lock = threading.Lock()
    color_image_acquisition_running = False
    cached_2d_color_image = None
    cached_2d_color_image_timestamp = None
    set_framerate = 0
    idx_if = None


    current_fps = 20
    cl_img_last_time = None


    max_framerate = 100

    last_brightness_setting = 50

    init_settings_dict = dict()
    settings_dict = dict()

    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
    drv_dict = dict()                          
    driver = None   
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
            self.msg_if.pub_warn("Nex_Dict: " + str(self.drv_dict))
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.driver_path = self.drv_dict['path']
            self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
            self.driver_module = self.driver_file.split('.')[0]
            self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        if self.device_path == "":
            #self.device_path = self.DEFAULT_DEVICE_PATH
            self.msg_if.pub_warn("No Device Path given")#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no No Device Path given")
            return
        # import driver class fromn driver module
        self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        if success:
            is_mjpg = False
            # Match MJPG_CAMS against the raw device_name (e.g. 'explorehd_213'),
            # not node_name, which is the alias (e.g. 'Vis_Zoom') and would never
            # match the camera-type keywords. Also check node_name as a fallback.
            for cam in self.MJPG_CAMS:
                if self.device_name.find(cam) != -1 or self.node_name.find(cam) != -1:
                    is_mjpg = True
                    break
            self.msg_if.pub_info("Launching driver with mjpg mode: " + str(is_mjpg))
            self.msg_if.pub_warn("device path: " + str(self.device_path))

            try:
                self.driver = self.driver_class(self.device_path, is_mjpg)
            except Exception as e:
                # Only log the error every 30 seconds -- don't want to fill up log in the case that the camera simply isn't attached.
                self.msg_if.pub_warn("Failed to instantiate driver: " + str(e) )
                sys.exit(-1)
        ################################################
        # Start node initialization
        
        if self.driver is None:
            self.msg_if.pub_warn("No Driver loaded")#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because No Driver loaded")
            return            
        if not self.driver.isConnected():
            self.msg_if.pub_warn("Failed to connect to camera device at " + str(self.device_path) +
                                 "; shutting down node instead of presenting a device that produces no images")
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because camera device not connected")
            return

        self.msg_if.pub_info("... Connected!")


        # Initialize controls
        self.factory_controls = self.FACTORY_CONTROLS
        self.current_controls = self.factory_controls # Updateded during initialization
        self.current_fps = self.DEFAULT_CURRENT_FPS # Should be updateded when settings read


        # Initialize settings
        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()
        

        # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
        self.msg_if.pub_info("Launching NEPI IDX () interface...")
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.idx_if = IDXDeviceIF(device_info = self.device_info_dict,
                                    data_source_description = 'camera',
                                    data_ref_description = 'camera_lense',
                                    getSettingsFunction=self.getSettingsFunction,
                                    setSettingFunction= self.setSettingFunction,
                                    factoryControls = self.factory_controls,
                                    setMaxFramerate =self.setMaxFramerate, 
                                    getFramerate = self.driver.getFramerate,
                                    getColorImage = self.getColorImg, 
                                    stopColorImageAcquisition = self.stopColorImg,
                                    data_products = ['color_image'])
                                    
        self.msg_if.pub_info(" " + " ... IDX interface running")

        # Update available IDX callbacks based on capabilities that the driver reports
        self.logDeviceInfo()
        time.sleep(1)
        self.getColorImg()
        # Now that all camera start-up stuff is processed, we can update the camera from the parameters that have been established
        time.sleep(1)

        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Now start the node
        nepi_sdk.spin()

    #**********************
    # Sensor setting functions

    def initSettingsDict(self):
        init_settings_dict = dict()
        device_controls_dict = self.driver.getCameraControls()

        ### Update Capabilities
        for setting_name in device_controls_dict.keys():
            setting_dict = dict()
            setting_current = None
            setting_dict['name'] = setting_name
            info = device_controls_dict[setting_name]
            #self.msg_if.pub_info("Got Init Camera Setting: " + str([setting_name,info]))
            setting_type = info['type']
            if setting_type == 'int':
                setting_type = 'Int'
            elif setting_type == 'float':
                setting_type = 'Float'
            elif setting_type == 'bool':
                setting_type = 'Toggle'
            elif setting_type == 'menu':
                setting_type = 'Menu'
            setting_dict['type'] = setting_type

            ###############
            setting_current = str(info['value'])
            if setting_type == 'Int':
                try:
                    setting_dict['default'] = int(setting_current)
                    setting_min = int(info['min'])
                    setting_max = int(info['max'])
                    setting_dict['bounds'] = [setting_min,setting_max]
                except:
                    pass
            if setting_type == 'Float':
                try:
                    setting_dict['default'] = float(setting_current)
                    setting_min = float(info['min'])
                    setting_max = float(info['max'])
                    setting_dict['bounds'] = [setting_min,setting_max]
                except:
                    pass

            elif setting_type == 'Toggle':
                setting_dict['default'] = (setting_current == True or setting_current == 'True' or setting_current == 'true')
            elif  setting_type == 'Selection' or setting_type == 'Selections' or setting_type == 'Menu':
                try:
                    setting_dict['default'] = int(setting_current)
                    legend = info['legend']
                    options = list(legend.keys())
                    setting_dict['options'] = options
                except:
                    pass
            else:
                setting_dict['default'] = str(setting_current)
            init_settings_dict[setting_name] = setting_dict    
        # Add Resolution Cap Settting
        try:
            [success,available_resolutions] = self.driver.getCurrentFormatAvailableResolutions()
            setting_dict = dict()
            setting_dict['type'] = 'Selection'
            options = []
            if len(available_resolutions) > 0:
                for res_dict in available_resolutions:
                    width = str(res_dict['width'])
                    height = str(res_dict['height'])
                    setting_option = (width + ":" + height)
                    if setting_option not in options:
                        options.append(setting_option)
                setting_dict['options'] = options
                setting_dict['name'] = 'resolution'

                [success,res_dict] = self.driver.getCurrentResolution()
                width = str(res_dict['width'])
                height = str(res_dict['height'])
                setting_value = (width + ":" + height)
                setting_dict['default'] = setting_value

                init_settings_dict['resolution'] = setting_dict


        except Exception as e:
            self.msg_if.pub_info(" " + "Driver returned invalid resolution options: " + str(e))
        # Add Framerate Cap setting_dict
        try:
            [success,framerates] = self.driver.getCurrentResolutionAvailableFramerates()
          
            setting_dict = dict()
            setting_dict['type'] = 'Selection'
            options = []
            if len(framerates) > 0:
                for rate in framerates:
                    setting_option = (str(round(rate,2)))
                    if setting_option not in options:
                        options.append(setting_option)
                setting_dict['options'] = options
                setting_dict['name'] = 'framerate'

                [success,framerate] = self.driver.getFramerate() 
                setting_dict['default'] = str(framerate)
                self.current_fps = framerate

                init_settings_dict['framerate'] = setting_dict
              
        except Exception as e:
            self.msg_if.pub_info(" " + "Driver returned invalid framerate options: " + str(e))

        self.init_settings_dict = init_settings_dict
        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        settings_dict_values = nepi_controls.get_controls_values_dict(settings_dict)
        self.msg_if.pub_warn("Initialized Settings: " + str(settings_dict_values))
        return settings_dict

    def refreshSettingsDict(self):
        settings_dict = copy.deepcopy(self.settings_dict)
        device_controls_dict = self.driver.getCameraControls()

        ### Update Capabilities
        for setting_name in device_controls_dict.keys():
            if setting_name in settings_dict.keys():
                ###############
                info = device_controls_dict[setting_name]
                #self.msg_if.pub_info("Got Refresh Camera Setting: " + str([setting_name,info]))
                setting_current = str(info['value'])
                setting_type = settings_dict[setting_name]['type']
                if setting_type == 'Int':
                    try:
                        settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, int(setting_current))
                        setting_min = int(info['min'])
                        setting_max = int(info['max'])
                        settings_dict = nepi_controls.set_control_bounds(settings_dict, setting_name, [setting_min,setting_max])
                    except:
                        pass
                elif setting_type == 'Float':
                    try:
                        settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, float(setting_current))
                        setting_min = float(info['min'])
                        setting_max = float(info['max'])
                        settings_dict = nepi_controls.set_control_bounds(settings_dict, setting_name, [setting_min,setting_max])
                    except:
                        pass
                elif setting_type == 'Toggle':
                    value = (setting_current == True or setting_current == 'True' or setting_current == 'true')
                    settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, value)
                elif  setting_type == 'Selection' or setting_type == 'Selections' or setting_type == 'Menu':
                    try:
                        settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, int(setting_current))
                        legend = info['legend']
                        options = list(legend.keys())
                        settings_dict = nepi_controls.set_control_options(settings_dict, setting_name, options)
                    except:
                        pass
                else:
                    settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, str(setting_current))
                #self.msg_if.pub_warn("Refreshed Current Controls: " + str([setting_name,settings_dict[setting_name]]))
        # Add Resolution Settting
        if 'resolution' in settings_dict.keys():
            try:
                setting_name = 'resolution'
                [success,available_resolutions] = self.driver.getCurrentFormatAvailableResolutions()
                options = []
                if len(available_resolutions) > 0:
                    for res_dict in available_resolutions:
                        width = str(res_dict['width'])
                        height = str(res_dict['height'])
                        setting_option = (width + ":" + height)
                        if setting_option not in options:
                            options.append(setting_option)
                    settings_dict = nepi_controls.set_control_options(settings_dict, setting_name, options)

                [success,res_dict] = self.driver.getCurrentResolution()
                width = str(res_dict['width'])
                height = str(res_dict['height'])
                setting_value = (width + ":" + height)
                settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, setting_value)
            except Exception as e:
                self.msg_if.pub_info(" " + "Driver returned invalid resolution options: " + str(e))
        # Add Framerate setting_dict
        if 'framerate' in settings_dict.keys():
            try:
                setting_name = 'framerate'
                [success,framerates] = self.driver.getCurrentResolutionAvailableFramerates()
                options = []
                if len(framerates) > 0:
                    for rate in framerates:
                        setting_option = (str(round(rate,2)))
                        if setting_option not in options:
                            options.append(setting_option)
                    settings_dict = nepi_controls.set_control_options(settings_dict, setting_name, options)
                [success,framerate] = self.driver.getFramerate() 
                settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, str(framerate))
                self.current_fps = framerate                 
            except Exception as e:
                self.msg_if.pub_info(" " + "Driver returned invalid framerate options: " + str(e))
        #self.msg_if.pub_warn("Refreshed Current Controls: " + str(settings_dict))
        settings_dict_values = nepi_controls.get_controls_values_dict(settings_dict)
        self.msg_if.pub_warn("Refreshed Current Settings: " + str(settings_dict_values))
        return settings_dict
        
    



    def getSettingsFunction(self):
        return self.settings_dict


    def setSettingFunction(self,setting_name, setting_value):
        setting_str = setting_name + ":" + str(setting_value)
        #self.msg_if.pub_warn("Got Setting Update Request" + setting_str)
        success = True
        msg = 'Success'
        found_setting = False
        if setting_name in self.settings_dict.keys():
                found_setting = True
                cur_val = nepi_controls.get_control_value(self.settings_dict, setting_name)
                if str(cur_val) != str(setting_value):
                    self.msg_if.pub_warn("Update Setting:" + setting_str + " to: " + str(cur_val))
                    needs_update = True
                    if setting_name != "resolution" and setting_name != "framerate":
                            
                            success, msg = self.driver.setCameraControl(setting_name,setting_value)
                            # if success:
                            #     msg = ("Setting Updated " + setting_str)
                            # else:
                            #     msg = (" FAILED to Update Setting: " + setting_str + " : " + str(success) + " : " + str(msg))
                            # self.msg_if.pub_warn(msg)
                    elif setting_name == "resolution":
                        data = data.split(":")
                        try:
                            [success,framerate] = self.driver.getFramerate()
                            [success,brightness] = self.driver.getCameraControl('brightness')
                            width = int(data[0])
                            height = int(data[1])
                            res_dict = {'width': width, 'height': height}
                            success, msg = self.driver.setResolution(res_dict)
                            if success:
                                # reset framerate if needed
                                if 'framerate' in self.settings_dict.keys():
                                    nepi_sdk.sleep(1)
                                    try:
                                        framerate = float(framerate)
                                        ret = self.driver.setFramerate(framerate)
                                        # if ret[0]:
                                        #     self.msg_if.pub_warn("Updated framerate: " + str(framerate) )
                                        # else:
                                        #     self.msg_if.pub_warn("Failed to update framerate: " + str(framerate) + " : " + ret[1])
                                    except Exception as e:
                                        self.msg_if.pub_warn("Failed to update Framerate setting to: " + str(framerate) + " : " + str(e))

                                # if 'brightness' in self.settings_dict.keys():
                                #     nepi_sdk.sleep(1)
                                #     try:
                                #         brightness = int(brightness)
                                #         ret = self.driver.setCameraControl('brightness', brightness)
                                #         # if ret[0]:
                                #         #     self.msg_if.pub_warn("Updated brightness: " + str(brightness) )
                                #         # else:
                                #         #     self.msg_if.pub_warn("Failed to update brightness: " + str(brightness) + " : " + ret[1])
                                #     except Exception as e:
                                #         self.msg_if.pub_warn("Failed to update Framerate setting to: " + str(brightness) + " : " + str(e))

            

                                # ## Force updates for remaining settings
                                # cur_settings = self.getSettings()
                                # for setting_name in cur_settings.keys():
                                        
                                #         setting = cur_settings[setting_name]
                                #         setting_str = str(setting)
                                #         try:
                                #             if setting_name != "resolution" and setting_name != "framerate":
                                #                 nepi_sdk.sleep(1)

                                #                 [setting_name, setting_type, data] = nepi_controls.get_data_from_setting(setting)
                                #                 ret = self.driver.setCameraControl(setting_name,data)
                                #                 if ret[0]:
                                #                     self.msg_if.pub_warn("Updated setting: " + setting_name + " : "  + str(data) )
                                #                 else:
                                #                     self.msg_if.pub_warn("Failed to update setting: " + setting_name + " : "  + setting_str  + " : "  + str(data) + " : " + ret[1])
                                #         except Exception as e:
                                #             self.msg_if.pub_warn("Failed to update setting: " + setting_name + " : "  + setting_str  + " : "  + str(data) + " : " + str(e))
                        except Exception as e:
                            self.msg_if.pub_info("Resoluton setting: " + data + " could not be parsed to float " + str(e))                               
                    elif setting_name == "framerate":
                        try:
                            framerate = float(data)
                            success, msg = self.driver.setFramerate(framerate)
                        except Exception as e:
                            self.msg_if.pub_info("Framerate setting: " + data + " could not be parsed to float " + str(e))
                    else:
                        success = False
                        needs_update = False
                    if needs_update == True:
                        self.settings_dict = self.refreshSettingsDict()

        if found_setting is False:
            success = False
            msg = (self.node_name  + " Setting name" + setting_str + " is not supported")        
        # settings_values_dict = nepi_controls.get_controls_values_dict(self.settings_dict)
        # self.msg_if.pub_warn("Returning Updated Settings: "  + str(settings_values_dict) )           
        return success, msg, self.settings_dict


    #**********************
    # IDX driver functions

    def logDeviceInfo(self):
        device_info_str = self.node_name + " info:\n"
        device_info_str += "\tDevice Path: " + self.driver.device_path + "\n"

        device_controls_dict = self.driver.getCameraControls()
        for key in device_controls_dict.keys():
            string = str(device_controls_dict[key])
            self.msg_if.pub_info(key + " " + string)
        



        _, format = self.driver.getCurrentFormat()
        device_info_str += "\tCamera Output Format: " + format + "\n"

        _, resolution_dict = self.driver.getCurrentResolution()
        device_info_str += "\tCurrent Resolution: " + str(resolution_dict['width']) + 'x' + str(resolution_dict['height']) + "\n"

        if (self.driver.hasAdjustableResolution()):
            _, available_resolutions = self.driver.getCurrentFormatAvailableResolutions()
            device_info_str += "\tAvailable Resolutions:\n"
            for res in available_resolutions:
                device_info_str += "\t\t" + str(res["width"]) + 'x' + str(res["height"]) + "\n"

        if (self.driver.hasAdjustableFramerate()):
            _, available_framerates = self.driver.getCurrentResolutionAvailableFramerates()
            device_info_str += "\tAvailable Framerates (current resolution): " + str(available_framerates) + "\n"
        
        self.msg_if.pub_info(device_info_str)

        
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


    def setDriverCameraControl(self, control_name, value):
        return self.driver.setScaledCameraControl(control_name, value)
    
    # Good base class candidate - Shared with ONVIF
    def getColorImg(self):
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
        msg = ''
        if need_data == False:
          return False, "Waiting for Timer", None, None, None  # Return None data
        else:
            self.cl_img_last_time = current_time

            encoding = "bgr8"


            self.img_lock.acquire()
            # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
            ret, msg = self.driver.startImageAcquisition()
            if ret is False:
                # Debug aid: surface why streaming could not start (results in no image at all).
                self.msg_if.pub_warn("Image acquisition failed to start for " + str(self.device_path) + ": " + str(msg), throttle_s = 5.0)
                self.img_lock.release()
                return ret, msg, None, None, None
            self.color_image_acquisition_running = True
            # Debug aid: log the negotiated format/resolution/framerate once, so when no
            # image appears we can confirm how the camera actually opened (e.g. YUYV vs MJPG).
            if getattr(self, '_acq_settings_logged', False) == False:
                self._acq_settings_logged = True
                try:
                    settings = self.driver.getCurrentVideoSettings()
                    self.msg_if.pub_info("Color image acquisition started for " + str(self.device_path) +
                                         " (mjpg=" + str(self.driver.mjpg) + ", settings=" + str(settings) + ")")
                except Exception as e:
                    self.msg_if.pub_debug("Could not read negotiated video settings for " + str(self.device_path) + ": " + str(e))
            timestamp = None
            start = time.time()

            cv2_img, timestamp, ret, msg = self.driver.getImage()
            stop = time.time()
            #print('GI: ', stop - start)
            if ret is False:
                # Debug aid: this is the "camera on but no image" case -- say exactly why.
                self.msg_if.pub_warn("No color image from " + str(self.device_path) + ": " + str(msg), throttle_s = 20.0)
                self.img_lock.release()
                return ret, msg, None, None, None
            if timestamp is None:
                timestamp = nepi_utils.get_time()
            self.img_lock.release()
            # Debug aid: confirm frames are actually decoding and their dimensions
            # (if frames flow here but the viewer is black, the issue is downstream).
            if cv2_img is not None:
                self.msg_if.pub_debug("Got color image " + str(getattr(cv2_img, 'shape', None)) + " from " + str(self.device_path), throttle_s = 10.0)
            return ret, msg, cv2_img, timestamp, encoding
    
    # Good base class candidate - Shared with ONVIF
    def stopColorImg(self):
        self.img_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        ret,msg = self.driver.stopImageAcquisition()
        self.color_image_acquisition_running = False
        self.cached_2d_color_image = None
        self.cached_2d_color_image_timestamp = None
        self.img_lock.release()
        return ret,msg

        
if __name__ == '__main__':
    node = V4l2CamNode()
