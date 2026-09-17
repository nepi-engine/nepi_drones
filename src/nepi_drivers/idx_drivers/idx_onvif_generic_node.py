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
import sys
import copy
import time
import math
import threading
import cv2

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_controls

from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'IDX_ONVIF_GENERIC' # Use in display menus
FILE_TYPE = 'NODE'


class OnvifCamNode:

    FACTORY_SETTINGS_OVERRIDES = dict(WhiteBalance_Mode = "AUTO",
                                      Exposure_Mode = "AUTO",
                                      Resolution = "2560:1440" )

 
    #Factory Control Values 
    FACTORY_CONTROLS = dict( 
    width_deg = 90,
    weight_deg = 60, 
    frame_id = 'sensor_frame' 
    )

    DEFAULT_CURRENT_FPS = 20 # Will be update later with actual

    init_settings_dict = dict()
    settings_dict = dict()
    
    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")


    idx_if = None

    current_fps = 20
    cl_img_last_time = None


    max_framerate = 100
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
            self.driver_path = self.drv_dict['path']
            self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
            self.driver_module = self.driver_file.split('.')[0]
            self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return       
      

        # Require the camera connection parameters to have been set
        if not nepi_sdk.has_param('~credentials/username'):
            self.msg_if.pub_warn("Missing credentials/username parameter... cannot start")
            return
        if not nepi_sdk.has_param('~credentials/password'):
            self.msg_if.pub_warn("Missing credentials/password parameter... cannot start")
            return
        if not nepi_sdk.has_param('~network/host'):
            self.msg_if.pub_warn("Missing network/host parameter... cannot start")
            return
                
        username = str(nepi_sdk.get_param('~credentials/username'))
        password = str(nepi_sdk.get_param('~credentials/password'))
        host = str(nepi_sdk.get_param('~network/host'))
        
        # Allow a default for the port, since it is part of onvif spec.
        onvif_port = nepi_sdk.get_param('~network/port', 80)
        nepi_sdk.set_param('~/network/port', onvif_port)


        self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        driver_constructed = False
        if success:
            attempts = 0
            while not nepi_sdk.is_shutdown() and driver_constructed == False and attempts < 5 and not nepi_sdk.is_shutdown():
                try:
                    self.driver = self.driver_class(username, password, host, onvif_port)
                    driver_constructed = True
                    self.msg_if.pub_info("ONVIF_NODE: Driver constructed")
                except Exception as e:
                    self.msg_if.pub_info("ONVIF_NODE: Failed to construct driver " + self.driver_module + " with exception: " + str(e))
                    time.sleep(1)
                attempts += 1 
        if driver_constructed == False:
            nepi_sdk.signal_shutdown("Shutting down Onvif node " + self.node_name + ", unable to connect to driver")
        else:
            ################################################
            self.msg_if.pub_info("... Connected!")
            self.dev_info = self.driver.getDeviceInfo()
            self.logDeviceInfo()        


            # Establish the URI indices (from ONVIF "Profiles") for the two image streams.
            # If these aren't the same, encoder param adjustments (resolution and framerate)
            # will only affect the first one.... so
            # TODO: Consider a scheme for adjusting parameters for separate streams independently
            # or in lock-step. Not sure if the uri_index and encoder_index have the same meaning
            self.img_uri_index = nepi_sdk.get_param('~/img_uri_index', 0)
            nepi_sdk.set_param('~/img_uri_index', self.img_uri_index)

            # Create threading locks for each URI index (currently just 1) to provide threadsafety
            self.img_uri_lock = threading.Lock()
            self.color_image_acquisition_running = False
            self.cached_2d_color_frame = None
            self.cached_2d_color_frame_timestamp = None


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
            self.msg_if.pub_info(" ... IDX interface running")
            # Now that all camera start-up stuff is processed, we can update the camera from the parameters that have been established

            # Now start the node
            nepi_sdk.spin()


    #**********************
    # Sensor setting functions

    def getRtspUrl(self):
        onvif_port = str(nepi_sdk.get_param('~network/port', 80))
        onvif_address = str(nepi_sdk.get_param('~network/host',""))
        url = "http://" + onvif_address #+ ":" + onvif_port
        username = str(nepi_sdk.get_param('~credentials/username'))
        password = str(nepi_sdk.get_param('~credentials/password'))
        return url, username, password


    def initSettingsDict(self):
        init_settings_dict = dict()
        try:
            controls_dict = self.driver.getCameraControls()
        except Exception as e:
            self.msg_if.pub_warn("Failed to get camera controls (camera may be disconnected): " + str(e))
            controls_dict = dict()

        for setting_name in controls_dict.keys():
            info = controls_dict[setting_name]
            setting_dict = dict()
            if 'type' in info:
                setting_type = info['type']
                if setting_type == 'int':
                    setting_type = 'Int'
                elif setting_type == 'float':
                    setting_type = 'Float'
                elif setting_type == 'bool':
                    setting_type = 'Toggle'
                elif setting_type == 'discrete':
                    # The retired cap-settings form called this 'Discrete'.
                    # The controls contract calls a named option list a Selection.
                    setting_type = 'Selection'
                setting_dict['type'] = setting_type
                setting_current = str(info['value'])
                if setting_current == 'Not Supported':
                    continue
                if setting_type == 'Int':
                    try:
                        setting_dict['bounds'] = [int(info['min']),int(info['max'])]
                        setting_dict['default'] = int(float(setting_current))
                    except:
                        continue
                elif setting_type == 'Float':
                    try:
                        setting_dict['bounds'] = [float(info['min']),float(info['max'])]
                        setting_dict['default'] = float(setting_current)
                    except:
                        continue
                elif setting_type == 'Selection':
                    try:
                        setting_dict['options'] = [str(option) for option in info['options']]
                        setting_dict['default'] = setting_current
                    except:
                        continue
                elif setting_type == 'Toggle':
                    setting_dict['default'] = (setting_current == 'True' or setting_current == 'true')
                else:
                    setting_dict['default'] = setting_current
                init_settings_dict[setting_name] = setting_dict
            elif 'value' in info:
                setting_value = info['value']
                if setting_value != "Not Supported":
                    setting_dict['type'] = 'String'
                    setting_dict['default'] = str(setting_value)
                    init_settings_dict[setting_name] = setting_dict

        # Add Resolution Setting
        try:
            [success,resolutions,encoder_cfg] = self.driver.getAvailableResolutions()
            if success:
                options = []
                for res_dict in resolutions:
                    setting_option = str(res_dict['Width']) + ":" + str(res_dict['Height'])
                    if setting_option not in options:
                        options.append(setting_option)
                if len(options) > 0:
                    setting_dict = dict()
                    setting_dict['type'] = 'Selection'
                    setting_dict['options'] = options
                    [success,res_dict] = self.driver.getResolution()
                    setting_dict['default'] = str(res_dict['Width']) + ":" + str(res_dict['Height'])
                    init_settings_dict['Resolution'] = setting_dict
        except Exception as e:
            self.msg_if.pub_warn(" " + "Driver returned invalid resolution options: " + str(e))

        # Add Framerate Setting
        try:
            [success,framerates,encoder_cfg] = self.driver.getFramerateRange()
            if success:
                setting_dict = dict()
                setting_dict['type'] = 'Int'
                setting_dict['bounds'] = [int(framerates['Min']),int(framerates['Max'])]
                [success,framerate] = self.driver.getFramerate()
                setting_dict['default'] = int(round(float(framerate)))
                self.current_fps = framerate
                init_settings_dict['Framerate'] = setting_dict
        except Exception as e:
            self.msg_if.pub_warn(" " + "Driver returned invalid framerate options: " + str(e))

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
        try:
            controls_dict = self.driver.getCameraControls()
        except Exception as e:
            self.msg_if.pub_warn("Failed to get camera controls (camera may be disconnected): " + str(e))
            return settings_dict

        for setting_name in controls_dict.keys():
            if setting_name not in settings_dict.keys():
                continue
            info = controls_dict[setting_name]
            if 'value' not in info:
                continue
            setting_current = str(info['value'])
            if setting_current == 'Not Supported':
                continue
            setting_type = settings_dict[setting_name]['type']
            try:
                if setting_type == 'Int':
                    settings_dict = nepi_controls.set_control_bounds(settings_dict, setting_name,
                                        [int(info['min']), int(info['max'])])
                    settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, int(float(setting_current)))
                elif setting_type == 'Float':
                    settings_dict = nepi_controls.set_control_bounds(settings_dict, setting_name,
                                        [int(info['min']), int(info['max'])])
                    settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, float(setting_current))
                elif setting_type == 'Selection':
                    settings_dict = nepi_controls.set_control_options(settings_dict, setting_name,
                                        [str(option) for option in info['options']])
                    settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, setting_current)
                else:
                    settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, setting_current)
            except Exception as e:
                self.msg_if.pub_debug("Failed to refresh setting " + setting_name + " : " + str(e))

        # Refresh Resolution
        if 'Resolution' in settings_dict.keys():
            try:
                [success,res_dict] = self.driver.getResolution()
                settings_dict = nepi_controls.set_control_value(settings_dict, 'Resolution',
                                    str(res_dict['Width']) + ":" + str(res_dict['Height']))
            except Exception as e:
                self.msg_if.pub_warn("Failed to get resolution (camera may be disconnected): " + str(e))

        # Refresh Framerate
        if 'Framerate' in settings_dict.keys():
            try:
                [success,framerate] = self.driver.getFramerate()
                settings_dict = nepi_controls.set_control_value(settings_dict, 'Framerate', int(round(float(framerate))))
                self.current_fps = framerate
            except Exception as e:
                self.msg_if.pub_warn("Failed to get framerate (camera may be disconnected): " + str(e))

        return settings_dict


    def getSettingsFunction(self):
        return self.settings_dict


    def setSettingFunction(self,setting_name, setting_value):
        setting_str = setting_name + ":" + str(setting_value)
        success = False
        msg = 'Success'
        if setting_name not in self.settings_dict.keys():
            msg = (self.node_name + " Setting name " + setting_str + " is not supported")
            return False, msg, self.settings_dict

        cur_val = nepi_controls.get_control_value(self.settings_dict, setting_name)
        if str(cur_val) == str(setting_value):
            return True, 'Already set', self.settings_dict

        if setting_name != "Resolution" and setting_name != "Framerate":
            success, msg = self.driver.setCameraControl(setting_name,setting_value)
            if success:
                msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
        elif setting_name == "Resolution":
            try:
                data_split = str(setting_value).split(":")
                width = int(data_split[0])
                height = int(data_split[1])
            except Exception as e:
                msg = "Resolution setting: " + str(setting_value) + " could not be parsed to int " + str(e)
                self.msg_if.pub_info(msg)
                return False, msg, self.settings_dict
            try:
                [fr_success,framerate] = self.driver.getFramerate()
                res_dict = {'Width': width, 'Height': height}
                success, msg = self.driver.setResolution(res_dict)
                # reset framerate if needed
                if success == True and 'Framerate' in self.settings_dict.keys():
                    nepi_sdk.sleep(1)
                    try:
                        self.driver.setFramerate(float(framerate))
                        self.msg_if.pub_info("Updated Framerate: " + str(framerate))
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to restore framerate " + str(framerate) + " : " + str(e))
            except Exception as e:
                msg = "setResolution function failed " + str(e)
                self.msg_if.pub_info(msg)
        elif setting_name == "Framerate":
            try:
                success, msg = self.driver.setFramerate(int(setting_value))
            except Exception as e:
                msg = "Framerate setting: " + str(setting_value) + " could not be parsed to int " + str(e)
                self.msg_if.pub_info(msg)

        self.settings_dict = self.refreshSettingsDict()
        return success, msg, self.settings_dict

    #**********************
    # Node driver functions

    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["HardwareId"] + "\n"
        self.msg_if.pub_info(dev_info_string)
    
        controls_dict = self.driver.getCameraControls()
        for key in controls_dict.keys():
            string = str(controls_dict[key])
            self.msg_if.pub_info(key + " " + string)
                
        
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


    def getFramerate(self):
        adj_fps =   nepi_img.adjust_framerate_ratio(self.current_fps,self.framerate_ratio)
        return adj_fps
    
    def getColorImg(self):
        # Check for control framerate adjustment
        last_time = self.cl_img_last_time
        current_time = nepi_utils.get_time()

        need_data = False
        if last_time != None and self.idx_if is not None:
          fr_delay = 1.0/self.max_framerate
          timer = current_time - last_time
          if timer > fr_delay:
            need_data = True
        else:
          need_data = True


        # Get and Process Data if Needed
        if need_data == False:
          return False, "Waiting for Timer", None, None, None  # Return None data
        else:
            self.cl_img_last_time = current_time

            encoding = "bgr8"
            self.img_uri_lock.acquire()
            # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
            ret, msg = self.driver.startImageAcquisition(uri_index = self.img_uri_index)
            if ret is False:
                self.img_uri_lock.release()
                return ret, msg, None, None, None
            self.color_image_acquisition_running = True
            timestamp = None
            start = nepi_utils.get_time()
            cv2_img, timestamp, ret, msg = self.driver.getImage(uri_index = self.img_uri_index)
            stop = nepi_utils.get_time()
            #print('GI: ', stop - start)
            if ret is False:
                self.img_uri_lock.release()
                return ret, msg, None, None, None
            if timestamp is None:
                timestamp = nepi_utils.get_time()  
            self.img_uri_lock.release() 
            return ret, msg, cv2_img, timestamp, encoding
        
    def stopColorImg(self):
        self.img_uri_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        ret,msg = self.driver.stopImageAcquisition(uri_index = self.img_uri_index)
        self.color_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None
        self.img_uri_lock.release()
        return ret,msg
    
if __name__ == '__main__':
	node = OnvifCamNode()

            


        

