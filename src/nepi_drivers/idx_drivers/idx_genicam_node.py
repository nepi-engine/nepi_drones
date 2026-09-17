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
from nepi_sdk import nepi_img
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_drvs

from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'IDX_GENICAM' # Use in display menus
FILE_TYPE = 'NODE'

class GenicamCamNode:
    FACTORY_SETTINGS_OVERRIDES = dict( BalanceWhiteAuto = 'Continuous',
                                      ColorCorrectionMode = 'Auto',
                                      ExposureAuto = 'Continuous',
                                      GainAuto = 'Continuous')

 

    #Factory Control Values 
    FACTORY_CONTROLS = dict( 
    frame_id = 'sensor_frame' 
    )

    DEFAULT_CURRENT_FPS = 20 # Will be update later with actual
    
    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")

    idx_if = None

    init_settings_dict = dict()
    settings_dict = dict()

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
            self.driver_module = self.driver_file.split(".")[0]
            self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
            model = self.drv_dict['DEVICE_DICT']['model']
            serial_number = self.drv_dict['DEVICE_DICT']['serial_number']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        # import driver class fromn driver module
        self.msg_if.pub_info(model)
        self.msg_if.pub_info(serial_number)
        self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        
        if success:
            try:
                self.driver = self.driver_class(model=model, serial_number=serial_number)
            except Exception as e:
                # Only log the error every 30 seconds -- don't want to fill up log in the case that the camera simply isn't attached.
                self.msg_if.pub_warn("Failed to instantiate driver - " + str(e) + ")")
                sys.exit(-1)
                return
                
        ################################################
        genicam_cfg_file_mappings = nepi_sdk.get_param("~genicam_mappings", {})
        if not self.driver.isConnected():
           self.msg_if.pub_warn("Failed to connect to camera device")

        self.msg_if.pub_info(f"{self.node_name}: ... Connected!")

        self.img_lock = threading.Lock()
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
        self.msg_if.pub_info("... IDX interface running")
        self.logDeviceInfo()


        #nepi_sdk.start_timer_process(1.0, self.checkAndUpdateCb, oneshot=True)

        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Now start the node
        nepi_sdk.spin()


    def checkAndUpdateCb(self,timer):
        self.msg_if.pub_warn("Node Running")
        nepi_sdk.start_timer_process(5, self.checkAndUpdateCb, oneshot=True)



    #**********************
    # Sensor setting functions

    def initSettingsDict(self):
        init_settings_dict = dict()
        controls_dict = self.driver.getCameraControls()

        for setting_name in controls_dict.keys():
            setting_dict = dict()
            info = controls_dict[setting_name]
            # self.msg_if.pub_info("Got Init Camera Setting: " + str(info))
            setting_type = info['type']
            if setting_type == 'int':
                setting_type = 'Int'
            elif setting_type == 'float':
                setting_type = 'Float'
            elif setting_type == 'bool':
                setting_type = 'Toggle'
            elif setting_type == 'enum':
                # The retired cap-settings form called this 'Discrete'. The
                # controls contract has no Discrete type; a named option list
                # is a Selection.
                setting_type = 'Selection'
            else:
                setting_type = 'String'
            setting_dict['type'] = setting_type
            setting_current = str(info.get('value',''))
            
            if setting_type == 'Int':
                try:
                    setting_dict['default'] = int(float(setting_current))
                    setting_dict['bounds'] = [int(info['min']),int(info['max'])]
                except:
                    continue
            elif setting_type == 'Float':
                try:
                    setting_dict['default'] = float(setting_current)
                    setting_dict['bounds'] = [float(info['min']),float(info['max'])]
                except:
                    continue
            elif setting_type == 'Selection':
                try:
                    options = [str(option) for option in info['options']]
                    setting_dict['options'] = options
                    setting_dict['default'] = setting_current
                except:
                    continue
            elif setting_type == 'Toggle':
                setting_dict['default'] = (setting_current == True or setting_current == 'True' or setting_current == 'true')
            else:
                setting_dict['default'] = setting_current
            init_settings_dict[setting_name] = setting_dict

        # Add Resolution Setting
        try:
            [success,available_resolutions] = self.driver.getCurrentFormatAvailableResolutions()
            if success == True:
                options = []
                for res_dict in available_resolutions:
                    setting_option = str(res_dict['width']) + ":" + str(res_dict['height'])
                    if setting_option not in options:
                        options.append(setting_option)
                if len(options) > 0:
                    setting_dict = dict()
                    setting_dict['type'] = 'Selection'
                    setting_dict['options'] = options
                    [success,res_dict] = self.driver.getCurrentResolution()
                    setting_dict['default'] = str(res_dict['width']) + ":" + str(res_dict['height'])
                    init_settings_dict['Resolution'] = setting_dict
        except Exception as e:
            self.msg_if.pub_warn("Driver returned invalid resolution options: " + str(e))

        # Add Framerate Setting
        try:
            [success,framerates] = self.driver.getCurrentResolutionAvailableFramerates()
            if success == True and len(framerates) > 0:
                # Framerate stays a Float, as it has always been. The old cap
                # form carried the discrete rate list in an 'options' key that
                # a Float control has no place for, so the same list becomes
                # the control's bounds.
                rates = [float(rate) for rate in framerates]
                setting_dict = dict()
                setting_dict['type'] = 'Float'
                setting_dict['bounds'] = [min(rates),max(rates)]
                [success,framerate] = self.driver.getFramerate()
                setting_dict['default'] = round(float(framerate),2)
                self.current_fps = framerate
                init_settings_dict['Framerate'] = setting_dict
        except Exception as e:
            self.msg_if.pub_warn("Driver returned invalid framerate options: " + str(e))

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
        controls_dict = self.driver.getCameraControls()

        for setting_name in controls_dict.keys():
            if setting_name not in settings_dict.keys():
                continue
            info = controls_dict[setting_name]
            setting_current = str(info.get('value',''))
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
                [success,available_resolutions] = self.driver.getCurrentFormatAvailableResolutions()
                options = []
                for res_dict in available_resolutions:
                    setting_option = str(res_dict['width']) + ":" + str(res_dict['height'])
                    if setting_option not in options:
                        options.append(setting_option)
                if len(options) > 0:
                    settings_dict = nepi_controls.set_control_options(settings_dict, 'Resolution', options)
                [success,res_dict] = self.driver.getCurrentResolution()
                settings_dict = nepi_controls.set_control_value(settings_dict, 'Resolution',
                                    str(res_dict['width']) + ":" + str(res_dict['height']))
            except Exception as e:
                self.msg_if.pub_warn("Failed to refresh current resolution: " + str(e))

        # Refresh Framerate
        if 'Framerate' in settings_dict.keys():
            try:
                [success,framerates] = self.driver.getCurrentResolutionAvailableFramerates()
                if len(framerates) > 0:
                    rates = [float(rate) for rate in framerates]
                    settings_dict = nepi_controls.set_control_bounds(settings_dict, 'Framerate',
                                       [int(info['min']), int(info['max'])])
                [success,framerate] = self.driver.getFramerate()
                settings_dict = nepi_controls.set_control_value(settings_dict, 'Framerate', round(float(framerate),2))
                self.current_fps = framerate
            except Exception as e:
                self.msg_if.pub_warn("Failed to refresh current framerate: " + str(e))

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

        self.msg_if.pub_info("Update Setting: " + setting_str + " from:to value: " + str([cur_val,setting_value]))
        if setting_name != "Resolution" and setting_name != "Framerate":
            success, msg = self.driver.setCameraControl(setting_name,setting_value)
        elif setting_name == "Resolution":
            try:
                data = str(setting_value).split(":")
                [fr_success,framerate] = self.driver.getFramerate()
                res_dict = {'width': int(data[0]), 'height': int(data[1])}
                self.msg_if.pub_info("Updating Res with Res Dict: " + str(res_dict))
                success, msg = self.driver.setResolution(res_dict)
                # reset framerate if needed
                if success == True and 'Framerate' in self.settings_dict.keys():
                    nepi_sdk.sleep(1)
                    try:
                        self.driver.setFramerate(float(framerate))
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to restore framerate " + str(framerate) + " : " + str(e))
            except Exception as e:
                msg = "Resolution setting: " + str(setting_value) + " could not be parsed: " + str(e)
                self.msg_if.pub_warn(msg)
        elif setting_name == "Framerate":
            try:
                success, msg = self.driver.setFramerate(float(setting_value))
            except Exception as e:
                msg = "Framerate setting: " + str(setting_value) + " could not be parsed to float " + str(e)
                self.msg_if.pub_warn(msg)

        self.settings_dict = self.refreshSettingsDict()
        return success, msg, self.settings_dict

    #**********************
    # Node driver functions

    def logDeviceInfo(self):
        device_info_str = f"{self.node_name} info:\n"\
                + f"\tModel: {self.driver.model}\n"\
                + f"\tS/N: {self.driver.serial_number}\n"

        controls_dict = self.driver.getCameraControls()
        #for key in controls_dict.keys():
            #string = str(controls_dict[key])
            #self.msg_if.pub_info(key + string)
            
        _, fmt = self.driver.getCurrentFormat()
        device_info_str += f"\tCamera Output Format: {fmt}\n"

        _, res_dict = self.driver.getCurrentResolution()
        device_info_str += "\tCurrent Resolution: " + f'{res_dict["width"]}x{res_dict["height"]}' + "\n"

        if (self.driver.hasAdjustableResolution()):
            _, available_resolutions = self.driver.getCurrentFormatAvailableResolutions()
            device_info_str += "\tAvailable Resolutions:\n"
            for res in available_resolutions:
                device_info_str += "\t\t" + f'{res["width"]}x{res["height"]}' + "\n"

        if (self.driver.hasAdjustableFramerate()):
            _, available_framerates = self.driver.getCurrentResolutionAvailableFramerates()
            device_info_str += "\t" + f'Available Framerates (current resolution): {available_framerates}' + "\n"
        #self.msg_if.pub_info(device_info_str)

        
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


    def getColorImg(self):
        # Check for control framerate adjustment
        last_time = self.cl_img_last_time
        current_time = nepi_utils.get_time()

        # # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
        # if self.color_image_acquisition_running == False:
        #     ret, msg = self.driver.startImageAcquisition()
        #     if ret is False:
        #         self.img_lock.release()
        #         return ret, msg, None, None, None
        ret, msg = self.driver.startImageAcquisition()
        if ret is False:
            self.img_lock.release()
            return ret, msg, None, None, None

        need_data = False
        if last_time != None and self.idx_if is not None:
          fr_delay = float(1) / self.max_framerate
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
            self.img_lock.acquire()


            self.color_image_acquisition_running = True
            timestamp = None
            start = time.time()
            cv2_img, timestamp, ret, msg = self.driver.getImage()
            if cv2_img is None:
                ret = False
                msg = "Got None Image"
                return ret, msg, None, None, None
            stop = time.time()
            #print('GI: ', stop - start)
            if timestamp is None:
                timestamp = nepi_utils.get_time()
            self.img_lock.release()
            return ret, msg, cv2_img, timestamp, encoding
        
    def stopColorImg(self):
        self.img_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        ret,msg = self.driver.stopImageAcquisition()
        self.color_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None
        self.img_lock.release()
        return ret,msg
    

if __name__ == '__main__':
    node = GenicamCamNode()
