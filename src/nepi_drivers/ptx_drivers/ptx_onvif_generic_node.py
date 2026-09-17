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

from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_nav

from nepi_api.device_if_ptx import PTXActuatorIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'PTX_ONVIF_GENERIC' # Use in display menus
FILE_TYPE = 'NODE'



class OnvifPanTiltNode:
    MAX_POSITION_UPDATE_RATE = 10

    DEFAULT_DRIVER_PATHS = ["/opt/nepi/nepi_engine/lib/nepi_drivers/"]




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

    last_pan_ratio_onvif = 0
    last_tilt_ratio_onvif = 0

    current_position = [0.0,0.0]
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
            while not nepi_sdk.is_shutdown() and driver_constructed == False and attempts < 5:
                try:
                    self.driver = self.driver_class(username, password, host, onvif_port)
                    driver_constructed = True
                    self.msg_if.pub_info("Driver constructed")
                except Exception as e:
                    self.msg_if.pub_info("Failed to construct driver: " + self.driver_module + "with exception: " + str(e))
                    nepi_sdk.sleep(1)
                attempts += 1 
        if driver_constructed == False:
            nepi_sdk.signal_shutdown("Shutting down Onvif node " + self.node_name + ", unable to connect to driver")
        else:
            ################################################
            self.msg_if.pub_info("... Connected!")
            self.dev_info = self.driver.getDeviceInfo()
            self.logDeviceInfo()

                    


            # Must pass a capabilities structure to ptx_interface constructor
            ptx_callback_names = dict()
            ptx_capabilities_dict = {}

            # Now check with the driver if any of these PTX capabilities are explicitly not present
            if not self.driver.hasAdjustableSpeed():
                ptx_callback_names["GetSpeed"] = None # Clear the method
                ptx_callback_names["SetSpeed"] = None # Clear the method
                ptx_capabilities_dict['has_speed_control'] = False
            else:
                ptx_capabilities_dict['has_speed_control'] = True
                
            if not self.driver.reportsPosition():
                ptx_callback_names["GetPosition"] = None # Clear the method
                
            if not self.driver.hasAbsolutePositioning():
                ptx_callback_names["GotoPosition"] = None # Clear the method

            
            self.has_absolute_positioning_and_feedback = self.driver.hasAbsolutePositioning() and self.driver.reportsPosition()
            ptx_capabilities_dict['has_absolute_positioning'] = self.has_absolute_positioning_and_feedback
            self.msg_if.pub_warn("hasAbsolutePositioning gnode check" + str(self.driver.hasAbsolutePositioning()))
            self.msg_if.pub_warn("reportsPosition gnode check" + str(self.driver.reportsPosition()))
                    
            if not self.driver.canHome() and not self.has_absolute_positioning_and_feedback:
                ptx_callback_names["GoHome"] = None
                ptx_capabilities_dict['has_homing'] = False
            else:
                ptx_capabilities_dict['has_homing'] = True
            
            if self.driver.reportsPosition == True:
                ptx_callback_names["GetPosition"] = self.getPosition()
            else:
                ptx_callback_names["GetPosition"] = None

                
            if not self.driver.homePositionAdjustable() and not self.has_absolute_positioning_and_feedback:
                ptx_callback_names["SetHomePositionHere"] = None
            

            # Following are implemented entirely in software because ONVIF has no support for setting HOME or PRESET by position
            if not self.has_absolute_positioning_and_feedback:
                ptx_callback_names["SetHomePosition"] = None    

            ptx_callback_names["StopMoving"] = self.stopMoving
            ptx_callback_names["MovePan"] = self.movePan
            ptx_callback_names["MoveTilt"] = self.movePan
            ptx_callback_names["MoveTilt"] = self.moveTilt

            # Now that we've updated the callbacks table, can apply the remappings... this is particularly useful since
            # many ONVIF devices report capabilities that they don't actually have, so need a user-override mechanism. In
            # that case, assign these to null in the config file
            # TODO: Not sure we actually need remappings for PTX: Makes sense for IDX because there are lots of controllable params.
            ptx_remappings = nepi_sdk.get_param('~ptx_remappings', {})
            self.msg_if.pub_info('Establishing PTX remappings')
            for from_name in ptx_remappings:
                to_name = ptx_remappings[from_name]
                if from_name not in ptx_callback_names or (to_name not in ptx_callback_names and to_name != None and to_name != 'None'):
                    pass
                elif to_name is None or to_name == 'None':
                    ptx_callback_names[from_name] = None
                elif ptx_callback_names[to_name] is None:
                    pass
                else:
                    ptx_callback_names[from_name] = ptx_callback_names[to_name]

            # Launch the PTX interface --  this takes care of initializing all the ptx settings from config. file, subscribing and advertising topics and services, etc.
            # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
            self.msg_if.pub_info("Launching NEPI PTX () interface...")
            self.device_info_dict["device_name"] = self.device_name
            self.device_info_dict["path"] = self.device_path
            self.device_info_dict["serial_number"] = self.driver.getDeviceSerialNumber()
            self.device_info_dict["hw_version"] = self.driver.getDeviceHardwareId()
            self.device_info_dict["sw_version"] = self.driver.getDeviceFirmwareVersion()

            #Factory Control Values 
            self.FACTORY_CONTROLS = {
                'frame_id' : self.node_name + '_frame',
                'pan_joint_name' : self.node_name + '_pan_joint',
                'tilt_joint_name' : self.node_name + '_tilt_joint',
                'reverse_pan_control' : False,
                'reverse_tilt_control' : False,
                'speed_ratio' : 0.5
            }
            
            # Driver can specify position limits via getPositionLimitsInDegrees. Otherwise, we hard-code them 
            # to arbitrary values here, but can be overridden in device config file (see ptx_if.py)
            self.default_settings = dict()
            if hasattr(self.driver, 'getPositionLimitsInDegrees'):
                driver_specified_limits = self.driver.getPositionLimitsInDegrees()
                self.default_settings['max_pan_hardstop_deg'] = driver_specified_limits['max_pan_hardstop_deg']
                self.default_settings['min_pan_hardstop_deg'] = driver_specified_limits['min_pan_hardstop_deg']
                self.default_settings['max_tilt_hardstop_deg'] = driver_specified_limits['max_tilt_hardstop_deg']
                self.default_settings['min_tilt_hardstop_deg'] = driver_specified_limits['min_tilt_hardstop_deg']
                self.default_settings['max_pan_softstop_deg'] = driver_specified_limits['max_pan_softstop_deg']
                self.default_settings['min_pan_softstop_deg'] = driver_specified_limits['min_pan_softstop_deg']
                self.default_settings['max_tilt_softstop_deg'] = driver_specified_limits['max_tilt_softstop_deg']
                self.default_settings['min_tilt_softstop_deg'] = driver_specified_limits['min_tilt_softstop_deg']
            else:
                self.default_settings['max_pan_hardstop_deg'] = 60.0
                self.default_settings['min_pan_hardstop_deg'] = -60.0
                self.default_settings['max_tilt_hardstop_deg'] = 60.0
                self.default_settings['min_tilt_hardstop_deg'] = -60.0
                self.default_settings['max_pan_softstop_deg'] = 59.0
                self.default_settings['min_pan_softstop_deg'] = -59.0
                self.default_settings['max_tilt_softstop_deg'] = 59.0
                self.default_settings['min_tilt_softstop_deg'] = -59.0

        
            self.speed_ratio = 0.5
            self.home_pan_deg = 0.0
            self.home_tilt_deg = 0.0

            self.ptx_if = PTXActuatorIF(device_info = self.device_info_dict, 
                                        factoryControls = self.FACTORY_CONTROLS,
                                        factoryLimits = self.default_settings,
                                        stopMovingCb = ptx_callback_names["StopMoving"],
                                        movePanCb = ptx_callback_names["MovePan"],
                                        moveTiltCb = ptx_callback_names["MoveTilt"],
                                        setSpeedRatioCb = None, #ptx_callback_names["SetSpeed"],
                                        getSpeedRatioCb = None, #ptx_callback_names["GetSpeed"],
                                        getPositionCb = ptx_callback_names["GetPosition"],
                                        gotoPositionCb = None, #ptx_callback_names["GotoPosition"],
                                        goHomeCb = ptx_callback_names["GoHome"],
                                        setHomePositionCb = None, #ptx_callback_names["SetHomePosition"],
                                        setHomePositionHereCb = None, #ptx_callback_names["SetHomePositionHere"],
                                        getNavPoseCb = self.getNavPoseDict,
                                        navpose_update_rate = self.MAX_POSITION_UPDATE_RATE)
                                        
            self.msg_if.pub_info(" ... PTX interface running")


            nepi_sdk.spin()

    def getPosition(self):
        return self.current_position


       
    def getNavPoseDict(self):
        pan_deg, tilt_deg = self.getCurrentPosition()
        navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
        navpose_dict['has_orientation'] = True
        navpose_dict['time_oreantation'] = nepi_utils.get_time()
        navpose_dict['roll_deg'] = 0.0
        navpose_dict['yaw_deg'] = pan_deg * self.PAN_DEG_DIR
        navpose_dict['pitch_deg'] = tilt_deg * self.TILT_DEG_DIR
        return navpose_dict

    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += ("Manufacturer: " + self.dev_info["Manufacturer"] + "\n")
        dev_info_string += ("Model: " + self.dev_info["Model"] + "\n")
        dev_info_string += ("Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n")
        dev_info_string += ("Serial Number: " + self.dev_info["HardwareId"] + "\n")
        self.msg_if.pub_info(dev_info_string)



    #######################
    ### PTX IF Functions

    def stopMoving(self):
        try:
            self.driver.stopMotion()
        except Exception as e:
            self.msg_if.pub_warn("Failed to stop PTZ motion (device may be disconnected): " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": PTZ device connection lost, shutting down")

    def movePan(self, direction, duration):
        if self.ptx_if is not None:
            driver_direction = self.driver.PT_DIRECTION_POSITIVE if direction == 1 else self.driver.PT_DIRECTION_NEGATIVE
            try:
                self.driver.jog(pan_direction = driver_direction, tilt_direction = self.driver.PT_DIRECTION_NONE, speed_ratio = self.speed_ratio, time_s = duration)
            except Exception as e:
                self.msg_if.pub_warn("Failed to pan (device may be disconnected): " + str(e))
                nepi_sdk.signal_shutdown(self.node_name + ": PTZ device connection lost, shutting down")

    def moveTilt(self, direction, duration):
        if self.ptx_if is not None:
            driver_direction = self.driver.PT_DIRECTION_POSITIVE if direction == 1 else self.driver.PT_DIRECTION_NEGATIVE
            try:
                self.driver.jog(pan_direction = self.driver.PT_DIRECTION_NONE, tilt_direction = driver_direction, speed_ratio = self.speed_ratio, time_s = duration)
            except Exception as e:
                self.msg_if.pub_warn("Failed to tilt (device may be disconnected): " + str(e))
                nepi_sdk.signal_shutdown(self.node_name + ": PTZ device connection lost, shutting down")

    def setSpeed(self, speed_ratio):
        # TODO: Limits checking and driver unit conversion?
        self.speed_ratio = speed_ratio

    def getSpeed(self):
        # TODO: Driver unit conversion?
        return self.speed_ratio
            
     
    def tiltDegToOvRatio(self, deg):
        ratio = 0.5
        max_ph = nepi_sdk.get_param('~ptx/limits/max_tilt_hardstop_deg', self.default_settings['max_tilt_hardstop_deg'])
        min_ph = nepi_sdk.get_param('~ptx/limits/min_tilt_hardstop_deg', self.default_settings['min_tilt_hardstop_deg'])
        reverse_tilt = False
        if self.ptx_if is not None:
            reverse_tilt = self.ptx_if.reverse_tilt_control
        if reverse_tilt == False:
          ratio = 1 - (deg - min_ph) / (max_ph - min_ph)
        else:
          ratio = (deg - min_ph) / (max_ph - min_ph)
        ovRatio = 2 * (ratio - 0.5)
        return ovRatio

    def panDegToOvRatio(self, deg):
        ratio = 0.5
        max_yh = nepi_sdk.get_param('~ptx/limits/max_pan_hardstop_deg', self.default_settings['max_pan_hardstop_deg'])
        min_yh = nepi_sdk.get_param('~ptx/limits/min_pan_hardstop_deg', self.default_settings['min_pan_hardstop_deg'])
        reverse_pan = False
        if self.ptx_if is not None:
            reverse_pan = self.ptx_if.reverse_pan_control 
        if reverse_pan == False:
          ratio = 1 - (deg - min_yh) / (max_yh - min_yh)
        else:
          ratio = (deg - min_yh) / (max_yh - min_yh)
        ovRatio = 2 * (ratio - 0.5)
        return (ovRatio)     

    def panOvRatioToDeg(self, ovRatio):
        pan_deg = 0
        max_yh = nepi_sdk.get_param('~ptx/limits/max_pan_hardstop_deg', self.default_settings['max_pan_hardstop_deg'])
        min_yh = nepi_sdk.get_param('~ptx/limits/min_pan_hardstop_deg', self.default_settings['min_pan_hardstop_deg'])
        reverse_pan = False
        if self.ptx_if is not None:
            reverse_pan = self.ptx_if.reverse_pan_control 
        if reverse_pan == False:
           pan_deg =  min_yh + (1-ovRatio) * (max_yh - min_yh)
        else:
           pan_deg =  max_yh - (1-ovRatio)  * (max_yh - min_yh)
        return  pan_deg
    
    def tiltOvRatioToDeg(self, ovRatio):
        tilt_deg = 0
        max_ph = nepi_sdk.get_param('~ptx/limits/max_tilt_hardstop_deg', self.default_settings['max_tilt_hardstop_deg'])
        min_ph = nepi_sdk.get_param('~ptx/limits/min_tilt_hardstop_deg', self.default_settings['min_tilt_hardstop_deg'])
        reverse_tilt = False
        if self.ptx_if is not None:
            reverse_tilt = self.ptx_if.reverse_tilt_control
        if reverse_tilt == False:
           tilt_deg =  min_ph + (1-ovRatio) * (max_ph - min_ph)
        else:
           tilt_deg =  max_ph - (1-ovRatio) * (max_ph - min_ph)
        return  tilt_deg



    def getCurrentPosition(self):
        try:
            pan_ratio_onvif, tilt_ratio_onvif = self.driver.getCurrentPosition()
        except Exception as e:
            self.msg_if.pub_warn("Failed to get PTZ position (device may be disconnected): " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": PTZ device connection lost, shutting down")
            return 0.0, 0.0
        if pan_ratio_onvif != -999 and tilt_ratio_onvif != -999:
            self.last_pan_ratio_onvif = pan_ratio_onvif
            self.last_tilt_ratio_onvif = tilt_ratio_onvif
        #print("Got pos_ratio_onvif from driver: " + str([pan_ratio_onvif, tilt_ratio_onvif]))
        # Recenter the -1.0,1.0 ratio from Onvif to the 0.0,1.0 used throughout the rest of NEPI
        pan_ratio_onvif = (0.5 * self.last_pan_ratio_onvif) + 0.5
        tilt_ratio_onvif = (0.5 * self.last_tilt_ratio_onvif) + 0.5
        #print("Got pos_ratio adj: " + str([pan_ratio_onvif, tilt_ratio_onvif]))
        if self.ptx_if is None:
            pan_deg = 0
            tilt_deg = 0
        else:
            pan_deg = self.panOvRatioToDeg(pan_ratio_onvif)
            tilt_deg = self.tiltOvRatioToDeg(tilt_ratio_onvif)
        #print("Got pos degs : " + str([pan_deg, tilt_deg]))
        return pan_deg, tilt_deg
        

    def gotoPosition(self, pan_deg, tilt_deg):
        pan_ratio_onvif = self.panDegToOvRatio(pan_deg)
        tilt_ratio_onvif = self.tiltDegToOvRatio(tilt_deg)
        # Recenter the ratios to the ONVIF -1.0,1.0 range
        self.driver.moveToPosition(pan_ratio_onvif, tilt_ratio_onvif, self.speed_ratio)
        
    def goHome(self):
        if self.driver.canHome() is True:
            self.driver.goHome(self.speed_ratio)
        elif self.driver.hasAbsolutePositioning() is True and self.ptx_if is not None:
            home_pan_ratio_onvif = self.panDegToOvRatio(self.home_pan_deg)
            home_tilt_ratio_onvif = self.tiltDegToOvRatio(self.home_tilt_deg)
            self.driver.moveToPosition(home_pan_ratio_onvif, home_tilt_ratio_onvif, self.speed_ratio)

    def setHomePosition(self, pan_deg, tilt_deg):
        # Have to implement these fully in s/w since ONVIF doesn't support absolute home position setting
        self.home_pan_deg = pan_deg
        self.home_tilt_deg = tilt_deg

    def setHomePositionHere(self):
        if self.driver.reportsPosition() is True:
            curr_pan_ratio_onvif, curr_tilt_ratio_onvif = self.driver.getCurrentPosition()
            if self.ptx_if is not None:
                self.home_pan_deg = self.panOvRatioToDeg(curr_pan_ratio_onvif)
                self.home_tilt_deg = self.tiltOvRatioToDeg(curr_tilt_ratio_onvif) 

        if self.driver.homePositionAdjustable is True:
            self.driver.setHomeHere()


    def updateFromParamServer(self):
        if self.ptx_if is not None:
            self.ptx_if.updateFromParamServer()


if __name__ == '__main__':
	node = OnvifPanTiltNode()
