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

##############################################################################
# IDX (Imaging / Camera) DRIVER TEMPLATE -- NODE
# ---------------------------------------------------------------------------
# Unlike the serial templates, the IDX node does NOT do hardware I/O itself.
# It:
#   1. reads '~drv_dict' (written by the LAUNCH-model discovery node),
#   2. dynamically imports the DRIVER class named in DRIVER_DICT via
#      nepi_drvs.importDriverClass() and constructs it,
#   3. registers an IDXDeviceIF, handing it image-acquisition callbacks,
#   4. spin()s. IDXDeviceIF pulls frames by calling getColorImg() -- there is
#      no publish loop in this file.
#
# getColorImg() returns a 5-TUPLE: (ret, msg, cv2_img, timestamp, encoding).
# Return (False, "...", None, None, None) when there is no new frame.
##############################################################################

import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs        # importDriverClass
from nepi_sdk import nepi_settings

from nepi_api.messages_if import MsgIF
from nepi_api.device_if_idx import IDXDeviceIF

PKG_NAME = 'IDX_NEW_CAM'
FILE_TYPE = 'NODE'


class ncamNode:

    # ---- Runtime-tunable settings, driven off the driver's control table.
    #      Built dynamically in getCapSettings() from driver.getCameraControls().
    FACTORY_SETTINGS_OVERRIDES = dict()

    # ---- Factory controls IDXDeviceIF expects (FOV + frame id, etc.).
    FACTORY_CONTROLS = dict(width_deg=90, height_deg=60, frame_id='sensor_frame')

    device_info_dict = dict(device_name="", path="", serial_number="",
                            hw_version="", sw_version="")

    max_framerate = 30.0
    cl_img_last_time = None
    idx_if = None
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

    ##########################################################################
    def __init__(self):
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        # ---- Read config + the DRIVER class coordinates from '~drv_dict'.
        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.driver_path = self.drv_dict['path']                 # injected by drivers_mgr
            self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
            self.driver_module = self.driver_file.split('.')[0]
            self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": no valid Device Dict")
            return

        # ---- Dynamically import + construct the raw-I/O driver class.
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(
            self.driver_file, self.driver_path, self.driver_module, self.driver_class_name)
        if not success:
            self.msg_if.pub_warn("Failed to import driver class: " + str(msg))
            nepi_sdk.signal_shutdown(self.node_name + ": driver import failed")
            return
        try:
            self.driver = self.driver_class(self.device_path)   # TODO: match your driver ctor
        except Exception as e:
            self.msg_if.pub_warn("Failed to construct driver: " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": driver construct failed")
            return
        if not self.driver.isConnected():
            nepi_sdk.signal_shutdown(self.node_name + ": camera not connected")
            return

        self.img_lock = threading.Lock()

        # ---- Build settings from the driver's control table + fill device_info.
        self.cap_settings = self.getCapSettings()
        self.factory_settings = self.getFactorySettings()
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = ""
        self.device_info_dict["hw_version"] = ""
        self.device_info_dict["sw_version"] = ""

        # ---- Register the camera with NEPI. Pass a getX/stopX pair for every
        #      data product the camera provides; omit the rest. This template
        #      provides only a color image.
        self.idx_if = IDXDeviceIF(
            device_info=self.device_info_dict,
            data_source_description='camera',
            data_ref_description='camera_lens',
            capSettings=self.cap_settings,
            factorySettings=self.factory_settings,
            settingUpdateFunction=self.settingUpdateFunction,
            getSettingsFunction=self.getSettings,
            factoryControls=self.FACTORY_CONTROLS,
            setMaxFramerate=self.setMaxFramerate,
            getFramerate=self.driver.getFramerate,
            getColorImage=self.getColorImg,
            stopColorImageAcquisition=self.stopColorImg,
            # getDepthMap=..., getPointcloud=...  # TODO: add for depth cameras
            data_products=['color_image'],
        )

        nepi_sdk.on_shutdown(self.cleanup_actions)
        self.msg_if.pub_info("Initialization complete")
        nepi_sdk.spin()

    #########################################################################
    # SETTINGS  (built from the driver's control table)
    #########################################################################
    def getCapSettings(self):
        settings = dict()
        for name, ctrl in self.driver.getCameraControls().items():
            settings[name] = {"type": "Int", "name": name,
                              "options": [str(ctrl.get('min', 0)), str(ctrl.get('max', 100))]}
        return settings

    def getFactorySettings(self):
        settings = self.getSettings()
        for name in settings.keys():
            if name in self.FACTORY_SETTINGS_OVERRIDES:
                settings[name]['value'] = self.FACTORY_SETTINGS_OVERRIDES[name]
        return settings

    def getSettings(self):
        settings = dict()
        for name, ctrl in self.driver.getCameraControls().items():
            settings[name] = {"name": name, "type": "Int", "value": str(ctrl.get('value', 0))}
        return settings

    def settingUpdateFunction(self, setting):
        [name, s_type, data] = nepi_settings.get_data_from_setting(setting)
        if data is None:
            return False, self.node_name + " setting data is None"
        success, msg = self.driver.setCameraControl(name, data)
        return success, (self.node_name + " UPDATED " + str(setting)) if success else msg

    def setMaxFramerate(self, rate):
        self.max_framerate = max(1.0, min(100.0, float(rate)))
        return True, ""

    #########################################################################
    # IMAGE ACQUISITION CALLBACKS  (the heart of an IDX driver)
    #########################################################################
    def getColorImg(self):
        # Rate-limit to max_framerate so we don't outrun the configured FPS.
        now = nepi_utils.get_time()
        if self.cl_img_last_time is not None:
            if (now - self.cl_img_last_time) < (1.0 / self.max_framerate):
                return False, "Waiting for timer", None, None, None

        with self.img_lock:
            self.driver.startImageAcquisition()
            cv2_img, timestamp, ret, msg = self.driver.getImage()
        if not ret:
            return False, msg, None, None, None
        self.cl_img_last_time = now
        # 5-tuple: (ret, msg, image, timestamp, encoding)
        return True, "Success", cv2_img, timestamp, "bgr8"

    def stopColorImg(self):
        with self.img_lock:
            ret, msg = self.driver.stopImageAcquisition()
        return ret, msg

    #########################################################################
    def cleanup_actions(self):
        try:
            self.driver.stopImageAcquisition()
        except Exception:
            pass


if __name__ == '__main__':
    node = ncamNode()
