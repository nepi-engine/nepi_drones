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


import copy
import time
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_controls

from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.device_if_npx import NPXDeviceIF
from nepi_api.node_if_ai_detector import AiDetectorIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'IDX_LIMELIGHT4_USB'
FILE_TYPE = 'NODE'


class Limelight4UsbCamNode:

    FACTORY_CONTROLS = dict(
        width_deg=90,
        weight_deg=60,
        frame_id='sensor_frame'
    )

    DEFAULT_CURRENT_FPS = 20

    device_info_dict = dict(device_name="", path="", serial_number="", hw_version="", sw_version="")

    idx_if = None
    npx_if = None

    init_settings_dict = dict()
    settings_dict = dict()
    ai_if = None
    AI_DEFAULT_CONFIG = {'threshold': 0.3, 'max_rate': 5}
    driver_navpose_dict = None
    current_fps = DEFAULT_CURRENT_FPS
    cl_img_last_time = None
    max_framerate = 100

    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
    drv_dict = dict()

    def __init__(self):
        #### NODE Initialization ####
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        ##############################
        # Load drv_dict
        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.driver_path = self.drv_dict['path']
            self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
            self.driver_module = self.driver_file.split('.')[0]
            self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        host = self.device_path
        api_port = 5807
        stream_port = 5800

        self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(
            self.driver_file, self.driver_path, self.driver_module, self.driver_class_name
        )

        driver_constructed = False
        if success:
            attempts = 0
            while not nepi_sdk.is_shutdown() and not driver_constructed and attempts < 5:
                try:
                    self.driver = self.driver_class(host, stream_port, api_port)
                    driver_constructed = True
                    self.msg_if.pub_info("LIMELIGHT4_NODE: Driver constructed")
                except Exception as e:
                    self.msg_if.pub_info("LIMELIGHT4_NODE: Failed to construct driver: " + str(e))
                    time.sleep(1)
                attempts += 1

        if not driver_constructed:
            nepi_sdk.signal_shutdown(
                "Shutting down Limelight4 node " + self.node_name + ", unable to connect to driver"
            )
        else:
            ################################################
            self.msg_if.pub_info("... Connected!")
            self.logDeviceInfo()

            # Threading state for image acquisition
            self.img_uri_lock = threading.Lock()
            self.color_image_acquisition_running = False
            self.cached_2d_color_frame = None
            self.cached_2d_color_frame_timestamp = None

            # Controls and settings
            self.factory_controls = self.FACTORY_CONTROLS
            self.current_controls = self.factory_controls
            self.current_fps = self.DEFAULT_CURRENT_FPS

            self.settings_dict = self.initSettingsDict()
            self.settings_dict = self.refreshSettingsDict()

            # Launch the IDX interface
            self.msg_if.pub_info("Launching NEPI IDX interface...")
            self.device_info_dict['device_name'] = self.device_name
            self.device_info_dict['path'] = self.device_path
            self.idx_if = IDXDeviceIF(
                device_info=self.device_info_dict,
                data_source_description='camera',
                data_ref_description='camera_lense',
                getSettingsFunction=self.getSettingsFunction,
                setSettingFunction=self.setSettingFunction,
                factoryControls=self.factory_controls,
                setMaxFramerate=self.setMaxFramerate,
                getFramerate=self.driver.get_framerate,
                getColorImage=self.getColorImg,
                stopColorImageAcquisition=self.stopColorImg,
                data_products=['color_image']
            )
            self.msg_if.pub_info("... IDX interface running")

            # Initialize navpose dict for IMU orientation
            self.driver_navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
            self.driver_navpose_dict['has_orientation'] = True

            # Populate device_info_dict with hardware details for NPX interface
            dev_info = self.driver.get_device_info()
            self.device_info_dict['serial_number'] = dev_info.get('serial_number', '')
            self.device_info_dict['hw_version'] = dev_info.get('hw_version', '')
            self.device_info_dict['sw_version'] = dev_info.get('sw_version', '')

            # Start IMU poll timer
            nepi_sdk.start_timer_process(
                1.0 / self.DEFAULT_CURRENT_FPS, self.pollImuCb, oneshot=False
            )

            # Launch NPX interface
            self.msg_if.pub_info("Launching NEPI NPX interface...")
            self.npx_if = NPXDeviceIF(
                device_info=self.device_info_dict,
                data_source_description='camera',
                data_ref_description='camera_lense',
                getNavPoseCb=self.getNavPoseCb,
                msg_if=self.msg_if
            )
            self.msg_if.pub_info("... NPX interface running")

            # Launch AI detector interface
            self.msg_if.pub_info("Launching NEPI AI Detector interface...")
            self.ai_if = AiDetectorIF(
                namespace=self.node_namespace,
                model_name=self.node_name,
                framework='limelight',
                description='Limelight 4 on-device neural network detector',
                proc_img_height=960,
                proc_img_width=1280,
                classes_list=[],
                default_config_dict=self.AI_DEFAULT_CONFIG,
                processImageFunction=self.processDetectionsCb,
                processFileFunction=self.processFileCb,
                msg_if=self.msg_if
            )
            self.msg_if.pub_info("... AI Detector interface running")

            nepi_sdk.spin()

    #**********************
    # Sensor setting functions

    def logDeviceInfo(self):
        dev_info = self.driver.get_device_info()
        info_str = self.node_name + " Device Info:\n"
        for k, v in dev_info.items():
            info_str += "  " + str(k) + ": " + str(v) + "\n"
        self.msg_if.pub_info(info_str)

    def initSettingsDict(self):
        init_settings_dict = dict()

        # Pipeline: indices 0-9. The retired cap-settings form called this
        # 'Discrete'; the controls contract calls a named option list a
        # Selection.
        ret, pipeline_idx = self.driver.get_pipeline()
        pipeline_val = str(pipeline_idx) if ret else '0'
        init_settings_dict['Pipeline'] = {
            'type': 'Selection',
            'options': ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9'],
            'default': pipeline_val
        }

        # Resolution: built from the driver-reported fixed set
        try:
            resolutions = self.driver.get_available_resolutions()
            options = [str(r['width']) + 'x' + str(r['height']) for r in resolutions]
        except Exception as e:
            self.msg_if.pub_warn("Driver returned invalid resolution options: " + str(e))
            options = []
        if '640x480' not in options:
            options.append('640x480')
        init_settings_dict['Resolution'] = {
            'type': 'Selection',
            'options': options,
            'default': '640x480'
        }

        self.init_settings_dict = init_settings_dict
        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        settings_dict_values = nepi_controls.get_controls_values_dict(settings_dict)
        self.msg_if.pub_info("Initialized Settings: " + str(settings_dict_values))
        return settings_dict


    def refreshSettingsDict(self):
        settings_dict = copy.deepcopy(self.settings_dict)
        try:
            ret, pipeline_idx = self.driver.get_pipeline()
            if ret:
                settings_dict = nepi_controls.set_control_value(settings_dict, 'Pipeline', str(pipeline_idx))
        except Exception as e:
            self.msg_if.pub_debug("Failed to refresh Pipeline setting: " + str(e))
        # Resolution is fixed by the active pipeline on the Limelight device,
        # so there is nothing live to read back for it.
        return settings_dict


    def getSettingsFunction(self):
        return self.settings_dict


    def setSettingFunction(self, setting_name, setting_value):
        success = False
        msg = "Unknown setting: " + str(setting_name)
        if setting_name not in self.settings_dict.keys():
            msg = self.node_name + " setting " + str(setting_name) + " is not supported"
            return False, msg, self.settings_dict

        if setting_name == 'Pipeline':
            try:
                success, msg = self.driver.set_pipeline(int(setting_value))
            except Exception as e:
                msg = "Failed to set pipeline: " + str(e)
        elif setting_name == 'Resolution':
            # MJPEG resolution is controlled by the active pipeline on the Limelight device;
            # we acknowledge the setting but cannot change it independently
            self.msg_if.pub_info(
                "Resolution changes require a pipeline switch on the Limelight device; "
                "acknowledging setting update only"
            )
            self.settings_dict = nepi_controls.set_control_value(self.settings_dict, setting_name, setting_value)
            return True, "Resolution noted; actual resolution is controlled by the active Limelight pipeline", self.settings_dict

        self.settings_dict = self.refreshSettingsDict()
        return success, msg, self.settings_dict

    #**********************
    # Node driver functions

    def setMaxFramerate(self, rate):
        if rate is None:
            return False, 'Got None Max Framerate'
        if rate < 1:
            rate = 1
        if rate > 100:
            rate = 100
        self.max_framerate = rate
        return True, ""

    def getColorImg(self):
        last_time = self.cl_img_last_time
        current_time = nepi_utils.get_time()

        need_data = False
        if last_time is not None and self.idx_if is not None:
            fr_delay = 1.0 / self.max_framerate
            timer = current_time - last_time
            if timer > fr_delay:
                need_data = True
        else:
            need_data = True

        if not need_data:
            return False, "Waiting for Timer", None, None, None

        self.cl_img_last_time = current_time
        encoding = "bgr8"

        self.img_uri_lock.acquire()
        ret, msg = self.driver.start_image_acquisition()
        if not ret:
            self.img_uri_lock.release()
            return ret, msg, None, None, None
        self.color_image_acquisition_running = True

        cv2_img, timestamp, ret, msg = self.driver.get_image()
        if not ret:
            self.img_uri_lock.release()
            return ret, msg, None, None, None

        if timestamp is None:
            timestamp = nepi_utils.get_time()
        self.img_uri_lock.release()

        return ret, msg, cv2_img, timestamp, encoding

    def stopColorImg(self):
        self.img_uri_lock.acquire()
        ret, msg = self.driver.stop_image_acquisition()
        self.color_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None
        self.img_uri_lock.release()
        return ret, msg

    def pollImuCb(self, timer):
        imu_data = self.driver.get_imu_data()
        if imu_data is not None:
            self.driver_navpose_dict['roll_deg'] = imu_data['roll_deg']
            self.driver_navpose_dict['pitch_deg'] = imu_data['pitch_deg']
            self.driver_navpose_dict['yaw_deg'] = imu_data['yaw_deg']
            self.driver_navpose_dict['time_orientation'] = nepi_utils.get_time()

    def getNavPoseCb(self):
        return self.driver_navpose_dict

    def processDetectionsCb(self, cv2_img, img_dict, threshold=0.3, resize=False, verbose=False):
        h, w = cv2_img.shape[:2]
        img_dict['image_width'] = w
        img_dict['image_height'] = h
        img_dict['prc_width'] = w
        img_dict['prc_height'] = h
        img_dict['ratio'] = 1
        img_dict['tiling'] = False
        raw_detections = self.driver.get_detection_data()
        if raw_detections is None:
            return [[], img_dict]
        img_area = w * h
        detect_dict_list = []
        for d in raw_detections:
            class_name = d['name']
            # Auto-register class names discovered from live Limelight detections
            if self.ai_if is not None and class_name not in self.ai_if.classes:
                self.ai_if.classes.append(class_name)
                self.ai_if.selected_classes.append(class_name)
            if d['prob'] < threshold:
                continue
            xmin = int(d['xmin'] * w)
            ymin = int(d['ymin'] * h)
            xmax = int(d['xmax'] * w)
            ymax = int(d['ymax'] * h)
            area_pixels = max(0, (xmax - xmin) * (ymax - ymin))
            area_ratio = area_pixels / img_area if img_area > 0 else 0.0
            detect_dict_list.append({
                'name': class_name,
                'id': d['id'],
                'uid': '',
                'prob': d['prob'],
                'xmin': xmin,
                'ymin': ymin,
                'xmax': xmax,
                'ymax': ymax,
                'area_pixels': area_pixels,
                'area_ratio': area_ratio,
            })
        return [detect_dict_list, img_dict]

    def processFileCb(self, img_file, img_dict, threshold=0.3, resize=False, verbose=False):
        # Limelight does not support offline file inference
        img_dict['image_width'] = 1
        img_dict['image_height'] = 1
        img_dict['prc_width'] = 1
        img_dict['prc_height'] = 1
        img_dict['ratio'] = 1
        img_dict['tiling'] = False
        return [[], img_dict]


if __name__ == '__main__':
    node = Limelight4UsbCamNode()
