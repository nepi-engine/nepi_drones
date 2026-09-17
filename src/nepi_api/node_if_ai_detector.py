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
import copy
import time 
import copy
import numpy as np
import math
import threading
import cv2

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image

from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import MgrSystemStatus
from nepi_interfaces.msg import StringArray
from nepi_interfaces.msg import ProcessStatus
from nepi_interfaces.msg import Detection, Detections, DetectorStatus
from nepi_interfaces.srv import DetectorStatusQuery, DetectorStatusQueryRequest, DetectorStatusQueryResponse
from nepi_interfaces.msg import Target, Targets, TargetingStatus


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_img

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodePublishersIF, NodeSubscribersIF, NodeClassIF
from nepi_api.system_if import SaveDataIF, StatesIF, TriggersIF
from nepi_api.process_if import DetectionsIF, TargetsIF


SYSTEM_ALL_TOPIC = 'all'

#########################################
# AI Detector Node IF
#########################################
DETECTIONS_ALL_TOPIC = 'detections'
TARGETS_ALL_TOPIC = 'targets'

EXAMPLE_DETECTION_DICT_ENTRY = {
    'name': 'chair', # Class String Name
    'id': 1, # Class Index from Classes List
    'uid': '', # Reserved for unique tracking by downstream applications
    'prob': .3, # Probability of detection
    'xmin': 10,
    'ymin': 10,
    'xmax': 100,
    'ymax': 100
}

MIN_THRESHOLD = 0.01
MAX_THRESHOLD = 1.0
DEFAULT_THRESHOLD = 0.3

MIN_MAX_RATE = 1
MAX_MAX_RATE = 20
DEFAULT_MAX_PROC_RATE = 10
DEFAULT_MAX_IMG_RATE = 10
DEFAULT_USE_LAST_IMAGE = True

DEFAULT_WAIT_FOR_DETECT = False

DEFAULT_IMG_TILING = False

DEFAULT_LABELS_OVERLAY = True
DEFAULT_CLF_OVERLAY = False
DEFAULT_IMG_OVERLAY = False

GET_IMAGE_TIMEOUT_SEC = 1 



class AiDetectorIF:
    

    DETECTIONS_DATA_PRODUCTS = ['detections','detections_image']
    TARGETS_DATA_PRODUCTS = ['targets','targets_image']
                                
    IMAGE_FILTERS = ['color_image']

    # A detector must never consume its own overlay outputs as an input image
    # source; skip these product basenames even if a stale/explicit selection
    # lists them (they now resolve as real topics under the image namespace).
    OUTPUT_IMG_PRODUCTS = ['detections_image', 'targets_image']

    BLANK_SIZE_DICT = { 'h': 350, 'w': 700, 'c': 3}
    BLANK_CV2_IMAGE = nepi_img.create_blank_image((BLANK_SIZE_DICT['h'],BLANK_SIZE_DICT['w'],BLANK_SIZE_DICT['c']))

    namespace = '~'
    detections_namespace = '~'
    targets_namespace = '~'
    all_namespace = None
    all_detections_namespace = None
    all_targets_namespace = None

    process_status_msg = ProcessStatus()
    detector_status_msg = DetectorStatus()
    targeting_status_msg = TargetingStatus()

    states_dict = None
    triggers_dict = dict()

    node_if = None
    save_data_if = None
    save_data_namespace = 'None'
    

    data_products = DETECTIONS_DATA_PRODUCTS + TARGETS_DATA_PRODUCTS

    available_source_topics = []

    api_lib_folder = '/opt/nepi/nepi_engine/lib/nepi_api'

    processImage = None
    processFile = None

    self_managed = True
    model_name = "model"

    last_detect_time = nepi_sdk.get_time()

    img_ifs_dict = dict()
    img_ifs_lock = threading.Lock()
    imgs_info_dict = dict()
    imgs_img_proc_dict = dict()


    imgs_has_subs_dict = dict()
    images_dict = dict()

    navpose_dict = dict()
    navpose_dict_lock = threading.Lock()

    depth_map_dict = dict()
    depth_map_dict_lock = threading.Lock()

    pointcloud_dict = dict()
    pointcloud_dict_lock = threading.Lock()



    msg_str = 'Loading'
    active_source_topics = []
    cur_source_topic = "None"

    img_msg = None
    get_source_topic = "None"
    got_source_topic = None

    get_source_file = False
    got_source_file = False



    detections_has_published = False
    first_detect_complete = False
    detecting_state = False

    targeting_topic = 'targets'
    targeting_state = False
    targeting_has_published = False

    source_receive_latencies = [0,0,0,0,0,0,0,0,0,0]
    source_receive_rates = [0,0,0,0,0,0,0,0,0,0]

    preprocess_times = [0,0,0,0,0,0,0,0,0,0]
    preprocess_latencies = [0,0,0,0,0,0,0,0,0,0]
    preprocess_rates = [0,0,0,0,0,0,0,0,0,0]
    
    process_times = [0,0,0,0,0,0,0,0,0,0]
    
    process_latencies = [0,0,0,0,0,0,0,0,0,0]
    process_rates = [0,0,0,0,0,0,0,0,0,0]

    is_processing = False
    process_state = False
    last_receive_source_time = nepi_sdk.get_time()
    last_process_detect_time = nepi_sdk.get_time()

    sleep_state = False


    enabled = True
    selected_classes = []
    selected_classes_targets = []
    sleep_enabled = False
    sleep_suspend_sec = 0
    sleep_run_sec = 0
    img_tiling = False
    overlay_labels = True
    overlay_range_bearing = True
    overlay_clf_name = False
    overlay_img_name = False
    threshold = DEFAULT_THRESHOLD
    max_process_rate_hz = DEFAULT_MAX_PROC_RATE
    max_image_pub_rate_hz = DEFAULT_MAX_IMG_RATE
    use_last_image = DEFAULT_USE_LAST_IMAGE

    auto_select_enabled = True
    auto_select_active = True
    selected_sources = []

    imaging_enabled=True
    launch_node_process=None
    pub_img_node_name = ""
    pub_img_namepace = ""

    source_file_path=None
    source_file_processing = False

    next_source_topic="None"

    node_if_prefix = model_name + '_'

    active_nodes = []
    active_topics = []
    active_topic_types = []
    active_services = []


    def __init__(self, 
                namespace,
                model_name, 
                framework, 
                description, 
                proc_img_height, 
                proc_img_width,  
                classes_list, 
                processImageFunction,
                processFileFunction,
                enable_image_pub = True,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()



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
        self.msg_if.pub_debug("Starting Node Class IF Initialization Processes", log_name_list = self.log_name_list)


        ##############################  
        # Setup All Namespaces

        # Collective controls publish on the shared namespaces, which fans a
        # single command out to every IDX image.
        self.all_namespace = nepi_sdk.create_namespace(self.base_namespace, SYSTEM_ALL_TOPIC)
        self.all_detections_namespace = nepi_sdk.create_namespace(self.all_namespace, DETECTIONS_ALL_TOPIC)
        self.all_targets_namespace = nepi_sdk.create_namespace(self.all_namespace, TARGETS_ALL_TOPIC)
 
 
        ##############################
        # Get for System Folders
        self.msg_if.pub_warn("Waiting for system folders")
        system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        while system_folders is None and nepi_sdk.is_shutdown() == False:
            system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
            nepi_sdk.sleep(1)

        self.msg_if.pub_warn("Got system folders: " + str(system_folders))
       
        if system_folders is not None:
            self.api_lib_folder = system_folders['api_lib']
        self.msg_if.pub_info("Using SDK Share Folder: " + str(self.api_lib_folder))
 

        ##############################  
        # Init Class Variables 

        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)
     

        self.detections_namespace = nepi_sdk.create_namespace(self.namespace,'detections')
        self.targets_namespace = nepi_sdk.create_namespace(self.namespace,'targets')


        self.enable_image_pub = enable_image_pub

        


        model_name = nepi_utils.get_clean_name(model_name)
        self.model_name = model_name
        self.model_framework = framework
        self.model_type = 'detection'
        self.model_description = description
        self.model_proc_img_height = proc_img_height
        self.model_proc_img_width = proc_img_width
        self.processImage = processImageFunction
        self.processFile = processFileFunction
        self.classes = classes_list
        self.msg_if.pub_warn("Detector provided classes list: " + str(self.classes))

        self.has_sleep = False

        self.selected_classes = self.classes
        self.selected_classes_targets = self.classes


        self.node_if_prefix =self.model_name + '_'


        ## Init Status Messages
        self.process_status_msg.node_name = self.node_name
        self.process_status_msg.namespace = self.namespace

        self.process_status_msg.name = self.model_name
        self.process_status_msg.group = self.model_framework
        self.process_status_msg.description = self.model_description

        self.process_status_msg.config_topic = self.node_namespace

     
        self.initCb(do_updates = False)

        ##############################
        ### Setup Node



        # Configs Dict ########################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.node_namespace,
        }


        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.enabled
            },
            'auto_select_enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.auto_select_enabled
            },
            'selected_sources': {
                'namespace': self.node_namespace,
                'factory_val': []
            },
            'selected_classes': {
                'namespace': self.node_namespace,
                'factory_val': self.selected_classes
            },
            'threshold': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_THRESHOLD
            },
            'max_process_rate_hz': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_MAX_PROC_RATE
            },
            'max_image_pub_rate_hz': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_MAX_IMG_RATE
            },
            'use_last_image': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_USE_LAST_IMAGE
            },
            'imaging_enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.imaging_enabled
            },
            'selected_classes_targets': {
                'namespace': self.node_namespace,
                'factory_val': self.selected_classes_targets
            },
        }



        # Services Config Dict ####################
        self.SRVS_DICT = {
            'detector_status_query': {
                'namespace': self.detections_namespace,
                'topic': 'detector_status_query',
                'srv': DetectorStatusQuery,
                'req': DetectorStatusQueryRequest(),
                'resp': DetectorStatusQueryResponse(),
                'callback': self.handleStatusRequest
            }
        }


        # Pubs Config Dict ####################
        # NOTE: The per-detector detections/targets data products and their
        # status messages are owned by DetectionsIF/TargetsIF (constructed
        # below), which reproduce the same wire topics
        # (<node_ns>/detections[/status], <node_ns>/targets[/status]). Only the
        # collective 'all' fan-out publishers remain inline here, since the
        # per-data-product IFs do not cover the shared /all/* namespaces.
        self.PUBS_DICT = {
            #######################
            # All Detections
            #######################
            'all_detections': {
                'msg': Detections,
                'namespace': self.all_namespace,
                'topic': DETECTIONS_ALL_TOPIC,
                'qsize': 1,
                'latch': False
            },

            #######################
            # All Targets
            #######################
            'all_targets': {
                'msg': Targets,
                'namespace': self.all_namespace,
                'topic': TARGETS_ALL_TOPIC,
                'qsize': 1,
                'latch': False
            }
        }


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            ############
            # Detector
            ############
            'detector_enable': {
                'namespace': self.detections_namespace,
                'topic': 'enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setEnableCb, 
                'callback_args': ()
            },
            'detector_set_auto_select_enable': {
                'namespace': self.detections_namespace,
                'topic': 'set_auto_select_enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setAutoSelectEnableCb, 
                'callback_args': ()
            },
            'detector_set_source_topic': {
                'namespace': self.detections_namespace,
                'topic': 'set_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.setImageTopicCb, 
                'callback_args': ()
            },
            'detector_set_source_topics': {
                'namespace': self.detections_namespace,
                'topic': 'set_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setImageTopicsCb, 
                'callback_args': ()
            },
            'detector_add_source_topic': {
                'namespace': self.detections_namespace,
                'topic': 'add_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.addImageTopicCb, 
                'callback_args': ()
            },
            'detector_add_source_topics': {
                'namespace': self.detections_namespace,
                'topic': 'add_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.addImageTopicsCb, 
                'callback_args': ()
            },
            'detector_remove_source_topic': {
                'namespace': self.detections_namespace,
                'topic': 'remove_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.removeImageTopicCb, 
                'callback_args': ()
            },
            'detector_remove_source_topics': {
                'namespace': self.detections_namespace,
                'topic': 'remove_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.removeImageTopicsCb, 
                'callback_args': ()
            },
            'detector_process_source_file': {
                'namespace': self.detections_namespace,
                'topic': 'process_source_file',
                'msg': String,
                'qsize': 10,
                'callback': self.processImageFileCb, 
                'callback_args': ()
            },
              'detector_set_class': {
                'namespace': self.detections_namespace,
                'topic': 'set_class',
                'msg': String,
                'qsize': 10,
                'callback': self.setClassCb, 
                'callback_args': ()
            },
            'detector_set_classes': {
                'namespace': self.detections_namespace,
                'topic': 'set_classes',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setClassesCb, 
                'callback_args': ()
            },
            'detector_add_class': {
                'namespace': self.detections_namespace,
                'topic': 'add_class',
                'msg': String,
                'qsize': 10,
                'callback': self.addClassCb, 
                'callback_args': ()
            },
            'detector_remove_class': {
                'namespace': self.detections_namespace,
                'topic': 'remove_class',
                'msg': String,
                'qsize': 10,
                'callback': self.removeClassCb, 
                'callback_args': ()
            },
            'detector_add_all_classes': {
                'namespace': self.detections_namespace,
                'topic': 'add_all_classes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.addAllClassesCb, 
                'callback_args': ()
            },
            'detector_remove_all_classes': {
                'namespace': self.detections_namespace,
                'topic': 'remove_all_classes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.removeAllClassesCb, 
                'callback_args': ()
            },
            'detector_set_threshold': {
                'namespace': self.detections_namespace,
                'topic': 'set_threshold',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setThresholdCb, 
                'callback_args': ()
            },
            'detector_set_image_pub': {
                'namespace': self.detections_namespace,
                'topic': 'set_image_pub',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setPubImageCb, 
                'callback_args': ()
            },
            'detector_set_max_process_rate': {
                'namespace': self.detections_namespace,
                'topic': 'set_max_process_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxProcessRateCb, 
                'callback_args': ()
            },
            'detector_set_max_image_pub_rate': {
                'namespace': self.detections_namespace,
                'topic': 'set_max_image_pub_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxImgRateCb, 
                'callback_args': ()
            },
            'detector_set_use_last_image': {
                'namespace': self.detections_namespace,
                'topic': 'set_use_last_image',
                'msg':Bool,
                'qsize': 10,
                'callback': self.setUseLastImageCb, 
                'callback_args': ()
            },
            ############
            # All Detector
            ############
            'all_detector_enable': {
                'namespace': self.all_detections_namespace,
                'topic': 'enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setEnableCb, 
                'callback_args': ()
            },
            'all_detector_set_source_topic': {
                'namespace': self.all_detections_namespace,
                'topic': 'set_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.setImageTopicCb, 
                'callback_args': ()
            },
            'all_detector_set_source_topics': {
                'namespace': self.all_detections_namespace,
                'topic': 'set_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setImageTopicsCb, 
                'callback_args': ()
            },
            'all_detector_add_source_topic': {
                'namespace': self.all_detections_namespace,
                'topic': 'add_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.addImageTopicCb, 
                'callback_args': ()
            },
            'all_detector_add_source_topics': {
                'namespace': self.all_detections_namespace,
                'topic': 'add_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.addImageTopicsCb, 
                'callback_args': ()
            },
            'all_detector_remove_source_topic': {
                'namespace': self.all_detections_namespace,
                'topic': 'remove_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.removeImageTopicCb, 
                'callback_args': ()
            },
            'all_detector_remove_source_topics': {
                'namespace': self.all_detections_namespace,
                'topic': 'remove_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.removeImageTopicsCb, 
                'callback_args': ()
            },
            'all_detector_process_source_file': {
                'namespace': self.all_detections_namespace,
                'topic': 'process_source_file',
                'msg': String,
                'qsize': 10,
                'callback': self.processImageFileCb, 
                'callback_args': ()
            },
              'all_detector_set_class': {
                'namespace': self.all_detections_namespace,
                'topic': 'set_class',
                'msg': String,
                'qsize': 10,
                'callback': self.setClassCb, 
                'callback_args': ()
            },
            'all_detector_set_classes': {
                'namespace': self.all_detections_namespace,
                'topic': 'set_classes',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setClassesCb, 
                'callback_args': ()
            },
            'all_detector_add_class': {
                'namespace': self.all_detections_namespace,
                'topic': 'add_class',
                'msg': String,
                'qsize': 10,
                'callback': self.addClassCb, 
                'callback_args': ()
            },
            'all_detector_remove_class': {
                'namespace': self.all_detections_namespace,
                'topic': 'remove_class',
                'msg': String,
                'qsize': 10,
                'callback': self.removeClassCb, 
                'callback_args': ()
            },
            'all_detector_add_all_classes': {
                'namespace': self.all_detections_namespace,
                'topic': 'add_all_classes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.addAllClassesCb, 
                'callback_args': ()
            },
            'all_detector_remove_all_classes': {
                'namespace': self.all_detections_namespace,
                'topic': 'remove_all_classes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.removeAllClassesCb, 
                'callback_args': ()
            },
            'all_detector_set_threshold': {
                'namespace': self.all_detections_namespace,
                'topic': 'set_threshold',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setThresholdCb, 
                'callback_args': ()
            },
            'all_detector_set_image_pub': {
                'namespace': self.all_detections_namespace,
                'topic': 'set_image_pub',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setPubImageCb, 
                'callback_args': ()
            },
            'all_detector_set_max_process_rate': {
                'namespace': self.all_detections_namespace,
                'topic': 'set_max_process_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxProcessRateCb, 
                'callback_args': ()
            },
            'all_detector_set_max_image_pub_rate': {
                'namespace': self.all_detections_namespace,
                'topic': 'set_max_image_pub_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxImgRateCb, 
                'callback_args': ()
            },
            'all_detector_set_use_last_image': {
                'namespace': self.all_detections_namespace,
                'topic': 'set_use_last_image',
                'msg':Bool,
                'qsize': 10,
                'callback': self.setUseLastImageCb, 
                'callback_args': ()
            },
            ############
            # Targeting
            ############
            'targeting_enable': {
                'namespace': self.targets_namespace,
                'topic': 'enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setEnableCb, 
                'callback_args': ()
            },
            'targeting_set_source_topic': {
                'namespace': self.targets_namespace,
                'topic': 'set_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.setImageTopicCb, 
                'callback_args': ()
            },
            'targeting_set_source_topics': {
                'namespace': self.targets_namespace,
                'topic': 'set_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setImageTopicsCb, 
                'callback_args': ()
            },
            'targeting_add_source_topic': {
                'namespace': self.targets_namespace,
                'topic': 'add_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.addImageTopicCb, 
                'callback_args': ()
            },
            'targeting_add_source_topics': {
                'namespace': self.targets_namespace,
                'topic': 'add_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.addImageTopicsCb, 
                'callback_args': ()
            },
            'targeting_remove_source_topic': {
                'namespace': self.targets_namespace,
                'topic': 'remove_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.removeImageTopicCb, 
                'callback_args': ()
            },
            'targeting_remove_source_topics': {
                'namespace': self.targets_namespace,
                'topic': 'remove_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.removeImageTopicsCb, 
                'callback_args': ()
            },
            'targeting_process_source_file': {
                'namespace': self.targets_namespace,
                'topic': 'process_source_file',
                'msg': String,
                'qsize': 10,
                'callback': self.processImageFileCb, 
                'callback_args': ()
            },
             'targeting_set_class': {
                'namespace': self.targets_namespace,
                'topic': 'set_class',
                'msg': String,
                'qsize': 10,
                'callback': self.setClassCb, 
                'callback_args': ()
            },
            'targeting_set_classes': {
                'namespace': self.targets_namespace,
                'topic': 'set_classes',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setClassesCb, 
                'callback_args': ()
            },
            'targeting_add_class': {
                'namespace': self.targets_namespace,
                'topic': 'add_class',
                'msg': String,
                'qsize': 10,
                'callback': self.addClassCb, 
                'callback_args': ()
            },
            'targeting_remove_class': {
                'namespace': self.targets_namespace,
                'topic': 'remove_class',
                'msg': String,
                'qsize': 10,
                'callback': self.removeClassCb, 
                'callback_args': ()
            },
            'targeting_add_all_classes': {
                'namespace': self.targets_namespace,
                'topic': 'add_all_classes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.addAllClassesCb, 
                'callback_args': ()
            },
            'targeting_remove_all_classes': {
                'namespace': self.targets_namespace,
                'topic': 'remove_all_classes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.removeAllClassesCb, 
                'callback_args': ()
            },
            'targeting_set_threshold': {
                'namespace': self.targets_namespace,
                'topic': 'set_threshold',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setThresholdCb, 
                'callback_args': ()
            },
            'targeting_set_image_pub': {
                'namespace': self.targets_namespace,
                'topic': 'set_image_pub',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setPubImageCb, 
                'callback_args': ()
            },
            'targeting_set_max_process_rate': {
                'namespace': self.targets_namespace,
                'topic': 'set_max_process_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxProcessRateCb, 
                'callback_args': ()
            },
            'targeting_set_max_image_pub_rate': {
                'namespace': self.targets_namespace,
                'topic': 'set_max_image_pub_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxImgRateCb, 
                'callback_args': ()
            },
            'targeting_set_use_last_image': {
                'namespace': self.targets_namespace,
                'topic': 'set_use_last_image',
                'msg':Bool,
                'qsize': 10,
                'callback': self.setUseLastImageCb, 
                'callback_args': ()
            },
            ############
            # All 
            ############
            'all_targeting_enable': {
                'namespace': self.all_targets_namespace,
                'topic': 'enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setEnableCb, 
                'callback_args': ()
            },
            'all_targeting_set_auto_select_enable': {
                'namespace': self.all_targets_namespace,
                'topic': 'set_auto_select_enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setAutoSelectEnableCb, 
                'callback_args': ()
            },
            'all_targeting_set_source_topic': {
                'namespace': self.all_targets_namespace,
                'topic': 'set_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.setImageTopicCb, 
                'callback_args': ()
            },
            'all_targeting_set_source_topics': {
                'namespace': self.all_targets_namespace,
                'topic': 'set_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setImageTopicsCb, 
                'callback_args': ()
            },
            'all_targeting_add_source_topic': {
                'namespace': self.all_targets_namespace,
                'topic': 'add_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.addImageTopicCb, 
                'callback_args': ()
            },
            'all_targeting_add_source_topics': {
                'namespace': self.all_targets_namespace,
                'topic': 'add_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.addImageTopicsCb, 
                'callback_args': ()
            },
            'all_targeting_remove_source_topic': {
                'namespace': self.all_targets_namespace,
                'topic': 'remove_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.removeImageTopicCb, 
                'callback_args': ()
            },
            'all_targeting_remove_source_topics': {
                'namespace': self.all_targets_namespace,
                'topic': 'remove_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.removeImageTopicsCb, 
                'callback_args': ()
            },
            'all_targeting_process_source_file': {
                'namespace': self.all_targets_namespace,
                'topic': 'process_source_file',
                'msg': String,
                'qsize': 10,
                'callback': self.processImageFileCb, 
                'callback_args': ()
            },
             'all_targeting_set_class': {
                'namespace': self.all_targets_namespace,
                'topic': 'set_class',
                'msg': String,
                'qsize': 10,
                'callback': self.setClassCb, 
                'callback_args': ()
            },
            'all_targeting_set_classes': {
                'namespace': self.all_targets_namespace,
                'topic': 'set_classes',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setClassesCb, 
                'callback_args': ()
            },
            'all_targeting_add_class': {
                'namespace': self.all_targets_namespace,
                'topic': 'add_class',
                'msg': String,
                'qsize': 10,
                'callback': self.addClassCb, 
                'callback_args': ()
            },
            'all_targeting_remove_class': {
                'namespace': self.all_targets_namespace,
                'topic': 'remove_class',
                'msg': String,
                'qsize': 10,
                'callback': self.removeClassCb, 
                'callback_args': ()
            },
            'all_targeting_add_all_classes': {
                'namespace': self.all_targets_namespace,
                'topic': 'add_all_classes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.addAllClassesCb, 
                'callback_args': ()
            },
            'all_targeting_remove_all_classes': {
                'namespace': self.all_targets_namespace,
                'topic': 'remove_all_classes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.removeAllClassesCb, 
                'callback_args': ()
            },
            'all_targeting_set_threshold': {
                'namespace': self.all_targets_namespace,
                'topic': 'set_threshold',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setThresholdCb, 
                'callback_args': ()
            },
            'all_targeting_set_image_pub': {
                'namespace': self.all_targets_namespace,
                'topic': 'set_image_pub',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setPubImageCb, 
                'callback_args': ()
            },
            'all_targeting_set_max_process_rate': {
                'namespace': self.all_targets_namespace,
                'topic': 'set_max_process_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxProcessRateCb, 
                'callback_args': ()
            },
            'all_targeting_set_max_image_pub_rate': {
                'namespace': self.all_targets_namespace,
                'topic': 'set_max_image_pub_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxImgRateCb, 
                'callback_args': ()
            },
            'all_targeting_set_use_last_image': {
                'namespace': self.all_targets_namespace,
                'topic': 'set_use_last_image',
                'msg':Bool,
                'qsize': 10,
                'callback': self.setUseLastImageCb, 
                'callback_args': ()
            },
            ############
            # Misc
            ############
            'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self.systemStatusCb
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

        #self.node_if.wait_for_ready()
        nepi_sdk.sleep(1)

        self.initCb(do_updates = True)



        self.msg_if.pub_warn("Launcing Image Pub Node")
        self.launch_image_pub_node()
        ###############################
        # Create System IFs
        # Setup States IF
        self.states_dict = {
                        "running": {
                            "name":"running",
                            "node_name": self.node_name,
                            "description": "Current detections running state",
                            "type":"Bool",
                            "options": [],
                            "value":"False"
                            },
                        "detections": {
                            "name":"detections",
                            "node_name": self.node_name,
                            "description": "Current detections state, cleared every 1 sec",
                            "type":"Bool",
                            "options": [],
                            "value":"False"
                            }
        }


        
        self.states_if_detections = StatesIF(
                        states_name = 'detectons',
                        get_states_dict_function = self.get_detections_states,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if)
                            # msg_if = self.msg_if,
                            # node_if = self.node_if
                            # )

        self.states_if_targets = StatesIF(
                        states_name = 'targets',
                        get_states_dict_function = self.get_targets_states,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if)
                            # msg_if = self.msg_if,
                            # node_if = self.node_if
                            # )



        # Setup Triggers IF
        self.triggers_dict = {
                        "detections_trigger": {
                            "name":"detections_trigger",
                            "node_name": self.node_name,
                            "description": "Triggered on AI detections",
                            "data_str_list":["None"],
                            "time":nepi_utils.get_time()
                            }

        }

        self.triggers_if = TriggersIF(triggers_dict = self.triggers_dict,
                            msg_if = self.msg_if)
                            # msg_if = self.msg_if,
                            # node_if = self.node_if
                            # )

        
        # Setup Save Data IF
        factory_data_rates= {}
        for d in self.data_products:
            factory_data_rates[d] = [1.0, 0.0, 100] 

        self.save_data_namespace = self.node_namespace + '/save_data'
        self.save_data_if = SaveDataIF(data_products = self.data_products, factory_rate_dict = factory_data_rates,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if)
                            # msg_if = self.msg_if,
                            # node_if = self.node_if
                            # )
        nepi_sdk.sleep(1)
        if self.save_data_if is not None:
            self.process_status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.msg_if.pub_info("Using save_data namespace: " + str(self.process_status_msg.save_data_topic))


        ###############################
        # Create Per-Data-Product IFs
        # DetectionsIF/TargetsIF own the detections/targets data products and
        # their per-product status messages. Given the detector node namespace
        # they publish on <node_ns>/detections[/status] and
        # <node_ns>/targets[/status] -- the same wire topics/types the removed
        # inline PUBS_DICT entries used. They share the detector's SaveDataIF so
        # detections/targets saving stays centralized (and rate-gated), and
        # follow the file convention of building their own node_if.
        self.msg_if.pub_warn("Pre DetectionsIF max_process_rate: " + str(self.max_process_rate_hz), log_name_list = self.log_name_list)
        self.detections_if = DetectionsIF(namespace = self.namespace,
                        data_product = 'detections',
                        save_data_if = self.save_data_if,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if)

        self.msg_if.pub_warn("Pre TargetsIF max_process_rate: " + str(self.max_process_rate_hz), log_name_list = self.log_name_list)
        self.targets_if = TargetsIF(namespace = self.namespace,
                        data_product = 'targets',
                        save_data_if = self.save_data_if,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if)
        nepi_sdk.sleep(1)


        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_sdk.start_timer_process((1.0), self.publishStatusCb)
        nepi_sdk.start_timer_process((0.1), self.updateImgSubsCb, oneshot = True)
        nepi_sdk.start_timer_process((0.1), self.updaterCb, oneshot = True)
        nepi_sdk.start_timer_process((0.1), self.updateNextTopicCb, oneshot = True)
        nepi_sdk.start_timer_process((1), self.processDetectionsCb, oneshot = True)

        self.msg_str = 'Loaded'
        ##########################
        self.msg_if.pub_warn("IF Initialization Complete", log_name_list = self.log_name_list)
        self.msg_if.pub_warn("max_process_rate: " + str(self.max_process_rate_hz), log_name_list = self.log_name_list)
        ##########################

    def systemStatusCb(self,msg):
        self.active_nodes = msg.active_nodes
        self.active_topics = msg.active_topics
        self.active_topic_types = msg.active_topic_types
        self.active_services = msg.active_services

    def launch_image_pub_node(self):
        """Launches the detections image publisher node as a subprocess.

        Resolves the image publisher node file path and starts the node with
        the correct namespace and parameters if the file exists and image
        publishing is enabled. Does nothing if the node is already running or
        the node file cannot be found.
        """
        node_name = self.node_name + "_img_pub"
        launch_namespace = os.path.dirname(self.node_namespace)
        node_namespace = self.node_namespace + "_img_pub"
        pkg_name = 'nepi_api'
        node_file_folder = self.api_lib_folder
        node_file_name = 'nepi_ai_detector_img_pub_node.py'

        self.msg_if.pub_warn("Launching Detction Img Node with with settings " + str([pkg_name, node_file_name, node_name]))
        ###############################
        # Launch Node
        node_file_path = os.path.join(node_file_folder, node_file_name)
        if self.launch_node_process is not None:
            self.msg_if.pub_warn("Node Already Launched: " + node_name)
        elif os.path.exists(node_file_path) == False or self.enable_image_pub == False:
            self.msg_if.pub_warn("Could not find Node File at: " + node_file_path)
        else:

            [success, msg, sub_process] = nepi_sdk.launch_node(pkg_name, node_file_name, node_name, namespace=launch_namespace)
            if success == True:
                self.launch_node_process = sub_process
                self.pub_img_node_name = node_name
                self.pub_img_namepace = node_namespace
            self.msg_if.pub_warn("Node launch return msg: " + str(msg))

    def kill_image_pub_node(self):
        """Terminates the running detections image publisher node.

        Sends a kill signal to the subprocess started by
        ``launch_image_pub_node`` and clears the process handle and node name
        on success. Logs a warning if the node is not currently running.
        """
        if self.launch_node_process is None:
            self.msg_if.pub_warn("Node Not Running")
        else:
            self.msg_if.pub_warn("Killing Node")
            success = nepi_sdk.kill_node_process(self.pub_img_node_name, self.launch_node_process)
            if success == True:
                self.launch_node_process = None
                self.pub_img_node_name = ""
                self.pub_img_namepace = ""
                self.msg_if.pub_warn("Node Killed")
            else:
                self.msg_if.pub_warn("Failed to Kill Node")


    def get_detections_states(self):
        """Returns the current states dictionary.

        Used as a callback by the StatesIF to retrieve live state values.

        Returns:
            dict: The states dictionary containing running and detections state
                entries.
        """
        states_dict = dict()
        
        states_dict['detections'] = self.detecting_state
        states_dict[self.node_name + '/detections'] = self.detecting_state
        self.detecting_state = False
        return self.states_dict

    def get_targets_states(self):
        """Returns the current states dictionary.

        Used as a callback by the StatesIF to retrieve live state values.

        Returns:
            dict: The states dictionary containing running and detections state
                entries.
        """
        states_dict = dict()
        
        states_dict['targets'] = self.targeting_state
        states_dict[self.node_name + '/targets'] = self.targeting_state
        self.targeting_state = False
       
        return states_dict

    def save_config(self):
        if self.node_if is not None:
            self.node_if.save_config()  


    def initCb(self,do_updates = False):
        self.msg_if.pub_info("Setting init values to param values", log_name_list = self.log_name_list)
        if self.node_if is not None:
            self.imaging_enabled = self.node_if.get_param('imaging_enabled')
            self.enabled = self.node_if.get_param('enabled')
            self.selected_classes = self.node_if.get_param('selected_classes')
            self.selected_classes_targets = self.node_if.get_param('selected_classes')
            self.threshold = self.node_if.get_param('threshold')
            self.max_process_rate_hz = self.node_if.get_param('max_process_rate_hz')
            self.msg_if.pub_warn("Init max process rate: " + str(self.max_process_rate_hz), log_name_list = self.log_name_list)
            self.max_image_pub_rate_hz = self.node_if.get_param('max_image_pub_rate_hz')
            self.use_last_image = self.node_if.get_param('use_last_image')

            # Restore the auto-select flags directly, the way every other value in
            # this method is restored. Routing them through setAutoSelectEnable ran
            # its first statement -- self.selected_sources = [] -- and wiped the
            # selection that had just been read back on the line above, so a saved
            # image source always came back empty.
            auto_select_enabled = self.node_if.get_param('auto_select_enabled')
            self.auto_select_enabled = auto_select_enabled
            self.auto_select_active = auto_select_enabled
            self.selected_sources = self.node_if.get_param('selected_sources')
            self.msg_if.pub_info("Init selected images: " + str(self.selected_sources), log_name_list = self.log_name_list)

            
        if do_updates == True:
            pass
        self.publish_status()

    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass # self.node_if.reset_params()
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def factoryResetCb(self,do_updates = True):
        if self.node_if is not None:
            pass # self.node_if.factory_reset_params()
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    ##########################################

    def setEnableCb(self,msg):
        #self.msg_if.pub_warn("Received AI enable msg: " + str(msg))
        enabled = msg.data
        self.setEnable(enabled)



    def setEnable(self,enabled):
        if self.enabled != enabled:
            self.enabled = enabled
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('enabled',self.enabled)
                
            if enabled == False and not nepi_sdk.is_shutdown():
                self.next_source_topic = "None"

            

    def setAutoSelectEnableCb(self,msg):
        #self.msg_if.pub_warn("Received AI auto select source topic msg: " + str(msg))
        enabled = msg.data
        self.setAutoSelectEnable(enabled)


    def setAutoSelectEnable(self, enabled):
        self.selected_sources = []
        self.auto_select_active = enabled
        self.auto_select_enabled = enabled
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('auto_select_enabled',self.auto_select_enabled)
            
       

    def setImageTopicCb(self,msg):
        #self.msg_if.pub_info("Received Set Image Topic: " + msg.data)
        source_topic = msg.data
        self.setImageTopic(source_topic)


    def setImageTopic(self, source_topic):
        #self.msg_if.pub_info("Set Image Topic: " + source_topic)     
        if self.selected_sources != [source_topic]:    
            self.selected_sources = [source_topic]
            self.auto_select_active = False
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_sources',self.selected_sources)
                

    def setImageTopicsCb(self,msg):
        #self.msg_if.pub_info("Received Set Image Topic: " + msg.data)
        source_topics = msg.data
        self.setImageTopics(source_topics)


    def setImageTopics(self, source_topics):
        #self.msg_if.pub_info("Set Image Topics: " + str(source_topics))     
        if self.selected_sources != source_topics:    
            self.selected_sources = source_topics
            self.auto_select_active = False
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_sources',self.selected_sources)
                


    def addImageTopicCb(self,msg):
        #self.msg_if.pub_info("Received Add Image Topic: " + msg.data)
        source_topic = msg.data
        self.addImageTopic(source_topic)


    def addImageTopicsCb(self,msg):
        #self.msg_if.pub_info("Received Add Image Topics: " + str(msg))
        source_topic_list = msg.array
        for source_topic in source_topic_list:
            self.addImageTopic(source_topic)


    def addImageTopic(self,source_topic):   
        #self.msg_if.pub_info("Adding Image Topic: " + source_topic)
        if source_topic not in self.selected_sources: 
            source_topics = copy.deepcopy(self.selected_sources)
            if source_topic not in source_topics:
                source_topics.append(source_topic)
            else:
                self.msg_if.pub_warn('Image topic allready selected')
            self.selected_sources = source_topics
            self.auto_select_active = False
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_sources',self.selected_sources)
                


    def removeImageTopicCb(self,msg):
        #self.msg_if.pub_info("Received Remove Image Topic: " + str(msg))
        source_topic = msg.data
        self.removeImageTopic(source_topic)


    def removeImageTopicsCb(self,msg):
        #self.msg_if.pub_info("Received Remove Image Topic: " + str(msg))
        source_topic_list = msg.array
        for source_topic in source_topic_list:
            self.removeImageTopic(source_topic)



    def removeImageTopic(self,source_topic):
        #self.msg_if.pub_info("Removing Image Topic: " + source_topic)  
        if source_topic in self.selected_sources:       
            source_topics = copy.deepcopy(self.selected_sources)
            if source_topic in source_topics:
                source_topics.remove(source_topic)
            self.selected_sources = source_topics
            self.auto_select_active = False
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_sources',self.selected_sources)
                


    ###################
    # Process Functions

    def setClassCb(self,msg):
        #self.msg_if.pub_info("Received Set class: " + msg.data)
        class_name = msg.data
        self.setClass(class_name)


    def setClass(self, class_name):
        #self.msg_if.pub_info("Set Class: " + class_name)      
        if self.selected_classes != [class_name]:  
            self.selected_classes = [class_name]
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_classes',self.selected_classes)
                

    def setClassesCb(self,msg):
        #self.msg_if.pub_info("Received Set classes: " + msg.data)
        class_names = msg.data
        self.setClasses(class_names)


    def setClasses(self, class_names):
        #self.msg_if.pub_info("Set Class: " + class_name) 
        if self.selected_classes != class_names:        
            self.selected_classes = class_names
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_classes',self.selected_classes)
                


    def addAllClassesCb(self,msg):
        #self.msg_if.pub_info('Got add all classes msg: ' + str(msg))
        self.addAllClasses()

    def addAllClasses(self):
        self.publish_status() # Updated Here
        if self.selected_classes != self.classes:
            self.selected_classes = self.classes
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_classes', self.classes)
                


    def removeAllClassesCb(self,msg):
        #self.msg_if.pub_info('Got remove all classes msg: ' + str(msg))
        if len(self.selected_classes) > 0:
            self.selected_classes = []
            self.publish_status() # Updated Here
            if self.node_if is not None:
                self.node_if.set_param('selected_classes',[])
                


    def addClassCb(self,msg):
        #self.msg_if.pub_info('Got add class msg: ' + str(msg))
        class_name = msg.data
        if class_name in self.classes and class_name not in self.selected_classes:
            sel_classes = copy.deepcopy(self.selected_classes)
            if class_name not in sel_classes:
                sel_classes.append(class_name)
            self.selected_classes = sel_classes
            self.publish_status() # Updated Here
            if self.node_if is not None:
                self.node_if.set_param('selected_classes', sel_classes)
                


    def removeClassCb(self,msg):
        #self.msg_if.pub_info('Got remove class msg: ' + str(msg))
        class_name = msg.data
        
        if class_name in self.selected_classes:
            sel_classes = copy.deepcopy(self.selected_classes)
            sel_classes.remove(class_name)
            self.selected_classes = sel_classes
            self.publish_status() # Updated Here
            if self.node_if is not None:
                self.node_if.set_param('selected_classes', sel_classes)
                



    def setThresholdCb(self,msg):
        threshold = msg.data
        self.setThreshold(threshold)

    def setThreshold(self,threshold):
        #self.msg_if.pub_info("Received Threshold Update: " + str(threshold))
        if threshold <  MIN_THRESHOLD:
            threshold = MIN_THRESHOLD
        elif threshold > MAX_THRESHOLD:
            threshold = MAX_THRESHOLD
        if self.threshold != threshold:
            self.threshold = threshold
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('threshold',self.threshold)
                


    def setMaxProcessRateCb(self,msg):
        max_rate = msg.data
        if max_rate <  MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        if max_rate != self.max_process_rate_hz:
            self.max_process_rate_hz = max_rate
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('max_process_rate_hz',self.max_process_rate_hz)
                

 


    def setMaxImgRateCb(self,msg):
        max_rate = msg.data
        if max_rate <  MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        if max_rate != self.max_image_pub_rate_hz:
            self.max_image_pub_rate_hz = max_rate
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('max_image_pub_rate_hz',self.max_image_pub_rate_hz)
                

    def setUseLastImageCb(self,msg):
        enable = msg.data
        if self.use_last_image != enable:
            self.use_last_image = enable
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('use_last_image',self.use_last_image)
                





    def setPubImageCb(self,msg):
        enable = msg.data
        self.set_pub_image(enable)




    def set_pub_image(self,enable):
        """Enables or disables image publishing and persists the setting.

        Args:
            enable (bool): True to enable detections image publishing,
                False to disable it.
        """
        if self.imaging_enabled != enable:
            self.imaging_enabled = enable
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('imaging_enabled',self.imaging_enabled)
                



    ###############.########################
    # Class Functions

    def statusPublishCb(self,timer):
        self.publish_status()



        
    def updaterCb(self,timer):
        #self.msg_if.pub_warn("Updating with image topic: " +  self.source_topic)
        selected_sources = copy.deepcopy(self.selected_sources)
        active_source_topics = copy.deepcopy(self.active_source_topics)

        ##############
        last_available = copy.deepcopy(self.available_source_topics)
        
        topics = nepi_sdk.find_topics_by_msg('Image', topics_list = self.active_topics, types_list = self.active_topic_types)
        available_source_topics = []
        for topic in topics:
            valid_topic = False
            for filter in self.IMAGE_FILTERS:
                if filter in topic:
                    valid_topic = True
            if valid_topic == True:
                available_source_topics.append(topic)
        if available_source_topics != last_available:
            self.available_source_topics = available_source_topics
            needs_publish = True

        ##############
        # self.msg_if.pub_warn("")
        # self.msg_if.pub_warn("Updating with available topics: " +  str(available_source_topics))
        # self.msg_if.pub_warn("Updating with selected topics: " +  str(selected_sources))
        # self.msg_if.pub_warn("Updating with active topics: " +  str(active_source_topics))
        purge_list = []
        # Update Image subscribers
        if len(selected_sources) == 0 and len(available_source_topics) > 0 and self.auto_select_enabled == True and self.auto_select_active == True:
            selected_sources = []
            for topic in available_source_topics:
                if 'idx' in topic:
                    self.selected_sources = [topic]
                    break
        
        for source_topic in selected_sources:
            if os.path.basename(source_topic) in self.OUTPUT_IMG_PRODUCTS:
                continue
            if source_topic not in active_source_topics and source_topic in available_source_topics:
                self.msg_if.pub_warn('Will subscribe to image topic: ' + source_topic)
                if source_topic not in self.active_source_topics:
                    self.active_source_topics.append(source_topic)
                success = self.subscribeImgTopic(source_topic)
        # Update Image Subs purge list
        for source_topic in active_source_topics:
            if source_topic not in available_source_topics or source_topic not in selected_sources:
                purge_list.append(source_topic)
        if len(purge_list) > 0:
            self.msg_if.pub_warn('Purging image topics: ' + str(purge_list))
        for topic in purge_list:
            self.msg_if.pub_warn('Will unsubscribe from image topic: ' + topic)
            try:
                self.active_source_topics.remove(source_topic)
            except:
                pass
            success = self.unsubscribeImgTopic(topic)

        # Check Image connected state
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        img_connects = []
        for source_topic in imgs_info_dict.keys():
            img_connects.append(imgs_info_dict[source_topic]['img_connected'])
        img_selected = len(img_connects) > 0
        img_connected = True in img_connects
       
        # Set Detector State
        enabled = self.enabled
        if enabled == True:
            if img_selected == 0:
                self.msg_str = "Waiting"
            elif img_connected == False:
                self.msg_str = "Listening"
            else:
                self.msg_str = "Detecting"
        else: # Loaded, but not enabled
            self.msg_str = "Disabled"



        ### Check on image topic data subs
        for source_topic in imgs_info_dict.keys(): 
            info_dict = imgs_info_dict[source_topic] 
            active = (source_topic in active_source_topics)
            depth_map_topic = info_dict['depth_map_topic']

            data_subs_dict = dict()
            ## Check on depth map
            dm_update_state = None
            #self.msg_if.pub_warn("Checking on depth map topic: " + str(depth_map_topic) + " with active state: " + str(active))
            if depth_map_topic != '':
                if active and (info_dict['depth_map_connected'] == False and info_dict['depth_map_connecting'] == False):
                    self.msg_if.pub_warn("Subscribing to depth map topic: " + str(depth_map_topic))
                    data_subs_dict.update( {
                        'np_depth_map': {
                                'namespace': depth_map_topic,
                                'msg': Image,
                                'topic': '',
                                'qsize': 1,
                                'callback': self.depthMapCb,
                                'callback_args': (source_topic)
                            }
                        }
                    )

                self.imgs_info_dict[source_topic]['depth_map_connecting'] = True
                self.imgs_info_dict[source_topic]['depth_map_connected'] = False

            if len(list(data_subs_dict.keys())) > 0:
                    self.img_ifs_lock.acquire()
                    self.img_ifs_dict[source_topic]['subs_if'].add_subs(data_subs_dict)
                    self.img_ifs_lock.release()



        nepi_sdk.start_timer_process((.1), self.updaterCb, oneshot = True)


    def subscribeImgTopic(self,source_topic):
        if source_topic == "None" or source_topic == "":
            return False

        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        if source_topic in imgs_info_dict.keys():
            img_info_dict = imgs_info_dict[source_topic]
            connecting = img_info_dict['img_connecting']
            connected = img_info_dict['img_connected']
            if connecting == True or connected == True:
                return False
            

        self.msg_if.pub_warn("Subscribing to image topic: " + str(source_topic))
        ####################
        # Create Pubs and Subs IF Dict 

        img_pubs_dict = {
            'detections_pub': {
                'msg': Detections,
                'namespace': source_topic,
                'topic': 'detections',
                'qsize': 1,
                'latch': False
            },
            'detections_status_pub': {
                'msg': Targets,
                'namespace': source_topic + '/detections',
                'topic': 'status',
                'qsize': 1,
                'latch': False
            },
            'targets_pub': {
                'msg': Targets,
                'namespace': source_topic,
                'topic': 'targets',
                'qsize': 1,
                'latch': False
            },
            'targets_status_pub': {
                'msg': Targets,
                'namespace': source_topic + '/targets',
                'topic': 'status',
                'qsize': 1,
                'latch': False
            }
        }

        img_subs_dict = {
            'image_sub': {
                    'namespace': source_topic,
                    'msg': Image,
                    'topic': '',
                    'qsize': 1,
                    'callback': self.imageCb,
                    'callback_args': (source_topic)
            },
            'status_sub': {
                    'namespace': source_topic,
                    'msg': ImageStatus,
                    'topic': 'status',
                    'qsize': 1,
                    'callback': self.imageStatusCb,
                    'callback_args': (source_topic)
            }
        }
    


        
        # Check if exists
        if source_topic in imgs_info_dict.keys():
            self.imgs_info_dict[source_topic]['img_connected'] = False
            self.imgs_info_dict[source_topic]['img_connecting'] = True
            self.msg_if.pub_info('Subsribing to image topic: ' + source_topic)  
            # Try and Reregister subs and pubs
            self.img_ifs_lock.acquire()
            self.img_ifs_dict[source_topic]['subs_if'].register_subs(img_subs_dict)
            self.img_ifs_dict[source_topic]['pubs_if'].register_pubs(img_pubs_dict)
            self.img_ifs_lock.release()
            self.msg_if.pub_warn('Registered : ' + source_topic +  ' ' + str(self.img_ifs_dict[source_topic]))
            # Set back to active
            return True
        else:
                
            # Create register new image topic
            self.msg_if.pub_warn('Registering to image topic: ' + source_topic)
            img_base_namespace = os.path.dirname(source_topic) 
            img_pub_topic = os.path.join(img_base_namespace,'detections_image')
            self.msg_if.pub_warn('Publishing on namespace: ' + img_pub_topic)

            ####################
            # Create img info dict
            img_info_dict = dict()  
            img_info_dict['img_source_topic'] = source_topic
            img_info_dict['img_pub_topic'] = img_pub_topic 
            img_info_dict['img_connecting'] = True
            img_info_dict['img_connected'] = False 


            img_info_dict['has_range'] = False
            img_info_dict['width_deg'] = 110
            img_info_dict['height_deg'] = 70

            img_info_dict['napose_topic'] = ''
            img_info_dict['napose_connecting'] = False
            img_info_dict['napose_connected'] = False
            img_info_dict['napose_last_connection'] = 0

            img_info_dict['depth_map_topic'] = ''
            img_info_dict['depth_map_connecting'] = False
            img_info_dict['depth_map_connected'] = False
            img_info_dict['depth_map_last_connection'] = 0

            img_info_dict['pointcloud_topic'] = ''
            img_info_dict['pointcloud_connecting'] = False
            img_info_dict['pointcloud_connected'] = False
            img_info_dict['pointcloud_last_connection'] = 0
            

            self.imgs_info_dict[source_topic] = img_info_dict

            self.msg_if.pub_info('Subsribing to image topic: ' + source_topic)


            #####################
            ## Initialized Data Dictionaries
            self.navpose_dict_lock.acquire()
            self.navpose_dict[source_topic] = None
            self.navpose_dict_lock.release()

            self.depth_map_dict_lock.acquire()
            self.depth_map_dict[source_topic] = None
            self.depth_map_dict_lock.release()

            self.pointcloud_dict_lock.acquire()
            self.pointcloud_dict[source_topic] = None
            self.pointcloud_dict_lock.release()

            
            ####################
            # Pubs Config Dict 
            img_pubs_if = NodePublishersIF(
                    pubs_dict = img_pubs_dict,
                    log_name_list = self.log_name_list,
                    msg_if = self.msg_if
                                        )

            ####################
            # Subs Config Dict 
            img_subs_if = NodeSubscribersIF(
                    subs_dict = img_subs_dict,
                    log_name_list = self.log_name_list,
                        msg_if = self.msg_if)
                        # msg_if = self.msg_if,
                        # node_if = self.node_if
                        # )





            ####################
            # Add Img Subs and Pubs IFs to Img IFs Dict
            self.img_ifs_lock.acquire()
            self.img_ifs_dict[source_topic] = {
                                            'pubs_if': img_pubs_if,
                                            'subs_if': img_subs_if
                                            }   

            self.img_ifs_lock.release()
            self.msg_if.pub_warn('Registered : ' + source_topic)

            time.sleep(1)
            ####################

            ####################
            # Create Img Sub Dict
            self.imgs_has_subs_dict[source_topic] = False

            ####################
            # Create Img Dict
            img_dict = dict()
            img_dict['lock'] = threading.Lock()
            img_dict['topic'] = source_topic
            img_dict['timestamp'] = nepi_utils.get_time()
            img_dict['cv2_img'] = None
            self.images_dict[source_topic] = img_dict


            return True

                

    def unsubscribeImgTopic(self,source_topic):

        # Unsubsribe from Pubs and Subs
        if source_topic in self.img_ifs_dict.keys():
            self.msg_if.pub_warn('Unregistering image topic subs for: ' + source_topic)
            self.img_ifs_lock.acquire()
            self.img_ifs_dict[source_topic]['subs_if'].unregister_subs()
            self.img_ifs_dict[source_topic]['pubs_if'].unregister_pubs()
            self.img_ifs_lock.release()
            #Leave img pub running in case it is switched back on
        if source_topic in self.img_ifs_dict.keys():
            # Clear info dict
            self.imgs_info_dict[source_topic]['img_connecting'] = False
            self.imgs_info_dict[source_topic]['img_connected'] = False 
            self.imgs_info_dict[source_topic]['image_latency_time'] = 0
            self.imgs_info_dict[source_topic]['detect_latency_time'] = 0
            self.imgs_info_dict[source_topic]['image_time'] = 0 
            self.imgs_info_dict[source_topic]['detect_time'] = 0 

            self.imgs_info_dict[source_topic]['napose_topic'] = ''
            self.imgs_info_dict[source_topic]['napose_connecting'] = False
            self.imgs_info_dict[source_topic]['napose_connected'] = False
            self.imgs_info_dict[source_topic]['napose_last_connection'] = 0

            self.imgs_info_dict[source_topic]['depth_map_topic'] = ''
            self.imgs_info_dict[source_topic]['depth_map_connecting'] = False
            self.imgs_info_dict[source_topic]['depth_map_connected'] = False
            self.imgs_info_dict[source_topic]['depth_map_last_connection'] = 0

            self.imgs_info_dict[source_topic]['pointcloud_topic'] = ''
            self.imgs_info_dict[source_topic]['pointcloud_connecting'] = False
            self.imgs_info_dict[source_topic]['pointcloud_connected'] = False
            self.imgs_info_dict[source_topic]['pointcloud_last_connection'] = 0

            # Clear Img Dict
        if source_topic in self.images_dict.keys():
            try:
                self.images_dict[source_topic]['lock'].acquire()
                self.images_dict[source_topic]['timestamp'] = nepi_utils.get_time()
                self.images_dict[source_topic]['cv2_img'] = None
                self.images_dict[source_topic]['lock'].release()
            except:
                pass


        #####################
        ## Clear Data Dictionaries
        self.navpose_dict_lock.acquire()
        self.navpose_dict[source_topic] = None
        self.navpose_dict_lock.release()

        self.depth_map_dict_lock.acquire()
        self.depth_map_dict[source_topic] = None
        self.depth_map_dict_lock.release()

        self.pointcloud_dict_lock.acquire()
        self.pointcloud_dict[source_topic] = None
        self.pointcloud_dict_lock.release()


        # Clear Img Subs Dict
        self.imgs_has_subs_dict[source_topic] = False

        return True





    def updateNextTopicCb(self,timer):
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        active_source_topics = copy.deepcopy(self.active_source_topics)
        enabled = self.enabled
        if enabled == True:
            source_topics = self.selected_sources
            connected_list = []
            for topic in source_topics:
                if topic in imgs_info_dict.keys():
                    if imgs_info_dict[topic]['img_connected'] == True:
                        connected_list.append(topic)
            if len(connected_list) == 0:
                #self.msg_if.pub_warn("No Connected Image Topics")
                self.cur_source_topic = "None"
                self.next_source_topic = "None"
            else:
                

                # Get image topic info
                cur_source_topic = copy.deepcopy(self.cur_source_topic)

                # Setup Next Img if needed
                num_connected_list = len(connected_list)
                if num_connected_list > 0:
                    if cur_source_topic in connected_list:
                        next_img_ind = connected_list.index(cur_source_topic) + 1
                        if next_img_ind >= num_connected_list:
                            self.next_source_topic = connected_list[0]
                        else:
                            self.next_source_topic = connected_list[next_img_ind]
                    else:
                        self.next_source_topic = connected_list[0]
                else:
                    self.next_source_topic = "None"

                # Check if current image topic active
                if cur_source_topic is not None and cur_source_topic in imgs_info_dict.keys():
                    active = cur_source_topic in active_source_topics
                    if active == False:
                        self.cur_source_topic = "None"

                # Check if image topic has been initialized.
                if cur_source_topic == "None" and self.next_source_topic != "None":
                    self.msg_if.pub_warn("Initializing get topic to: " +  self.next_source_topic)
                    self.got_source_topic = None
                    self.cur_source_topic = copy.deepcopy(self.next_source_topic)
                    self.get_source_topic = copy.deepcopy(self.next_source_topic)
                    #self.last_detect_time = nepi_sdk.get_time()


                ##############################
                # Check for non responding image streams              
                last_detect_delay = round((nepi_utils.get_time() - self.last_detect_time), 3)     
                if self.got_source_topic is None and last_detect_delay > (GET_IMAGE_TIMEOUT_SEC):
                    #self.msg_if.pub_warn("Topic " + cur_source_topic + " timed out. Setting next topic to: " +  self.next_source_topic)
                    if cur_source_topic is not None and cur_source_topic in imgs_info_dict.keys():
                        imgs_info_dict[cur_source_topic]['img_connected'] = False
                    self.cur_source_topic = self.next_source_topic
                    #self.last_detect_time = nepi_sdk.get_time()

                elif self.got_source_topic is None:
                    # Set Next Image Topic on Delay
                    self.cur_source_topic = copy.deepcopy(self.next_source_topic)
                    self.get_source_topic = copy.deepcopy(self.next_source_topic)
                    #self.msg_if.pub_warn("Get Topic set to " + self.get_source_topic + " with time: " +  str(timer))
    
            
                    #self.msg_if.pub_warn("Reset Current and Get Image Topic: " +  self.cur_source_topic)
                    #self.msg_if.pub_warn("With Delay and Timer: " + str(delay_time) + " " + str(timer))
        # self.msg_if.pub_warn("Current Image Topic set to: " + str(self.cur_source_topic))
        # self.msg_if.pub_warn("Get Image Topic set to: " + str(self.get_source_topic))
        # self.msg_if.pub_warn("Got Image Topic set to: " + str(self.got_source_topic))
        # self.msg_if.pub_warn("Next Image Topic set to: " + str(self.next_source_topic))
                    
        nepi_sdk.start_timer_process((0.01), self.updateNextTopicCb, oneshot = True)


    def depthMapCb(self,img_msg, args):     
        source_topic = args
        
        if self.depth_map_dict[source_topic] is None:
             self.msg_if.pub_warn("Depth Map Connected Image Topic : " + source_topic)
        #self.msg_if.pub_warn("Get Image Topic set to: " + self.get_source_topic)

        # self.imgs_info_dict[source_topic]['img_connected'] = True
        
        # stamp = img_msg.header.stamp
        # timestamp = copy.deepcopy(float(stamp.to_sec()))
        np_depth_map = nepi_img.rosimg_to_cv2img(img_msg)
        self.depth_map_dict_lock.acquire()
        self.depth_map_dict[source_topic] = np_depth_map
        self.depth_map_dict_lock.release()

        self.imgs_info_dict[source_topic]['depth_map_last_connection'] = nepi_utils.get_time()
        self.imgs_info_dict[source_topic]['depth_map_connecting'] = False
        self.imgs_info_dict[source_topic]['depth_map_connected'] = True





    def imageStatusCb(self,status_msg, args):     
        source_topic = args
        if source_topic in self.imgs_info_dict.keys():
            self.imgs_info_dict[source_topic]['width_deg'] = status_msg.width_deg
            self.imgs_info_dict[source_topic]['height_deg'] = status_msg.height_deg
            self.imgs_info_dict[source_topic]['navpose_topic'] = status_msg.navpose_topic
            self.imgs_info_dict[source_topic]['depth_map_topic'] = status_msg.depth_map_topic
            self.imgs_info_dict[source_topic]['pointcloud_topic'] = status_msg.pointcloud_topic


    def imageCb(self,img_msg, args):     
        source_topic = args
        
        #self.msg_if.pub_warn("Recieved Image Topic : " + source_topic)
        #self.msg_if.pub_warn("Get Image Topic set to: " + self.get_source_topic)

        self.imgs_info_dict[source_topic]['img_connected'] = True
        self.imgs_info_dict[source_topic]['img_connecting'] = False
        
        stamp = img_msg.header.stamp
        timestamp = copy.deepcopy(float(stamp.to_sec()))
        
        ###############################
        source_receive_latency = round(nepi_sdk.get_time() - timestamp, 3)
        source_receive_delay = (nepi_sdk.get_time() - self.last_receive_source_time)
        if source_receive_delay > 0.01:

            self.source_receive_latencies.pop(0)
            self.source_receive_latencies.append(source_receive_latency)

            source_receive_rate = round( 1.0 / source_receive_delay , 3)
            self.source_receive_rates.pop(0)
            self.source_receive_rates.append(source_receive_rate)

            self.last_receive_source_time = nepi_sdk.get_time()

        #####################################
        if source_topic == self.get_source_topic: #and self.got_source_topic is None:   
            self.got_source_topic = source_topic

            #self.msg_if.pub_warn("Processing Image Topic " + source_topic)    

            

            # Update img_dict
            # img_dict = dict()
            
            cv2_img = nepi_img.rosimg_to_cv2img(img_msg)
            
            if source_topic in self.images_dict.keys():
                try:
                    self.images_dict[source_topic]['lock'].acquire()
                    self.images_dict[source_topic]['topic'] = source_topic
                    self.images_dict[source_topic]['timestamp'] = timestamp 
                    self.images_dict[source_topic]['cv2_img'] = cv2_img    
                    self.images_dict[source_topic]['lock'].release()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to read from img_dict " + str(source_topic) + " : " + str(e))    
                    





    


    def processImageFileCb(self,str_msg):    
        source_file = str_msg.data
        source_topic = source_file


        ##############################
        ### Get CV2 Image)
        if os.path.exists(source_file) == False:
            self.msg_if.pub_warn("Process Image File Failed. Image File Not Found:  " + source_file)
            return 
        
        source_file_processing = True

        timestamp = nepi_sdk.get_time() 
        
        # Update images_dict

        if 'file' not in self.images_dict.keys():
            img_dict = dict()
            img_dict['source_file'] = source_file
            img_dict['timestamp'] = timestamp
            img_dict['lock'] = threading.Lock()
            self.images_dict['file'] = img_dict
            
        else:
            self.images_dict['file']['lock'].acquire()
            self.images_dict['file']['source_file'] = source_file
            self.images_dict['file']['timestamp'] = timestamp   
            self.images_dict['file']['lock'].release()



    def processDetectionsCb(self,timer):
        start_time = nepi_sdk.get_time()  
        if self.is_processing == True:
            return
        
        ##############################
        ### Get CV2 Image
        ###############################

        cv2_img = None
        img_dict = None
        source_file = None
        source_topic = copy.deepcopy(self.got_source_topic)
        if 'file' in self.images_dict.keys():
            img_dict = dict()
            self.images_dict['file']['lock'].acquire()
            source_file = copy.deepcopy(self.images_dict['file']['source_file'])
            self.images_dict['file']['source_file'] = None
            img_dict['topic'] = source_file
            img_dict['timestamp'] = self.images_dict['file']['timestamp']    
            self.images_dict['file']['lock'].release()
        if source_file is None and  source_topic is not None:
            if source_topic in self.images_dict.keys():
                img_dict = dict()
                self.images_dict[source_topic]['lock'].acquire()
                img_dict['topic'] = self.images_dict[source_topic]['topic']
                img_dict['timestamp'] = self.images_dict[source_topic]['timestamp'] 
                cv2_img = copy.deepcopy(self.images_dict[source_topic]['cv2_img'])   
                self.images_dict[source_topic]['cv2_img'] = None
                self.images_dict[source_topic]['lock'].release()
                if cv2_img is not None:
                    self.got_source_topic = None

        #####################################
        if cv2_img is not None or source_file is not None:
            ##############################
            ### Start Processing
            ###############################
            self.is_processing = True
            
            source_topic = img_dict['topic']
            timestamp = img_dict['timestamp']
            preprocess_time = round( (nepi_sdk.get_time() - start_time ) , 3)
            self.preprocess_times.pop(0)
            self.preprocess_times.append(preprocess_time)
            # self.msg_if.pub_warn("")
            # self.msg_if.pub_warn("Image_Process Timestamp: " + str(timestamp))
            # self.msg_if.pub_warn("Image_Process Time: " + str(preprocess_latency))
            # self.msg_if.pub_warn("Image_Process Times: " + str(self.preprocess_latencies))

            
            #####################################
            # Update Depth Map Data if available
            np_depth_map = None
            if cv2_img is not None and source_topic in self.imgs_info_dict.keys():
                depth_map_connected = self.imgs_info_dict[source_topic]['depth_map_connected']
                depth_map_last_connection = self.imgs_info_dict[source_topic]['depth_map_last_connection']
                depth_map_age = start_time - depth_map_last_connection
                if depth_map_connected == True and depth_map_age < 1:
                    self.depth_map_dict_lock.acquire()
                    np_depth_map = copy.deepcopy(self.depth_map_dict[source_topic])
                    self.depth_map_dict_lock.release()

            preprocess_latency = (nepi_sdk.get_time() - timestamp)
            self.preprocess_latencies.pop(0)
            self.preprocess_latencies.append(preprocess_latency)

            preprocess_rate = round( 1.0 / (nepi_sdk.get_time() - start_time) , 3)
            self.preprocess_rates.pop(0)
            self.preprocess_rates.append(preprocess_rate)

            ##############################
            # Process Detections
            detect_dict_list = []
            detect_dicts = []
            threshold = self.threshold
            start_process_time = nepi_sdk.get_time()
            try:
                ##################################
                if cv2_img is not None:
                    [detect_dicts, img_dict] = self.processImage(cv2_img, img_dict, threshold = threshold, resize = False, verbose = False) 
                elif source_file is not None:
                    [detect_dicts, img_dict] = self.processFile(source_file, img_dict, threshold = threshold, resize = False, verbose = False) 
                #self.msg_if.pub_warn("AIF got img_dict: " + str(img_dict))
                #self.msg_if.pub_warn("AIF got back detect_dict: " + str(detect_dicts))
                ##################################
                success = True
                self.first_detect_complete = True
            except Exception as e:
                nepi_sdk.sleep(1)
                self.msg_if.pub_warn("Failed to process detections img with exception: " + str(e))
            self.is_processing = False


            #self.msg_if.pub_warn("Processed Image Topic " + source_topic) 
            timestamp = nepi_utils.get_time()
            self.last_detect_time = nepi_sdk.get_time()
            ##############################
            # Publish Detections
            # Filter selected classes
            sel_classes = copy.deepcopy(self.selected_classes)
            sel_detect_ind = []
            for i, detect in enumerate(detect_dicts):
                if detect['name'] in sel_classes:
                    sel_detect_ind.append(i)
            for ind in sel_detect_ind:
                detect_dict_list.append(detect_dicts[ind])


            ###############################
            process_time = round( (nepi_sdk.get_time() - start_process_time ) , 3)
            self.process_times.pop(0)
            self.process_times.append(process_time)

            ##################################
            self.publishDetectionsData(source_topic, img_dict, detect_dict_list, timestamp, np_depth_map = np_depth_map)
            self.publishTargetsData(source_topic, img_dict, detect_dict_list, timestamp, np_depth_map = np_depth_map)
            ##################################

            process_latency = (nepi_sdk.get_time() - timestamp)

            self.process_latencies.pop(0)
            self.process_latencies.append(process_latency)

            process_rate = round( 1.0 / (nepi_sdk.get_time() - self.last_process_detect_time) , 3)
            self.process_rates.pop(0)
            self.process_rates.append(process_rate)
            self.last_process_detect_time = nepi_sdk.get_time()

        #####################################
        process_time = nepi_utils.get_time() - start_time
        max_rate = self.max_process_rate_hz
        delay_time = (float(1) / max_rate) - process_time
        if delay_time < 0.01:
            delay_time = 0.01

        #self.msg_if.pub_warn("Delay and Timer: " + str(delay_time) + " " + str(timer))
        nepi_sdk.start_timer_process((delay_time), self.processDetectionsCb, oneshot = True)
                

    def cleanBoxes(self,detect_dict_list):
        size_dict = dict()
        detect_dict = dict()
        sorted_dict = dict()
        center_dict = dict()
        clean_dict = dict()
        clean_boxes = []
        for class_name in self.classes:
            size_dict[class_name] = []
            detect_dict[class_name] = []
            sorted_dict[class_name] = []
            center_dict[class_name] = []
            clean_dict[class_name] = []
        for det in detect_dict_list:
            class_name = det['name']
            xmin = det['xmin']
            ymin = det['ymin']
            xmax = det['xmax']
            ymax = det['ymax']
            xysize = (xmax - xmin) * (ymax - ymin)
            size_dict[class_name].append(xysize)
            detect_dict[class_name].append(det)
        for class_name in size_dict.keys():
            size_list = size_dict[class_name]
            list_tmp = size_list.copy()
            list_sorted = size_list.copy()
            list_sorted.sort()

            list_index = []
            for x in list_sorted:
                list_index.insert(0,list_tmp.index(x))
                list_tmp[list_tmp.index(x)] = -1
            for ind in list_index:
                sorted_dict[class_name].append(detect_dict[class_name][ind])
        for class_name in sorted_dict.keys():
            for det in sorted_dict[class_name]:
                xmin = det['xmin']
                ymin = det['ymin']
                xmax = det['xmax']
                ymax = det['ymax']
                xcent = xmin + (xmax - xmin)/2
                ycent = ymin + (ymax - ymin)/2
                best = True
                for bdet in clean_dict[class_name]:
                    bxmin = bdet['xmin']
                    bymin = bdet['ymin']
                    bxmax = bdet['xmax']
                    bymax = bdet['ymax']
                    if (xcent > bxmin and xcent < bxmax) and (ycent > bymin and ycent < bymax):
                        best = False
                if best == True:
                    clean_dict[class_name].append(det)
        for class_name in clean_dict.keys():
            for det in clean_dict[class_name]:
                clean_boxes.append(det)
        
        return clean_boxes

            
            


    def publishDetectionsData(self, source_topic, img_dict, detect_dict_list, timestamp, np_depth_map = None):
        detect_dict_list = self.cleanBoxes(detect_dict_list)
        #self.msg_if.pub_warn("Publisher got img_dict: " + str(img_dict))
        det_count = len(detect_dict_list)
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        active_source_topics = copy.deepcopy(self.active_source_topics)
        if source_topic in active_source_topics:

            ###############################
            # Calculate Localization Data

            detection_msg_list = []
            l_msg_list = []
            targets_msg_list = []
            for detect_dict in detect_dict_list:

                # Calculate target bearings
                if source_topic in self.imgs_info_dict.keys():
                    image_fov_vert = self.imgs_info_dict[source_topic]['height_deg']
                    image_fov_horz = self.imgs_info_dict[source_topic]['width_deg']
                else:
                    image_fov_vert = 70
                    image_fov_horz = 100

                object_loc_y_pix = float(detect_dict['ymin'] + ((detect_dict['ymax'] - detect_dict['ymin']))  / 2) 
                object_loc_x_pix = float(detect_dict['xmin'] + ((detect_dict['xmax'] - detect_dict['xmin']))  / 2)
                object_loc_y_ratio_from_center = float(object_loc_y_pix - img_dict['image_height']/2) / float(img_dict['image_height']/2)
                object_loc_x_ratio_from_center = float(object_loc_x_pix - img_dict['image_width']/2) / float(img_dict['image_width']/2)
                vert_angle_deg = (object_loc_y_ratio_from_center * float(image_fov_vert/2))
                horz_angle_deg = - (object_loc_x_ratio_from_center * float(image_fov_horz/2))


                target_range_m = -999
                if np_depth_map is not None:
                    self.imgs_info_dict[source_topic]['has_range'] = True
                    try:
                        target_range_m = nepi_img.get_range_from_npDepthMap(np_depth_map, detect_dict)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to get target depth from np_depth_map: " + str(e))



                ### Print the range and bearings for each detected object
                #self.msg_if.pub_warn("")
                #self.msg_if.pub_warn(target_label)
                #self.msg_if.pub_warn(str(depth_box_adj.shape) + " detections box size")
                #self.msg_if.pub_warn("%.2f" % target_range_m + "m : " + "%.2f" % horz_angle_deg + "d : " + "%.2f" % vert_angle_deg + "d : ")
                #self.msg_if.pub_warn("")

                ################
                # Bounding Boxes
                try:
                    detection_msg = Detection()
                    detection_msg.name = detect_dict['name']
                    detection_msg.id = detect_dict['id']
                    detection_msg.uid = detect_dict['uid']
                    detection_msg.confidence = detect_dict['prob']
                    detection_msg.xmin = detect_dict['xmin']
                    detection_msg.ymin = detect_dict['ymin']
                    detection_msg.xmax = detect_dict['xmax']
                    detection_msg.ymax = detect_dict['ymax']
                    area_pixels = (detect_dict['xmax'] - detect_dict['xmin']) * (detect_dict['ymax'] - detect_dict['ymin'])
                    img_area = img_dict['prc_width']* img_dict['prc_height']
                    if img_area > 1:
                        area_ratio = area_pixels / img_area
                    else:
                        area_ratio = -999
                    detection_msg.area_pixels = img_area
                    detection_msg.area_ratio = area_ratio
                    detection_msg_list.append(detection_msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get all data from detect dict: " + str(e)) 

                try:
                    # Ranl Bearing, Nav, and Pose Data ENU Reference Frame
                    detection_msg.range_m = target_range_m
                    detection_msg.azimuth_deg = horz_angle_deg
                    detection_msg.elevation_deg = vert_angle_deg
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get all data from detect dict: " + str(e))



                    area_pixels = (detect_dict['xmax'] - detect_dict['xmin']) * (detect_dict['ymax'] - detect_dict['ymin'])
                    img_area = img_dict['prc_width']* img_dict['prc_height']
                    if img_area > 1:
                        area_ratio = area_pixels / img_area
                    else:
                        area_ratio = -999

            detections_msg = Detections()
            detections_msg.timestamp = float(timestamp)

            detections_msg.process_name = self.node_name
            detections_msg.process_namespace = self.node_namespace

            detections_msg.source_topic = source_topic
            detections_msg.source_timestamp = float(img_dict['timestamp'])
            detections_msg.detections = detection_msg_list
            #self.msg_if.pub_warn("Publisher create detection msg: " + str(detections_msg))
            # detections data product (publish + rate-gated save) is owned by
            # DetectionsIF; the collective 'all' fan-out stays inline.
            self.detections_if.publish_data(detections_msg, timestamp = timestamp)
            self.node_if.publish_pub('all_detections', detections_msg)
            try:
               self.img_ifs_dict[source_topic]['pubs_if'].publish_pub('detections_pub',detections_msg)
            except Exception as e:
                self.msg_if.pub_warn("Failed to publish detections to source topic: " + str(e), throttle_s = 5)

            if det_count > 0:
                if 'detections_trigger' in self.triggers_dict.keys():
                    trigger_dict = self.triggers_dict['detections_trigger']
                    trigger_dict['time']=nepi_utils.get_time()
                    try:
                        self.triggers_if.publish_trigger(trigger_dict)
                    except:
                        pass
                self.detecting_state = True
                self.process_state = True

            # NOTE: detections/targets saving is now handled inside
            # DetectionsIF.publish_data / TargetsIF.publish_data (rate-gated via
            # the shared SaveDataIF), so the previous unconditional inline save
            # of the detections/targets messages has been removed here.


    def publishTargetsData(self, source_topic, img_dict, detect_dict_list, timestamp, np_depth_map = None):
        detect_dict_list = self.cleanBoxes(detect_dict_list)
        #self.msg_if.pub_warn("Publisher got img_dict: " + str(img_dict))
        det_count = len(detect_dict_list)
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        active_source_topics = copy.deepcopy(self.active_source_topics)
        if source_topic in active_source_topics:

            ###############################
            # Calculate Localization Data

            targets_msg_list = []
            for detect_dict in detect_dict_list:

                # Calculate target bearings
                if source_topic in self.imgs_info_dict.keys():
                    image_fov_vert = self.imgs_info_dict[source_topic]['height_deg']
                    image_fov_horz = self.imgs_info_dict[source_topic]['width_deg']
                else:
                    image_fov_vert = 70
                    image_fov_horz = 100

                object_loc_y_pix = float(detect_dict['ymin'] + ((detect_dict['ymax'] - detect_dict['ymin']))  / 2) 
                object_loc_x_pix = float(detect_dict['xmin'] + ((detect_dict['xmax'] - detect_dict['xmin']))  / 2)
                object_loc_y_ratio_from_center = float(object_loc_y_pix - img_dict['image_height']/2) / float(img_dict['image_height']/2)
                object_loc_x_ratio_from_center = float(object_loc_x_pix - img_dict['image_width']/2) / float(img_dict['image_width']/2)
                vert_angle_deg = (object_loc_y_ratio_from_center * float(image_fov_vert/2))
                horz_angle_deg = - (object_loc_x_ratio_from_center * float(image_fov_horz/2))


                target_range_m = -999
                if np_depth_map is not None:
                    self.imgs_info_dict[source_topic]['has_range'] = True
                    try:
                        target_range_m = nepi_img.get_range_from_npDepthMap(np_depth_map, detect_dict)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to get target depth from np_depth_map: " + str(e))


                ########################
                # Targeting
                try:
                    target_msg = Target()

                    target_msg.timestamp = float(img_dict['timestamp'])

                    target_msg.name = detect_dict['name']
                    target_msg.uid = detect_dict['uid']
                    target_msg.confidence = detect_dict['prob']

                    # 2D Data ENU Reference Frame
                    target_msg.xmin_pixel = detect_dict['xmin']
                    target_msg.xmax_pixel = detect_dict['xmax']

                    target_msg.ymin_pixel = detect_dict['ymin']
                    target_msg.ymax_pixel = detect_dict['ymax']

                    target_msg.width_pixels = detect_dict['xmax'] - detect_dict['xmin']
                    target_msg.height_pixels = detect_dict['ymax'] - detect_dict['ymin']


                    target_msg.area_pixels = (detect_dict['xmax'] - detect_dict['xmin']) * (detect_dict['ymax'] - detect_dict['ymin'])

                    area_pixels = (detect_dict['xmax'] - detect_dict['xmin']) * (detect_dict['ymax'] - detect_dict['ymin'])
                    img_area = img_dict['prc_width']* img_dict['prc_height']
                    if img_area > 1:
                        area_ratio = area_pixels / img_area
                    else:
                        area_ratio = -999
                    target_msg.area_pixels = img_area
                    target_msg.area_ratio = area_ratio
                    #target_msg.vel_pixels

                    # 3D Data in ENU Reference Frame
                    # target_msg.width_meters = detect_dict['width_meters']
                    # target_msg.height_meters = detect_dict['height_meters']
                    # target_msg.depth_meters = detect_dict['depth_meters']
                    # target_msg.area_meters = detect_dict['area_meters']

                    #target_msg.center_xyz_meters = detect_dict['center_xyz_meters']

                    # Range, Bearing, Nav, and Pose Data ENU Reference Frame
                    target_msg.range_m = target_range_m
                    target_msg.azimuth_deg = horz_angle_deg
                    target_msg.elevation_deg = vert_angle_deg   
                    targets_msg_list.append(target_msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get all data from detect dict: " + str(e)) 



            targets_msg = Targets()

            
            targets_msg.timestamp = float(timestamp)

            targets_msg.process_name = self.node_name
            targets_msg.process_namespace = self.node_namespace


            targets_msg.source_topic = source_topic
            targets_msg.source_timestamp = float(img_dict['timestamp'])
            #targets_msg.source_nav_pose

            # targets_msg.has_2d_data = True
            # targets_msg.has_3d_data = False
            # targets_msg.has_range_data = True
            # targets_msg.has_bearing_data = True

            # targets_msg.has_navpose_data = True

            # targets_msg.has_color_data = False
            # targets_msg.has_countour_data = False
            # targets_msg.has_shape_data = False

            targets_msg.targets = targets_msg_list

            #self.msg_if.pub_warn("Publisher targets msg " + str(targets_msg))
            # targets data product (publish + rate-gated save) is owned by
            # TargetsIF; the collective 'all' fan-out stays inline.
            self.targets_if.publish_data(targets_msg, timestamp = timestamp)
            self.node_if.publish_pub('all_targets', targets_msg)

            try:
               self.img_ifs_dict[source_topic]['pubs_if'].publish_pub('targets_pub',targets_msg)
            except Exception as e:
                self.msg_if.pub_warn("Failed to publish targets to source topic: " + str(e), throttle_s = 5)

            if det_count > 0:
                if 'targeting_trigger' in self.triggers_dict.keys():
                    trigger_dict = self.triggers_dict['targets_trigger']
                    trigger_dict['time']=nepi_utils.get_time()
                    self.triggers_if.publish_trigger(trigger_dict)
                self.targeting_state = True
                self.process_state = True
                

            # NOTE: detections/targets saving is now handled inside
            # DetectionsIF.publish_data / TargetsIF.publish_data (rate-gated via
            # the shared SaveDataIF), so the previous unconditional inline save
            # of the detections/targets messages has been removed here.


    def updateImgSubsCb(self,timer):
        # Check for data subscribers every second
        has_subs_list = []
        # Find active img topics
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        source_topics = imgs_info_dict.keys()
        active_topics = []
        active_topics = copy.deepcopy(self.active_source_topics)
               
        
        # Check if for all topic subscribers
        topic_names = []
        filters = ['detections','targets']
        for data_product in filters:
            namespace = os.path.join(self.node_namespace, data_product)
            topic_names.append(namespace)
    
        try:
            [has_subs,has_subs_dict] = nepi_sdk.find_subscribers(topic_names,filters, log_name_list = self.log_name_list)
        except:
            [has_subs,has_subs_dict] = [ False, dict() ]
        
        # Check if save_data_if needs data
        ds_dict = self.save_data_if.data_products_should_save_dict()
        for ds in ds_dict.keys():
            if ds == True:
                has_subs = has_subs or ds

        if has_subs == True:
            self.img_ifs_lock.acquire()
            for source_topic in source_topics:
                if source_topic not in active_topics:
                    self.imgs_has_subs_dict[source_topic] = False
                else:
                    self.imgs_has_subs_dict[source_topic] = True
            self.img_ifs_lock.release()
        else:           
            # Check image topic subscribers
            for source_topic in source_topics:
                if source_topic not in active_topics:
                    self.imgs_has_subs_dict[source_topic] = False
                else:
                    filters = ['detections_image']
                    topic_names = []      
                    if source_topic in self.imgs_info_dict.keys():
                        topic_names.append(self.imgs_info_dict[source_topic]['img_pub_topic'])
                    try:
                        [has_subs,has_subs_dict] = nepi_sdk.find_subscribers(topic_names,filters, log_name_list = self.log_name_list)
                    except:
                        [has_subs,has_subs_dict] = [ False, dict() ]


                    if source_topic in self.imgs_info_dict.keys():
                        if source_topic in has_subs_dict.keys():
                            self.imgs_has_subs_dict[source_topic] = True
                        else:
                            self.imgs_has_subs_dict[source_topic] = False



               

        nepi_sdk.start_timer_process((0.1), self.updateImgSubsCb, oneshot = True)

    def handleStatusRequest(self,_):
        resp = self.process_status_msg
        #self.msg_if.pub_warn("Returning Detector Info Response: " + str(resp))
        return resp


    def updateProcessStatus(self):

        self.process_status_msg.max_process_rate_hz = self.max_process_rate_hz

        self.process_status_msg.available_source_topics = self.available_source_topics
        self.process_status_msg.auto_select_enabled = self.auto_select_enabled
        if self.auto_select_enabled == False:
            self.auto_select_active = False
        self.process_status_msg.auto_select_active = self.auto_select_active
        self.process_status_msg.selected_sources = self.selected_sources


        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        active_source_topics = copy.deepcopy(self.active_source_topics)
        


        img_connects = []
        for source_topic in imgs_info_dict.keys():
            img_connected = imgs_info_dict[source_topic]['img_connected']
            img_connects.append(img_connected)
        self.process_status_msg.sources_connected = img_connects
        img_selected = len(img_connects) > 0 or self.source_file_processing
        self.process_status_msg.source_selected = img_selected 
        img_connected = True in img_connects or self.source_file_processing
        self.process_status_msg.source_connected = img_connected 

        self.process_status_msg.has_image_pub = True
        self.process_status_msg.image_pub_name = 'detections_image'
        self.process_status_msg.image_pub_enabled = self.imaging_enabled
        self.process_status_msg.max_image_pub_rate_hz = self.max_image_pub_rate_hz
        self.process_status_msg.use_last_image = self.use_last_image
        img_source_topics = []
        img_det_namespaces = []
        img_pub_topics = []
        for source_topic in imgs_info_dict.keys():
                state = (source_topic in active_source_topics)
                if state == True:
                    img_source_topics.append(source_topic)
                    img_pub_topics.append(imgs_info_dict[source_topic]['img_pub_topic'])
        self.process_status_msg.imaging_source_topics = img_source_topics
        self.process_status_msg.imaging_pub_topics = img_pub_topics


        #################

        self.process_status_msg.enabled = self.enabled
        running = self.enabled and img_selected and img_connected and self.sleep_state == False
        self.process_status_msg.running = running
        state = False
        state = self.process_state
        self.process_status_msg.state = state
        self.process_status_msg.msg_str = self.msg_str

        #################
        source_receive_latencies = sum(self.source_receive_latencies) / len(self.source_receive_latencies)
        self.process_status_msg.avg_source_latency = source_receive_latencies
        self.process_status_msg.avg_source_rate = sum(self.source_receive_rates) / len(self.source_receive_rates)

        preprocess_latencies = source_receive_latencies + sum(self.preprocess_latencies) / len(self.preprocess_latencies)
        self.process_status_msg.avg_preprocess_latency = preprocess_latencies
        self.process_status_msg.avg_preprocess_rate = sum(self.preprocess_rates) / len(self.preprocess_rates)
        
        self.process_status_msg.avg_process_latency = preprocess_latencies + sum(self.process_latencies) / len(self.process_latencies)
        self.process_status_msg.avg_process_rate = sum(self.process_rates) / len(self.process_rates)


        avg_process_time = sum(self.process_times) / len(self.process_times)
        if avg_process_time > 0.001:
            max_process_rate= 1.0 / avg_process_time
        else:
            max_process_rate= 0
        self.process_status_msg.max_process_rate = max_process_rate

    

    def publishStatusCb(self,timer):
        self.updateProcessStatus()
        self.publish_status()
        self.process_state = False

    def publish_status(self):
        self.publish_detector_status()
        #self.publish_targeting_status()


    def publish_detector_status(self):
        """Assembles and publishes the AI detector status message.

        Populates all fields of the DetectorStatus message from current
        internal state — including model metadata, class selections, sleep
        configuration, overlay flags, rate limits, image topic lists, and
        performance metrics — then publishes it on the status topic.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to
                True.
        """
        #self.msg_if.pub_warn("Starting Detector Status Pub")

        detector_status_msg = DetectorStatus()
        detector_status_msg.process_status = self.process_status_msg
        detector_status_msg.process_status.namespace = self.detections_namespace
        detector_status_msg.available_classes = self.classes
        detector_status_msg.selected_classes = self.selected_classes
        detector_status_msg.threshold_filter = self.threshold


        #self.msg_if.pub_warn("Ending Detector Status Pub")
        #self.msg_if.pub_warn("Sending Detection Status Msg: " + str(self.process_status_msg), throttle_s = 5)
        # DetectorStatus is published on <node_ns>/detections/status by
        # DetectionsIF (same wire topic/type as the removed 'detector_status'
        # inline pub).


        
        detections_if = getattr(self, 'detections_if', None)
        if detections_if is not None:
            detections_if.publish_status(detector_status_msg)
            # Publish for each connected image
            sources_connected = []
            for source_topic in self.imgs_info_dict.keys():
                if self.imgs_info_dict[source_topic]['img_connected']:
                    sources_connected.append(source_topic)

            for source_topic in sources_connected:
                try:
                    self.img_ifs_dict[source_topic]['pubs_if'].publish_pub('detections_status_pub',detector_status_msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to publish detections status to source topic: " + str(e), throttle_s = 5)



    def publish_targeting_status(self, do_updates = True):
        """Assembles and publishes the AI detector status message.

        Populates all fields of the DetectorStatus message from current
        internal state — including model metadata, class selections, sleep
        configuration, overlay flags, rate limits, image topic lists, and
        performance metrics — then publishes it on the status topic.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to
                True.
        """
        #self.msg_if.pub_warn("Starting Detector Status Pub")

        
    
        targeting_status_msg = TargetingStatus()
        # targeting_status_msg.process_status = self.process_status_msg
        # targeting_status_msg.process_status.namespace = self.targets_namespace
        targeting_status_msg.available_classes = self.classes
        targeting_status_msg.selected_classes = self.selected_classes
        targeting_status_msg.threshold_filter = self.threshold
        
        #self.msg_if.pub_warn("Publishing Targeting Status Msg: " + str(targeting_status_msg), throttle_s = 5)
        # TargetingStatus is published on <node_ns>/targets/status by TargetsIF
        # (same wire topic/type as the removed 'targeting_status' inline pub).
        targets_if = getattr(self, 'targets_if', None)
        if targets_if is not None:
            targets_if.publish_status(targeting_status_msg)
            # Publish for each connected image
            sources_connected = []
            for source_topic in self.imgs_info_dict.keys():
                if self.imgs_info_dict[source_topic]['img_connected']:
                    sources_connected.append(source_topic)
            for source_topic in sources_connected:
                try:
                    self.img_ifs_dict[source_topic]['pubs_if'].publish_pub('targets_status_pub',targeting_status_msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to publish targets status to source topic: " + str(e), throttle_s = 5)
