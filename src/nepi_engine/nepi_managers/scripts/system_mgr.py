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
from socket import INADDR_UNSPEC_GROUP
import time
import shutil
from collections import deque
import re
from datetime import datetime, timezone

import subprocess
import importlib
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_mgrs
from nepi_sdk import nepi_keys
from nepi_sdk import nepi_settings

                 
#####################
# System         

from nepi_sdk import nepi_states
from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import MgrSystemStatus, WarningFlags, StampedString, StringArray, \
                                DictString, DictStringEntry, UpdateBool, UpdateString, UpdateStringDict, UpdateOrder
                      
from nepi_interfaces.srv import SystemStatusQuery, SystemStatusQueryRequest, SystemStatusQueryResponse



from nepi_interfaces.msg import SystemState, SystemStates
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SettingsIF, StatesIF, TriggersIF, SaveDataIF


BYTES_PER_MEGABYTE = 2**20



NEPI_FOLDER='/opt/nepi'

class SystemMgrNode():


    
    STATUS_PERIOD = 1.0  # TODO: Configurable update period?

    DISK_FULL_MARGIN_MB = 250  # MB TODO: Configurable?

    SYS_ETC_PATH = NEPI_FOLDER + "/etc"
    FW_VERSION_PATH = NEPI_FOLDER + "/nepi_engine/etc/fw_version.txt"

    STATES_DICT = dict()

    RUN_MODES = ['develop','deploy']

    REQD_STORAGE_SUBDIRS = ["ai_models", 
                        "ai_training",
                        "data", 
                        "databases", 
                        "install",
                        "install/updates",
                        "install/apps",
                        "install/drivers",
                        "license", 
                        "logs", 
                        "logs/ros_log",
                        "logs/nepi_scripts_logs", 
                        "nepi_src",
                        "nepi_scripts",
                        "user_cfg",
                        "user_cfg/cals",]

    REQD_CONFIG_SUBDIRS = ["docker_cfg", 
                            "factory_cfg",
                            "system_cfg"]
                            
    
    STORAGE_CHECK_SKIP_LIST = ["ai_training",
                            "data",
                            "logs", 
                            "logs/ros_log",
                            "logs/nepi_scripts_logs", 
                            "nepi_src",
                            "tmp",
                            "docker_cfg", 
                            "factory_cfg",
                            "system_cfg"]
    


    SUPPORTED_DEVICE_TYPES = [ 'IDX', 'PTX', 'LSX', 'NPX', 'RBX', 'SVX']
    SUPPORTED_DEVICE_DESCRIPTIONS = [ 'Imaging', 'PanTilt', 'Lights', 'NavPose', 'Robot', 'Servo']

    USER_RESTRICTION_OPTIONS =  \
        ['MANAGER-DEVICE','MANAGER-NEPI','MANAGER-SYSTEM','MANAGER-NETWORK','MANAGER-TIME','MANAGER-DATA','MANAGER-SOFTWARE','NEPI_LICENSE', \
        'MANAGER-NAVPOSE','MANAGER-DRIVERS','MANAGER-APPS','MANAGER-AI-MODELS','MANAGER-AI-DETECTORS','MANAGER-SCRIPTS'] + \
        ['SYSTEM-ADMIN','SYSTEM-MESSAGES','SYSTEM-SAVE','SYSTEM-CONFIG','SYSTEM-SETTINGS','SYSTEM-TRIGGERS','SYSTEM-STATES','SYSTEM-TRANSFORMS',] + \
        ['DATA-IMAGE-ALL','DATA-IMAGE-OVERLAY','DATA-IMAGE-RENDER','DATA-IMAGE-INFO','DATA-IMAGE-NAVPOSE','DATA-IMAGE-CONFIG'] + \
        ['DATA-NAVPOSE'] + \
        ['DEVICE-IDX','DEVICE-PTX','DEVICE-LSX','DEVICE-NPX','DEVICE-RBX', 'DEVICE_SVX']
    



    # Sentinel the nepi system config file uses for an entry that is not set
    CONFIG_NONE_VALUE = 'NONE'

    # Seconds to wait for the host service to pick up an NEPI_UPDATE_CONFIG request,
    # and then to wait for the update it started to finish
    UPDATE_START_WAIT_S = 10
    UPDATE_FINISH_WAIT_S = 300

    SYSTEM_SETTINGS_KEYS = []
    SYSTEM_SETTINGS_DICT = dict()
    system_capSettings = None
    system_factorySettings = None
    system_update_time = 0
    system_update_delay = 60
    nepi_service_running = False
    nepi_update_requested = False
    nepi_updating_config = False
    nepi_expand_requested = False
    nepi_expanding_fs = False
                  


    node_if = None
    status_msg = MgrSystemStatus()
    status_published = False

    storage_subdirs = dict()
    user_folders = dict()
    system_folders = dict()

    nepi_uid = 'nepi' #1000 # default to nepi
    nepi_gid = 'nepi'

    nepiadmin_uid = 'nepiadmin' #1000 # default to nepi
    nepiadmin_gid = 'nepiadmin'

    nepihost_uid = 'nepihost' #1000 # default to nepi
    nepihost_gid = 'nepihost'


    # Shorter period for more responsive updates
    disk_usage_deque = deque(maxlen=3)
    disk_required_percent = 10
    disk_clean_percent = 10

    in_container = False

    triggers_list = []
    triggers_status_interval = 1.0

    states_pub_interval = 1.0

    current_throttle_ratio = 1.0


    admin_enabled = False
    admin_password_valid = False
    admin_mode_set = False
    admin_mode_updated = True


    managers_dict = dict()
    managers_process_dict = dict()
    managers_base_list = nepi_mgrs.BASE_MANAGERS
    managers_failed_list = []
    managers_active_list = []
    managers_active_list_last = []
    managers_running_list = []
    managers_param_folder = '/opt/nepi/nepi_engine/share/nepi_managers/params'
    
    run_mode = 'deploy'
    debug_mode_enabled = False

    user_restriction_options = USER_RESTRICTION_OPTIONS
    user_restrictions = []

    supported_device_types = SUPPORTED_DEVICE_TYPES
    supported_device_descriptions = SUPPORTED_DEVICE_DESCRIPTIONS

    user_login_enabled = False
    user_login_password_valid = False

    space_available = True

    topics_name_list = []
    topics_types_list = []

    services_name_list = []
    services_type_list = []


    init_complete = False
    ready = False

    config_saved = False

    timezone_str = 'UTC'
    internet_connected = False
    date_time_str = ''
    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "system_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ###############################
        # Initialize System Params

        # Create an aware datetime object
        dt = datetime.now(timezone.utc)
        tzs = dt.strftime('%Z')
        nepi_system.set_timezone(tzs)
        nepi_system.set_navposes_dict(dict())
        nepi_system.set_active_drivers([])
        nepi_system.set_devices_alias_dict(dict())
        nepi_system.set_active_apps([])
        nepi_system.set_ai_models_dict(dict())
        nepi_system.set_active_ai_detectors([])

        ###############################
        # Initialize Class Variables
        
        self.system_config = nepi_system.load_nepi_system_config()
        self.msg_if.pub_warn("Got System Config: " + str(self.system_config))
        if self.system_config is None:
            self.system_config = dict()
        if len(self.system_config.keys()) == 0:
            self.msg_if.pub_warn("Failed to Read NEPI config file")
            nepi_sdk.signal_shutdown("Shutting Down: Failed to Read NEPI config file")
            return
        self.system_capSettings = self.getCapSettings(self.system_config)
        self.system_factorySettings = self.getSettings(self.system_config)
        
        # self.factory_config = nepi_system.load_nepi_factory_config()
        # self.msg_if.pub_warn("Got System Config: " + str(self.factory_config))
        # if self.factory_config is None:
        #     self.factory_config = dict()
        # if len(self.factory_config.keys()) == 0:
        #     self.factory_config = self.system_config
        # else:
        #     for key in self.system_config.keys():
        #         if key not in self.factory_config.keys():
        #             self.factory_config[key] = self.system_config[key]
        # self.system_factorySettings = self.getSettings(self.factory_config)
       
        

        self.nepi_uid = self.system_config['NEPI_USER']
        self.nepi_gid = self.nepi_uid


        self.nepiadmin_uid = self.system_config['NEPI_ADMIN_USER']
        self.nepiadmin_gid = self.system_config['NEPI_ADMIN_USER']

        self.nepihost_uid = self.system_config['NEPI_HOST_USER']
        self.nepihost_gid = self.system_config['NEPI_HOST_USER']

        check_path=self.system_config['NEPI_STORAGE']
        if os.path.exists(check_path)==False:
            os.mkdir(check_path)
        check_path=self.system_config['NEPI_CONFIG']
        if os.path.exists(check_path)==False:
            os.mkdir(check_path)
            os.mkdir(self.system_config['FACTORY_CONFIG'])
            os.mkdir(self.system_config['SYSTEM_CONFIG'])
            os.mkdir(self.system_config['DOCKER_CONFIG'])

        check_path=self.system_config['NEPI_IMPORT_PATH']
        if check_path not in self.REQD_STORAGE_SUBDIRS:
            self.REQD_STORAGE_SUBDIRS.append(check_path)
        check_path=self.system_config['NEPI_EXPORT_PATH']
        if check_path not in self.REQD_STORAGE_SUBDIRS:
            self.REQD_STORAGE_SUBDIRS.append(check_path)

        nepi_system.set_nepi_config(self.system_config)

        self.status_msg.serial_number = str(self.system_config['NEPI_DEVICE_SN'])
        hw_type = self.system_config['NEPI_HW_TYPE']
        if hw_type != 'unknown':
            self.status_msg.hw_type = hw_type
        hw_model = self.system_config['NEPI_HW_MODEL']
        if hw_model != 'unknown':
            self.status_msg.hw_model = hw_model
        self.status_msg.sw_desc = self.system_config['NEPI_SW_DESC']
        self.status_msg.has_cuda = self.system_config['NEPI_HAS_CUDA'] == 1
        self.status_msg.manages_time = self.system_config['NEPI_MANAGES_TIME'] == 1
        self.status_msg.manages_network = self.system_config['NEPI_MANAGES_NETWORK'] == 1

        self.in_container = self.system_config['NEPI_IN_CONTAINER'] == 1
        self.status_msg.in_container = self.in_container

        self.config_folder = self.system_config['NEPI_CONFIG']
        self.storage_folder = self.system_config['NEPI_STORAGE']
        self.data_folder = self.storage_folder + "/data"

        
        self.SDK_PATH_DICT = {
            'sdk_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_sdk',
            'sdk_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_sdk',
            }
        self.API_PATH_DICT = {
            'api_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_api',
            'api_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_api',
            }
        self.ETC_PATH_DICT = {
            'etc': NEPI_FOLDER + '/nepi_engine/etc'
            }
        self.CONFIG_FOLDER_DICT = {
            'docker_cfg': self.config_folder + "/docker_cfg",
            'factory_cfg':  self.config_folder + "/factory_cfg",
            'system_cfg':  self.config_folder + "/system_cfg",
            'user_cfg':  self.storage_folder + "/user_cfg",
            }
        self.RUI_PATH_DICT = {
            'rui_env': NEPI_FOLDER + '/rui/.nvm',
            'rui_bld': NEPI_FOLDER + '/rui/src/rui_webserver/rui-app',
            'rui_src': NEPI_FOLDER + '/rui/src/rui_webserver/rui-app/src'
            }
        self.MANAGERS_PATH_DICT = {
            'managers_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_managers',
            'managers_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_managers',
            'managers_param': NEPI_FOLDER + '/nepi_engine/share/nepi_managers/params',
            }
        self.DRIVERS_PATH_DICT = {
            'drivers_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_drivers',
            'drivers_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_drivers',
            'drivers_param': NEPI_FOLDER + '/nepi_engine/lib/nepi_drivers',
            'drivers_install': '/mnt/nepi_storage/install/drivers'
            }
        self.AIFS_PATH_DICT = {
            'aifs_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_aifs',
            'aifs_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_aifs',
            'aifs_param': NEPI_FOLDER + '/nepi_engine/share/nepi_aifs',
            'aifs_models': '/mnt/nepi_storage/ai_models/',
            'aifs_install': '/mnt/nepi_storage/install/ai_frameworks'
            }
        self.APPS_PATH_DICT = {
            'apps_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_apps',
            'apps_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_apps',
            'apps_param': NEPI_FOLDER + '/nepi_engine/share/nepi_apps/params',
            'apps_install': '/mnt/nepi_storage/install/apps'
            }

        self.SYSTEM_CHECK_SKIP_LIST = ['rui','etc']
        self.SYSTEM_PATH_DICT = {
            'sdk': self.SDK_PATH_DICT,
            'api': self.API_PATH_DICT,
            'etc': self.ETC_PATH_DICT,
            'cfg': self.CONFIG_FOLDER_DICT,
            'rui': self.RUI_PATH_DICT,
            'managers': self.MANAGERS_PATH_DICT,
            'drivers': self.DRIVERS_PATH_DICT,
            'aifs': self.AIFS_PATH_DICT,
            'apps': self.APPS_PATH_DICT
        }


        self.req_storage_subdirs = self.REQD_STORAGE_SUBDIRS
        self.req_config_subdirs = self.REQD_CONFIG_SUBDIRS
        


        self.msg_if.pub_warn("Checking User Storage Partition")
        # First check that the storage partition is actually mounted
        # if not os.path.ismount(self.storage_folder):
        #     self.msg_if.pub_warn("NEPI Storage partition is not mounted... attempting to mount")
        #     ret, msg = self.nepi_image.mountPartition(self.nepi_storage_device, self.storage_folder)
        #     if ret is False:
        #         self.msg_if.pub_warn("Unable to mount NEPI Storage partition... system may be dysfunctional")
        #         #return False # Allow it continue on local storage...
                
        # ... as long as there is enough space
        self.update_storage()
        if self.status_msg.warnings.flags[WarningFlags.DISK_FULL] is True:
            self.msg_if.pub_warn("Insufficient space on storage partition")
            self.storage_folder = ""
            #return False




        self.msg_if.pub_warn("Checking System Folders")
        # Ensure that the user partition is properly laid out
        self.storage_subdirs = {} # Populated in function below
        self.ensure_reqd_subdirs()


        self.msg_if.pub_warn("Storing User Folders")
        nepi_system.set_user_folders(self.user_folders)
        #self.msg_if.pub_warn("Stored user folders: " + str(self.user_folders))

        self.msg_if.pub_info("Waiting for user folders")
        user_folders = nepi_system.get_user_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got user folders: " + str(user_folders))
        
        self.msg_if.pub_warn("Storing System Folders")
        nepi_system.set_system_folders(self.system_folders)
        #self.msg_if.pub_warn("Stored System Folders: " + str(self.system_folders))

        self.msg_if.pub_info("Waiting for system folders")
        system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got system folders: " + str(system_folders))



        self.msg_if.pub_warn("Checking valid device id")
        self.valid_device_id_re = re.compile(r"^[a-zA-Z][\w]*$")


        self.msg_if.pub_warn("Updating Boot Fail Counter")
        nepi_system.update_nepi_docker_config("NEPI_FAIL_COUNT",0)


        self.status_msg.sys_run_mode_options = self.RUN_MODES
        self.status_msg.user_restriction_options = self.user_restriction_options
        #self.status_msg.sys_managers_base_list = self.managers_base_list
        
        ### Load System Manager Params
        self.msg_if.pub_warn("Updating From Param Server")

        # user_cfg_file = self.node_name + '.yaml'
        # user_cfg_path = nepi_sdk.create_namespace(self.CONFIG_FOLDER_DICT['user_cfg'],user_cfg_file)
        # params_dict = nepi_sdk.load_params_from_file(user_cfg_path,self.node_namespace)
        # if len(list(params_dict.keys())) > 0:
        #     self.msg_if.pub_warn("Loaded System Manager Params from: " + str(user_cfg_path) + " : " + str(params_dict))
        # else:
        system_cfg_file = self.node_name + '.yaml'
        system_cfg_path = nepi_sdk.create_namespace(self.CONFIG_FOLDER_DICT['system_cfg'],system_cfg_file)
        params_dict = nepi_sdk.load_params_from_file(system_cfg_path,self.node_namespace)
        if len(list(params_dict.keys())) > 0:
            self.msg_if.pub_warn("Loaded System Manager Params from: " + str(system_cfg_path) + " : " + str(params_dict))
        # system_cfg_file = self.node_name + '.yaml'
        # system_cfg_path = nepi_sdk.create_namespace(self.CONFIG_FOLDER_DICT['system_cfg'],system_cfg_file)
        # params_dict = nepi_sdk.load_params_from_file(system_cfg_path,self.node_namespace)
        # if len(list(params_dict.keys())) > 0:
        #     self.msg_if.pub_warn("Loaded System Manager Params from: " + str(system_cfg_path) + " : " + str(params_dict))
        nepi_sdk.sleep(1)

        self.msg_if.pub_warn("Starting Node IF Setup")    
        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'clear_params': False,
            'namespace': self.node_namespace
        }


        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'admin_enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.admin_enabled
            },
            'managers_active_list': {
                'namespace': self.node_namespace,
                'factory_val': self.managers_active_list
            },
            'run_mode': {
                'namespace': self.node_namespace,
                'factory_val': self.run_mode
            },
            'user_restriction_options': {
                'namespace': self.node_namespace,
                'factory_val': self.user_restriction_options
            },
            'user_restrictions': {
                'namespace': self.node_namespace,
                'factory_val': self.user_restrictions
            },
            'user_login_enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.user_login_enabled
            },
            'storage_folder': {
                'namespace': self.node_namespace,
                'factory_val': self.storage_folder
            },
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'system_status_query': {
                'namespace': self.base_namespace,
                'topic': 'system_status_query',
                'srv': SystemStatusQuery,
                'req': SystemStatusQueryRequest(),
                'resp': SystemStatusQueryResponse(),
                'callback': self.provide_system_status
            }
        }



        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.base_namespace,
                'topic': 'status',
                'msg': MgrSystemStatus,
                'qsize': 1,
                'latch': True
            },
            'apply_throttle': {
                'namespace': self.base_namespace,
                'topic': 'apply_throttle',
                'msg': Float32,
                'qsize': 3,
                'latch': True
            },
            'save_data_pub': {
                'namespace': self.base_namespace,
                'topic': 'save_data',
                'msg': Bool,
                'qsize': 1,
                'latch': True
            },
            'states_pub': {
                'namespace': self.base_namespace,
                'topic': 'states',
                'msg': SystemStates,
                'qsize': 1,
                'latch': True
            }
        }  

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'enable_admin': {
                'namespace': self.base_namespace,
                'topic': 'admin_mode_enable',
                'msg': Bool,
                'qsize': None,
                'callback': self.enableAdminCb, 
                'callback_args': ()
            },
            'set_admin_password': {
                'namespace': self.base_namespace,
                'topic': 'set_admin_password',
                'msg': String,
                'qsize': None,
                'callback': self.setAdminPasswordCb,
                'callback_args': ()
            },
            'enable_all_managers': {
                'namespace': self.node_namespace,
                'topic': 'enable_all_managers',
                'msg': Empty,
                'qsize': 10,
                'callback': self.enableAllCb, 
                'callback_args': ()
            },
            'disable_all_managers': {
                'namespace': self.node_namespace,
                'topic': 'disable_all_managers',
                'msg': Empty,
                'qsize': 10,
                'callback': self.disableAllCb, 
                'callback_args': ()
            },
            'update_manager_state': {
                'namespace': self.node_namespace,
                'topic': 'update_manager_state',
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.updateStateCb, 
                'callback_args': ()
            },
            'update_manager_order': {
                'namespace': self.node_namespace,
                'topic': 'update_manager_order',
                'msg': UpdateOrder,
                'qsize': 10,
                'callback': self.updateOrderCb, 
                'callback_args': ()
            },

            'set_run_mode': {
                'namespace': self.base_namespace,
                'topic': 'set_run_mode',
                'msg': String,
                'qsize': None,
                'callback': self.setRuiModeCb, 
                'callback_args': ()
            },
            'enable_debug_mode': {
                'namespace': self.base_namespace,
                'topic': 'enable_debug_mode',
                'msg': Bool,
                'qsize': None,
                'callback': self.enableDebugCb, 
                'callback_args': ()
            },
            'add_user_restriction': {
                'namespace': self.base_namespace,
                'topic': 'add_user_restriction',
                'msg': String,
                'qsize': None,
                'callback': self.addUserRestrictionCb, 
                'callback_args': ()
            },
            'remove_user_restriction': {
                'namespace': self.base_namespace,
                'topic': 'remove_user_restriction',
                'msg': String,
                'qsize': None,
                'callback': self.removeUserRestrictionCb, 
                'callback_args': ()
            },
            'enable_user_login': {
                'namespace': self.base_namespace,
                'topic': 'user_login_mode_enable',
                'msg': Bool,
                'qsize': None,
                'callback': self.enableUserLoginCb, 
                'callback_args': ()
            },
            'set_user_login_password': {
                'namespace': self.base_namespace,
                'topic': 'set_user_login_password',
                'msg': String,
                'qsize': None,
                'callback': self.setUserLoginPasswordCb, 
                'callback_args': ()
            },  
            'save_data': {
                'namespace': self.base_namespace,
                'topic': 'save_data',
                'msg': Bool,
                'qsize': None,
                'callback': self.setSaveStatusCb, 
                'callback_args': ()
            },
            'clear_data_folder': {
                'namespace': self.base_namespace,
                'topic': 'clear_data_folder',
                'msg': Empty,
                'qsize': None,
                'callback': self.clearDataFolderCb, 
                'callback_args': ()
            },

            'set_device_id': {
                'namespace': self.base_namespace,
                'topic': 'set_device_id',
                'msg': String,
                'qsize': None,
                'callback': self.setDeviceIdCb, 
                'callback_args': ()
            },        
            'submit_system_error_msg': {
                'namespace': self.base_namespace,
                'topic': 'submit_system_error_msg',
                'msg': String,
                'qsize': None,
                'callback': self.systemErrorCb, 
                'callback_args': ()
            },
            'set_system_configs': {
                'namespace': self.base_namespace,
                'topic': 'set_system_configs',
                'msg': UpdateStringDict,
                'qsize': None,
                'callback': self.setNepiConfigsCb, 
                'callback_args': ()
            },
            'update_system_config': {
                'namespace': self.base_namespace,
                'topic': 'update_system_config',
                'msg': Empty,
                'qsize': None,
                'callback': self.updateNepiConfigCb, 
                'callback_args': ()
            },
            'connect_internet': {
                'namespace': self.base_namespace,
                'topic': 'connect_internet',
                'msg': Empty,
                'qsize': None,
                'callback': self.connectInternetCb, 
                'callback_args': ()
            },
            'restart_nepi_software': {
                'namespace': self.base_namespace,
                'topic': 'restart_nepi_software',
                'msg': Empty,
                'qsize': None,
                'callback': self.restartNepiSoftwareCb,
                'callback_args': ()
            },
            'restart_nepi_container': {
                'namespace': self.base_namespace,
                'topic': 'restart_nepi_container',
                'msg': Empty,
                'qsize': None,
                'callback': self.restartNepiContainerCb,
                'callback_args': ()
            },
            'expand_storage_drive': {
                'namespace': self.base_namespace,
                'topic': 'expand_storage_drive',
                'msg': Empty,
                'qsize': None,
                'callback': self.expandStorageDriveCb,
                'callback_args': ()
            },


        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        wait_cfg_mgr = False,
                        msg_if = self.msg_if
        )

        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()

        # Config mgr not running yet, so have to load saved configs ourselfs


        self.initCb(do_updates = True)


        ###############################
        # Setup System Settings IF Class ####################
        self.msg_if.pub_debug("Starting Settings IF Initialization", log_name_list = [self.node_name])
        system_settings_ns = self.base_namespace

        self.SYSTEM_SETTINGS_DICT = {
                    'capSettings': self.system_capSettings, 
                    'factorySettings': self.system_factorySettings,
                    'setSettingFunction': self.systemSettingUpdateFunction, 
                    'getSettingsFunction': self.systemGetSettingsFunction
                    
        }

    
        self.system_settings_if = SettingsIF(namespace = system_settings_ns,
                        settings_dict = self.SYSTEM_SETTINGS_DICT,
                        log_name_list = [self.node_name],
                        save_params = False,
                            msg_if = self.msg_if
                        )

        #######################
        # Setup NEPI Managers Updater Process
        self.msg_if.pub_info(":" + self.class_name + ": Starting states status pub service: ")
        nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)


        #######################
        # Setup System IF Classes
        self.managers_param_folder = self.MANAGERS_PATH_DICT['managers_param']
        self.msg_if.pub_info("Using Managers Param Folder: " + str(self.managers_param_folder))


        self.msg_if.pub_warn("Starting System IF Setup")   
        #######################
        # Setup System IF Classes
        
        # Setup States IF
        self.STATES_DICT = {
        }
        self.states_if = StatesIF(self.getStatesDictCb,
                        msg_if = self.msg_if)
        time.sleep(1)


        self.msg_if.pub_warn("Completing Initialization Processes") 
        ########################
        # Complete Initialization


        # Create States Status Pub Processes
        self.states_pub_interval = 1.0

        self.msg_if.pub_info(":" + self.class_name + ": Starting states status pub service: ")
        nepi_sdk.start_timer_process(self.states_pub_interval, self.systemStatesPubCb)

    
        # Want to update the op_environment (from param server) through the whole system once at
        # start-up, but the only reasonable way to do that is to delay long enough to let all nodes start
        self.msg_if.pub_warn("Updating From Param Server")
        self.initConfig()
    


        # Crate system status pub
        self.msg_if.pub_warn("Starting System Status Messages")
        nepi_sdk.start_timer_process(self.STATUS_PERIOD, self.publishStatusCb)
        nepi_sdk.start_timer_process(1, self.updateTopicsServicesCb, oneshot = True)
        nepi_sdk.start_timer_process(60000, self.ClearLogsCb, oneshot = True)
        nepi_sdk.start_timer_process(1, self.updateDockerCb)
        self.msg_if.pub_warn("System status ready")

        ##################################
        # Setup Save Data IF Class ####################
        self.msg_if.pub_debug("Starting Save Data IF Initialization")
        self.data_products_list = ['All']
        factory_data_rates= {}
        factory_data_rates['All'] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
       

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "idx",
            'add_node_name': True
            }

        self.msg_if.pub_debug("Starting save_rate_dict: " + str(factory_data_rates))
        sd_namespace = self.base_namespace
        self.save_data_if = SaveDataIF(namespace = sd_namespace,
                                data_products = self.data_products_list,
                                factory_rate_dict = factory_data_rates,
                                factory_filename_dict = factory_filename_dict,
                                log_name_list = [self.node_name],
                                msg_if = self.msg_if
                        )


        #########################################################
        ## Initiation Complete
        self.ready = True
        self.msg_if.pub_warn("Initialization Complete")
        nepi_sdk.spin()


    def initConfig(self):
        if self.node_if is not None:

            user_restriction_options = self.node_if.get_param("user_restriction_options")
            if user_restriction_options is not None:
                for option in user_restriction_options:
                    if option not in self.user_restriction_options:
                        self.user_restriction_options.append(option)
    
    
            supported_device_types = self.node_if.get_param("supported_device_types")
            if supported_device_types is not None:
                for option in supported_device_types:
                    if option not in self.supported_device_types:
                        self.supported_device_types.append(option)
            self.status_msg.supported_device_types = self.supported_device_types

            supported_device_descriptions = self.node_if.get_param("supported_device_descriptions")
            if supported_device_descriptions is not None:
                for option in supported_device_descriptions:
                    if option not in self.supported_device_descriptions:
                        self.supported_device_descriptions.append(option)
            self.status_msg.supported_device_descriptions = self.supported_device_descriptions


            # Now gather all the params and set members appropriately
            # Keep the value __init__ already resolved from the system config
            # (NEPI_STORAGE) whenever the param server has nothing usable for
            # us yet. get_param() returns None when the param is not on the
            # server, and the very next thing initConfig does with this is
            # os.statvfs(self.storage_folder), which raises TypeError on None
            # -- an UNCAUGHT crash in __init__, so system_mgr dies outright.
            # That takes the whole platform with it (every other manager waits
            # on system_mgr, and the RUI blocks forever on its status topic),
            # and nothing respawns it. Confirmed live 2026-08-12 after an
            # in-place `supervisorctl restart nepi_engine`: the restart wipes
            # the param server with roscore, and this read lost the race
            # against the params being re-registered, so the platform came up
            # with 11 managers but no system_mgr and a publisher-less
            # /nepi/device1/status. It is intermittent -- the same restart
            # succeeded on the immediate retry -- which is exactly why it
            # needs a fallback rather than being left to timing.
            storage_folder_param = self.node_if.get_param("storage_folder")
            if storage_folder_param:
                self.storage_folder = storage_folder_param
            else:
                self.msg_if.pub_warn("No usable storage_folder param yet; keeping " +
                                     str(self.storage_folder) + " from the system config")


            # nepi_storage has some additional logic
            self.getNEPIStorageDevice()
            

            fw_str = self.get_fw_rev()
            self.system_config = nepi_system.update_nepi_system_config('NEPI_VERSION',fw_str)
            self.status_msg.firmware_version = fw_str


            # TODO: Determine how many temperature readings we have. On Jetson, for example
            #      there are 8 "thermal zones" in /sys/class/thermal/
            self.status_msg.temperature_sensor_names.append('CPU Zone 0')
            # TODO: Configurable warning/error temperatures
            self.status_msg.warning_temps.append(60.0)
            self.status_msg.critical_temps.append(70.0)

            # Reporting an unknown disk capacity is a cosmetic degradation; an
            # exception here is fatal, because this runs inside __init__ and
            # nothing respawns system_mgr -- and without system_mgr no other
            # manager initializes and the RUI blocks forever on its status
            # topic. Never let a bad/blank storage path take the platform down
            # over one status field (self.storage_folder is also deliberately
            # set to "" further up when the disk is full, which would raise
            # here too).
            try:
                statvfs = os.statvfs(self.storage_folder)
                self.status_msg.disk_capacity = statvfs.f_frsize * statvfs.f_blocks / \
                    BYTES_PER_MEGABYTE     # Size of data filesystem in Megabytes
            except Exception as e:
                self.status_msg.disk_capacity = 0
                self.msg_if.pub_warn("Could not determine disk capacity of " +
                                     str(self.storage_folder) + ": " + str(e))

            for i in self.status_msg.temperature_sensor_names:
                self.status_msg.temperatures.append(0.0)

            self.status_msg.save_all_enabled = False

    
    def initCb(self, do_updates = False):
        if self.node_if is not None:
            #self.admin_enabled = self.node_if.get_param('admin_enabled')
            managers_active_list = self.node_if.get_param('managers_active_list')
            if managers_active_list is None:
                managers_active_list = []
            if 'MANAGER-NETWORK' in managers_active_list and self.system_config['NEPI_MANAGES_NETWORK'] == 0:
                managers_active_list.remove('MANAGER-NETWORK')
            if 'MANAGER-TIME' in managers_active_list and self.system_config['NEPI_MANAGES_TIME'] == 0:
                managers_active_list.remove('MANAGER-TIME')
            if 'MANAGER-SOFTWARE' in managers_active_list and self.system_config['NEPI_MANAGES_SOFTWARE'] == 0:
                managers_active_list.remove('MANAGER-SOFTWARE')
            self.managers_active_list = managers_active_list
            #self.run_mode = self.node_if.get_param('run_mode')
            self.user_restriction_options = self.node_if.get_param('user_restriction_options')
            self.user_restrictions = self.node_if.get_param('user_restrictions')
            self.user_login_enabled = self.node_if.get_param('user_login_enabled')

            if do_updates == True:
                managers_dict = nepi_mgrs.refreshManagersDict(self.managers_param_folder,self.managers_dict)
                purge_list = []
                if 'MANAGER-NETWORK' in managers_dict.keys() and self.system_config['NEPI_MANAGES_NETWORK'] == 0:
                    purge_list.append('MANAGER-NETWORK')
                if 'MANAGER-TIME' in managers_dict.keys() and self.system_config['NEPI_MANAGES_TIME'] == 0:
                    purge_list.append('MANAGER-TIME')
                if 'MANAGER-SOFTWARE' in managers_dict.keys() and self.system_config['NEPI_MANAGES_SOFTWARE'] == 0:
                    purge_list.append('MANAGER-SOFTWARE')
                for manager in purge_list:
                    del managers_dict[manager]
                self.managers_dict = managers_dict

                for manager_name in self.managers_dict:
                    self.managers_dict[manager_name]['active'] = manager_name in self.managers_active_list
                
                self.updateSystemAdminSettings()
                if self.node_if is not None:
                    self.node_if.set_param('managers_active_list', self.managers_active_list)
            self.publish_status() 

    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def factoryResetCb(self,do_updates = True):
        self.aifs_classes_dict = dict()
        self.aif_classes_dict = dict()
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    ###################################
    # System Settings Functions

    def getCapSettings(self, config_dict):
        cap_settings = dict()
        if config_dict is None:
            cap_settings = copy.deepcopy(nepi_settings.NONE_CAP_SETTINGS)
        else:
            for key in config_dict.keys():
                cap_setting = None
                val = str(config_dict[key])
                try:
                    val_int = int(val)
                    cap_setting = {"name":key,"type":"Int","optons":[]}
                except:
                    cap_setting = {"name":key,"type":"String","optons":[]}
                if cap_setting is not None:
                    cap_settings[key] = cap_setting
        return cap_settings
    
    def getSettings(self, config_dict):
        settings = dict()
        if config_dict is None:
            settings = copy.deepcopy(nepi_settings.NONE_SETTINGS)
        else:
            for key in config_dict.keys():
                setting = None
                val = str(config_dict[key])
                try:
                    val_int = int(val)
                    setting = {"name":key,"type":"Int","value":val}
                except:
                    val_str = val
                    setting = {"name":key,"type":"String","value":val}
                if setting is not None:
                    settings[key] = setting
        return settings
                
        
    def systemGetSettingsFunction(self):
        settings = self.getSettings(self.system_config)
        return settings

    def systemSettingUpdateFunction(self, setting):
      success = False
      msg = ""
      setting_str = str(setting)
      [s_name, s_type, data] = nepi_settings.get_data_from_setting(setting)
      if data is not None:
        setting_name = setting['name']
        setting_data = data
        # NONE is the config file's sentinel for an unset entry and is the shipped
        # value for NEPI_GATEWAY_IP, NEPI_ALIAS_IP_2/3, and NEPI_NAV_IP, so it has to
        # survive the IP check or those keys can never be cleared once set.
        if 'IP' in setting_name and setting_data != self.CONFIG_NONE_VALUE:
            if nepi_utils.is_valid_ip(setting_data) == False:
                msg = (self.node_name  + " Setting data" + setting_str + " is Not a valid IP address")
                self.add_info_string(msg, StampedString.PRI_HIGH)
                return success, msg
        if 'NEPI_DEVICE_SN' == setting_name:
            if nepi_utils.is_valid_serial_number(setting_data) == False:
                msg = (self.node_name  + " Serial Number" + setting_str + " is Not a valid 6 Diget Number")
                self.add_info_string(msg, StampedString.PRI_HIGH)
                return success, msg
        self.system_config[setting_name] = setting_data
        # nepi_system.update_nepi_system_config(setting_name,setting_data)         
        success = True
        msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)   
      else:
        msg = (self.node_name  + " Setting data" + setting_str + " is None")
      return success, msg

    def setNepiConfigsCb(self, msg):
        self.msg_if.pub_info("Got Set Configs msg: " + str(msg))
        if self.system_settings_if is not None:
            key_strs = msg.key_strs
            value_strs = msg.value_strs
            if len(key_strs) == len(value_strs):
                for i, key_str in enumerate(key_strs):
                    self.system_settings_if.update_setting_value(key_str, value_strs[i])
                    nepi_sdk.sleep(0.2)

    def updateNepiConfigCb(self, msg):
        self.msg_if.pub_info("Got Update Config msg: " + str(msg))
        start_config = nepi_system.load_nepi_system_config()
        if self.nepi_update_requested == False and self.nepi_updating_config == False:
            self.msg_if.pub_warn("Starting System Update Process")
            umsg = "Sending NEPI Update Request"
            self.msg_if.pub_info(str(umsg))
            self.status_msg.nepi_update_msg = umsg
            self.nepi_update_requested = True
            system_config = copy.deepcopy(self.system_config)
            nepi_system.update_nepi_system_configs(system_config) 
            nepi_sdk.sleep(1)
            update_config = 1
            nepi_system.update_nepi_docker_config('NEPI_UPDATE_CONFIG',update_config)
            # Wait for updates
            success = False
            umsg = 'NEPI Config Failed to Update'
            last_time = nepi_utils.get_time()
            timer = 0
            while (timer < self.UPDATE_START_WAIT_S):
                    timer = nepi_utils.get_time() - last_time
                    if self.nepi_updating_config == True:
                        success = True
                        umsg = 'NEPI Config Updating'
                        self.status_msg.nepi_update_msg = umsg
                        break
                    nepi_sdk.sleep(0.5)

            if success == False:
                self.msg_if.pub_info(str(umsg))
                self.status_msg.nepi_update_msg = umsg
            else:
                # The host cleared the flag means the update finished. Bounded by the
                # timer this loop already computes so a host that never clears it
                # cannot wedge this subscriber callback thread forever.
                success = False
                umsg = 'NEPI Config Update Did Not Finish'
                last_time = nepi_utils.get_time()
                timer = 0
                while (self.nepi_updating_config == True and timer < self.UPDATE_FINISH_WAIT_S):
                        timer = nepi_utils.get_time() - last_time
                        nepi_sdk.sleep(0.5)
                if self.nepi_updating_config == False:
                    success = True
                    umsg = 'NEPI Config Updated'
            self.msg_if.pub_info(str(umsg))
            self.status_msg.nepi_update_msg = umsg
            self.nepi_update_requested = False
            self.system_update_time = nepi_utils.get_time()
            if success == False:
                nepi_system.save_nepi_system_config(start_config)
                nepi_sdk.sleep(1)
            updated_config = nepi_system.load_nepi_system_config()
            self.msg_if.pub_warn("Got Updted System Config: " + str(updated_config))
            if updated_config is None:
                self.system_config = dict()
            for key in updated_config.keys():
                self.system_settings_if.update_setting_value(key,updated_config[key])
            
        else:
            self.msg_if.pub_warn("System Update Process Allready in Progress")
        
    def connectInternetCb(self, msg):
        nepi_system.update_nepi_docker_config('NEPI_ETC_INTERNET_UPDATE',1)

    def restartNepiSoftwareCb(self, msg):
        nepi_system.update_nepi_docker_config('NEPI_ETC_INTERNET_UPDATE',1)

    def restartNepiContainerCb(self, msg):
        nepi_system.restart_nepi_software()

    def expandStorageDriveCb(self, msg):
        self.msg_if.pub_info("Got Expand Storage Drive msg: " + str(msg))
        if self.nepi_update_requested == True or self.nepi_updating_config == True:
            self.msg_if.pub_warn("System Config Update in Progress, Ignoring Expand Storage Request")
            self.status_msg.nepi_update_msg = 'CONFIG UPDATE IN PROGRESS - EXPAND REQUEST IGNORED'
        elif self.nepi_expand_requested == True or self.nepi_expanding_fs == True:
            self.msg_if.pub_warn("Storage Expansion Allready in Progress")
            self.status_msg.nepi_update_msg = 'STORAGE EXPANSION ALLREADY IN PROGRESS'
        else:
            self.msg_if.pub_warn("Starting Storage Drive Expansion Process")
            self.nepi_expand_requested = True
            umsg = 'STORAGE EXPANSION REQUESTED'
            self.msg_if.pub_info(str(umsg))
            self.status_msg.nepi_update_msg = umsg
            nepi_system.update_nepi_docker_config('NEPI_EXPAND_FS', 1)

    #######################
    # System Info Functions
    def add_info_string(self, string, level):
        self.status_msg.info_strings.append(StampedString(
            timestamp=nepi_sdk.get_msg_stamp(), payload=string, priority=level))


    def get_fw_rev(self):
        if (os.path.exists(self.FW_VERSION_PATH) and (os.path.getsize(self.FW_VERSION_PATH) > 0)):
            with open(self.FW_VERSION_PATH, "r") as f:
                fw_version = f.readline().strip()
                return fw_version

        return "UNSPECIFIED"
    
          

    def update_temperatures(self):
        # Get the current temperatures
        if len(self.status_msg.temperatures) == 0:
            return
        
        # TODO: Should the temperature sensor or the entire subproc. cmd line be configurable?
        temp_string_mdegC = subprocess.check_output(
            ["cat", "/sys/class/thermal/thermal_zone0/temp"])
       
        self.status_msg.temperatures[0] = float(temp_string_mdegC) / 1000.0

        # Check for temperature warnings and do thermal throttling
        throttle_ratio_min = 1.0
        for i, t in enumerate(self.status_msg.temperatures):
            if (t > self.status_msg.critical_temps[i]):
                # Critical implies high
                self.status_msg.warnings.flags[WarningFlags.CRITICAL_TEMPERATURE] = True
                self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
                # Make sure to send a user string
                self.add_info_string(
                    WarningFlags.CRITICAL_TEMPERATURE_STRING, StampedString.PRI_HIGH)
                # Set the throttle ratio to 0% globally
                self.msg_if.pub_error("temperature: " + str(WarningFlags.CRITICAL_TEMPERATURE_STRING) + " " + str(t), throttle_s = 600)
                throttle_ratio_min = 0.0
            else:
                self.status_msg.warnings.flags[WarningFlags.CRITICAL_TEMPERATURE] = False
                if (t > self.status_msg.warning_temps[i]):
                    self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
                    throttle_ratio_i = 1.0 - ((t - self.status_msg.warning_temps[i]) / (
                        self.status_msg.critical_temps[i] - self.status_msg.warning_temps[i]))
                    throttle_ratio_min = min(
                        throttle_ratio_i, throttle_ratio_min)
                else:
                    self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = False
            # If a new thermal throttle ratio was computed, publish it globally
            if (throttle_ratio_min != self.current_throttle_ratio):
                if self.node_if is not None:
                    self.node_if.publish_pub('throttle_ratio_pub', Float32(throttle_ratio_min))
                self.current_throttle_ratio = throttle_ratio_min
                #self.msg_if.pub_warn("New thermal rate throttle value: %f%%", self.current_throttle_ratio)

    def update_storage(self):
        # Data partition
        try:
            statvfs = os.statvfs(self.storage_folder)
        except Exception as e:
            # str(e), not e.what() -- the latter is C++ idiom; on a Python
            # exception it raises AttributeError from inside this very handler,
            # converting a handled disk-check failure back into the fatal,
            # platform-killing crash the try/except exists to prevent. Also
            # guard against storage_folder being None/"" here, since string
            # concatenation with None raises TypeError the same way.
            warn_str = "Error checking data storage status of " + str(self.storage_folder) + ": " + str(e)
            self.msg_if.pub_warn(warn_str)
            self.add_info_string("warn_str")
            self.status_msg.disk_usage = 0
            self.storage_rate = 0
            return

        disk_free = float(statvfs.f_frsize) * \
            statvfs.f_bavail / BYTES_PER_MEGABYTE  # In MB
        self.status_msg.disk_usage = self.status_msg.disk_capacity - disk_free

        self.disk_usage_deque.append(self.status_msg.disk_usage)
        self.status_msg.storage_rate = (
            self.disk_usage_deque[-1] - self.disk_usage_deque[0]) / (len(self.disk_usage_deque)*self.STATUS_PERIOD)

        # Check for disk warnings
        if (disk_free < self.DISK_FULL_MARGIN_MB):
            self.status_msg.warnings.flags[WarningFlags.DISK_FULL] = True
            self.add_info_string("Max disk usage exceeded",
                                 StampedString.PRI_HIGH)
            # Force all nodes to stop data saving
            if self.node_if is not None:
                self.node_if.publish_pub('save_data_pub', False)
        else:
            self.status_msg.warnings.flags[WarningFlags.DISK_FULL] = False

 

    def updateDockerCb(self, event):
        nepi_system.update_nepi_docker_config("NEPI_FAIL_COUNT" , 0)

        netlist_text = ''
        netlist_file = '/mnt/nepi_config/system_cfg/etc/netlist.txt'
        if os.path.exists(netlist_file):
            try:
                with open(netlist_file, "r", encoding="utf-8") as file:
                    netlist_text = file.read()
            except:
                pass
        if netlist_text != '' and 'end_file' in netlist_text:
            netlist_text = netlist_text.replace('end_file','')
            self.status_msg.netlist_str = str(netlist_text)

        nepi_service_running = False
        nepi_updating_config = False
        nepi_expanding_fs = False
        nepi_docker_config = nepi_system.load_nepi_docker_config()
        if nepi_docker_config is not None:
            if 'NEPI_UPDATING_CONFIG' in nepi_docker_config.keys():
                nepi_updating_config = nepi_docker_config['NEPI_UPDATING_CONFIG'] == 1

            if 'NEPI_EXPANDING_FS' in nepi_docker_config.keys():
                nepi_expanding_fs = nepi_docker_config['NEPI_EXPANDING_FS'] == 1

            if 'NEPI_SERVICE_RUNNING' in nepi_docker_config.keys():
                nepi_service_running = nepi_docker_config['NEPI_SERVICE_RUNNING'] == 1
                #Reset for next check
                # nepi_docker_config = nepi_system.update_nepi_docker_config('NEPI_SERVICE_RUNNING', 0)

        self.nepi_service_running = nepi_service_running
        self.status_msg.nepi_service_running = nepi_service_running

        self.nepi_updating_config = nepi_updating_config
        self.status_msg.nepi_updating_config = nepi_updating_config

        if self.nepi_updating_config == True:
            self.nepi_update_requested = False
        self.status_msg.nepi_update_requested = self.nepi_update_requested

        if nepi_expanding_fs == True:
            self.nepi_expand_requested = False
            self.status_msg.nepi_update_msg = 'EXPANDING STORAGE PARTITION - DO NOT POWER OFF SYSTEM'
        elif self.nepi_expanding_fs == True:
            # Expansion just finished, clear the in-progress warning
            self.status_msg.nepi_update_msg = 'STORAGE EXPANSION FINISHED'
        self.nepi_expanding_fs = nepi_expanding_fs

        
        

        



    def updateTopicsServicesCb(self, event):
        active_nodes = []
        nodes_list = nepi_sdk.get_node_list()
        for node in nodes_list:
            active_nodes.append(os.path.basename(node))
    
        self.status_msg.active_nodes = active_nodes
        [self.topics_name_list,self.topics_types_list] = nepi_sdk.get_topics_data_list()
        #self.msg_if.pub_warn("Got Topics List: " + str(topics_name_list))
        self.status_msg.active_topics = self.topics_name_list
        self.status_msg.active_topic_types = self.topics_types_list

        self.services_name_list = nepi_sdk.get_service_list()
        #self.msg_if.pub_warn("Got Services List: " + str(self.services_name_list))
        self.status_msg.active_services = self.services_name_list
        nepi_sdk.start_timer_process(1, self.updateTopicsServicesCb, oneshot = True)



    def ClearLogsCb(self, event):
        subprocess.run(["rosclean", "purge", "-y"])
        nepi_sdk.start_timer_process(60000, self.ClearLogsCb, oneshot = True)
 




    def setSaveStatusCb(self, save_msg):
        self.status_msg.save_all_enabled = save_msg.data



    ##################################
    # System State and Trigger

    def getStatesDictCb(self):
        return self.STATES_DICT

    def systemTriggersCb(self,msg):
        trigger_name = msg.name
        if trigger_name not in self.triggers_list:
            self.triggers_list.append(trigger_name)


    def systemStatesPubCb(self,timer):
        states_dict = dict()
        msg = SystemStates()
        namespaces = nepi_states.get_states_publisher_namespaces(self.topics_name_list, self.topics_types_list)
        if namespaces is not None:
            for namespace in namespaces:
                topic = os.path.join(namespace,'system_states_query')
                if topic not in self.service_dict.keys():
                    service = nepi_sdk.create_service(topic,SystemState)
                    if service is not None:
                        self.service_dict[topic] = service
                elif topic in self.service_dict.keys():
                    service = self.service_dict[topic]
                    req = SystemStatesQueryRequest()
                    try:
                        resp = nepi_sdk.call_service(service, req)
                        state_names = resp.state_names
                        states_msg_list = resp.states_msg_list
                        for i, state_name in enumerate(state_names):
                            state = states_msg_list[i].state
                            if state_name not in states_dict.keys() or state == True:
                                states_dict[state_name] = state
                    except:
                        self.msg_if.pub_info(":" + self.class_name + ": Failed to call service: " + str(e))

            states_status_msg = None
            try:
                states_status_msg = nepi_states.create_states_msg(states_dict)
            except Exception as e:
                self.msg_if.pub_info(":" + self.class_name + ": Failed to create status msg: " + str(e))
            if self.node_if is not None and states_status_msg is not None:
                self.node_if.publish_pub('states_pub', states_status_msg)
        #nepi_sdk.start_timer_process(self.states_pub_interval, self.systemStatesPubCb)


    #######################

    def updaterCb(self,timer):
        needs_update = False
        managers_dict = copy.deepcopy(self.managers_dict)
        managers_ordered_list = nepi_mgrs.getManagersOrderedList(managers_dict)
        managers_active_list_last = self.managers_active_list_last
        managers_active_list = self.managers_active_list
        if (managers_active_list != managers_active_list_last):
            needs_update = True
            for manager_name in self.managers_dict:
                self.managers_dict[manager_name]['active'] = manager_name in managers_active_list
        self.managers_active_list_last = copy.deepcopy(self.managers_active_list)

        ####################
        # Running process check
        managers_running_list = []
        for manager_name in self.managers_process_dict.keys():
            node_name = managers_dict[manager_name]['node_name']
            running = nepi_sdk.check_node_by_name(node_name)
            if running == True:
                managers_running_list.append(manager_name)
                #self.msg_if.pub_warn("Manager RUNNING: " + str(manager_name) + " node_name: " + str(node_name))
            else:
                #self.msg_if.pub_warn("Manager NOT running: " + str(manager_name) + " node_name: " + str(node_name))
                pass
        if self.managers_running_list != managers_running_list:
            needs_update = True

        ####################
        # Run purge process check
        purge_list = []
        for manager_name in self.managers_process_dict.keys():
            if manager_name not in managers_active_list:
                purge_list.append(manager_name)
        for manager_name in purge_list:
            node_namespace = managers_dict[manager_name]['node_namespace']
            subprocess = self.managers_process_dict[manager_name]
            self.msg_if.pub_warn("Killing Deactivated Manager: " + str(manager_name))
            success = nepi_mgrs.killManagerNode(node_namespace,subprocess)
            del self.managers_process_dict[manager_name]
            needs_update = True


        ####################
        # Launch process check
        for manager_name in managers_ordered_list:
            if  manager_name in managers_active_list and manager_name not in self.managers_process_dict.keys() and manager_name not in self.managers_failed_list:
                file_name = managers_dict[manager_name]['node_file']
                node_name = managers_dict[manager_name]['node_name']
                etc_file = managers_dict[manager_name]['etc_file']
                [success, msg, subprocess] = nepi_mgrs.launchManagerNode(file_name, node_name, etc_file)
                if success == True:
                    self.managers_process_dict[manager_name] = subprocess
                    self.msg_if.pub_warn("Manager launched successfully: " + str(manager_name))
                else:
                    self.msg_if.pub_warn("Manager Failed to Launch. Will not retry: " + str(manager_name))
                    self.managers_failed_list.append(manager_name)

        self.managers_active_list = managers_active_list
        self.managers_running_list = managers_running_list


        if needs_update == True:
            self.status_msg.sys_managers_ordered_list = nepi_mgrs.getManagersOrderedList(managers_dict)
            self.status_msg.sys_managers_active_list = self.managers_active_list
            self.status_msg.sys_managers_running_list = self.managers_running_list
            self.publish_status()



        needs_save = self.config_saved == False or needs_update == True
        if needs_save == True and self.node_if is not None:
            config_folders = nepi_system.get_config_folders()
            if len(list(config_folders.keys())) != 0:
                self.config_saved = True
                self.node_if.set_param('managers_active_list', self.managers_active_list)
                #self.msg_if.pub_warn("Config Mgr Ready. Saving System Config with managers dict: " + str(self.managers_dict))
                self.node_if.save_config()


        ###############
        # Check free disk space
        data_path = os.path.join(self.storage_folder, 'data')
        space_available = nepi_utils.check_path_space(data_path,self.disk_required_percent)
        if space_available == False and self.space_available == True:
            self.msg_if.pub_warn("Data folder minimum free space percent limit reached: " + str(self.disk_required_percent))
            free_space = self.disk_required_percent + self.disk_clean_percent
            self.msg_if.pub_warn("Deleting old data files to free up space in folder: " + str(data_path))
            nepi_utils.free_path_space(data_path,free_space)
        space_available = nepi_utils.check_path_space(data_path,self.disk_required_percent)
        if space_available == False:
            self.space_available = False
            self.msg_if.pub_warn("Failed to free disk space percent to: " + str(self.disk_required_percent))
            self.msg_if.pub_warn("Data Saving Disabled")
            
        nepi_system.set_space_available(space_available)
        
        self.internet_connected = nepi_utils.internet_check()
        self.status_msg.internet_connected = self.internet_connected
        self.date_time_str = nepi_utils.get_datetime_str_from_timestamp()
        self.status_msg.date_time_str = self.date_time_str

        nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)



    def updateSystemAdminSettings(self):
       
        admin_password_valid = self.admin_password_valid #(self.admin_password_valid or self.run_mode == 'develop')
        self.admin_mode_set = self.admin_enabled and admin_password_valid
        self.status_msg.sys_admin_enabled = self.admin_enabled
        self.status_msg.sys_admin_password_valid = admin_password_valid 
        self.status_msg.sys_admin_mode_set = self.admin_mode_set   

        self.status_msg.sys_run_mode = self.run_mode
        self.status_msg.sys_debug_enabled = self.debug_mode_enabled


        self.status_msg.user_restrictions = self.user_restrictions
        user_restricted = []
        if self.run_mode == 'deploy':
            user_restricted = self.user_restrictions
        self.status_msg.user_restricted = user_restricted

        self.user_login_mode_set = self.user_login_enabled and self.user_login_password_valid
        self.status_msg.user_login_enabled = self.user_login_enabled
        self.status_msg.user_login_password_valid = self.user_login_password_valid 



        self.publish_status()
        nepi_system.set_admin_mode(self.admin_mode_set)
        nepi_system.set_debug_mode(self.debug_mode_enabled)
        nepi_system.set_managers_running(self.managers_running_list)



    def enableAdminCb(self, msg):
        self.admin_enabled = msg.data
        if self.admin_enabled == False:
            self.admin_password_valid = False
        self.updateSystemAdminSettings()
        if self.node_if is not None:
            self.node_if.set_param('admin_enabled',msg.data)
            self.node_if.save_config()

    def setAdminPasswordCb(self, msg):
        password_encripted = msg.data
        self.msg_if.pub_warn("Got admin password msg: " + str(msg))
        password = nepi_keys.decrypt_rui_msg(password_encripted)
        self.msg_if.pub_warn("Got decrypted admin password: " + str(password))
        self.msg_if.pub_warn("Checking from user: " + str(self.nepi_uid))
        if password is not None:
            if password != '':

                password_valid = nepi_utils.check_password(self.nepi_uid,'nepiadmin',password)
                self.admin_password_valid = password_valid
                self.updateSystemAdminSettings()


    def setRuiModeCb(self, msg):
        if self.admin_mode_set == True:
            run_mode = msg.data
            if run_mode in self.RUN_MODES:
                self.run_mode = msg.data
                self.updateSystemAdminSettings()
                if self.node_if is not None:
                    self.node_if.set_param('run_mode',msg.data)
                    self.node_if.save_config()

    def enableDebugCb(self, msg):
        self.debug_mode_enabled = msg.data
        self.updateSystemAdminSettings()


    def enableUserLoginCb(self, msg):
        if self.admin_mode_set == True:
            self.user_login_enabled = msg.data
            self.updateSystemAdminSettings()
            if self.node_if is not None:
                self.node_if.set_param('user_login_enabled',msg.data)
                self.node_if.save_config()

    def setUserLoginPasswordCb(self, msg):
        user = self.nepi_uid
        password = msg.data
        password_valid = nepi_utils.check_password(self.nepi_uid,user,password)
        if password_valid:
                self.user_login_password_valid = True
                self.updateSystemAdminSettings()


    def addUserRestrictionCb(self, msg):
        if self.admin_mode_set == True:
            name = msg.data
            if name not in self.user_restriction_options:
                self.user_restriction_options.append(name)

            if name not in self.user_restrictions:
                self.user_restrictions.append(name)
                self.updateSystemAdminSettings()
        if self.node_if is not None:
            self.node_if.set_param('user_restrictions',self.user_restrictions)
            self.node_if.save_config()


    def removeUserRestrictionCb(self, msg):
        if self.admin_mode_set == True:
            name = msg.data
            if name in self.user_restrictions:  
                self.user_restrictions.remove(name)
                self.updateSystemAdminSettings()
                if self.node_if is not None:
                    self.node_if.set_param('user_restrictions',self.user_restrictions)
                    self.node_if.save_config()


  
    ###################
    ## Managers Mgr Callbacks
    def enableAllCb(self,msg):
        self.msg_if.pub_info("Got enable all msg: " + str(msg))
        mgrs_names = list(self.managers_dict.keys())
        for manager_name in mgrs_names and manager_name not in self.managers_base_list:
            if manager_name not in self.managers_active_list:
                self.managers_active_list.append(manager_name)
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param("managers_active_list",self.managers_active_list)
                self.node_if.save_config() # Save config on options change

    def disableAllCb(self,msg):
        self.msg_if.pub_info("Got disable all msg: " + str(msg))
        mgrs_names = list(self.managers_dict.keys())
        for manager_name in mgrs_names and manager_name not in self.managers_base_list:
            if manager_name in self.managers_active_list:
                self.managers_active_list.remove(manager_name)
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param("managers_active_list",self.managers_active_list)
                self.node_if.save_config() # Save config on options change


    def updateStateCb(self,msg):
        self.msg_if.pub_info("Got update manager state msg: " + str(msg))
        manager_name = msg.name
        enabled = msg.value
        mgrs_names = list(self.managers_dict.keys())
        if manager_name in mgrs_names and manager_name not in self.managers_base_list:

            if enabled == True and manager_name not in self.managers_active_list:
                self.managers_active_list.append(manager_name)
            elif enabled == False and manager_name in self.managers_active_list:
                self.managers_active_list.remove(manager_name)
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param("managers_active_list",self.managers_active_list)
                self.node_if.save_config() # Save config on options change



    def updateOrderCb(self,msg):
        self.msg_if.pub_info("Got Update manager order msg: " + str(msg))
        manager_name = msg.name
        move_cmd = msg.move_cmd
        moveFunction = self.getOrderUpdateFunction(move_cmd)
        managers_dict = copy.deepcopy(self.managers_dict)
        if manager_name in managers_dict.keys() and manager_name not in self.managers_base_list:
            if manager_name in self.managers_active_list:
                self.managers_active_list.remove(manager_name)
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param("managers_active_list",self.managers_active_list)
                self.node_if.save_config() # Save config on options change


    def getOrderUpdateFunction(self,move_cmd):
        if move_cmd == 'top':
            updateFunction = nepi_mgrs.moveManagerTop
        elif move_cmd == 'bottom':
            updateFunction = nepi_mgrs.moveManagerBottom
        elif move_cmd == 'up':
            updateFunction = nepi_mgrs.moveManagerUp
        elif move_cmd == 'down':
            updateFunction = nepi_mgrs.moveManagerDown
        else:
            updateFunction = self.moveManagerNone
        return updateFunction

    def moveManagerNone(self,manager_name,managers_dict):
        return managers_dict


 



    def ensure_reqd_subdirs(self):
        # Check for and create subdirectories as necessary
        self.msg_if.pub_warn("Checking nepi config folders")
        for subdir in self.req_config_subdirs:
            full_path_subdir = os.path.join(self.config_folder, subdir)
            if not os.path.isdir(full_path_subdir):
                self.msg_if.pub_warn("Required config subdir " + subdir + " not present... will create")
                os.makedirs(full_path_subdir)
                # And set the owner:group and permissions. Do this every time to fix bad settings e.g., during SSD setup
                # TODO: Different owner:group for different folders?
            if subdir not in self.STORAGE_CHECK_SKIP_LIST:
                self.msg_if.pub_warn("Checking nepi config folder permissions: " + subdir)
                os.system('chown  ' + str(self.nepi_uid) + ':' + str(self.nepi_gid) + ' ' + full_path_subdir) # Use os.system instead of os.chown to have a recursive option
                #os.chown(full_path_subdir, self.nepi_uid, self.nepi_gid)
                os.system('chmod  0775 ' + full_path_subdir)
            self.storage_subdirs[subdir] = full_path_subdir
            self.user_folders[subdir] = full_path_subdir


        self.msg_if.pub_warn("Checking nepi storage folders")
        for subdir in self.req_storage_subdirs:
            full_path_subdir = os.path.join(self.storage_folder, subdir)
            if not os.path.isdir(full_path_subdir):
                self.msg_if.pub_warn("Required storage subdir " + subdir + " not present... will create")
                os.makedirs(full_path_subdir)
                # And set the owner:group and permissions. Do this every time to fix bad settings e.g., during SSD setup
                # TODO: Different owner:group for different folders?
            if subdir not in self.STORAGE_CHECK_SKIP_LIST:
                self.msg_if.pub_warn("Checking nepi storage folder permissions: " + subdir)
                os.system('chown -R ' + str(self.nepi_uid) + ':' + str(self.nepi_gid) + ' ' + full_path_subdir) # Use os.system instead of os.chown to have a recursive option
                #os.chown(full_path_subdir, self.nepi_uid, self.nepi_gid)
                os.system('chmod -R 0775 ' + full_path_subdir)
            self.storage_subdirs[subdir] = full_path_subdir
            self.user_folders[subdir] = full_path_subdir



        # Check system folders
        self.msg_if.pub_warn("Checking nepi config folder")
        if not os.path.isdir(self.SYS_ETC_PATH):
                self.msg_if.pub_warn("Folder " + self.SYS_ETC_PATH + " not present... will create")
                os.makedirs(self.SYS_ETC_PATH)
        os.system('chown -R ' + str(self.nepi_uid) + ':' + str(self.nepi_gid) + ' ' + self.SYS_ETC_PATH) # Use os.system instead of os.chown to have a recursive option
        os.system('chmod -R 0775 ' + self.SYS_ETC_PATH)
        self.storage_subdirs['config'] = self.SYS_ETC_PATH


        for entry in self.SYSTEM_PATH_DICT.keys():
            path_dict = self.SYSTEM_PATH_DICT[entry]
            for key in path_dict:
                path_entry = path_dict[key]
                if entry not in self.SYSTEM_CHECK_SKIP_LIST and key not in self.SYSTEM_CHECK_SKIP_LIST:
                    self.msg_if.pub_warn("Checking system folder: " + key + " at: " + path_entry)
                    if not os.path.isdir(path_entry):
                            self.msg_if.pub_warn("Folder " + path_entry + " not present... will create")
                            os.makedirs(path_entry)
                    
                    os.system('chown -R ' + str(self.nepi_uid) + ':' + str(self.nepi_gid) + ' ' + path_entry) # Use os.system instead of os.chown to have a recursive option
                    os.system('chmod -R 0775 ' + path_entry)
                    #nepi_utils.remove_pycache_folders(path_entry)
                self.storage_subdirs[key] = path_entry
                self.system_folders[key] = path_entry
        return True




    def clearDataFolderCb(self, msg):
        if (self.status_msg.save_all_enabled is True):
            self.msg_if.pub_warn(
                "Refusing to clear data folder because data saving is currently enabled")
            return

        self.msg_if.pub_info("Clearing data folder by request")
        data_folder = self.storage_subdirs['data']
        if not os.path.isdir(data_folder):
            self.msg_if.pub_warn(
                "No such folder " + data_folder + "... nothing to clear"
            )
            return

        for filename in os.listdir(data_folder):
            file_path = os.path.join(data_folder, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                self.msg_if.pub_warn('Failed to delete: ' + file_path + " " + str(e))


    def setDeviceIdCb(self, msg):
        # First, validate the characters in the msg as namespace chars -- blank string is okay here to clear the value
        if (msg.data) and (not self.valid_device_id_re.match(msg.data)):
            self.msg_if.pub_warn("Invalid device ID: " +  str(msg.data))
            return

        nepi_system.update_nepi_system_config("NEPI_DEVICE_ID",msg.data)
        etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_hostname.sh"
        subprocess.call([etc_update_script])
        nepi_utils.sleep(1)
        self.system_config = self.get_nepi_system_config()
        
        # if 'NEPI_HW_TPE' not in self.system_config.keys():
        #     self.system_config = nepi_system.update_nepi_system_config('NEPI_HW_TYPE','unknown')
        
        # if 'NEPI_SW_DESC' not in self.system_config.keys():
        #      self.system_config = nepi_system.update_nepi_system_config('NEPI_SW_DESC','unknown')


        self.msg_if.pub_warn("Device ID Updated - Requires device reboot")
        self.add_info_string(
            "Device ID updated - Requires device reboot", StampedString.PRI_ELEVATED)
        self.node_if.save_config()

    def systemErrorCb(self, msg):
        self.add_info_string(msg.data, StampedString.PRI_HIGH)
    

    def save_data_prefix_callback(self, msg):
        save_data_prefix = msg.data

        data_folder = self.storage_subdirs['data']
        if data_folder is None:
            return # No data directory

        # Now ensure the directory exists if this prefix defines a subdirectory
        full_path = os.path.join(data_folder, save_data_prefix)
        parent_path = os.path.dirname(full_path)
        if not os.path.exists(parent_path):
            self.msg_if.pub_info("Creating new data subdirectory " + parent_path)
            os.makedirs(parent_path)
            
            # Gather owner and group details for data folder to propagate them
            # TODO: Do we need to do this recursively in case this we are creating multiple levels of subdirectory here
            stat_info = os.stat(data_folder)
            new_dir_uid = stat_info.st_uid
            new_dir_guid = stat_info.st_gid

            os.chown(parent_path, new_dir_uid, new_dir_guid)





    def getNEPIStorageDevice(self):
        if self.in_container == True:
            self.nepi_storage_device = self.storage_folder
        else:
          # Try to read the NEPI storage device out of /etc/fstab
          if os.path.exists('/etc/fstab'):
              with open('/etc/fstab', 'r') as fstab:
                  lines = fstab.readlines()
                  for line in lines:
                      if self.storage_folder in line and not line.startswith('#'):
                          candidate = line.split()[0] # First token is the device
                          if candidate.startswith('/dev/'):
                              self.nepi_storage_device = candidate
                              self.msg_if.pub_info('Identified NEPI storage device ' + self.nepi_storage_device + ' from /etc/fstab')
                              return
                          else:
                              self.msg_if.pub_warn('Candidate NEPI storage device from /etc/fstab is of unexpected form: ' + candidate)
            
          # If we get here, failed to get the storage device from /etc/fstab
          self.msg_if.pub_warn('Failed to get NEPI storage device from /etc/fstab -- falling back to system_mgr config file')
    

   
    def provide_system_status(self, req):
        response = SystemStatusQueryResponse()
        response.system_status = self.status_msg
        #self.msg_if.pub_warn("Returning status query response: " + str(response))
        return response


    def publishStatusCb(self, event):
        self.publish_status()



    def publish_status(self):
        
        self.update_temperatures()
        # Populate the rest of the message contents
        # Temperature(s)
        self.update_temperatures()

        # Disk usage
        self.update_storage()

        # Now publish it
        if self.node_if is not None:
            if self.status_published == False:
                self.status_published = True
                #self.msg_if.pub_warn("Publishing status message: " + str(self.status_msg))
            self.node_if.publish_pub('status_pub', self.status_msg)

        # Always clear info strings after publishing
        del self.status_msg.info_strings[:]



    



if __name__ == '__main__':
    SysMgr = SystemMgrNode()