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
import shutil

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_system"
logger = Logger(log_name = log_name)

NEPI_SYSTEM_CONFIG_FILE = '/opt/nepi/etc/nepi_system_config.yaml'
SYSTEM_CONFIG_FILE = '/mnt/nepi_config/system_cfg/etc/nepi_system_config.yaml'
FACTORY_CONFIG_FILE= '/mnt/nepi_config/system_cfg/etc/nepi_system_config.factory'


DOCKER_CONFIG_BLANK_FILE = '/mnt/nepi_config/docker_cfg/nepi_docker_config.blank'
DOCKER_CONFIG_FILE = '/mnt/nepi_config/docker_cfg/nepi_docker_config.yaml'

NEPI_ALL_CONFIG_IDS = ['idx','ptx','lsx','npx','rbx']

#######################
## Get System Data Functions

def get_nepi_config(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'nepi_config')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_nepi_config(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'nepi_config')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################

def get_admin_mode(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'admin_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_admin_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'admin_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_develop_mode(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'develop_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_develop_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'develop_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_debug_mode(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'debug_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_debug_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'debug_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success


def get_managers_running(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'managers_running')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_managers_running(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'managers_running')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_user_restrictions(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'user_restrictions')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_user_restrictions(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'user_restrictions')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################



def get_user_folders(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'user_folders')
    #print("Waiting for User Folders in param: " + str(param_namespace))
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_user_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'user_folders')
    #print("Storing User Folders in param: " + str(param_namespace))
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_system_folders(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'system_folders')
    #logger.log_warn("Got system folders namespace: " +  str(param_namespace))
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_system_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'system_folders')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_space_available(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'space_available')
    #logger.log_warn("Got system folders namespace: " +  str(param_namespace))
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_space_available(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'space_available')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success


##########################

def get_config_folders(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'config_folders')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_config_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'config_folders')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success



def supports_all_config(namespace):
    supports_all = False
    for id in NEPI_ALL_CONFIG_IDS:
        check_str = '/' + id
        if check_str in namespace:
            supports_all = True
            break
    return supports_all



def get_devices_alias_dict(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'devices_alias_dict')
    devices_alias_dict = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    if devices_alias_dict is None:
        devices_alias_dict = dict()
    return devices_alias_dict

def set_devices_alias_dict(devices_alias_dict, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'devices_alias_dict')
    success = nepi_sdk.set_param(param_namespace, devices_alias_dict, log_name_list = log_name_list)
    return success

def get_device_alias(device_name):
    alias_name = device_name
    devices_alias_dict = get_devices_alias_dict()
    if devices_alias_dict is not None:
        if device_name in devices_alias_dict.keys():
            alias_name = devices_alias_dict[device_name]
    return alias_name


##########################

def get_navposes_dict(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'navposes_dict')
    navposes_dict = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    if navposes_dict is None:
        navposes_dict = dict()
    return navposes_dict

def set_navposes_dict(navposes_dict, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'navposes_dict')
    success = nepi_sdk.set_param(param_namespace, navposes_dict, log_name_list = log_name_list)
    return success

##########################

def get_timezone(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'timezone')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_timezone(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'timezone')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################

def get_active_drivers(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_drivers')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_active_drivers(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_drivers')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success


##########################

def get_active_apps(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_apps')
    devices_alias_dict = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    if devices_alias_dict is None:
        devices_alias_dict = dict()
    return devices_alias_dict

def set_active_apps(devices_alias_dict, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_apps')
    success = nepi_sdk.set_param(param_namespace, devices_alias_dict, log_name_list = log_name_list)
    return success


##########################

def get_active_ai_detectors(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_ai_detectors')
    devices_alias_dict = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    if devices_alias_dict is None:
        devices_alias_dict = dict()
    return devices_alias_dict

def set_active_ai_detectors(devices_alias_dict, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_ai_detectors')
    success = nepi_sdk.set_param(param_namespace, devices_alias_dict, log_name_list = log_name_list)
    return success

''' Need to add
def get_active_ai_frameworks(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_ai_frameworks')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_ai_models(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_ai_models')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_apps(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_apps')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_auto_scrips(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_auto_scrips')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data
'''
##########################

def get_ai_models_dict(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'ai_models_dict')
    ai_models_dict = None
    ai_models_dict = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return ai_models_dict

def set_ai_models_dict(ai_models_dict, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'ai_models_dict')
    success = nepi_sdk.set_param(param_namespace, ai_models_dict, log_name_list = log_name_list)
    return success


########################

def load_nepi_system_config():
    config_dict=dict()

    target_file=SYSTEM_CONFIG_FILE
    backup_file=NEPI_SYSTEM_CONFIG_FILE
    if os.path.exists(target_file) == False:
        folder=os.path.dirname(target_file)
        if os.path.exists(folder) == False:       
            try:
                os.makedirs(folder)
            except Exception as e:
                print("Unable to create folder: " + str(e))
        try:
            shutil.copyfile(backup_file, target_file)
        except Exception as e:
            print("Unable to copy file: " + str(e))

    config_dict = nepi_utils.read_dict_from_file(target_file)
    if config_dict is None:
        config_dict = dict()
    for key in config_dict.keys(): # Fixe empty arrays
        if config_dict[key] is None:
            config_dict[key]=[]
    # print("Printing System Config Dict " + str(config_dict))
    return config_dict

def load_nepi_factory_config():
    config_dict=dict()

    target_file=FACTORY_CONFIG_FILE
    backup_file=SYSTEM_CONFIG_FILE
    if os.path.exists(target_file) == False:
        folder=os.path.dirname(target_file)
        if os.path.exists(folder) == False:       
            try:
                os.makedirs(folder)
            except Exception as e:
                print("Unable to create folder: " + str(e))
        try:
            shutil.copyfile(backup_file, target_file)
        except Exception as e:
            print("Unable to copy file: " + str(e))

    config_dict = nepi_utils.read_dict_from_file(target_file)
    if config_dict is None:
        config_dict = dict()
    for key in config_dict.keys(): # Fixe empty arrays
        if config_dict[key] is None:
            config_dict[key]=[]
    # print("Printing System Config Dict " + str(config_dict))
    return config_dict

def save_nepi_system_config(config_dict):
    target_file=SYSTEM_CONFIG_FILE
    backup_file=NEPI_SYSTEM_CONFIG_FILE
    if os.path.exists(target_file) == False:
        folder=os.path.dirname(target_file)
        if os.path.exists(folder) == False:       
            try:
                os.makedirs(folder)
            except Exception as e:
                print("Unable to create folder: " + str(e))
        try:
            shutil.copyfile(backup_file, target_file)
        except Exception as e:
            print("Unable to copy file: " + str(e))
    success = nepi_utils.write_dict_to_file(config_dict, target_file)
    return success

def update_nepi_system_configs(config_dict):
    success=False
    logger.log_warn('Got System Configs update ' + str(config_dict))
    cur_config_dict=load_nepi_system_config()
    if cur_config_dict is not None:
        for key in cur_config_dict.keys():
            if key not in config_dict.keys():
                config_dict[key] = cur_config_dict[key]
    logger.log_warn('Updating System Configs file with ' + str(config_dict))
    success=save_nepi_system_config(config_dict)
    return config_dict

def update_nepi_system_config(config_key, config_value, config_dict = None):
    success=False
    #logger.log_warn('Got System Config update ' + str([config_dict,config_key]))
    if config_dict is None:
        config_dict=load_nepi_system_config()
    if config_dict is not None:
        config_dict[config_key] = config_value
        #logger.log_warn('Updating System Config file with ' + str(config_dict[config_key]))
        success=save_nepi_system_config(config_dict)
    return config_dict

def restart_nepi_software():
    nepi_utils.bash_nepi_cmd('nepistart')

########################
def load_nepi_docker_config():
    config_dict = None

    target_file=DOCKER_CONFIG_FILE
    backup_file=DOCKER_CONFIG_BLANK_FILE
    if os.path.exists(target_file) == False or os.path.getsize(target_file) == 0:
        folder=os.path.dirname(target_file)
        if os.path.exists(folder) == False:       
            try:
                os.makedirs(folder)
            except Exception as e:
                print("Unable to create folder: " + str(e))
        try:
            shutil.copyfile(backup_file, target_file)
        except Exception as e:
            print("Unable to copy file: " + str(e))
    if os.path.exists(target_file) == True:
        config_dict = nepi_utils.read_dict_from_file(target_file)
        if config_dict is not None:
            if len(list(config_dict.keys())) == 0:
                config_dict = None
            else:
                for key in config_dict.keys(): # Fixe empty arrays
                    if config_dict[key] is None:
                        config_dict[key]=[]
            
    return config_dict

def save_nepi_docker_config(config_dict):
    if len(list(config_dict.keys())) == 0:
        return False
    target_file=DOCKER_CONFIG_FILE
    backup_file=DOCKER_CONFIG_BLANK_FILE
    if os.path.exists(target_file) == False or os.path.getsize(target_file) == 0:
        folder=os.path.dirname(target_file)
        if os.path.exists(folder) == False:       
            try:
                os.makedirs(folder)
            except Exception as e:
                print("Unable to create folder: " + str(e))
        try:
            shutil.copyfile(backup_file, target_file)
        except Exception as e:
            print("Unable to copy file: " + str(e))

    success = nepi_utils.write_dict_to_file(config_dict, DOCKER_CONFIG_FILE)
    return success

def update_nepi_docker_config(config_key, config_value):
    success=False
    config_dict=load_nepi_docker_config()
    if config_dict is not None:
        config_dict[config_key] = config_value
        success=save_nepi_docker_config(config_dict)
    return success