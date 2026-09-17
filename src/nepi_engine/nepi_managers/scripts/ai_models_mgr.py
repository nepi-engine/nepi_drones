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

import sys
import os
import os.path
import copy
import errno
import glob
import subprocess
import threading
import yaml
import time
import numpy as np
import cv2


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_img


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import MgrSystemStatus

from nepi_interfaces.msg import UpdateBool, MgrAiModelsStatus, AiModelStatus, MgrSystemStatus

from nepi_interfaces.srv import AiModelStatusQuery, AiModelStatusQueryRequest, AiModelStatusQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


DELAY_STARTUP_SEC = 0



class AIDetectorManager:   

    MODEL_TYPE_LIST = ['detection']
    MODEL_INFO_INTERVAL = 2


    node_if = None

    aifs_classes_dict = dict()
    aifs_dict = dict() 
    models_dict = dict()
    models_dict_lock = threading.Lock()
    models_namespace_dict = dict()

    
    all_namespace = 'ai'

    status_msg = MgrAiModelsStatus()
    status_published = False

    active_nodes = []
    active_topics = []
    active_topic_types = []
    active_services = []

    #######################
    ### Node Initialization


    DEFAULT_NODE_NAME = "ai_models_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### MGR NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        nepi_sdk.sleep(1)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        
        
        ##############################
        # Get for System Folders
        self.msg_if.pub_info("Waiting for system folders")
        system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got system folders: " + str(system_folders))

        self.aifs_param_folder = system_folders['aifs_param']
        self.msg_if.pub_info("Using AI Frameworks Params Folder: " + str(self.aifs_param_folder))

        self.aifs_api_folder = system_folders['api_pkg']
        self.msg_if.pub_info("Using AI Frameworks API Folder: " + str(self.aifs_api_folder))

        self.aifs_install_folder = system_folders['aifs_install']
        self.msg_if.pub_info("Using AI Frameworks Install Folder: " + str(self.aifs_install_folder))
        
        self.ai_models_folder = system_folders['aifs_models']
        self.msg_if.pub_info("Using AI Models Folder: " + str(self.ai_models_folder))

        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders()

        # self.msg_if.pub_info("Waiting for driver manager to start")
        # active_drivers = nepi_system.get_active_drivers(log_name_list = [self.node_name])
        # nepi_sdk.sleep(5) # Some extra time for drivers to load
        
       
        ##############################
        # Initialize Variables


        message = "NO MODELS ENABLED"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_no_models_img = nepi_img.cv2img_to_rosimg(cv2_img) 

        message = "LOADING MODELS"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_loading_img = nepi_img.cv2img_to_rosimg(cv2_img) 

        message = "WAITING FOR MODELS TO PUBLISH"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_waiting_img = nepi_img.cv2img_to_rosimg(cv2_img) 

        self.initCb(do_updates = False)

        
        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace
        }


        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'aifs_dict': {
                'namespace': self.node_namespace,
                'factory_val': dict()
            },  
            'models_dict': {
                'namespace': self.node_namespace,
                'factory_val': dict()
            }

        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'model_status_query': {
                'namespace': self.node_namespace,
                'topic': 'model_status_query',
                'srv': AiModelStatusQuery,
                'req': AiModelStatusQueryRequest(),
                'resp': AiModelStatusQueryResponse(),
                'callback': self.handleModelStatusRequest
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': MgrAiModelsStatus,
                'qsize': 1,
                'latch': True
            }
        }  


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'refresh_frameworks': {
                'namespace': self.node_namespace,
                'topic': 'refresh_frameworks',
                'msg': Empty,
                'qsize': 10,
                'callback': self.refreshCb, 
                'callback_args': ()
            },
            'update_framework_state': {
                'namespace': self.node_namespace,
                'topic': 'update_framework_state',
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.updateFrameworkStateCb, 
                'callback_args': ()
            },
            'enable_all_frameworks': {
                'namespace': self.node_namespace,
                'topic': 'enable_all_frameworks',
                'msg': Empty,
                'qsize': 10,
                'callback': self.enableAllFwsCb, 
                'callback_args': ()
            },
            'disable_all_frameworks': {
                'namespace': self.node_namespace,
                'topic': 'disable_all_frameworks',
                'msg': Empty,
                'qsize': 10,
                'callback': self.disableAllFwsCb, 
                'callback_args': ()
            },
            'enable_all_models': {
                'namespace': self.node_namespace,
                'topic': 'enable_all_models',
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.enableAllModelsCb, 
                'callback_args': ()
            },
            'disable_all_models': {
                'namespace': self.node_namespace,
                'topic': 'disable_all_models',
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.disableAllModelsCb, 
                'callback_args': ()
            },
            'update_model_state': {
                'namespace': self.node_namespace,
                'topic': 'update_model_state',
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.updateModelStateCb, 
                'callback_args': ()
            },
            'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self.systemStatusCb
            }

        }



        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        msg_if = self.msg_if
        )


        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()
        self.msg_if.pub_info("Delaying Startup Secs: " + str(DELAY_STARTUP_SEC))
        nepi_sdk.sleep(DELAY_STARTUP_SEC)
        ###########################

        success = self.initCb(do_updates = True)


        ###########################


        self.msg_if.pub_info("Staring AI Status Pub process")
        nepi_sdk.start_timer_process(1.0, self.statusPubCb)
        self.msg_if.pub_info("Staring AI Framework and Model update process")
        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)


        self.ros_waiting_img.header.stamp = nepi_sdk.get_msg_stamp()
        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_sdk.spin()
        #########################################################


    ####################
    # Wait for System and Config Statuses Callbacks
    def systemStatusCb(self,msg):
        self.active_nodes = msg.active_nodes
        self.active_topics = msg.active_topics
        self.active_topic_types = msg.active_topic_types
        self.active_services = msg.active_services
        

    def initCb(self, do_updates = False):
      success = True
      if self.node_if is not None:
        self.msg_if.pub_warn("Initializing Node Variables")
        self.aifs_dict = self.node_if.get_param('aifs_dict')

        models_dict = self.node_if.get_param('models_dict')
        self.updateModelsDict(models_dict)

        self.msg_if.pub_warn("Calling Refresh Process")
        success = self.refresh()

      return success



    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def factoryResetCb(self,do_updates = True):
        self.aifs_classes_dict = dict()
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)



    ####################
    # Wait for System and Config Statuses Callbacks
        

    def configStatusCb(self,msg):
        self.cfg_status = True
        

    #######################
    ### Mgr Config Functions

    def refreshCb(self,msg):
        self.refresh()

    def refresh(self):
        success = False
        #self.msg_if.pub_warn("Starting Refresh with models dict with keys: " + str(self.models_dict.keys()))
        self.msg_if.pub_warn("Stopping all running models")
        models_dict = self.copyModelsDict()
        for model_name in models_dict.keys():
                if models_dict[model_name]['active'] == True:
                  node_name = models_dict[model_name]['node_name']
                  success = self.killModel(model_name)
                  if success == False:
                    self.killModel(node_name)
        time.sleep(1)
        aif_classes = list(self.aifs_classes_dict.keys())
        for aif_class in aif_classes:
            del self.aifs_classes_dict[aif_class]


        if self.node_if is not None:

            ## Find AI Frameworks
            # Get ai framework dict form param server and update
            #self.msg_if.pub_warn("Refreshing latest ais dict " + str(self.aifs_dict))

            # Update AI Frameworks Dict
            orig_aifs_dict = copy.deepcopy(self.aifs_dict)
            cur_aif_keys = orig_aifs_dict.keys()
            #self.msg_if.pub_warn("Got last aifs " + str(cur_aif_keuys))
            aifs_dict = nepi_aifs.getAIFsDict(self.aifs_param_folder, self.aifs_api_folder)
            #self.msg_if.pub_warn("Refreshed aifs dict with keys: " + str(self.aifs_dict.keys()))
            #self.msg_if.pub_warn("Refreshed active aifs list: " + str(self.getActiveAifs())) 
            new_aif_keys = aifs_dict.keys()
            #self.msg_if.pub_warn("Got updated aifs " + str(new_aif_keys))
            if cur_aif_keys != new_aif_keys:
                self.msg_if.pub_warn("Got updated ai framework list: " + str(new_aif_keys))

            orig_models_dict = self.copyModelsDict()
            models_dict = dict()

            # Refresh Frameworks if Needed
            purge_list = []
            for aif_name in aifs_dict.keys():
                aif_dict = aifs_dict[aif_name]

                self.msg_if.pub_info(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> " + aif_name)
                self.msg_if.pub_info("Processing ais dict for aif name " + aif_name)
                success = False
                if aif_name not in self.aifs_classes_dict.keys():
                    self.msg_if.pub_info("Updating ai dict for framework: " + str(aif_name))
                    file_name = aif_dict['if_file_name']
                    file_path = aif_dict['api_path']
                    module_name = aif_dict['if_module_name']
                    class_name = aif_dict['if_class_name']
                    #self.msg_if.pub_warn("Got aif dict: " + aif_name + " : " + str(aif_dict))
                    sys.path.append(file_path)
                    self.msg_if.pub_warn("Importing AIF Class File: " + str(file_path)  + " " + file_name + " from module " + module_name)
                    [success, msg, aif_class] = nepi_aifs.importAIFClass(file_name,file_path,module_name,class_name)
                    if success == True:
                        success = False
                        try:
                            self.msg_if.pub_warn("Instantiating IF class for framework: " + str(aif_name))
                            launch_namespace = self.base_namespace #os.path.join(self.base_namespace, "ai")
                            aif_if_class_instance = aif_class(aif_dict, launch_namespace, self.ai_models_folder)
                            self.msg_if.pub_warn("Created aif instantiated if class: " + aif_name )
                            supported = aif_if_class_instance.checkFrameworkSupport()
                            if supported == False:
                                self.msg_if.pub_warn("Framework not supported: " + aif_name )
                                if aif_name not in purge_list:
                                        purge_list.append(aif_name)
                                success = False
                            else:
                                self.aifs_classes_dict[aif_name] = aif_if_class_instance
                                success = True
                                time.sleep(1) # Give some time for publishers to set in class init
                        except Exception as e:
                            self.msg_if.pub_warn("Failed to instantiate ai framework if class " + class_name + " " + str(e))
                            if aif_name not in purge_list:
                                purge_list.append(aif_name)
                    else:
                        self.msg_if.pub_warn("Failed to Import ai framework if class " + class_name + " " + str(e))
                        if aif_name not in purge_list:
                                purge_list.append(aif_name)

                else:
                    aif_if_class_instance = self.aifs_classes_dict[aif_name]
                    self.msg_if.pub_warn("Got aif instantiated class: " + aif_name)
                    success = True

                # Refresh Models Dict
                if success == True:
                    if aif_name in orig_aifs_dict.keys():
                        aifs_dict[aif_name]['active'] = orig_aifs_dict[aif_name]['active']
                    else:
                        aifs_dict[aif_name]['active'] = False
                    try:
                        aif_models_dict = aif_if_class_instance.getModelsDict()
                    except Exception as e:
                        aif_models_dict = dict()
                        self.msg_if.pub_warn("Failed to Get Model Info for ai framework: " + aif_name + " " + str(e))
                    if (len(aif_models_dict.keys()) < 1):
                        self.msg_if.pub_warn("No models found for this ai framework: " + aif_name)
                    else:
                        self.msg_if.pub_info("Got models for framework: " + str(aif_name) + " from param server " + str(aif_models_dict.keys()))
                        pass
                    for model_name in aif_models_dict.keys():
                        try:
                            if model_name in orig_models_dict.keys():
                                aif_models_dict[model_name]['active'] = orig_models_dict[model_name]['active']
                            else: 
                                aif_models_dict[model_name]['active'] = False

                            models_dict[model_name] = aif_models_dict[model_name]
                          
                            success = True
                        except Exception as e:
                            self.msg_if.pub_warn("Failed to get models from class " + class_name + " " + str(e))
                            continue
                        
                        
            # Purge unsupported frameworks
            for aif_name in purge_list:
                del aifs_dict[aif_name]
    
           
            self.aifs_dict = aifs_dict
            self.node_if.set_param('aifs_dict', aifs_dict)
            self.updateModelsDict(models_dict)
            self.node_if.set_param('models_dict',models_dict)      


        return success
        
    def updateModelsDict(self, models_dict):
        self.models_dict_lock.acquire()
        self.models_dict = models_dict
        self.models_dict_lock.release()
        return models_dict

    def copyModelsDict(self):
        self.models_dict_lock.acquire()
        models_dict = copy.deepcopy(self.models_dict)
        self.models_dict_lock.release()
        return models_dict
        

    def printModelsDict(self,models_dict):
        self.msg_if.pub_warn("" )
        for model_name in models_dict.keys():
            model_dict = models_dict[model_name]
            self.printModelDict(model_dict)
        self.msg_if.pub_warn("" )


    def printModelDict(self,model_dict):
        #self.msg_if.pub_warn("" )
        for key in model_dict.keys():    
            value = model_dict[key]
            self.msg_if.pub_warn("Model key: " + str(key) + " value: " + str(value))
        #self.msg_if.pub_warn("" )
 

    def getActiveAifs(self):
        aaifs = []
        aifs_dict = copy.deepcopy(self.aifs_dict)
        for aif_name in aifs_dict.keys():
            if 'active' in aifs_dict[aif_name].keys():
                if aifs_dict[aif_name]['active'] == True:
                    aaifs.append(aif_name)
        return aaifs

    def getActiveModels(self):
        active_models = []
        models_dict = self.copyModelsDict()
        for model_name in models_dict.keys():
            if 'active' in models_dict[model_name].keys():
                if models_dict[model_name]['active'] == True:
                    active_models.append(model_name)
        return active_models


    def updaterCb(self,timer):
        #self.msg_if.pub_warn("Starting Updater with models dict with keys: " + str(self.models_dict.keys()))
        # for key in self.models_dict.keys():
        #     self.msg_if.pub_warn("Starting Updater with g model dict with keys: " + str(key) + ' : ' + str(self.models_dict[key].keys()))
        # self.msg_if.pub_warn("**********************")
        # self.msg_if.pub_warn(" ")

        active_aifs = self.getActiveAifs()
        #self.msg_if.pub_warn("Update")
        #self.msg_if.pub_warn("Updating aifs dict with keys: " + str(self.aifs_dict.keys()))
        #self.msg_if.pub_warn("Updating active aifs list: " + str(self.getActiveAifs()))        
        models_dict = self.copyModelsDict()


        active_models_list = self.getActiveModels()
        # self.msg_if.pub_warn("Purge Check with active models list: " + str(active_models_list))
        # self.msg_if.pub_warn("Purge Check with models_namespace_dict keys: " + str(self.models_namespace_dict.keys()))        
        purge_list = []
        for model_name in self.models_namespace_dict.keys():
            model_aif = models_dict[model_name]['framework']
            if model_name not in active_models_list or model_aif not in active_aifs:
                self.msg_if.pub_warn("Adding model to purge list: " + str(model_name))
                purge_list.append(model_name)

        for model_name in purge_list:
            try:
                node_name = models_dict[model_name]['node_name']
                #self.msg_if.pub_warn("Calling Mgr Kill model: " + node_name)
                models_dict[model_name]['active'] = False
                del self.models_namespace_dict[model_name]
                node_name = models_dict[model_name]['node_name']
                success = self.killModel(model_name)
                if success == False:
                    self.killModel(node_name)
                

                nepi_sdk.wait()
            except Exception as e:
                self.msg_if.pub_warn("Failed to Kill model: " + node_name + " : " + str(e))

        ######## Launch Models
        active_models_list = self.getActiveModels()
        #self.msg_if.pub_warn("Launch Check with active models list: " + str(active_models_list))
        # self.msg_if.pub_warn("Launch Check with models_namespace_dict keys: " + str(self.models_namespace_dict.keys())) 
        for model_name in active_models_list:
            model_aif = models_dict[model_name]['framework']
            if model_name not in self.models_namespace_dict.keys() and model_aif in active_aifs:
                self.msg_if.pub_warn("Calling Launch Model process for Model: " + model_name)
                model_dict = models_dict[model_name]
                node_namespace = self.loadModel(model_name, model_dict)

                #self.models_dict_lock.acquire()
                #self.models_dict[model_name]['running'] = False

                if node_namespace is not None:
                    self.msg_if.pub_warn("Got Launch Namespace for Model: " + model_name + " : " + str(node_namespace))
                    self.models_dict[model_name]['msg'] = "Model launched"
                    self.models_namespace_dict[model_name] = node_namespace
                    self.publish_status()
                else:
                    self.msg_if.pub_warn("Model failed to launch: " + model_name)
                    self.msg_if.pub_warn("Setting model to disabled: " + model_name)
                    self.models_dict[model_name]['msg'] = "Model failed to launch"
                    if model_name in list(self.models_namespace_dict.keys()):
                        del self.models_namespace_dict[model_name]
                    self.models_dict[model_name]['active'] = False
                #self.models_dict_lock.release()

        ######## Check Running Models
        active_models_list = self.getActiveModels()
        #self.msg_if.pub_warn("Running Check with active models list: " + str(active_models_list))
        # self.msg_if.pub_warn("Run Check with models_namespace_dict keys: " + str(self.models_namespace_dict.keys())) 
        for model_name in models_dict.keys():
            model_dict = models_dict[model_name]
            was_running = False
            if 'running' in model_dict.keys():
                was_running =  model_dict['running']
            model_node_name = model_dict['node_name']
            #self.msg_if.pub_warn("Check for running model node name: " + str(model_node_name) + " in " + str(self.active_nodes))

            running = model_node_name in self.active_nodes 
            #self.models_dict_lock.acquire()
            self.models_dict[model_name]['running'] = running
            if running == True:
                self.models_dict[model_name]['msg'] = "Model running"
                if was_running == False:
                    self.msg_if.pub_warn("Model Running: " + model_node_name)
                    
            elif running == False:
                if was_running == True and model_name in active_models_list:
                    self.models_dict[model_name]['msg'] = "Model stopped not running"
                    self.models_dict[model_name]['active'] = False
                else:
                    self.models_dict[model_name]['msg'] = "Model not running"
            #self.models_dict_lock.release()

                
    

        # self.msg_if.pub_warn("Updated models dict with keys: " + str(self.models_dict.keys()))
        # for key in self.models_dict.keys():
        #     self.msg_if.pub_warn("Updated model dict with keys: " + str(key) + ' : ' + str(self.models_dict[key].keys()))

        #self.msg_if.pub_warn("Updated active models list: " + str(self.getActiveModels()))
        if len(active_models_list) == 0:
            self.ros_no_models_img.header.stamp = nepi_sdk.get_msg_stamp()
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param("models_dict",self.models_dict)


        #self.msg_if.pub_warn("Ending Updater with models dict with keys: " + str(self.models_dict.keys()))
        # for key in self.models_dict.keys():
        #     self.msg_if.pub_warn("Ending Updater with g model dict with keys: " + str(key) + ' : ' + str(self.models_dict[key].keys()))
        # self.msg_if.pub_warn("**********************")
        # self.msg_if.pub_warn(" ")


        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)


    def loadModel(self, model_name, model_dict):       
        success = False
        node_namespace = None
        if model_name != "None": 
            aif_name=self.models_dict[model_name]['framework']
            if aif_name not in self.aifs_classes_dict.keys():
                self.msg_if.pub_warn("Model Framework Class not instantiated")
            else:
                try:
                    aif_class = self.aifs_classes_dict[aif_name]
                    self.msg_if.pub_warn("Launching Node for Model: " + model_name)
                    [success,node_namespace] = aif_class.launchModel(model_dict)
                    if success == True:
                        self.msg_if.pub_warn("Model Node Launched with namespace: " + node_namespace)
                    else:
                        self.msg_if.pub_warn("Model Failed to Launch: " + model_name)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to Launch Node for model: " + model_name + " : " + str(e))


        return node_namespace
        
    def killModel(self, model_name):
        success = False
        if model_name != "None": 
            models_dict = self.copyModelsDict()
            self.msg_if.pub_warn("Kill Model Requested with model keys: " + str(models_dict.keys()))
            #self.msg_if.pub_warn("Kill Model Requested with model dict: " + str(models_dict))
            if model_name not in models_dict.keys():
                self.msg_if.pub_warn("Unknown model model requested: " + model_name)
            else:
                node_name = models_dict[model_name]['node_name']
                aif_name=self.models_dict[model_name]['framework']
                if aif_name not in self.aifs_classes_dict.keys():
                    self.msg_if.pub_warn("Model Framework Class not instantiated")
                else:
                    # Call Kill model
                    aif_class = self.aifs_classes_dict[aif_name]
                    #self.msg_if.pub_info("Calling AIF Kill model " + model_name)
                    #self.msg_if.pub_info("Checking for model in namespace dict " + str(self.models_namespace_dict.keys()))
                    if model_name in self.models_namespace_dict.keys():
                        #node_name = os.path.basename(self.models_namespace_dict[model_name])
                        del self.models_namespace_dict[model_name]
                    aif_class.killModel(node_name)
                    # Kill Img Pub Node if running
                    nepi_sdk.kill_node(node_name + '_img_pub')
                    self.models_dict_lock.acquire()
                    self.models_dict[model_name]['msg'] = "Model killed"
                    self.models_dict_lock.release()
                    success = True
        return success
  
    def save_config(self):
        # Save framework and model dictionaries
        self.node_if.save_config() # Save config after initialization


    def updateFrameworkStateCb(self,msg):
        self.msg_if.pub_warn("Recieved Framework State Update: " + str(msg))
        framework_name = msg.name
        state = msg.value
        active_aifs = self.getActiveAifs()
        aifs_dict = copy.deepcopy(self.aifs_dict)
        if state == True:
            if framework_name not in active_aifs:
                self.msg_if.pub_warn("Setting AI Framework: " + str(framework_name) + " Active")
                if framework_name in aifs_dict.keys():
                    aifs_dict[framework_name]['active'] = True
                    self.aifs_dict = aifs_dict
                    self.publish_status()
                    if self.node_if is not None:
                        self.node_if.set_param('aifs_dict', aifs_dict)

        else:
            if framework_name in active_aifs:
                self.msg_if.pub_warn("Setting AI Framework: " + str(framework_name) + " Inctive")
                aifs_dict[framework_name]['active'] = False
                self.aifs_dict = aifs_dict
                self.publish_status()
                if self.node_if is not None:
                    self.node_if.set_param('aifs_dict', aifs_dict)

        
        


    def disableAllFwsCb(self,msg):
        aifs_dict = copy.deepcopy(self.aifs_dict)
        for aif in aifs_dict.keys():
            aifs_dict[aif]['active'] = False
        self.aifs_dict = aifs_dict
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('aifs_dict', aifs_dict)


    def enableAllFwsCb(self,msg):
        aifs_dict = copy.deepcopy(self.aifs_dict)
        available_frameworks = list(aifs_dict.keys())
        for aif in aifs_dict.keys():
            aifs_dict[aif]['active'] = True
        self.aifs_dict = aifs_dict
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('aifs_dict', aifs_dict)


    def enableAllModelsCb(self,msg):
        self.msg_if.pub_warn("Recieved Enable All Models msg: " + str(msg))
        aif_name = msg.name
        state = msg.value
        active_aifs = self.getActiveAifs()
        if aif_name not in active_aifs:
            self.msg_if.pub_warn("Ignoring request. AI Framework: " + str(aif_name) + " not enabled")
        else:
            models_dict = self.copyModelsDict()
            for model_name in models_dict.keys():
                model_dict = models_dict[model_name]
                if model_dict['framework'] == aif_name:
                    models_dict[model_name]['active'] = True
        self.updateModelsDict(models_dict)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param("models_dict",models_dict)

 

    def disableAllModelsCb(self,msg):
        self.msg_if.pub_warn("Recieved Disable All Models msg: " + str(msg))
        aif_name = msg.name
        state = msg.value
        active_aifs = self.getActiveAifs()
        if aif_name not in active_aifs:
            self.msg_if.pub_warn("Ignoring request. AI Framework: " + str(aif_name) + " not enabled")
        else:
            models_dict = self.copyModelsDict()
            for model_name in models_dict.keys():
                model_dict = models_dict[model_name]
                if model_dict['framework'] == aif_name:
                    models_dict[model_name]['active'] = False
                
        self.updateModelsDict(models_dict)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param("models_dict",models_dict)


    def updateModelStateCb(self,msg):
        self.msg_if.pub_warn("Recieved Model State Update: " + str(msg))
        model_name = msg.name
        state = msg.value
        models_dict = self.copyModelsDict()
        if model_name in models_dict.keys():
            model_dict = models_dict[model_name]
            model_aif = model_dict['framework']
            active_aifs = self.getActiveAifs()
            if model_aif not in active_aifs:
                self.msg_if.pub_warn("Ignoring request. Model's AI Framework: " + str(model_aif) + " not enabled")
            else:
                active_state = model_dict['active']
                if state != active_state:
                    if state == True:
                        self.msg_if.pub_warn("Changing Model State to: True")
                        models_dict[model_name]['active'] = True
                    else:
                        self.msg_if.pub_warn("Changing Model State to: False")
                        models_dict[model_name]['active'] = False
        if self.models_dict != models_dict:
            self.updateModelsDict(models_dict)
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param("models_dict",models_dict)



    def getModelStatus(self, model_name):
        models_dict = self.copyModelsDict()
        model_status_msg = AiModelStatus()
        if model_name in self.models_dict.keys():
            model_dict = models_dict[model_name]
            try:
                model_status_msg.model_name = model_name

                model_status_msg.display_name = model_dict['display_name']
                model_status_msg.description = model_dict['description']

                model_status_msg.node_name = model_dict['node_name']
                if model_name in list(self.models_namespace_dict.keys()):
                    model_status_msg.namespace = self.models_namespace_dict[model_name]

                model_status_msg.framework = model_dict['framework']
                model_status_msg.type = model_dict['type']
                size_mb = -999
                try:
                    size_mb = int(model_dict['size'])
                except:
                    pass
                model_status_msg.size_mbytes = size_mb

                model_status_msg.proc_img_height = model_dict['img_height']
                model_status_msg.proc_img_width = model_dict['img_width']
                model_status_msg.classes = model_dict['classes']


                model_status_msg.enabled = model_dict['active']

                running = False
                if 'running' in model_dict.keys():
                    running = model_dict['running']

                model_status_msg.running = running
                model_status_msg.order = 0

                msg_str = 'Not running'
                if 'msg' in model_dict.keys():
                    msg_str = model_dict['msg']
                model_status_msg.msg_str = msg_str  
            except Exception as e:
                self.msg_if.pub_warn("Failed to create status message for model: " + model_name + " : " + str(e), throttle_s = 5)

        return model_status_msg    



    def handleModelStatusRequest(self,req):
        model_name = req
        self.msg_if.pub_warn("Handling Request for model info query for mode: " + str(model_name))
        resp = AiModelStatusQueryResponse()
        resp = self.getModelStatus(model_name)
        #self.msg_if.pub_warn("Respose for model info query " + str(resp))
        return resp

    def statusPubCb(self,timer):
        self.publish_status()

    def publish_status(self, do_updates = False):

        last_status_msg = copy.deepcopy(self.status_msg)
        aifs_dict = copy.deepcopy(self.aifs_dict)
        models_dict = self.copyModelsDict()

        #self.msg_if.pub_warn("Status Using AIFs Dict: " + str(aifs_dict.keys()))
        # self.msg_if.pub_warn("Status Using Models Dict: " + str(models_dict.keys()))

        status_msg = MgrAiModelsStatus()

        ai_frameworks =  nepi_aifs.getAIFsSortedList(aifs_dict)
        status_msg.ai_frameworks_list = ai_frameworks
        active_aifs = self.getActiveAifs()
        ai_frameworks_folders = []

        ai_frameworks_active = []
        for aif_name in ai_frameworks:
            if aif_name in active_aifs:
                ai_frameworks_active.append(aif_name)
            ai_frameworks_folders.append(aifs_dict[aif_name]['models_folder_name'])
        status_msg.ai_frameworks_active_list = ai_frameworks_active
        status_msg.ai_frameworks_folder_list = ai_frameworks_folders



        ai_models_ordered = nepi_aifs.getModelsSortedList(models_dict)
        status_msg.ai_models_ordered_list = ai_models_ordered


        ai_models_ordered_name_list = []
        ai_models_ordered_framework_list = []
        ai_models_ordered_type_list = []
        ai_models_ordered_status_list = []


        ai_models_active_list = []


        ai_models_running_list = []
        ai_models_running_name_list = []
        ai_models_running_type_list = []
        ai_models_running_namespace_list = []


        for model_name in ai_models_ordered:
            ai_models_ordered_name_list.append(models_dict[model_name]['display_name'])
            ai_models_ordered_framework_list.append(models_dict[model_name]['framework'])
            ai_models_ordered_type_list.append(models_dict[model_name]['type'])
            ai_models_ordered_status_list.append(self.getModelStatus(model_name))
            active = False
            if 'active' in models_dict[model_name].keys():
                if models_dict[model_name]['active']== True:
                    ai_models_active_list.append(model_name)

            if 'running' in models_dict[model_name].keys():
                if models_dict[model_name]['running'] == True:
                    ai_models_running_list.append(model_name)
                    ai_models_running_name_list.append(models_dict[model_name]['display_name'])
                    ai_models_running_type_list.append(models_dict[model_name]['type'])
                if model_name in self.models_namespace_dict.keys():
                    ai_models_running_namespace_list.append(self.models_namespace_dict[model_name])
                

        status_msg.ai_models_ordered_name_list = ai_models_ordered_name_list
        status_msg.ai_models_ordered_framework_list = ai_models_ordered_framework_list
        status_msg.ai_models_ordered_type_list = ai_models_ordered_type_list
        status_msg.ai_models_ordered_status_list = ai_models_ordered_status_list


        status_msg.ai_models_active_list = ai_models_active_list


        status_msg.ai_models_running_list = ai_models_running_list
        status_msg.ai_models_running_name_list = ai_models_running_name_list
        status_msg.ai_models_running_type_list = ai_models_running_type_list
        status_msg.ai_models_running_namespace_list = ai_models_running_namespace_list


        status_msg.all_namespace = self.all_namespace


        self.status_msg = status_msg
        if self.node_if is not None:
            if self.status_published == False:
                self.status_published = True
                #self.msg_if.pub_info("Publishing Status Msg: " + str(self.status_msg))

            self.node_if.publish_pub('status_pub', self.status_msg)
            if last_status_msg != self.status_msg:
                self.node_if.save_config() # Save config
                status_dict = nepi_sdk.convert_msg2dict(status_msg)
                nepi_system.set_ai_models_dict(status_dict)

if __name__ == '__main__':
    AIDetectorManager()
