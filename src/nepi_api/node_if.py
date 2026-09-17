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
import time 
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_system

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import UpdateString

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyRequest as EmptySrvRequest
from std_srvs.srv import EmptyResponse as EmptySrvResponse


from nepi_api.messages_if import MsgIF







#########################################
# Connect Node Setup Classes
#########################################


##################################################
### Node Config Class

'''
EXAMPLE_CONFIGS_DICT = {
        'namespace': None,
        'alt_namespace': None,
        'init_callback': None,
        'reset_callback': None,
        'factory_reset_callback': None,
        'init_configs': True,
        'manage_configs': True,
        'clear_params': True
}
'''


class NodeConfigsIF:

    MAX_SAVE_RATE = 1

    msg_if = None
    ready = False
    configs_dict = None
    namespace = '~'
    alt_namespace = None

    initCb = None
    sysResetCb = None
    resetCb = None
    factoryResetCb = None

    save_triggered = False
    save_all_triggered = False
    last_save = nepi_sdk.get_time()
    ### IF Initialization
    def __init__(self, 
                configs_dict,
                wait_cfg_mgr = True,
                log_name = None,
                log_class_name = True,
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
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_debug("Starting Node Configs IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################    



        self.msg_if.pub_debug("Got Config Dict: " + str(configs_dict), log_name_list = self.log_name_list)

        if 'init_callback' in configs_dict.keys():  
            self.initCb = configs_dict['init_callback']

        if 'reset_callback' in configs_dict.keys():
            self.resetCb = configs_dict['reset_callback']

        if 'factory_reset_callback' in configs_dict.keys():
            self.factoryResetCb = configs_dict['factory_reset_callback']
        

        if 'namespace' not in configs_dict.keys():
            configs_dict['namespace'] = None
        namespace = configs_dict['namespace']
        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        if 'alt_namespace' in configs_dict.keys():
            self.alt_namespace = configs_dict['alt_namespace']
        #self.msg_if.pub_warn("Using Config namespace: " + str(self.namespace), log_name_list = self.log_name_list)
        #self.msg_if.pub_warn("Using Base namespace: " + str(self.base_namespace), log_name_list = self.log_name_list)

        clear_params = True
        if 'clear_params' in configs_dict.keys():
            clear_params = configs_dict['clear_params']
        if clear_params == True:
            self.msg_if.pub_debug("Clearing params for namespace: " + str(self.namespace), log_name_list = self.log_name_list)
            nepi_sdk.delete_params(self.namespace)
        

        manage_configs = True
        if 'manage_configs' in configs_dict.keys():
            manage_configs = configs_dict['manage_configs']
        if manage_configs == True:
            ###############
            # Create Config Publishers
            ns = nepi_sdk.create_namespace(self.base_namespace,'save_params')
            self.save_params_pub = nepi_sdk.create_publisher(ns, String, queue_size=1)
            ns = nepi_sdk.create_namespace(self.base_namespace,'save_params_all')
            self.save_params_all_pub = nepi_sdk.create_publisher(ns, String, queue_size=1)
            ns = nepi_sdk.create_namespace(self.base_namespace,'reset_params')
            self.reset_params_pub = nepi_sdk.create_publisher(ns, UpdateString, queue_size=1)
            ns = nepi_sdk.create_namespace(self.base_namespace,'delete_configs')
            self.delete_configs_pub = nepi_sdk.create_publisher(ns, UpdateString, queue_size=1)


            nepi_sdk.sleep(1)

            ###############
            # Create Config Subscribers
            # Subscribe to save config for node namespace
            nepi_sdk.create_subscriber(self.namespace + '/save_config', Empty, self._saveCb, queue_size=5)

            nepi_sdk.create_subscriber(self.namespace + '/init_config', Empty, self._initCb, queue_size=5)
            nepi_sdk.create_subscriber(self.namespace + '/reset_config', Empty, self._resetCb, queue_size=5)
            nepi_sdk.create_subscriber(self.namespace + '/factory_reset_config', Empty, self._factoryResetCb, queue_size=5)


            # Global Topic Subscribers
            nepi_sdk.create_subscriber('save_config', Empty, self._saveCb, queue_size=5)
            nepi_sdk.create_subscriber('init_config', Empty, self._initCb, queue_size=5)
            nepi_sdk.create_subscriber('reset_config', Empty, self._resetCb, queue_size=5)
            nepi_sdk.create_subscriber('factory_reset_config', Empty, self._factoryResetCb, queue_size=5)

            if nepi_system.supports_all_config(self.namespace) == True:
                nepi_sdk.create_subscriber(self.namespace + '/save_config_all', Empty, self._saveAllCb)
            nepi_sdk.create_subscriber(self.namespace + '/delete_configs', Empty, self._deleteConfigsCb)


            ################
            self.msg_if.pub_warn("Resetting Params", log_name_list = self.log_name_list)
            self.reset_params(do_updates = False)
            nepi_sdk.sleep(1)

        # params_dict = nepi_sdk.get_param(self.namespace, dict())
        # self.msg_if.pub_warn("Got Reset Params: " + str(params_dict), log_name_list = self.log_name_list)

        if 'init_configs' in configs_dict.keys():
            
            init_configs = configs_dict['init_configs']
            if init_configs == True:
                self.msg_if.pub_warn("Calling parent reset function", log_name_list = self.log_name_list)
                self.reset_config()
        nepi_sdk.start_timer_process(self.MAX_SAVE_RATE, self._updaterCb, oneshot = True)
        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("Node Configs IF Initialization Complete", log_name_list = self.log_name_list)
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("Ready", log_name_list = self.log_name_list)
        return self.ready

    def reset_params(self, do_updates = True):
        msg = UpdateString()
        msg.name = self.namespace
        if self.alt_namespace is not None:
            msg.name2 = self.alt_namespace
            self.msg_if.pub_warn("Reseting Params: " + str(self.namespace) + " including alt namespace" + str(self.alt_namespace))
        else:
            self.msg_if.pub_warn("Reseting Params: " + str(self.namespace))
        self.reset_params_pub.publish(msg)




    def init_config(self, do_updates = False):
        if (self.initCb is not None):
            try:
                self.initCb(do_updates = do_updates) # Callback provided by the container class to set init values to current values, etc.
            except:
                # self.initCb()
                pass

    def save_config(self):
        self.save_triggered = True


    def save_config_all(self):
        self.msg_if.pub_info("Saving Config All: " + str(self.namespace))
        self.save_all_triggered = True



    def delete_config_all(self):
        self.msg_if.pub_debug("Deleting Config All: " + str(self.namespace))
        msg = UpdateString()
        msg.name = self.namespace
        msg.name2 = self.namespace
        if self.alt_namespace is not None:
            msg.name2 = self.alt_namespace
        self.delete_configs_pub.publish(msg)



    def reset_config(self):
        self.msg_if.pub_warn("Resetting Config: " + str(self.namespace))
        if self.resetCb is not None and not nepi_sdk.is_shutdown():
           self.resetCb() # Callback provided by container class to update based on param server, etc.

    def factory_save_config(self):
        self.msg_if.pub_warn("Factory Saving Config: " + str(self.namespace))


    def factory_reset_config(self):
        self.msg_if.pub_warn("Factory Resetting Config: " + str(self.namespace))
        if self.factoryResetCb is not None and not nepi_sdk.is_shutdown():
            self.factoryResetCb() # Callback provided by container class to update based on param server, etc.
  



    ###############################
    # Class Private Methods
    ###############################

    def _updaterCb(self, timer):
        if self.save_all_triggered == True:
            self.save_params_all_pub.publish(self.namespace)
        elif self.save_triggered == True:
            self.save_params_pub.publish(self.namespace)
        self.save_triggered = False
        self.save_all_triggered = False
        nepi_sdk.start_timer_process(self.MAX_SAVE_RATE, self._updaterCb, oneshot = True)

    def _saveCb(self,msg):
        self.save_config()

    def _saveAllCb(self,msg):
        self.msg_if.pub_warn("Got Save Config All Msg", log_name_list = self.log_name_list)
        self.save_config_all()

    def _deleteConfigsCb(self,msg):
        self.msg_if.pub_warn("Got Delete Config All Msg", log_name_list = self.log_name_list)
        self.delete_config_all()

    def _initCb(self,msg):
        self.init_config(do_updates = True)

    def _resetCb(self,msg):
        self.msg_if.pub_warn("Got Reset Config Request", log_name_list = self.log_name_list)
        self.reset_config() 

    def _factorySaveCb(self,msg):
        self.factory_save_config()

    def _factoryResetCb(self,msg):
        self.msg_if.pub_warn("Got Factory Reset Config Request", log_name_list = self.log_name_list)
        self.factory_reset_config()






##################################################
### Node Params Class
'''
EXAMPLE_PARAMS_DICT = {
    'param1_name': {
        'namespace':  self.node_namespace,
        'factory_val': 100,
        'current_val: 20  # Optional
    },
    'param2_name': {
        'namespace':  self.node_namespace,
        'factory_val': "Something"
    }
}
'''


class NodeParamsIF:

    msg_if = None
    ready = False
    params_dict = dict()

    initCb = None
    resetCb = None
    factoryResetCb = None

    params_ns_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                params_dict = None,
                log_name = None,
                log_class_name = True,
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
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_debug("Starting Node Params IF Initialization Processes", log_name_list = self.log_name_list)
        ##############################   

        ##############################  
        # Initialize Params System

        self.params_dict = params_dict
        if self.params_dict is None:
            self.params_dict = dict()
        self.initialize_params()


        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("Ready", log_name_list = self.log_name_list)
        return self.ready

    def load_params(self, file_path):
        self.nepi_sdk.load_params_from_file(file_path,self.namespace)        

    def getNestedInitVal(self, ns_param_dict, param_name):
        # A param key may contain '/' ('home_position/pan_deg'), and get_params returns
        # the namespace as a NESTED dict, so a slashed key never matches a top-level
        # key. Matching against the top level made every slashed param fall through to
        # its factory_val and then overwrite the correctly reloaded server value, which
        # is why svx and ptx home position could not survive a restart. Walk the
        # segments instead. A flat key still resolves on the first pass.
        val = ns_param_dict
        for key in param_name.split('/'):
            if isinstance(val, dict) and key in val.keys():
                val = val[key]
            else:
                return None
        return val

    def initialize_params(self):

        self.msg_if.pub_warn("Initializing params: " + str(self.params_dict.keys()), log_name_list = self.log_name_list)
        self.msg_if.pub_warn("Initializing params: " + str(self.params_dict), log_name_list = self.log_name_list)
        params_dict = copy.deepcopy(self.params_dict)
        init_val = None
        got_params_dict = dict()
        for param_name in params_dict.keys():
            namespace = params_dict[param_name]['namespace']
            if namespace not in got_params_dict.keys():
                get_params_dict = nepi_sdk.get_params(namespace)
                if get_params_dict is not None:
                    if isinstance(get_params_dict, dict):
                        got_params_dict[namespace] = get_params_dict
            if 'init_val' not in params_dict[param_name].keys():
                param_dict = params_dict[param_name]
                factory_val = param_dict['factory_val']

                ns_param_dict = got_params_dict[namespace]
                init_val = self.getNestedInitVal(ns_param_dict, param_name)
                if init_val is None:
                    init_val = factory_val
                    params_dict[param_name]['init_val'] = init_val

                self.set_param(param_name, init_val)
                cur_val = self.get_param(param_name)
                self.msg_if.pub_warn("Initialized param factory,init,value " + str([param_name,factory_val,init_val,cur_val]), log_name_list = self.log_name_list)

        self.params_dict = params_dict
        return True
            
    def reset_params(self):
        self.msg_if.pub_warn("Resetting params", log_name_list = self.log_name_list)
        success = self.initialize_params()
        for param_name in self.params_dict.keys():

            cur_val = self.get_param(param_name)

            init_val = None
            if 'init_val' in self.params_dict[param_name].keys():
                init_val = self.params_dict[param_name]['init_val']
            if init_val is None:
                init_val = self.params_dict[param_name]['factory_val']
            self.msg_if.pub_warn("Resetting param from:to " + str([param_name,cur_val,init_val]), log_name_list = self.log_name_list)
            self.set_param(param_name, init_val)

    def factory_reset_params(self):
        self.msg_if.pub_warn("Factory resetting params", log_name_list = self.log_name_list)
        for param_name in self.params_dict.keys():

            cur_val = self.get_param(param_name)
            
            factory_val = self.params_dict[param_name]['factory_val']
            self.msg_if.pub_warn("Factory Resetting param from:to " + str([param_name,cur_val,factory_val]), log_name_list = self.log_name_list)
            self.set_param(param_name, factory_val)

    def save_params(self, file_path):
        if not nepi_sdk.is_shutdown():
            self.nepi_sdk.save_params_to_file(file_path,self.namespace)       

    def has_param(self, param_name):
        namespace = self.get_param_namespace(param_name)
        if namespace is not None:
            return nepi_sdk.has_param(namespace)
        return False

    def get_param(self, param_name):
        value = None
        if param_name in self.params_dict.keys():
            param_dict = self.params_dict[param_name]
            namespace = self.get_param_namespace(param_name)
            if namespace is not None:
                if namespace in self.params_ns_dict.keys():
                    value = self.params_ns_dict[namespace]

                fallback = None
                if 'init_val' in param_dict.keys():
                    fallback = param_dict['init_val']
                
                if fallback is None:  
                    fallback = param_dict['factory_val']

                if value is None:
                    value = fallback
        return value

    def set_param(self, param_name, value):
        if not nepi_sdk.is_shutdown():
            namespace = self.get_param_namespace(param_name)
            if namespace is not None:
                self.params_ns_dict[namespace] = value
                nepi_sdk.set_param(namespace,value)
                

    def reset_param(self, param_name):
        if param_name in self.params_dict.keys():
            init_val = self.params_dict[param_name]['init_val']
            self.set_param(param_name, init_val)

    def factory_reset_param(self, param_name):
        if param_name in self.params_dict.keys():
            factory_val = self.params_dict[param_name]['factory_val']
            self.set_param(param_name, factory_val)

    def get_params(self):
        return list(self.params_dict.keys())


    def get_param_namespace(self,param_name):
        namespace = None
        if param_name in self.params_dict.keys() and not nepi_sdk.is_shutdown():
            param_dict = self.params_dict[param_name]
            namespace = nepi_sdk.create_namespace(param_dict['namespace'],param_name)
        return namespace


    def add_param(self, param_name, namespace, value):
        if not nepi_sdk.is_shutdown():
            if param_name is not None and namespace is not None and value is not None:
                if param_name not in self.params_dict.keys():
                    self.params_dict[param_name] = {
                'namespace': namespace,
                'factory_val': value
            }

    def add_params(self,params_dict):
        self.params_dict.update(params_dict)
        self.initialize_params()




    ###############################
    # Class Private Methods
    ###############################





##################################################
### Node Services Class
def EXAMPLE_CALLBACK_FUNCTION(request):
    response = EmptySrvResponse()
    return response
'''
EXAMPLE_SRVS_DICT = {
    'service_name': {
        'namespace':  self.node_namespace,
        'topic': 'empty_query',
        'srv': EmptySrv,
        'req': EmptySrvRequest(),
        'resp': EmptySrvResponse(),
        'callback': EXAMPLE_CALLBACK_FUNCTION
    }
}
'''
class NodeServicesIF:

    msg_if = None
    ready = False
    srvs_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                services_dict = None,
                log_name = None,
                log_class_name = True,
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
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_debug("Starting Node Services IF Initialization Processes", log_name_list = self.log_name_list)
        ##############################   

        ##############################  
        # Initialize Services System
        self.srvs_dict = services_dict
        if self.srvs_dict is None:
            self.srvs_dict = dict()
        self._initializeServices()

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("Ready", log_name_list = self.log_name_list)
        return self.ready

       
    def get_services(self):
        return list(self.srvs_dict.keys())

    def create_request_msg(self,service_name):
        req = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'req' in srv_dict.keys():
                req = srv_dict['req']
        return req

    def create_response_msg(self,service_name):
        resp = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'resp' in srv_dict.keys():
                resp = srv_dict['resp']
        return resp

    def register_service(self,service_name, service_dict):
        self.srvs_dict[service_name] = service_dict
        self._initializeServices()

    def register_services(self,services_dict):
        for service_name in services_dict.keys():
            service_dict = services_dict[service_name]
            self.srvs_dict[service_name] = service_dict
        self._initializeServices()

    def unregister_service(self,service_name):
        self._unregisterService(service_name)

    def unregister_services(self):
        service_names = list(self.srvs_dict.keys())
        for service_name in service_names:
            self._unregisterService(service_name)

    def add_services(self,services_dict):
        self.srvs_dict.update(services_dict)
        self._initializeServices()
    ###############################
    # Class Private Methods
    ###############################

    def _initializeServices(self):
        for service_name in self.srvs_dict.keys():
            self.msg_if.pub_debug("Will try to create service for: " + service_name )
            srv_dict = self.srvs_dict[service_name]
            if 'service' not in srv_dict.keys() and srv_dict['callback'] is not None:
                srv_callback = None
                try:
                    srv_namespace = nepi_sdk.create_namespace(srv_dict['namespace'],srv_dict['topic'])
                    srv_msg = srv_dict['srv']
                    srv_callback = srv_dict['callback']
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get service info from dict: " + service_name + " " + str(e), throttle_s = 5.0, log_name_list = self.log_name_list) 
                if srv_callback is not None and not nepi_sdk.is_shutdown():
                    self.msg_if.pub_debug("Created service for: " + service_name + " with namespace: " + str(srv_namespace), log_name_list = self.log_name_list)
                    service = None
                    try:
                        service = nepi_sdk.create_service(srv_namespace, srv_msg, srv_callback, log_name_list = self.log_name_list)   
                        self.srvs_dict[service_name]['service'] = service
                        self.srvs_dict[service_name]['namespace'] = srv_namespace
                        self.msg_if.pub_debug("Created service for: " + service_name + " with namespace: " + str(srv_namespace), \
                                        throttle_s = 5.0, log_name_list = self.log_name_list)                 
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to get service connection: " + service_name + " " + str(e), log_name_list = self.log_name_list)  
                    

    def _unregisterService(self, service_name):
        purge = False
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            purge = True
            if 'service' in srv_dict.keys() and not nepi_sdk.is_shutdown():
                try:
                    self.srvs_dict[service_name]['service'].shutdown()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister service: " + service_name + " " + str(e), log_name_list = self.log_name_list) 
        if purge == True:
            del self.srvs_dict[service_name]
                    







##################################################
### Node Publishers Class
'''

EXAMPLE_PUBS_DICT = {
    'pub_name': {
        'namespace':  self.node_namespace,
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'latch': False
    }
}
'''

class NodePublishersIF:

    msg_if = None
    ready = False
    pubs_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                pubs_dict = None,
                log_name = None,
                log_class_name = True,
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
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_debug("Starting Node Pubs IF Initialization Processes", log_name_list = self.log_name_list)
        ##############################   


        ##############################  
        # Initialize Publishers System
        self.pubs_dict = pubs_dict
        if self.pubs_dict is None:
            self.pubs_dict = dict()
        self.msg_if.pub_debug("Initializing with pub dict: " + str(self.pubs_dict) )
        self._initializePubs()

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("Ready", log_name_list = self.log_name_list)
        return self.ready

        

    def get_pubs(self):
        return list(self.pubs_dict.keys())

    def has_subscribers_check(self,pub_name):
        has_subs = False
        if pub_name in self.pubs_dict.keys() and not nepi_sdk.is_shutdown():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                if pub_dict['pub'] is not None:
                    has_subs = pub_dict['pub'].get_num_connections() > 0
                    self.msg_if.pub_debug("Pub has subscribers: " + pub_dict['namespace'] + "/" + pub_dict['topic'] + " " + str(has_subs), \
                                        throttle_s = 5.0, log_name_list = self.log_name_list) 
        return has_subs

    def publish_pub(self,pub_name,pub_msg):
        success = False
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                if pub_dict['pub'] is not None and not nepi_sdk.is_shutdown():
                    try:
                        nepi_sdk.publish_pub(pub_dict['pub'], pub_msg, log_name_list = self.log_name_list)
                        success = True
                    except Exception as e:
                        namespace =  pub_dict['namespace']
                        self.msg_if.pub_warn("Failed to publish msg: " + pub_name + \
                            " " + str(namespace)  + " " + str(pub_msg) + str(e), throttle_s = 5.0, log_name_list = self.log_name_list)   
        return success

    def register_pub(self,pub_name, pub_dict):
        self.pubs_dict[pub_name] = pub_dict
        self._initializePubs(print_msg = True)

    def register_pubs(self,pubs_dict = None):
        if pubs_dict is not None:
            self.pubs_dict.update(pubs_dict)
        self._initializePubs(print_msg = True)



 
    def unregister_pub(self,pub_name):
        self._unregisterPub(pub_name)

    def unregister_pubs(self):
        pub_names = list(self.pubs_dict.keys())
        for pub_name in pub_names:
            self._unregisterPub(pub_name)

    def add_pubs(self,pubs_dict):
        self.msg_if.pub_debug("Adding pubs dict: " + str(pubs_dict) , log_name_list = self.log_name_list) 
        self.pubs_dict.update(pubs_dict)
        #self.msg_if.pub_debug("Updated pubs dict: " + str(pubs_dict) , log_name_list = self.log_name_list) 
        self._initializePubs(print_msg = True)

    ###############################
    # Class Private Methods
    ###############################
    def _initializePubs(self, print_msg = False):
        for pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            add_pub = False
            if 'pub' not in pub_dict.keys():
                add_pub = True
            elif pub_dict['pub'] is None:
                add_pub = True
            if add_pub == True:  
                if 'topic' in pub_dict.keys() and 'msg' in pub_dict.keys() and not nepi_sdk.is_shutdown():
                    pub_namespace = nepi_sdk.create_namespace(pub_dict['namespace'] ,pub_dict['topic'])
                    self.msg_if.pub_debug("Creating pub for: " + pub_name + " with namespace: " + pub_namespace , log_name_list = self.log_name_list) 
                    pub = None
                    if 'qsize' not in pub_dict.keys():
                        self.pubs_dict[pub_name]['qsize'] = 1
                        pub_dict['qsize'] = 1
                    if 'latch' not in pub_dict.keys():
                        self.pubs_dict[pub_name]['latch'] = False
                        pub_dict['latch'] = False
                    try:
                        pub = nepi_sdk.create_publisher(pub_namespace, pub_dict['msg'], queue_size = pub_dict['qsize'], \
                                latch = pub_dict['latch'], log_name_list = self.log_name_list)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to create publisher: " + pub_name + " " + str(e), log_name_list = self.log_name_list) 
                    self.pubs_dict[pub_name]['pub_namespace'] = pub_namespace
                    self.pubs_dict[pub_name]['pub'] = pub
                    if print_msg == True:
                        self.msg_if.pub_warn("Added Pub: " + pub_name, log_name_list = self.log_name_list)
            else:
                #self.msg_if.pub_warn("Pubublisher already exists for: " + pub_name, log_name_list = self.log_name_list) 
                pass


    def _unregisterPub(self, pub_name):
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            purge = True
            if 'pub' in pub_dict.keys() and not nepi_sdk.is_shutdown():
                if pub_dict['pub'] is not None:
                    try:
                        self.pubs_dict[pub_name]['pub'].unregister()
                        self.msg_if.pub_warn("Unregister pub: " + pub_name, log_name_list = self.log_name_list)                    
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to get unregister pub: " + pub_name + " " + str(e), log_name_list = self.log_name_list) 
                    self.pubs_dict[pub_name]['pub'] = None



##################################################
### Node Subscribers Class
'''
def EXAMPLE_SUB_CALLBACK(msg):
    return msg

EXAMPLE_SUBS_DICT = {
    'sub_name': {
        'namespace':  self.node_namespace,
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'callback': EXAMPLE_SUB_CALLBACK, 
        'callback_args': ()
    }
}
'''
class NodeSubscribersIF:

    msg_if = None
    ready = False
    subs_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                subs_dict = None,
                log_name = None,
                log_class_name = True,
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
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_debug("Starting Node Subs IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################   


        self.subs_dict = subs_dict
        if self.subs_dict is None:
            self.subs_dict = dict()
        self._initializeSubs()


        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("Ready", log_name_list = self.log_name_list)
        return self.ready

        
    def get_subs(self):
        return list(self.subs_dict.keys())


    def register_sub(self,sub_name, sub_dict):
        self.subs_dict[sub_name] = sub_dict
        self._initializeSubs()

    def register_subs(self,subs_dict):
        for sub_name in subs_dict.keys():
            sub_dict = subs_dict[sub_name]
            self.subs_dict[sub_name] = sub_dict
        self._initializeSubs()

    def unregister_sub(self,sub_name):
        self._unregisterSub(sub_name)

    def unregister_subs(self):
        sub_names = list(self.subs_dict.keys())
        for sub_name in sub_names:
            self._unregisterSub(sub_name)


    def add_subs(self,subs_dict):
        self.subs_dict.update(subs_dict)
        self._initializeSubs()
    ###############################
    # Class Private Methods
    ###############################
    def _initializeSubs(self):
        for sub_name in self.subs_dict.keys():
            sub_dict = self.subs_dict[sub_name]
            self.msg_if.pub_debug("Will try to create sub for: " + sub_name )
            if 'sub' not in sub_dict.keys() and sub_dict['callback'] is not None and not nepi_sdk.is_shutdown():
                sub_namespace = nepi_sdk.create_namespace(sub_dict['namespace'],sub_dict['topic'])
                self.msg_if.pub_debug("Creating sub for: " + sub_name + " with namespace: " + sub_namespace, log_name_list = self.log_name_list) 
                if 'callback_args' not in sub_dict.keys():
                    sub_dict['callback_args'] = ()
                if sub_dict['callback_args'] is None:
                    sub_dict['callback_args'] = ()
                try:
                    if len(sub_dict['callback_args']) == 0:
                        sub = nepi_sdk.create_subscriber(sub_namespace, sub_dict['msg'], sub_dict['callback'], queue_size = sub_dict['qsize'], \
                            log_name_list = self.log_name_list)
                    else:
                        sub = nepi_sdk.create_subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'], \
                             callback_args=sub_dict['callback_args'], log_name_list = self.log_name_list)
                    self.subs_dict[sub_name]['sub'] = sub
                    self.subs_dict[sub_name]['sub_namespace'] = sub_namespace
                    success = True
                    self.msg_if.pub_debug("Created sub for: " + sub_name + " with namespace: " + sub_namespace, log_name_list = self.log_name_list) 
                except Exception as e:
                    self.msg_if.pub_warn("Failed to create subscriber: " + sub_name + " " + str(e), log_name_list = self.log_name_list)   
                    self.subs_dict[sub_name]['sub'] = None
            

    def _unregisterSub(self, sub_name):
        purge = False
        if sub_name in self.subs_dict.keys():
            sub_dict = self.subs_dict[sub_name]
            purge = True
            if 'sub' in sub_dict.keys() and not nepi_sdk.is_shutdown():
                try:
                    self.subs_dict[sub_name]['sub'].unregister()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister sub: " + sub_name + " " + str(e), throttle_s = 5.0)  
        if purge == True:
            del self.subs_dict[sub_name]


##################################################
### Node Class

# Configs Dict ####################
'''
EXAMPLE_CONFIGS_DICT = {
        'init_callback': None,
        'reset_callback': None,
        'factory_reset_callback': None,
        'init_configs': True,
        'namespace':  self.node_namespace
}


# Params Dict ####################
EXAMPLE_PARAMS_DICT = {
    'param1_name': {
        'namespace':  self.node_namespace,
        'factory_val': 100,
        'current_val: 20  # Optional
    },
    'param2_name': {
        'namespace':  self.node_namespace,
        'factory_val': "Something"
    }
}


# Services Dict ####################
EXAMPLE_SRVS_DICT = {
    'service_name': {
        'namespace':  self.node_namespace,
        'topic': 'empty_query',
        'srv': EmptySrv,
        'req': EmptySrvRequest(),
        'resp': EmptySrvResponse(),
        'callback': EXAMPLE_CALLBACK_FUNCTION
    }
}


# Publishers Dict ####################
EXAMPLE_PUBS_DICT = {
    'pub_name': {
        'namespace':  self.node_namespace,
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'latch': False
    }
}


# Subscribers Dict ####################
EXAMPLE_SUBS_DICT = {
    'sub_name': {
        'namespace':  self.node_namespace,
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'callback': EXAMPLE_SUB_CALLBACK, 
        'callback_args': ()
    }
}


# Create Node Class ####################

EXAMPLE_NODE_IF = NodeClassIF(
                configs_dict = EXAMPLE_CONFIGS_DICT,
                params_dict = EXAMPLE_PARAMS_DICT,
                services_dict = EXAMPLE_SRVS_DICT,
                pubs_dict = EXAMPLE_PUBS_DICT,
                subs_dict = EXAMPLE_SUBS_DICT,
                log_class_name = True
)
'''

class NodeClassIF:

    ready = False
    node_if = None
    msg_if = None
    log_name = None

    configs_dict = None
    configs_if = None
    params_if = None
    services_if = None
    pubs_if = None
    subs_if = None


    #######################
    ### IF Initialization
    def __init__(self, 
                node_name = None,
                configs_dict = None,
                params_dict = None,
                services_dict = None,
                pubs_dict = None,
                subs_dict = None,
                wait_cfg_mgr = True,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        if node_name is not None:
            nepi_sdk.init_node(name = node_name) # Can be overwitten by luanch command
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_info("Starting Node IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################  
        # Create Sub Classes
           
        ##############################  
        # Create Config Class After Params
        if configs_dict is None:
            self.configs_dict = configs_dict
        else:
            # Need to inject our own config callback functions that call the params_if functions first
            self.configs_dict = configs_dict
            # Copy and override rather than rebuild from scratch, so alt_namespace,
            # clear_params, manage_configs and anything else the caller set survive the
            # injection. The rebuilt literal this replaced carried only namespace and
            # silently dropped alt_namespace, which reset_params and delete_config_all
            # both read. Copying also keeps NodeConfigsIF's own 'namespace' default
            # assignment off the caller's dict.
            configs_dict = dict(configs_dict)
            configs_dict['init_callback'] = self._initConfigCb
            configs_dict['reset_callback'] = self._resetConfigCb
            configs_dict['factory_reset_callback'] = self._factoryResetConfigCb

        if self.configs_dict is not None:
            # Pass the INJECTED dict, not self.configs_dict. Passing the caller's dict
            # here made the three wrappers above dead code: /init_config never re-ran
            # initialize_params, and /reset_config and /factory_reset_config never reset
            # a param. self.configs_dict deliberately stays the caller's dict, because
            # each wrapper does its params work and then delegates to the caller's
            # callback through it.
            # params_if does not exist yet at this point, so the construction-time
            # reset_config that init_configs triggers still no-ops on the params step
            # exactly as it did before. Only the runtime topic callbacks change.
            self.configs_if = NodeConfigsIF(configs_dict = configs_dict,
                                            wait_cfg_mgr = wait_cfg_mgr, 
                                            msg_if = self.msg_if, 
                                            log_name_list = self.log_name_list)
        nepi_sdk.sleep(1)


        self.params_if = NodeParamsIF(params_dict = params_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)
        self.services_if = NodeServicesIF(services_dict = services_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)

        self.pubs_if = NodePublishersIF(pubs_dict = pubs_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)
        self.subs_if = NodeSubscribersIF(subs_dict = subs_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)

        nepi_sdk.sleep(0.1)
      
        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("Node IF Initialization Complete", log_name_list = self.log_name_list)
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("Ready", log_name_list = self.log_name_list)
        return self.ready

    # Config Methods ####################
    def save_config(self):
        if self.configs_if is not None:
            self.configs_if.save_config()

    def save_config_all(self):
        if self.configs_if is not None:
            self.configs_if.save_config_all()

    def reset_config(self):
        if self.configs_if is not None:
            self.configs_if.reset_config()

    def factory_reset_config(self):
        if self.configs_if is not None:
            self.configs_if.factory_reset_config()


    # Param Methods ####################
    def add_param(self,param_name, namespace, value):
        if self.params_if is not None:
            params = self.params_if.add_param(param_name, namespace, value)

    def add_params(self,params_dict):
        if self.params_if is not None:
            self.params_if.add_params(params_dict)

    def get_params(self):
        params = None
        if self.params_if is not None:
            params = self.params_if.get_params()
        return params
    
    def load_params(self, file_path):
        if self.params_if is not None:
            self.params_if.load_params(file_path)

    def initialize_params(self):
        if self.params_if is not None:
            self.params_if.initialize_params()


    def reset_params(self):
        if self.params_if is not None:
            self.params_if.reset_params()

    def factory_reset_params(self):
        if self.params_if is not None:
            self.params_if.factory_reset_params()


    def save_params(self):
        if self.params_if is not None:
            self.params_if.save_params()



    def has_param(self, param_name):
        exists = False
        if self.params_if is not None:
            exists = self.params_if.has_param(param_name)
        return exists

    def get_param(self, param_name):
        value = None
        if self.params_if is not None:
            value = self.params_if.get_param(param_name)

        return value

    def set_param(self, param_name, value):
        success = False
        if self.params_if is not None:
            self.params_if.set_param(param_name,value)
        return success

    def reset_param(self, param_name):
        success = False
        if self.params_if is not None:
            self.params_if.reset_param(param_name)
        return success

    def factory_reset_param(self, param_name):
        success = False
        if self.params_if is not None:
            self.params_if.reset_param(param_name)

        return success

    # Service Methods ####################
    def get_services(self):
        srvs = None
        if self.services_if is not None:
            srvs = self.services_if.get_services()
        return srvs

    def register_service(self,service_name, service_dict):
        if self.services_if is not None:
            self.services_if.register_service(service_name, service_dict)

    def register_services(self, services_dict):
        if self.services_if is not None:
            self.services_if.register_services( services_dict)


    def unregister_service(self,service_name):
        if self.services_if is not None:
            self.services_if.unregister_service(service_name)

    def unregister_services(self):
        service_names = list(self.srvs_dict.keys())  
        for service_name in service_names:  
            self.services_if.unregister_services()


    # Publisher Methods ####################
    def get_pubs(self):
        pubs = []
        if self.pubs_if is not None:
            pubs = self.pubs_if.get_pubs()
        return pubs

    def pub_has_subscribers(self,pub_name):
        has_subs = False
        if self.pubs_if is not None:
            has_subs = self.pubs_if.has_subscribers_check(pub_name)
        return has_subs

    def register_pub(self,pub_name, pub_dict):
        if self.pubs_if is not None:
            self.pubs_if.register_pub(pub_name, pub_dict)

    def register_pubs(self,pubs_dict = None):
        if self.pubs_if is not None:
            self.pubs_if.register_pubs(pubs_dict)


    def unregister_pub(self,pub_name):
        if self.pubs_if is not None:
            self.pubs_if.unregister_pub(pub_name)

    def unregister_pubs(self):
        if self.pubs_if is not None:
            self.pubs_if.unregister_pubs()

    def publish_pub(self,pub_name, pub_msg):
        success = False
        if self.pubs_if is not None and not nepi_sdk.is_shutdown():
            # Was "succes" (typo) -- the real result was computed and
            # discarded every time, so this always reported False to the
            # caller regardless of whether the underlying publish actually
            # succeeded. Found live while diagnosing why the RBX Settings
            # panel never appeared -- turned out to be a stale deployed RUI
            # file, not this, but the return value here was still always
            # wrong and worth fixing while it was in view.
            success = self.pubs_if.publish_pub(pub_name, pub_msg)
        return success
            
    # Subscriber Methods ####################
    def get_subs(self):
        subs = []
        if self.subs_if is not None:
            subs = self.subs_if.get_subs()
        return subs


    def register_sub(self,sub_name, sub_dict):
        if self.subs_if is not None:
            self.subs_if.register_sub(sub_name, sub_dict)

    def register_subs(self, subs_dict):
        if self.subs_if is not None:
            self.subs_if.register_subs(subs_dict)

    def unregister_sub(self,sub_name):
        if self.subs_if is not None:
            self.subs_if.unregister_sub(sub_name)

    def unregister_subs(self):
        if self.subs_if is not None:
            self.subs_if.unregister_subs()

    # Class Methods ####################
    def unregister_class(self):
        if self.services_if is not None:
            self.services_if.unregister_services()
        if self.pubs_if is not None:
            self.pubs_if.unregister_pubs()
        if self.subs_if is not None:
            self.subs_if.unregister_subs()

    ###############################
    # Class Private Methods
    ###############################

    def _initConfigCb(self, do_updates = False):
        self.initialize_params()
        if self.configs_dict is not None:
            if 'init_callback' in self.configs_dict.keys():
                if self.configs_dict['init_callback'] is not None:
                    self.configs_dict['init_callback'](do_updates = do_updates)
            

    def _resetConfigCb(self):
        self.msg_if.pub_warn("Node Got Reset Params Request", log_name_list = self.log_name_list)
        self.reset_params()
        if self.configs_dict is not None:
            if 'reset_callback' in self.configs_dict.keys():
                if self.configs_dict['reset_callback'] is not None:
                    self.configs_dict['reset_callback']()

    def _factoryResetConfigCb(self):
        self.msg_if.pub_warn("Node Got Factory Reset Params Request", log_name_list = self.log_name_list)
        self.factory_reset_params()
        if self.configs_dict is not None:
            if 'factory_reset_callback' in self.configs_dict.keys():
                if self.configs_dict['factory_reset_callback'] is not None:
                    self.configs_dict['factory_reset_callback']()