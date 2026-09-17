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
import numpy as np
import copy
import threading

os.environ['EGL_PLATFORM'] = 'surfaceless'   # Ubuntu 20.04+
import open3d as o3d

import cv2


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_states
from nepi_sdk import nepi_triggers
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_controls


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64


from nepi_interfaces.msg import Control, ControlsStatus, SettingsStatus, UpdateControl, MgrSystemStatus

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus, FilenameConfig
from nepi_interfaces.srv import SaveDataCapabilitiesQuery, SaveDataCapabilitiesQueryRequest, SaveDataCapabilitiesQueryResponse


from nepi_interfaces.msg import Transform, TransformStatus

from nepi_interfaces.msg import SystemState, SystemStates, SystemStatesStatus
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_interfaces.msg import SystemTrigger

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import  NodeClassIF






#########################################
# Controls IF Class
#########################################


class ControlsIF:
    
    msg_if = None
    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'controls_'
    namespace = ''

    controls_name = 'controls'
    controls_display_name = ''
    controls_description = ''
    controls_dict = dict()
    controls_hidden = False
    controls_status_msg = ControlsStatus()

    controls_node_pubs_dict = None
    controls_node_subs_dict = None
    controls_ready = False

    active_nodes = []
    active_topics = []
    active_topic_types =  []
    active_services =  []  

    status_has_published = False

    controls_updated_callback = None, # if not None: Calls function with with control_name when msg is recieved, after changine controls_dict and publishing status
    controls_updater_max_rate = 1,
    controls_updater_callback = None, # if not None: Calls function at the begining of each loop

    #######################
    ### IF Initialization
    def __init__(self, 
                controls_name = 'controls',
                controls_display_name = 'Controls',
                controls_description = 'Controls',
                controls_init_dict = dict(),
                controls_updated_callback = None, # if not None: Calls function with with control_name when msg is recieved, after changine controls_dict and publishing status
                controls_updater_max_rate = -1,# set to -1 to disable updater thread
                controls_updater_callback = None, # if not None: Calls function at the begining of each loop
                has_status_pub = True,
                show_controls = True,
                has_show_control = False,
                hidden = False,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None,
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
            log_name = nepi_utils.get_clean_name(log_name)
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Controlses", log_name_list = self.log_name_list)

        # Create Namespace
        self.controls_name = nepi_utils.get_clean_name(controls_name)
        if controls_name is None or controls_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(controls_name)) 
            return
        self.msg_if.pub_info("Using Name: " + self.controls_name)
        # Built from the sanitized self.controls_name, not the raw argument -- the
        # namespace must match the name reported in ControlsStatus.
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,self.controls_name)
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        ##############################    
        # Initialize Class Variables

        self.controls_display_name = str(controls_display_name)
        self.controls_description = str(controls_description)
        self.controls_dict = nepi_controls.create_controls_dict(controls_init_dict)
        self.controls_status_msg = nepi_controls.create_status_msg(self.controls_name, self.controls_display_name, self.controls_description,
                                                                    show_controls, has_show_control)
        self.controls_hidden = hidden

        self.controls_updated_callback = controls_updated_callback
        self.controls_updater_max_rate = controls_updater_max_rate
        self.controls_updater_callback = controls_updater_callback
    

        # if source_callback_dict is not None:
        #     for key in source_callback_dict.keys():
        #         self.source_callback_dict[key] = source_callback_dict[key]
      


        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        # Persist the selected topic under the connect namespace so the
        # selection survives node restarts (via the config manager). Passing a
        # params_dict is what enables config management on NodeClassIF.
        PARAMS_DICT = {
            self.node_if_prefix + 'controls_dict': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict
            },
            # controls_hidden lives outside controls_dict but is driven by the live
            # set_controls_hidden topic and published in ControlsStatus, so it needs
            # its own key or the hide-all toggle is lost on every restart.
            self.node_if_prefix + 'controls_hidden': {
                'namespace': self.namespace,
                'factory_val': self.controls_hidden
            }
        }


        # Publishers Config Dict ####################
        if has_status_pub == False:
            self.controls_node_pubs_dict = dict()
        else:
            self.controls_node_pubs_dict = {
                self.node_if_prefix + 'status_pub': {
                    'namespace': self.namespace,
                    'topic': 'status',
                    'msg': ControlsStatus,
                    'qsize': 1,
                    'latch': True
                }
            }


        # Subscribers Config Dict ####################
        self.controls_node_subs_dict = {
            #####################
            # Control Subs
            ####################
            self.node_if_prefix + 'update_control': {
                'msg': UpdateControl,
                'namespace': self.namespace,
                'topic': 'update_control',
                'qsize': 5,
                'callback': self._updateControlCb
            },
            #####################
            # Display Subs
            #####################
            #  self.node_if_prefix + 'set_control_hidden': {
            #     'msg': UpdateBool,
            #     'namespace': self.namespace,
            #     'topic': 'set_control_hidden',
            #     'qsize': 5,
            #     'callback': self._setHiddenValueCb
            # },
            #  self.node_if_prefix + 'set_controls_hidden': {
            #     'msg': UpdateBool,
            #     'namespace': self.namespace,
            #     'topic': 'set_controls_hidden',
            #     'qsize': 5,
            #     'callback': self._setControlsHiddenCb
            # },
            #  self.node_if_prefix + 'set_control_order': {
            #     'msg': UpdateInt,
            #     'namespace': self.namespace,
            #     'topic': 'set_control_order',
            #     'qsize': 5,
            #     'callback': self._setOrderValueCb
            # },
            #  self.node_if_prefix + 'set_control_up': {
            #     'msg': UpdateTrigger,
            #     'namespace': self.namespace,
            #     'topic': 'set_control_up',
            #     'qsize': 5,
            #     'callback': self._setOrderTopCb
            # },
            #  self.node_if_prefix + 'set_control_down': {
            #     'msg': UpdateTrigger,
            #     'namespace': self.namespace,
            #     'topic': 'set_control_down',
            #     'qsize': 5,
            #     'callback': self._setOrderDownCb
            # },
            #  self.node_if_prefix + 'set_control_top': {
            #     'msg': UpdateTrigger,
            #     'namespace': self.namespace,
            #     'topic': 'set_control_top',
            #     'qsize': 5,
            #     'callback': self._setOrderTopCb
            # },
            #  self.node_if_prefix + 'set_control_bottom': {
            #     'msg': UpdateTrigger,
            #     'namespace': self.namespace,
            #     'topic': 'set_control_bottom',
            #     'qsize': 5,
            #     'callback': self._setOrderBottomCb
            # },

            #####################
            # Misc Subs
            #####################
             self.node_if_prefix + 'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self._systemStatusCb
            },
        }

    
        
        if node_if is None:
            self.config_topic = self.namespace
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.controls_node_pubs_dict,
                            subs_dict = self.controls_node_subs_dict,
                            log_name_list = [],
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()
        else:
            self.node_if_shared = True
            try:
                self.node_if = node_if
                self.node_if.register_pubs(self.controls_node_pubs_dict)
                self.node_if.register_subs(self.controls_node_subs_dict)
                # Register the persisted controls dict on the shared node_if too,
                # under the same prefixed key init() reads and set_control_value()
                # writes. This registered a bare 'selected_sources' instead -- a name
                # ControlsIF never reads or writes, and a generic key that would
                # collide with the ProcessIF and detector params of the same name --
                # so on a shared node_if every control value write resolved to no
                # namespace and the controls box came back at factory defaults.
                # PARAMS_DICT keys already carry the controls_name prefix, so they
                # are domain-unique on a shared registry.
                self.node_if.add_params(PARAMS_DICT)
                nepi_sdk.sleep(1)
            except Exception as e:
                self.msg_if.pub_info("Failed to register pubs and subs: " + str(e))
                return


        ##############################
        # Start updater controls
        if self.controls_updater_max_rate != -1:
            nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
        if has_status_pub == True:
            nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

        ##############################
        # Complete Initialization
        self.controls_ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_controls_ready_state(self):
        """Return the ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.controls_ready

    def wait_for_controls_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.controls_ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.controls_ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.controls_ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.controls_ready  

    def get_namespace(self):
        """Return the fully-resolved ROS namespace this controls set publishes under.

        This is create_namespace(node_namespace, controls_name) -- the same
        namespace the status publisher and every set_*_control_value subscriber
        are registered on, so it is what a remote subscriber should use.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.namespace
    
    def unregister(self):
        """Shut down this controls interface and release its ROS resources.

        Returns:
            bool: True if teardown completed, False if it raised.
        """
        success = False
        # Called self.unsubscribe_topic() twice, a method this class does not
        # define -- so unregister() always raised before it released anything.
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.sleep(1)
            else:
                if self.node_if is not None:
                    if self.controls_node_subs_dict is not None:
                        for sub_name in self.controls_node_subs_dict.keys():
                            self.node_if.unregister_sub(sub_name)
                self.controls_node_subs_dict = None

                if self.node_if is not None:
                    if self.controls_node_pubs_dict is not None:
                        for pub_name in self.controls_node_pubs_dict.keys():
                            self.node_if.unregister_pub(pub_name)
                self.controls_node_pubs_dict = None
                
        time.sleep(1)
        try:
            self.node_if = None
            self.controls_ready = False
            success = True
        except Exception as e:
            self.msg_if.pub_warn("Failed to unregister:  " + str(e))
        return success


    ##################
    # Controls Functions
    def get_controls_dict(self):
        controls_dict = copy.deepcopy(self.controls_dict)
        return controls_dict

    def get_control_value(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        value = nepi_controls.get_control_value(controls_dict, control_name)
        return value

    def set_control_value(self, control_name, update_value, index = None):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.set_control_value(controls_dict, control_name, update_value, index = index)
        self.controls_dict = controls_dict
        self.publish_status()
        if self.controls_updated_callback is not None:
            self.controls_updated_callback(control_name)
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'controls_dict'
            self.node_if.set_param(param_name, self.controls_dict)


    def reset_control_value(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.reset_control_value(controls_dict, control_name)
        self.controls_dict = controls_dict

    def reset_control_values(self):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.reset_control_values(controls_dict)
        self.controls_dict = controls_dict


    def get_control_options(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        options = nepi_controls.get_control_options(controls_dict, control_name)
        return options

    def set_control_options(self, control_name, options):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.set_control_options(controls_dict, control_name, options)
        self.controls_dict = controls_dict
        self.publish_status()
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'controls_dict'
            self.node_if.set_param(param_name, self.controls_dict)

    def get_control_bounds(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        bounds = nepi_controls.get_control_bounds(controls_dict, control_name)
        return bounds

    def set_control_bounds(self, control_name, min_bound = None, max_bound = None):
        # Called set_control_options here, which writes the string option list
        # instead of the numeric bounds -- so setting bounds on an Int or Float
        # control changed nothing and clobbered its options.
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.set_control_bounds(controls_dict, control_name, min_bound = min_bound, max_bound = max_bound)
        self.controls_dict = controls_dict
        self.publish_status()
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'controls_dict'
            self.node_if.set_param(param_name, self.controls_dict)



    ##################
    # Display Functions

    def get_control_display_name(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        display_name = nepi_controls.get_control_display_name(controls_dict, control_name)
        return display_name

    def set_control_display_name(self, control_name, display_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.set_control_display_name(controls_dict, control_name, display_name)
        self.controls_dict = controls_dict


    def get_control_description(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        description = nepi_controls.get_control_description(controls_dict, control_name)
        return description

    def set_control_description(self, control_name, description):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.set_control_description(controls_dict, control_name, description)
        self.controls_dict = controls_dict

    def get_control_hidden(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        hidden = nepi_controls.get_control_hidden(controls_dict, control_name)
        return hidden

    def set_control_hidden(self, control_name, hidden):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.set_control_hidden(controls_dict, control_name, hidden)
        self.controls_dict = controls_dict
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'controls_dict'
            self.node_if.set_param(param_name, self.controls_dict)

    def get_controls_hidden(self):
        return self.controls_hidden

    def set_controls_hidden(self, hidden):
        self.controls_hidden = bool(hidden)
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'controls_hidden'
            self.node_if.set_param(param_name, self.controls_hidden)

    def get_control_display_order(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        order = nepi_controls.get_control_display_order(controls_dict, control_name)
        return order

    def set_control_display_order(self, control_name, update_order = 0):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.set_control_display_order(controls_dict, control_name, update_order)
        self.controls_dict = controls_dict
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'controls_dict'
            self.node_if.set_param(param_name, self.controls_dict)

    def move_control_display_top(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.move_control_display_top(controls_dict, control_name)
        self.controls_dict = controls_dict
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'controls_dict'
            self.node_if.set_param(param_name, self.controls_dict)

    def move_control_display_bottom(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.move_control_display_bottom(controls_dict, control_name)
        self.controls_dict = controls_dict
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'controls_dict'
            self.node_if.set_param(param_name, self.controls_dict)

    def move_control_display_up(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.move_control_display_up(controls_dict, control_name)
        self.controls_dict = controls_dict
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'controls_dict'
            self.node_if.set_param(param_name, self.controls_dict)

    def move_control_display_down(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.move_control_display_down(controls_dict, control_name)
        self.controls_dict = controls_dict
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'controls_dict'
            self.node_if.set_param(param_name, self.controls_dict)


    ##################
    # Misc Functions

    def publish_status(self, status_msg = None):
        ###########
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_status_msg = nepi_controls.update_status_msg(self.controls_status_msg, controls_dict, self.controls_hidden)
        self.controls_status_msg.config_topic = self.config_topic
        if self.node_if is not None:
            if self.status_has_published == False:
                self.msg_if.pub_warn("Publishing Status: " + str(self.controls_status_msg))
                self.status_has_published = True
            self.node_if.publish_pub(self.node_if_prefix + 'status_pub', self.controls_status_msg) 
        return status_msg

    def init(self, do_updates = False):
        """Initialize or re-initialize controls from the parameter server and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            # Prefixed key, matching how the param is registered and how
            # set_control_value()/set_control_options()/set_control_bounds()
            # write it back. get_param() returns None for a name it does not
            # know, so the unprefixed name wiped the controls dict on every
            # config init, reset and factory reset, leaving ControlsStatus with
            # empty control lists and the RUI with an empty controls box.
            param_name = self.node_if_prefix + 'controls_dict'
            controls_dict = self.node_if.get_param(param_name)
            if controls_dict is not None:
                self.controls_dict = controls_dict
            param_name = self.node_if_prefix + 'controls_hidden'
            controls_hidden = self.node_if.get_param(param_name)
            if controls_hidden is not None:
                self.controls_hidden = controls_hidden

        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset parameters to their last-saved (user) values and reinitialize.

        Calls node_if.reset_params() to reload the user configuration tier, then
        reinitializes from the param server.
        """
        self.controls_dict = nepi_controls.reset_control_values(self.controls_dict)
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset parameters to factory defaults and reinitialize.

        Calls node_if.factory_reset_params() to restore factory values, then
        reinitializes from the param server.
        """
        self.controls_dict = nepi_controls.reset_control_values(self.controls_dict)
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Factory resetting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    # ROS callback for the system status msg. Populates the active topic/type
    # lists that discovery searches. NOTE: this MUST NOT share a name with the
    # discovery timer below -- a duplicate name silently shadows this method, so
    # active_topics never gets populated and discovery finds nothing.
    def _systemStatusCb(self,msg):
            self.active_nodes = msg.active_nodes
            self.active_topics = msg.active_topics
            self.active_topic_types = msg.active_topic_types
            self.active_services = msg.active_services


    # Discovery/connection timer. Finds available topics of the connect status
    # msg type among the active topics, auto-selects, and subscribes.
    def _updaterCb(self,timer):
        needs_publish = False
        start_time = nepi_utils.get_time()
        ##############
        if self.controls_updater_callback is not None:
            needs_publish = self.controls_updater_callback()
        ##################
        # Get settings from param server
        if needs_publish == True:
          self.publish_status()

        ##################
        # Setup Next Update
        delay_time = float(1) / self.controls_updater_max_rate
        update_time = nepi_utils.get_time() - start_time
        next_time = delay_time - update_time
        if next_time < 0.01:
            next_time = 0.01
        nepi_sdk.start_timer_process(next_time, self._updaterCb, oneshot = True)


    def _updateControlCb(self,msg):
            #self.msg_if.pub_warn("Got Control Update msg: " + str(msg))
            control_name = msg.name
            # The value setters share this single callback. Most Update* msgs carry
            # a 'value' field; UpdateRangeWindow (FloatSliders) carries start/stop_range
            # and UpdateTrigger (Trigger) carries no value at all.
            controls_dict = copy.deepcopy(self.controls_dict)
            self.controls_dict = nepi_controls.apply_update_control_msg(self.controls_dict, msg)
            self.publish_status()
            if self.controls_dict != controls_dict:
                if self.controls_updated_callback is not None:
                    self.controls_updated_callback(control_name)
                if self.node_if is not None:
                    param_name = self.node_if_prefix + 'controls_dict'
                    self.node_if.set_param(param_name, self.controls_dict)
    

    def _setHiddenValueCb(self,msg):
            self.set_control_hidden(msg.name, msg.value)

    def _setControlsHiddenCb(self,msg):
            self.set_controls_hidden(msg.value)

    def _setOrderValueCb(self,msg):
            self.set_control_display_order(msg.name, msg.value)

    def _setOrderTopCb(self,msg):
            self.move_control_display_top(msg.name)

    def _setOrderBottomCb(self,msg):
            self.move_control_display_bottom(msg.name)

    def _setOrderDownCb(self,msg):
            self.move_control_display_down(msg.name)

    def _publishStatusCb(self,timer):
            self.publish_status()



           


################################################
## ReadWriteIF


class ReadWriteIF:

    ready = False
    node_name = ''
    # Save data variables
    filename_dict = {
        'prefix': "",
        'suffix': "",
        'add_timestamp': True, 
        'use_utc_tz': True,
        'add_ms': True,
        'add_us': False,
        'add_tz': True,
        'add_node_name': True
        }

    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'read_write_'


    #######################
    ### IF Initialization
    def __init__(self,
                filename_dict = None,
                node_name = None,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
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
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)
        



        #############################
        # Initialize Class Variables

        if node_name is not None:
            self.node_name = node_name.replace(' ','-')
        self.msg_if.pub_info("Using node_name: " + self.node_name, log_name_list = self.log_name_list)

        if filename_dict is not None:
            for key in self.filename_dict.keys():
                if key in filename_dict.keys():
                    self.filename_dict[key] = filename_dict[key]

        self.data_dict = {
            'dict': {
                'data_type': dict,
                'file_types': ['yaml'],
                'read_function': self.read_dict_file,
                'write_function': self.write_dict_file
            },
            'array': {
                'data_type': np.ndarray,
                'file_types': ['npy','csv','txt'],
                'read_function': self.read_array_file,
                'write_function': self.write_array_file 
            },
            'image': {
                'data_type': np.ndarray,
                'file_types': ['png','PNG','jpg','jpeg','JPG'],
                'read_function': self.read_image_file,
                'write_function': self.write_image_file 
            },
            'pointcloud': {
                'data_type': o3d.geometry.PointCloud,
                'file_types': ['pcd'],
                'read_function': self.read_pointcloud_file,
                'write_function': self.write_pointcloud_file 
            },
        }

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return the current ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready


    def get_namespace(self):
        """Return the ROS namespace used by this interface.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                # if self.SRVS_DICT is not None:
                #         for service_name in self.SRVS_DICT.keys():
                #             self.node_if.unregister_service(service_name)
                # self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                # if self.node_if is not None:
                #     if self.PUBS_DICT is not None:
                #         for pub_name in self.PUBS_DICT.keys():
                #             self.node_if.unregister_pub(pub_name)


    def get_supported_data_types(self):
        """Return the list of data type keys supported for file I/O.

        Returns:
            list: List of supported data type strings (e.g. 'dict', 'array', 'image',
                'pointcloud').
        """
        return list(self.data_dict.keys())


    def get_filename_prefix(self):
        """Return the current filename prefix string.

        Returns:
            str: The prefix prepended to generated filenames.
        """
        return self.filename_dict['prefix']

    def set_filename_prefix(self, prefix = ''):
        """Set the filename prefix string used when generating file names.

        Args:
            prefix (str, optional): Prefix string to prepend to filenames.
                Defaults to ''.
        """
        self.filename_dict['prefix'] = prefix

    def get_use_utc_tz(self):
        """Return whether UTC timezone is used for file timestamps.

        Returns:
            bool: True if UTC is used, False if local timezone is used.
        """
        return self.filename_dict['use_utc_tz']

    def set_use_utc_tz(self, use_utc_tz):
        """Set whether timestamps in filenames use UTC or local timezone.

        Args:
            use_utc_tz (bool): True to use UTC, False to use local timezone.
        """
        self.filename_dict['use_utc_tz'] = use_utc_tz

    def get_add_timestamp(self):
        """Return whether a timestamp is appended to generated filenames.

        Returns:
            bool: True if a timestamp is included in filenames.
        """
        return self.filename_dict['add_timestamp']

    def set_add_timestamp(self, add_timestamp):
        """Set whether a timestamp is included in generated filenames.

        Args:
            add_timestamp (bool): True to include a timestamp in filenames.
        """
        self.filename_dict['add_timestamp'] = add_timestamp

    def get_add_ms(self):
        """Return whether milliseconds are included in the filename timestamp.

        Returns:
            bool: True if milliseconds are appended to the timestamp portion.
        """
        return self.filename_dict['add_ms']

    def set_add_ms(self, add_ms):
        """Set whether milliseconds are included in the filename timestamp.

        Args:
            add_ms (bool): True to append milliseconds to the timestamp.
        """
        self.filename_dict['add_ms'] = add_ms

    def get_add_us(self):
        """Return whether microseconds are included in the filename timestamp.

        Returns:
            bool: True if microseconds are appended to the timestamp portion.
        """
        return self.filename_dict['add_us']

    def set_add_us(self, add_us):
        """Set whether microseconds are included in the filename timestamp.

        Args:
            add_us (bool): True to append microseconds to the timestamp.
        """
        self.filename_dict['add_us'] = add_us

    def get_add_tz(self):
        """Return whether the timezone abbreviation is included in filenames.

        Returns:
            bool: True if the timezone string is appended to the timestamp.
        """
        return self.filename_dict['add_tz']

    def set_add_tz(self, add_tz):
        """Set whether the timezone abbreviation is included in filenames.

        Args:
            add_tz (bool): True to append the timezone string to the timestamp.
        """
        self.filename_dict['add_tz'] = add_tz


    def get_filename_dict(self):
        """Return the full filename configuration dictionary.

        Returns:
            dict: Dictionary containing all filename formatting options (prefix,
                suffix, add_timestamp, use_utc_tz, add_ms, add_us, add_tz,
                add_node_name).
        """
        return self.filename_dict

    def set_filename_dict(self,filename_dict):
        """Merge a partial filename configuration dict into the current settings.

        Any keys missing from the provided dict are filled in from the current
        filename_dict, ensuring all required keys remain present.

        Args:
            filename_dict (dict): Dictionary with one or more filename config keys
                to update.

        Returns:
            dict: The updated filename configuration dictionary.
        """
        for key in self.filename_dict.keys():
            if key not in filename_dict.keys():
                filename_dict[key] = self.filename_dict[key]

        self.filename_dict = filename_dict
        return self.filename_dict


    def get_folder_files(self, path, ext_str = ""):
        """Return a list of files in a folder, optionally filtered by extension.

        Args:
            path (str): Filesystem path of the directory to list.
            ext_str (str, optional): File extension filter (e.g. 'yaml'). Defaults
                to '' (all files).

        Returns:
            list: List of file paths matching the extension filter.
        """
        file_list = nepi_utils.get_file_list(path,ext_str=ext_str)
        return file_list

    def get_time_from_filename(self,filename):
        """Extract and return the timestamp embedded in a NEPI-style filename.

        Args:
            filename (str): Filename string containing an embedded datetime token.

        Returns:
            float: Unix timestamp (seconds) parsed from the filename, or None if
                parsing fails.
        """
        file_time = None
        dt_str = self._getDtStr(filename)
        file_time = nepi_utils.get_time_from_datetime_str(dt_str)
        return file_time


    def get_data_type(self,data):
        """Infer the NEPI data type category of a data object.

        Inspects the Python type and, for numpy arrays, the dtype and number of
        dimensions to distinguish images from generic arrays.

        Args:
            data: The data object to classify.

        Returns:
            str: One of 'dict', 'pointcloud', 'image', 'array', or the string
                representation of the object's type if unrecognized.
        """
        dtype = type(data)

        if dtype == dict:
            return 'dict'
        elif dtype == o3d.geometry.PointCloud:
            return 'pointcloud'
        elif dtype == np.ndarray:
            # Check for valid image dtypes (uint8 or float32 are common)
            image_dtypes = [np.uint8]
            if data.dtype not in image_dtypes:
                return 'array'
            elif data.ndim == 2 or data.ndim == 3 :
                if data.dtype == np.uint8:
                    return 'image'
                else:
                    return 'array'
            else:
                return 'array'
        else:
            return str(dtype)
            





    def write_data_file(self, filepath, data, data_name, timestamp = None, timezone = None, filename = None, key_name = None):
        """Write a data object to disk using the appropriate format for its type.

        The data type is inferred automatically via get_data_type(). Supported types
        are 'dict' (YAML), 'array' (npy/csv), 'image' (png/jpg), and 'pointcloud'
        (pcd). Unsupported types are logged as warnings.

        Args:
            filepath (str): Directory path where the file will be written.
            data: Data object to save. Must be one of the supported types.
            data_name (str): Name token embedded in the generated filename.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the filename timestamp.
                Defaults to None (UTC).
        """
        data_type = self.get_data_type(data)
        found_type = False
        if data_type in self.data_dict.keys():
                save_function = self.data_dict[data_type]['write_function']
                #self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone), log_name_list = self.log_name_list, throttle_s = 5.0)
                filename = save_function(filepath, data, data_name, timestamp = timestamp, timezone = timezone, filename = filename, key_name = key_name)
                found_type = True
        if found_type == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type) + ' for data name: ' + str(data_name) + ' with data: ' + str(data), log_name_list = self.log_name_list, throttle_s = 5.0)
        return filename


    def read_dict_file(self, filepath, filename):
        """Read a YAML file and return its contents as a dictionary.

        Args:
            filepath (str): Directory path containing the file.
            filename (str): Name of the YAML file to read (must have a supported
                extension such as 'yaml').

        Returns:
            dict: Parsed dictionary from the YAML file, or None if the file type
                is unsupported or reading fails.
        """
        data_key = 'dict'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            try:
                data = nepi_utils.read_yaml_2_dict(file_path)
            except:
                self.msg_if.pub_warn("Failed to read file: " + file_path, log_name_list = self.log_name_list)
        return data

    def write_dict_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'yaml', filename = None, key_name = None):
        """Write a dictionary to a YAML file using a generated filename.

        Args:
            filepath (str): Directory path where the file will be written.
            data (dict): Dictionary to serialize.
            data_name (str): Name token embedded in the generated filename.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the filename timestamp.
                Defaults to None (UTC).
            ext_str (str, optional): File extension. Defaults to 'yaml'.

        Returns:
            bool: True if the file was written successfully, False otherwise.
        """
        data_key = 'dict'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                if filename is None:
                    filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    try:
                        success = nepi_utils.add_dict_2_yaml(file_path, data, key_name = key_name)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5)
                else:
                    try:
                        data_to_write = {key_name: data} if key_name is not None else data
                        success = nepi_utils.write_dict_2_yaml(file_path, data_to_write)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5)
        return filename


    def read_array_file(self, filepath, filename):
        """Read a numeric array from a .npy, .csv, or .txt file.

        Args:
            filepath (str): Directory path containing the file.
            filename (str): Name of the array file to read (must have a supported
                extension such as 'npy', 'csv', or 'txt').

        Returns:
            numpy.ndarray: Array loaded from the file, or None if the file type is
                unsupported or reading fails.
        """
        data_key = 'array'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            try:
                if ext_str == 'npy':
                    data = np.genfromtxt(file_path, delimiter=',', dtype=float, filling_values=-999)
                else:
                    data = np.genfromtxt(file_path, delimiter=',', dtype=float, filling_values=-999)
            except:
                self.msg_if.pub_warn("Failed to read file: " + file_path, log_name_list = self.log_name_list)
                
        return data

    def write_array_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'npy', filename = None, key_name = None):
        """Write a numpy array to a file using a generated filename.

        Args:
            filepath (str): Directory path where the file will be written.
            data (numpy.ndarray): Array to save.
            data_name (str): Name token embedded in the generated filename.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the filename timestamp.
                Defaults to None (UTC).
            ext_str (str, optional): File extension ('npy', 'csv', or 'txt').
                Defaults to 'npy'.

        Returns:
            bool: True if the file was written successfully, False otherwise.
        """
        data_key = 'array'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(type(data)), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    try:
                        if ext_str == 'npy':
                            file_path = file_path.replace('.' + ext_str, '')
                            success = np.save(file_path, data)
                        else:
                            success = np.savetxt(file_path, data)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5)

        return success


    def read_image_file(self, filepath, filename):
        """Read an image from disk and return it as a numpy array.

        Args:
            filepath (str): Directory path containing the file.
            filename (str): Name of the image file to read (must have a supported
                extension such as 'png', 'jpg', or 'jpeg').

        Returns:
            numpy.ndarray: Image array, or None if the file type is unsupported or
                reading fails.
        """
        data_key = 'image'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            try:
                data = nepi_img.read_image_file(file_path)
            except:
                self.msg_if.pub_warn("Failed to read file: " + file_path, log_name_list = self.log_name_list)
                
        return data

    def write_image_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'png', filename = None, key_name = None):
        """Write a numpy image array to an image file using a generated filename.

        Args:
            filepath (str): Directory path where the file will be written.
            data (numpy.ndarray): Image array (uint8) to save.
            data_name (str): Name token embedded in the generated filename.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the filename timestamp.
                Defaults to None (UTC).
            ext_str (str, optional): Image format extension ('png', 'jpg', etc.).
                Defaults to 'png'.

        Returns:
            bool: True if the file was written successfully, False otherwise.
        """
        data_key = 'image'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(type(data)), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    try:
                        success = nepi_img.write_image_file(file_path, data)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5)
        return success


    def read_pointcloud_file(self, filepath, filename):
        """Read a point cloud from a .pcd file and return an Open3D PointCloud object.

        Args:
            filepath (str): Directory path containing the file.
            filename (str): Name of the point cloud file to read (must have a
                supported extension such as 'pcd').

        Returns:
            open3d.geometry.PointCloud: Loaded point cloud, or None if the file type
                is unsupported or reading fails.
        """
        data_key = 'pointcloud'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            try:
                data = nepi_pc.read_pointcloud_file(file_path)
            except:
                self.msg_if.pub_warn("Failed to read file: " + file_path, log_name_list = self.log_name_list)
                
        return data

    def write_pointcloud_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'pcd', filename = None, key_name = None):
        """Write an Open3D point cloud to a file using a generated filename.

        Args:
            filepath (str): Directory path where the file will be written.
            data (open3d.geometry.PointCloud): Point cloud object to save.
            data_name (str): Name token embedded in the generated filename.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the filename timestamp.
                Defaults to None (UTC).
            ext_str (str, optional): Point cloud format extension. Defaults to 'pcd'.

        Returns:
            bool: True if the file was written successfully, False otherwise.
        """
        data_key = 'pointcloud'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    try:
                        success = nepi_pc.write_pointcloud_file(file_path,data)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5)
                    
        return success


    def get_example_filename(self, data_name = 'data_product', timestamp = None, timezone = None, ext_str = 'ext'):
        """Generate and return an example filename using the current naming settings.

        Useful for previewing what output filenames will look like without writing
        any data.

        Args:
            data_name (str, optional): Name token to embed in the filename. Defaults
                to 'data_product'.
            timestamp (float, optional): Unix timestamp used in the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the timestamp. Defaults to
                None (UTC).
            ext_str (str, optional): File extension for the example filename. Defaults
                to 'ext'.

        Returns:
            str: The generated example filename string.
        """
        filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
        return filename
    ###############################
    # Class Private Methods
    ###############################

    def _createFileName(self, data_name_str, timestamp = None, timezone = None, ext_str = ""):
        if timestamp == None:
            timestamp = nepi_utils.get_time()
        prefix = self.filename_dict['prefix']
        if len(prefix) > 0:
            if prefix[-1] != '_':
                prefix = prefix + '_'
        suffix = self.filename_dict['suffix']
        if len(suffix) > 0:
            if suffix[0] != '_':
                suffix = '_' + suffix
        add_time = self.filename_dict['add_timestamp']
        data_time_str = ''
        if add_time == True:
            time_ns = nepi_sdk.sec_from_timestamp(timestamp)
            add_ms = self.filename_dict['add_ms']
            add_us = self.filename_dict['add_us']
            add_tz = self.filename_dict['add_tz']
            data_time_str = nepi_utils.get_datetime_str_from_timestamp(time_ns, add_ms = add_ms, add_us = add_us, add_tz = add_tz, timezone = timezone) + '_'
        node_name_str = ""
        if self.filename_dict['add_node_name'] == True:
            node_name_str = self.node_name
        if len(ext_str) > 0:
            ext_str  = '.' + ext_str
        if len(data_name_str) >  0:
            data_name_str = '-' + data_name_str
        filename = prefix + data_time_str + node_name_str + data_name_str + suffix + ext_str
        return filename

    
    def _getDtStr(self,filename):
        d_inds = nepi_utils.find_all_indexes(filename, 'D')
        dt_ind = None
        dt_str = filename
        for ind in d_inds:
            if len(filename) >= ind + 1:
                if filename[ind + 1].isdigit() == True:
                    dt_ind = ind
                    break
        dt_str = dt_str[dt_ind:]
        dt_str = dt_str.split('_')[0]
        return dt_str
                




################################################
## SaveDataIF

FALLBACK_DATA_FOLDER = '/mnt/nepi_storage/data'

SUPPORTED_DICT_FILE_TYPES = ['yaml']
SUPPORTED_IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']  
SUPPORTED_PC_FILE_TYPES = ['pcd']
SUPPORTED_VID_FILE_TYPES = ['avi','AVI']

SUPPORTED_DATA_TYPES = ['dict','cv2_image','o3d_pointcloud']

'''
EXAMPLE_RATE_DICT = {
    'data_product_1' : [save_rate_hz, last_time, max_rate],
    'data_product_2' : [save_rate_hz, last_time, max_rate]
}
'''

EXAMPLE_FILENAME_DICT = {
    'prefix': "", 
    'subfolder': "",
    'add_timestamp': True, 
    'use_utc_tz': True,
    'add_ms': True,
    'add_us': False,
    'add_tz': True,
    'add_node_name': False
    }

class SaveDataIF:

    DEFAULT_TIMEZONE = 'UTC'

    ready = None
    namespace = "~"
    all_save_namespace = None
    status_msg = SaveDataStatus
    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'save_data'
    read_write_if = None
 
    snapshot_dict = dict()

    sys_mgr_if = None
    read_write_if = None

    save_data_root_directory = FALLBACK_DATA_FOLDER

    filename_dict = {
        'prefix': "", 
        'subfolder': "", 
        'add_timestamp': True, 
        'use_utc_tz': True,
        'add_ms': True,
        'add_us': False,
        'add_tz': True,
        'add_node_name': True
        }

    save_rate_dict = dict()
    save_data = False
    use_utc_tz = False

    disabled = False
    save_all_enabled = False
    save_all_rate = 0.0

    space_available = False


    file_prefix = ""
    subfolder = ""

    was_saving = False

    pub_status = False

    ### IF Initialization
    def __init__(self, 
                namespace = None,
                save_data_name = 'save_data',
                data_products = [], 
                pub_status = True,
                factory_rate_dict = None, 
                factory_filename_dict = None, 
                ignore_global_rate_updates = False,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
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

        self.msg_if.pub_info("Starting SaveData IF Initialization Processes", log_name_list = self.log_name_list)
        ############################## 
        # Initialize Class Variables
        # Create Namespace
        save_data_name = nepi_utils.get_clean_name(save_data_name)
        if save_data_name is None or save_data_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(save_data_name)) 
            return
        self.msg_if.pub_info("Using States Name: " + save_data_name)
        # An explicit namespace lets a caller place this interface somewhere other than
        # its own node namespace. system_mgr passes the base namespace to own the global
        # '<base>/save_data' namespace, which is where the RUI's All selection subscribes
        # for status and where every other instance subscribes for global commands.
        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.create_namespace(namespace,save_data_name)
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'
        
        self.msg_if.pub_warn("Using save data namespace: " + self.namespace, log_name_list = self.log_name_list)
        
        self.pub_status = pub_status

        tzd = nepi_utils.get_timezone_description(self.DEFAULT_TIMEZONE)
        self.timezone = tzd

        all_save_namespace = nepi_sdk.create_namespace(self.base_namespace,'/save_data')
        if all_save_namespace != self.namespace:
            self.all_save_namespace = all_save_namespace

        ###############################
        # Connect Sys Mgr Services

        
        ##############################
        # Get for System Folders
        self.msg_if.pub_info("Waiting for user folders")
        user_folders = nepi_system.get_user_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got user folders: " + str(user_folders))
        if user_folders is not None and 'data' in user_folders.keys():
            self.save_data_root_directory = user_folders['data']
        self.msg_if.pub_info("Using SDK Share Folder: " + str(self.save_data_root_directory))

        # Ensure the data folder exists with proper ownership
        if not os.path.exists(self.save_data_root_directory):
            self.msg_if.pub_warn("Reported data folder does not exist... data saving is disabled", log_name_list = self.log_name_list)
            self.save_data_root_directory = None # Flag it as non-existent
            return # Don't enable any of the ROS interface stuff
        self.save_path = self.save_data_root_directory
        # And figure out user/group so that we know what ownership to create subfolders with
        stat_info = os.stat(self.save_data_root_directory)
        self.DATA_UID = stat_info.st_uid
        self.DATA_GID = stat_info.st_gid

 
 


        # Setup System IF Classes
        # Initialize with empty dict, then call update function

        self.read_write_if = ReadWriteIF(
                            filename_dict = dict(),
                            node_name = self.node_name
                            )
        nepi_sdk.sleep(1)
        self.msg_if.pub_debug("Got starting filename dict: " + str(self.filename_dict), log_name_list = self.log_name_list)
        if factory_filename_dict is not None:
            self.update_filename_dict(factory_filename_dict)
        self.msg_if.pub_debug("Got Updated filename dict: " + str(self.filename_dict), log_name_list = self.log_name_list)

        
        # Config initial data products dict
        self.msg_if.pub_debug("^^^^^^^^^^^^^^^^^^^^^^", log_name_list = self.log_name_list)
        self.msg_if.pub_debug("Starting Save_Data_IF with data products: " + str(data_products), log_name_list = self.log_name_list)
        self.msg_if.pub_debug("Starting Save_Data_IF with rate dict: " + str(factory_rate_dict), log_name_list = self.log_name_list)
        save_rate_dict = dict()
        save_rate = 0.0
        last_time = 0.0
        max_rate = 100
        for data_product in data_products:
            save_rate = 0.0
            last_time = 0.0
            max_rate = 100
            save_rate_entry = [save_rate, last_time, max_rate]
            if factory_rate_dict is not None:
                if data_product in factory_rate_dict.keys():
                    save_rate = factory_rate_dict[data_product][0]
            save_rate_entry[0] = save_rate
            save_rate_dict[data_product] = save_rate_entry
            self.snapshot_dict[data_product] = False
        self.save_rate_dict = save_rate_dict
        self.msg_if.pub_debug("Got defualt data rate dict: " + str(self.save_rate_dict), log_name_list = self.log_name_list)
            


        ##############################    
        # Node Setup
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }


        # Params Config Dict ####################
        self.PARAMS_DICT = {
            # 'disabled': {
            #     'namespace': self.namespace,
            #     'factory_val': self.disabled
            # },
            self.node_if_prefix + 'save_rate_dict': {
                'namespace': self.namespace,
                'factory_val': self.save_rate_dict
            },
            self.node_if_prefix + 'filename_dict': {
                'namespace': self.namespace,
                'factory_val': self.filename_dict
            }
        }


        # Services Config Dict ####################
        if self.namespace == self.namespace:
            self.SRVS_DICT = {
                self.node_if_prefix + 'save_data_capabilities_query': {
                    'namespace': self.namespace,
                    'topic': 'capabilities_query',
                    'srv': SaveDataCapabilitiesQuery,
                    'req': SaveDataCapabilitiesQueryRequest(),
                    'resp': SaveDataCapabilitiesQueryResponse(),
                    'callback': self._capabilitiesHandler
                }
            }
        else:
            self.SRVS_DICT = None


        # Publishers Config Dict ####################
        self.PUBS_DICT = dict()

        if self.pub_status == True:
            self.PUBS_DICT[self.node_if_prefix + 'save_data_status_pub'] = {
                'namespace': self.namespace,
                'msg': SaveDataStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            self.node_if_prefix + 'save_data_disable': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'disable',
                'qsize': 5,
                'callback': self._disableCb, 
                'callback_args': ()
            }, 
            self.node_if_prefix + 'save': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'save_data_enable',
                'qsize': 5,
                'callback': self._saveEnableCb, 
                'callback_args': ()
            },  
            self.node_if_prefix + 'prefix': {
                'namespace': self.namespace,
                'msg': String,
                'topic': 'save_data_prefix',
                'qsize': 5,
                'callback': self._setPrefixCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'subfolder': {
                'namespace': self.namespace,
                'msg': String,
                'topic': 'save_data_subfolder',
                'qsize': 5,
                'callback': self._setSubfolderCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'save_data_utc': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'save_data_utc',
                'qsize': 5,
                'callback': self._setLocalTzCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'filename': {
                'namespace': self.namespace,
                'msg': FilenameConfig,
                'topic': 'filename_config',
                'qsize': 5,
                'callback': self._setFilenameCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'rate': {
                'namespace': self.namespace,
                'msg': SaveDataRate,
                'topic': 'save_data_rate',
                'qsize': 5,
                'callback': self._saveRateCb, 
                'callback_args': ()
            },          
            self.node_if_prefix + 'snapshot': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'snapshot_trigger',
                'qsize': 5,
                'callback': self._snapshotCb,  
                'callback_args': ()
            },
            self.node_if_prefix + 'save_data_reset': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'reset_save_data',
                'qsize': 5,
                'callback': self._resetSaveDataCb,  
                'callback_args': ()
            },

        }

        if self.all_save_namespace is not None:
            ALL_SUBS_DICT =  {
                self.node_if_prefix + 'disable_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'saving_disable',
                    'qsize': 5,
                    'callback': self._disableCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'save_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'save_data_enable',
                    'qsize': 5,
                    'callback': self._saveEnableCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'prefix_all': {
                    'namespace': self.all_save_namespace,
                    'msg': String,
                    'topic': 'save_data_prefix',
                    'qsize': 5,
                    'callback': self._setPrefixCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'subfolder_all': {
                    'namespace': self.all_save_namespace,
                    'msg': String,
                    'topic': 'save_data_subfolder',
                    'qsize': 5,
                    'callback': self._setSubfolderCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'use_local_tz_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'save_data_utc',
                    'qsize': 5,
                    'callback': self._setLocalTzCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'filename_all': {
                    'namespace': self.all_save_namespace,
                    'msg': FilenameConfig,
                    'topic': 'filename_config',
                    'qsize': 5,
                    'callback': self._setFilenameCb,
                    'callback_args': ()
                },
                self.node_if_prefix + 'snapshot_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Empty,
                    'topic': 'snapshot_trigger',
                    'qsize': 5,
                    'callback': self._snapshotCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'save_all_config': {
                    'namespace': self.all_save_namespace,
                    'msg': Empty,
                    'topic': 'save_config',
                    'qsize': 5,
                    'callback': self._saveConfigCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'all_save_sub': {
                    'namespace': self.all_save_namespace,
                    'msg': SaveDataStatus,
                    'topic': 'status',
                    'qsize': 5,
                    'callback': self._saveAllStatusCb, 
                    'callback_args': ()
                }
            }
            if ignore_global_rate_updates == False:
                ALL_SUBS_DICT[self.node_if_prefix + 'rate_all'] = {
                        'namespace': self.all_save_namespace,
                        'msg': SaveDataRate,
                        'topic': 'save_data_rate',
                        'qsize': 5,
                        'callback': self._saveRateCb, 
                        'callback_args': ()
                    } 

            self.SUBS_DICT.update(ALL_SUBS_DICT)
        


        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            if self.PARAMS_DICT is not None:
               self.node_if.add_params(self.PARAMS_DICT) 
            self.node_if.register_services(self.SRVS_DICT)
            self.node_if.register_pubs(self.PUBS_DICT)
            self.node_if.register_subs(self.SUBS_DICT)
        else:
            self.config_topic = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = self.SRVS_DICT,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                                                )

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()
        
        self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################



    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return the current ready state of the SaveDataIF.

        Returns:
            bool: True if initialization completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the SaveDataIF is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready


    def get_namespace(self):
        """Return the ROS namespace used by this SaveDataIF.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace
    
    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SRVS_DICT is not None:
                        for service_name in self.SRVS_DICT.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)


    def get_data_products(self):
        """Return the list of registered data product names.

        Returns:
            list: List of data product name strings currently tracked by this interface.
        """
        return list(self.save_rate_dict.keys())

    def register_data_product(self, data_product,factory_rate = 0):
        """Register a new data product with an optional factory save rate.

        If the data product is not already registered it is added to the save rate
        dictionary, the snapshot dictionary, and the ROS parameter server.

        Args:
            data_product (str): Unique name for the data product.
            factory_rate (float, optional): Default save rate in Hz. Defaults to 0
                (disabled).
        """
        save_rate_dict = self.save_rate_dict
        if data_product not in save_rate_dict.keys():
            save_rate_dict[data_product] =  [factory_rate, 0.0, 100] # Default to 1Hz save rate, max rate = 100Hz
            self.save_rate_dict = save_rate_dict
            self.snapshot_dict[data_product] = False
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix + 'save_rate_dict',save_rate_dict)

    def unregister_data_product(self, data_product):
        """Remove a previously registered data product from the save rate tracking dict.

        Args:
            data_product (str): Name of the data product to remove.
        """
        purge = False
        if data_product in self.save_rate_dict.keys():
            purge = True
        if purge == True:
            del self.save_rate_dict[data_product]

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SRVS_DICT is not None:
                        for service_name in self.SRVS_DICT.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)


    def update_filename_dict(self,filename_dict):
        """Apply updates from a filename configuration dictionary.

        Handles prefix sanitization, subfolder creation with proper ownership, and
        updates the underlying ReadWriteIF and ROS parameter server. Only applies
        if the new dict differs from the current one.

        Args:
            filename_dict (dict): Dictionary containing one or more filename config
                keys to update (prefix, subfolder, add_timestamp, use_utc_tz,
                add_ms, add_us, add_tz, add_node_name).
        """

        if self.filename_dict != filename_dict and filename_dict is not None:
            if 'prefix' in filename_dict.keys():
               new_prefix = filename_dict['prefix']
               self.filename_dict['prefix'] = nepi_utils.get_clean_name(new_prefix)

            if 'subfolder' in filename_dict.keys():
                new_subfolder= nepi_utils.get_clean_name(filename_dict['subfolder'])
                 

                if new_subfolder != "" and self.save_data_root_directory != None:
                    full_path = os.path.join(self.save_data_root_directory, new_subfolder)
                elif self.save_data_root_directory != None:
                    full_path = self.save_data_root_directory
                else:
                    full_path = ""

                if not os.path.exists(full_path):
                    self.msg_if.pub_debug("Creating new data subdirectory " + full_path)
                    try:
                        os.makedirs(full_path)
                    except Exception as e:
                        self.msg_if.pub_warn("Could not create save folder " + new_subfolder + str(e) )
                if os.path.exists(full_path):
                    try:
                        os.chown(full_path, self.DATA_UID, self.DATA_GID)
                    except Exception as e:
                        self.msg_if.pub_warn("Could not chmod on save folder " + full_path + str(e) )
                    self.save_path = full_path
                    filename_dict['subfolder'] = new_subfolder

            # Apply Updates
            for key in self.filename_dict.keys():
                if key in filename_dict.keys():
                    self.filename_dict[key] = filename_dict[key]  

            self.publish_status()
            if self.read_write_if is not None and self.filename_dict is not None:
                self.read_write_if.set_filename_dict(filename_dict)
            if self.node_if is not None and self.filename_dict is not None:
                self.node_if.set_param(self.node_if_prefix + 'filename_dict',self.filename_dict)
                self.node_if.save_config()

    def set_save_rate(self,data_product,save_rate_hz=0):
        """Set the save rate for one or more data products.

        Supports special sentinel values for data_product to target all products
        (ALL_DATA_PRODUCTS), none (NONE_DATA_PRODUCTS), or only currently-active
        ones (ACTIVE_DATA_PRODUCTS). Per-product rates are clamped to the
        configured max rate for that product.

        Args:
            data_product (str): Name of the data product, or a sentinel constant
                from SaveDataRate (ALL_DATA_PRODUCTS, NONE_DATA_PRODUCTS,
                ACTIVE_DATA_PRODUCTS).
            save_rate_hz (float, optional): Target save rate in Hz. Defaults to 0
                (disabled).
        """
        save_all = SaveDataRate().ALL_DATA_PRODUCTS
        save_none = SaveDataRate().NONE_DATA_PRODUCTS
        save_active = SaveDataRate().ACTIVE_DATA_PRODUCTS
        save_rate_dict = self.save_rate_dict
        if (data_product == save_active):
            for d in save_rate_dict.keys():
                # Respect the max save rate
                if save_rate_dict[d][0] > 0:
                    save_rate_dict[d][0] = save_rate_hz if save_rate_hz <= save_rate_dict[d][2] else save_rate_dict[d][2]
        elif (data_product == save_all):
            for d in save_rate_dict.keys():
                save_rate_dict[d][0] = save_rate_hz
        elif (data_product == save_none):
            for d in save_rate_dict.keys():
                save_rate_dict[d][0] = 0.0
        elif (data_product in save_rate_dict.keys()):
            save_rate_dict[data_product][0] = save_rate_hz if save_rate_hz <= save_rate_dict[data_product][2] else save_rate_dict[data_product][2]
        else:
            self.msg_if.pub_warn("Requested unknown data product: " + data_product)    
        self.save_rate_dict = save_rate_dict     
        #self.msg_if.pub_warn("Updated save rate dict: " + str(self.save_rate_dict))   
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'save_rate_dict',save_rate_dict)
            self.node_if.save_config()
        
    def disable(self, enabled):
        """Enable or disable data saving for all registered data products.

        When enabling after a disabled period, all last-save timestamps are reset
        so that each product triggers a save on the next call to data_product_should_save.

        Args:
            enabled (bool): True to enable saving, False to disable.
        """
        #self.msg_if.pub_warn("Setting Saving Disabled to: " + str(enabled))  
        self.disabled = enabled
        self.publish_status()    

    def save_data_enable(self, enabled):
        """Enable or disable data save state for all registered data products.

        When enabling after a disabled period, all last-save timestamps are reset
        so that each product triggers a save on the next call to data_product_should_save.

        Args:
            enabled (bool): True to enable save, False to disable.
        """
        if enabled == True and self.was_saving == False:
            for d in self.save_rate_dict.keys():
                self.save_rate_dict[d][1] = 0.0
            self.was_saving = True
        else:
            self.was_saving = False
        self.msg_if.pub_warn("Setting Save Enable to: " + str(enabled))  
        self.save_data = enabled
        self.publish_status()    


    def get_saving_enabled(self):
        """Return whether data saving is currently enabled.

        Returns:
            bool: True if saving is enabled, False if disabled.
        """
        return self.save_data and self.disabled == False

    def data_product_save_enabled(self, data_product):
        """Return whether a specific data product is currently configured to save.

        Returns False if global saving is disabled or if the product's save rate
        is zero.

        Args:
            data_product (str): Name of the data product to check.

        Returns:
            bool: True if saving is enabled and the product has a non-zero rate,
                False otherwise.
        """
        # If saving is disabled for this node, then no data products are saving
        try:
            save_rate_dict = self.save_rate_dict
            if self.save_data == False:
                return False

            if data_product not in save_rate_dict:
                self.msg_if.pub_warn("Unknown data product " + data_product)
                return False

            save_rate = save_rate_dict[data_product][0]
            return (save_rate > 0.0) and self.disabled == False
        except:
            return False
        
    def data_product_save_rate(self, data_product):
        """Return the configured save rate for a specific data product.

        Returns False if global saving is disabled or the product is unknown.

        Args:
            data_product (str): Name of the data product.

        Returns:
            float: Save rate in Hz, or False if saving is disabled or the product
                is unrecognized. Returns 0.0 on unexpected errors.
        """
        # If saving is disabled for this node, then no data products are saving
        try:
            save_rate_dict = self.save_rate_dict
            if self.save_data == False:
                return False

            if data_product not in save_rate_dict:
                self.msg_if.pub_warn("Unknown data product " + data_product)
                return False

            save_rate = save_rate_dict[data_product][0]
            return save_rate
        except:
            return 0.0



    def data_product_should_save(self, data_product):
        """Determine whether it is time to save a data product based on its rate and elapsed time.

        Checks the configured save rate and the time since the last save. Also
        returns True if a snapshot has been triggered for the product. Returns
        False if global saving is disabled, the product is unknown, or the rate
        is zero.

        Args:
            data_product (str): Name of the data product to check.

        Returns:
            bool: True if the product should be saved now, False otherwise.
        """
        # If saving is disabled for this node, then it is not time to save this data product!
        save_rate_dict = self.save_rate_dict
        #self.msg_if.pub_debug("Checking should save for save rate dict: " + str(save_rate_dict), log_name_list = self.log_name_list, throttle_s = 5)
        
        if self.save_data == False or  self.disabled == True:
            return False

        if data_product not in save_rate_dict.keys():
            self.msg_if.pub_warn("Unknown data product " + data_product, log_name_list = self.log_name_list, throttle_s = 5)
            return False

        save_rate = save_rate_dict[data_product][0]
        if save_rate == 0.0:
            return False

        save_period = float(1) / float(save_rate)
        now = nepi_utils.get_time()
        elapsed = now - save_rate_dict[data_product][1]
        #self.msg_if.pub_debug("Checking should save: " + str([save_period,elapsed]), log_name_list = self.log_name_list, throttle_s = 5)
        snapshot = self.snapshot_dict[data_product]
        if (elapsed >= save_period or snapshot):
            #self.msg_if.pub_debug("Should save: " + data_product + " : " + str([save_period,elapsed]), log_name_list = self.log_name_list, throttle_s = 5)
            self.save_rate_dict = save_rate_dict
            return True 
        return False



    def data_product_snapshot_enabled(self, data_product):
        """Return whether a one-shot snapshot is pending for a data product.

        Args:
            data_product (str): Name of the data product to check.

        Returns:
            bool: True if a snapshot is pending, False if not or if the product
                is unrecognized.
        """
        try:
            enabled = self.snapshot_dict[data_product]  and self.disabled == False
            return enabled
        except:
            self.msg_if.pub_warn("Unknown snapshot data product " + data_product, log_name_list = self.log_name_list, throttle_s = 5)
            return False
        return False

    def data_product_snapshot_reset(self, data_product):
        """Clear the snapshot-pending flag for a data product after it has been saved.

        Args:
            data_product (str): Name of the data product to reset.

        Returns:
            bool: True if the flag was cleared, False if the product is unrecognized.
        """
        try:
            self.snapshot_dict[data_product] = False
            return True
        except:
            self.msg_if.pub_warn("Unknown snapshot data product " + data_product, log_name_list = self.log_name_list, throttle_s = 5)
            return False
        return False


    def data_products_should_save_dict(self):
        """Return a dictionary mapping each data product to whether it should save now.

        Combines the snapshot flag and the rate-based save logic for every
        registered data product.

        Returns:
            dict: Mapping of data product name to bool (True = should save now).
        """
        dps_dict=copy.deepcopy(self.snapshot_dict)
        for dp in dps_dict.keys():
            ss =  dps_dict[dp]
            sr = self.save_rate_dict[dp][0] > 0
            dps_dict[dp] = (ss or (self.save_data and sr)) and self.disabled == False
        return dps_dict


    def get_timestamp_string(self):
        """Return the current time as a formatted datetime string with milliseconds.

        Returns:
            str: Current datetime string in NEPI format including milliseconds.
        """
        return nepi_utils.get_datetime_str_now(add_ms = True, add_us = False)

    def get_filename_path_and_prefix(self):
        """Return the full save path joined with the current filename prefix.

        Returns:
            str: Combined path string, or empty string if the save path is None.
        """
        if self.save_path is None:
            return ""
        return os.path.join(self.save_path, self.read_write_if.get_filename_prefix())




    #***************************
    # NEPI data saving utility functions
    def save(self,data_product,data,timestamp = None, save_check=True, filename = None, key_name = None):
        """Save a data object for a named data product if conditions are met.

        Checks whether disk space is available, whether the product should save
        (rate-based or snapshot), and then delegates to ReadWriteIF.write_data_file.
        Resets the snapshot flag and updates the last-save timestamp after writing.

        Args:
            data_product (str): Name of the data product being saved.
            data: Data object to write. Type is inferred automatically.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            save_check (bool, optional): When True, the rate/snapshot logic gates
                the write. When False, the data is written unconditionally (if
                space is available). Defaults to True.
        """
        if self.space_available == False:
            self.data_product_snapshot_reset(data_product)
            return ''
        else:
            if self.disabled == False and self.filename_dict is not None:
                should_save = self.data_product_should_save(data_product)
                snapshot_enabled = self.data_product_snapshot_enabled(data_product)
                # Save data if enabled
                self.msg_if.pub_debug("******", log_name_list = self.log_name_list, throttle_s = 5)
                save_check = [should_save, snapshot_enabled, save_check]
                self.msg_if.pub_debug("Checking save checks: " + data_product + " " + str(save_check) , log_name_list = self.log_name_list, throttle_s = 5)
                if (should_save or snapshot_enabled or save_check == False):
                    if self.filename_dict['use_utc_tz'] == False:
                        timezone = self.timezone
                    else:
                        timezone = 'UTC'
                    self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone) , log_name_list = self.log_name_list, throttle_s = 5)
                    if should_save == False:
                        filename = None
                    filename = self.read_write_if.write_data_file(self.save_path, data, data_product, timezone = timezone, timestamp = timestamp, filename = filename, key_name = key_name)
                    self.data_product_snapshot_reset(data_product)
                    self.save_rate_dict[data_product][1] = nepi_utils.get_time()
                self.msg_if.pub_debug("Finished Checking save data: " + data_product , log_name_list = self.log_name_list, throttle_s = 5)
                self.msg_if.pub_debug("******", log_name_list = self.log_name_list, throttle_s = 5)
                return filename


    def create_filename_msg(self):
        """Build and return a FilenameConfig ROS message from the current filename dict.

        Returns:
            nepi_interfaces.msg.FilenameConfig: Populated FilenameConfig message
                reflecting the current naming settings.
        """
        fn_msg = FilenameConfig()
        try:
            fn_dict = self.filename_dict
            fn_msg.save_prefix = fn_dict['prefix']
            fn_msg.save_subfolder = fn_dict['subfolder']
            fn_msg.add_timestamp = fn_dict['add_timestamp']
            fn_msg.use_utc_tz = fn_dict['use_utc_tz']
            fn_msg.add_ms = fn_dict['add_ms']
            fn_msg.add_us = fn_dict['add_us']
            fn_msg.add_tz = fn_dict['add_tz']
        except:
            pass
        return fn_msg


    def publish_status(self):
        """Build and publish a SaveDataStatus message on the status topic.

        Compiles current save rates, filename config, data directory, timezone,
        and an example filename into the status message and latches it to the
        'status' publisher. No-ops if pub_status is False or node_if is None.
        """
        if self.node_if is not None and self.pub_status == True:
            save_rates_msg = []
            save_rate_dict = self.save_rate_dict

            #self.msg_if.pub_debug("Status pub save_rate_dict " + str(save_rate_dict), log_name_list = self.log_name_list, throttle_s = 5)
            for name in save_rate_dict.keys():
                save_rate_msg = SaveDataRate()
                save_rate_msg.data_product = name
                save_rate_msg.save_rate_hz = 0
                if self.disabled == False:
                    save_rate_msg.save_rate_hz = save_rate_dict[name][0]
                save_rates_msg.append(save_rate_msg)
                #self.msg_if.pub_debug("data_rates_msg " + str(save_rates_msg), log_name_list = self.log_name_list, throttle_s = 5)
            status_msg = SaveDataStatus()
            status_msg.node_name = self.node_name
            status_msg.save_data_topic = self.namespace
            status_msg.config_topic = self.config_topic
            status_msg.filename_config = self.create_filename_msg()
            status_msg.data_dir = self.save_path
            if self.filename_dict is not None:
                filename_dict = copy.deepcopy(self.filename_dict)
                try:
                    status_msg.filename_prefix = filename_dict['prefix']
                    status_msg.save_subfolder = filename_dict['subfolder']
                    status_msg.save_data_utc = filename_dict['use_utc_tz']
                except Exception as e:
                    self.msg_if.pub_warn("Failed to Publish filename dict: " + str(e), log_name_list = self.log_name_list, throttle_s = 5)
                #self.msg_if.pub_warn("Publishing filename dict: " + str(filename_dict), log_name_list = self.log_name_list, throttle_s = 5)

            status_msg.timezone = self.timezone
            status_msg.data_products = list(save_rate_dict.keys())
            status_msg.save_data_rates = save_rates_msg
            status_msg.save_data_enabled = self.save_data
            status_msg.disabled = self.disabled

            if self.all_save_namespace is None:
                status_msg.save_all_enabled = self.save_data
                # Guarded: publish_status runs on a 1 Hz timer, so a KeyError here would
                # silence the global status topic entirely.
                status_msg.save_all_rate = save_rate_dict.get('All',[0.0])[0]
            else:
                status_msg.save_all_enabled = self.save_all_enabled
                status_msg.save_all_rate = self.save_all_rate   


            if self.save_data_root_directory is not None:
                status_msg.data_dir = self.save_data_root_directory

            timezone = 'UTC'
            if self.filename_dict is not None:
                if self.filename_dict['use_utc_tz'] == False:
                    timezone = self.timezone

            #self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone) , log_name_list = self.log_name_list, throttle_s = 5)
            exp_filename = self.read_write_if.get_example_filename(timezone = timezone)
            status_msg.example_filename = exp_filename
            if self.node_if is not None:
                self.node_if.publish_pub(self.node_if_prefix +  'save_data_status_pub', status_msg)

    def init(self, do_updates = False):
        """Load save rate and filename parameters from the ROS param server and publish status.

        Reads the persisted save_rate_dict from the param server, merges it into the
        current dict (preserving last-save timers at zero), writes the merged dict back,
        and then publishes the current status.

        Args:
            do_updates (bool, optional): Reserved for future use; has no effect currently.
                Defaults to False.
        """
        #self.msg_if.pub_warn("Param updated save rate dict: " + str(self.save_rate_dict))
        if self.node_if is not None:
            # Prefixed keys, matching how the params are registered in PARAMS_DICT
            # and how set_save_rate()/update_filename_dict()/register_data_product()
            # write them back. get_param() returns None for a name it does not know,
            # so the unprefixed name wiped filename_dict on every config init, reset
            # and factory reset -- and save() no-ops entirely when filename_dict is
            # None, so nothing was ever written to disk.
            save_rate_dict = self.node_if.get_param(self.node_if_prefix + 'save_rate_dict')
            if save_rate_dict is not None:
                for data_product in self.save_rate_dict.keys():
                    if data_product in save_rate_dict.keys():
                        self.save_rate_dict[data_product][0] = save_rate_dict[data_product][0]
                    self.save_rate_dict[data_product][1] = 0.0 # Reset timer
            self.node_if.set_param(self.node_if_prefix + 'save_rate_dict',self.save_rate_dict)
            filename_dict = self.node_if.get_param(self.node_if_prefix + 'filename_dict')
            if filename_dict is not None:
                self.filename_dict = filename_dict
            disabled = self.node_if.get_param('disabled')
            if disabled is not None:
                self.disabled = disabled
        self.publish_status()


    def reset(self):
        """Reset parameters to their last-saved (user) values and reinitialize.

        Calls node_if.reset_params() to reload the user configuration tier, then
        reinitializes from the param server.
        """
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset parameters to factory defaults and reinitialize.

        Calls node_if.factory_reset_params() to restore factory values, then
        reinitializes from the param server.
        """
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Factory resetting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        if self.node_if is not None:
            pass
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def updaterCb(self,timer):
        tzd = nepi_system.get_timezone()
        last_tz = copy.deepcopy(self.timezone)
        self.timezone = tzd
        self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)

        self.space_available = nepi_system.get_space_available()


    def _capabilitiesHandler(self, req):
        return_list = []
        save_rate_dict = self.save_rate_dict
        for d in save_rate_dict:
            return_list.append(SaveDataRate(data_product = d, save_rate_hz = save_rate_dict[d][0]))
        return SaveDataCapabilitiesQueryResponse(return_list)





    def _logNavPoseEnableCb(self, msg):
        self.msg_if.pub_info("Recieved Log NavPose Enable Update: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data



    def _saveAllStatusCb(self,msg):
        #self.msg_if.pub_warn("Recieved save All status msg: " + str(msg), log_name_list = self.log_name_list)
        data_product_list = msg.data_products
        if 'All' in data_product_list:
            self.save_all_enabled = msg.save_data_enabled
            index = data_product_list.index('All')
            self.save_all_rate = msg.save_data_rates[index].save_rate_hz

    def _disableCb(self, msg):
        #self.msg_if.pub_info("Recieved Disable Update: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data
        self.disable(enabled)

    def _saveEnableCb(self, msg):
        self.msg_if.pub_info("Recieved Enable Update: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data
        self.save_data_enable(enabled)
        

    def _saveRateCb(self, msg):
        self.msg_if.pub_info("Recieved Rate Update: " + str(msg), log_name_list = self.log_name_list)
        data_product = msg.data_product
        save_rate_hz = msg.save_rate_hz
        self.set_save_rate(data_product,save_rate_hz)
    
        
    def _setPrefixCb(self, msg):
        prefix = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        if filename_dict is not None:
            filename_dict['prefix'] = prefix
            self.update_filename_dict(filename_dict)


    def _setSubfolderCb(self, msg):
        subfolder = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        if filename_dict is not None:
            filename_dict['subfolder'] = subfolder
            self.update_filename_dict(filename_dict)


    def _setLocalTzCb(self, msg):
        use_utc = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        if filename_dict is not None:
            filename_dict['use_utc_tz'] = use_utc
            self.update_filename_dict(filename_dict)


    def _setFilenameCb(self, msg):
        filename_dict = nepi_utils.convert_msg2dict(msg)
        if self.filename_dict is not None:
            self.update_filename_dict(filename_dict)



    def _snapshotCb(self,msg):
        self.msg_if.pub_info("Recieved Snapshot Trigger", log_name_list = self.log_name_list)
        save_rate_dict = self.save_rate_dict
        if self.disabled == False:
            for data_product in save_rate_dict.keys():
                save_rate = save_rate_dict[data_product][0]
                enabled = (save_rate > 0.0)
                if enabled:
                    self.snapshot_dict[data_product] = True

    def _saveConfigCb(self,msg):
        if self.node_if is not None:
            self.node_if.save_config()

    def _resetSaveDataCb(self,msg):
        self.reset()

    def _publishStatusCb(self, timer):
        self.publish_status()

#######################################
# Transform3DIF

# Transform_List = [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, heading_deg]



class Transform3DIF:

    ZERO_TRANSFORM = [0,0,0,0,0,0,0]
    # Class Vars ####################

    msg_if = None
    ready = False
    namespace = '~'

    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'transform_'

    transform = copy.deepcopy(ZERO_TRANSFORM)
    source = ''
    end = ''
    has_transform = True
    supports_updates = True

    status_msg = TransformStatus()
    
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
                transform_name = 'transform',
                source_ref_description = '',
                end_ref_description = '',
                get_3d_transform_function = None,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
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
        self.msg_if.pub_info("Starting Settings IF Initialization Processes", log_name_list = self.log_name_list)
        

        #############################
        # Initialize Class Variables
        transform_name = nepi_utils.get_clean_name(transform_name)
        if transform_name is None or transform_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(transform_name)) 
            return
        self.msg_if.pub_info("Using States Name: " + transform_name)
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,transform_name)
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        self.source = source_ref_description
        self.end = end_ref_description
        self.get_3d_transform_function = get_3d_transform_function
        if self.get_3d_transform_function is not None:
            self.supports_updates = False

        self.status_msg.has_transform = self.has_transform
        self.status_msg.supports_updates = self.supports_updates
        ##############################  
        # Create NodeClassIF Class  
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }
        
        # Params Config Dict ####################
        self.PARAMS_DICT = {
            self.node_if_prefix + 'transform': {
                'namespace': self.namespace,
                'factory_val': self.transform
            },
            self.node_if_prefix + 'source': {
                'namespace': self.namespace,
                'factory_val': self.source
            },
            self.node_if_prefix + 'end': {
                'namespace': self.namespace,
                'factory_val': self.end
            }
        }

        # Services Config Dict ####################
        self.SRVS_DICT = None
        

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'transform_status_pub': {
                'namespace': self.namespace,
                'msg': TransformStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            },
            self.node_if_prefix + 'transform_pub': {
                'namespace': self.namespace,
                'msg': Transform,
                'topic': '',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################

        if self.supports_updates == False:
            self.SUBS_DICT = None
        else:
            self.SUBS_DICT = {
                self.node_if_prefix + 'clear_navpose_frame_transform': {
                    'namespace': self.namespace,
                    'topic': 'clear_3d_transform',
                    'msg': Empty,
                    'qsize': 5,
                    'callback': self._clearFrame3dTransformCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'set_navpose_frame_transform': {
                    'namespace': self.namespace,
                    'topic': 'set_3d_transform',
                    'msg': Transform,
                    'qsize': 5,
                    'callback': self._setFrame3dTransformCb,
                    'callback_args': ()
                },
                self.node_if_prefix + 'set_source_ref_description': {
                    'namespace': self.namespace,
                    'topic': 'set_source_ref',
                    'msg': String,
                    'qsize': 5,
                    'callback': self._setSourceRefCb,
                    'callback_args': ()
                }
        }

        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            if self.PARAMS_DICT is not None:
               self.node_if.add_params(self.PARAMS_DICT) 
            self.node_if.register_pubs(self.PUBS_DICT)
            self.node_if.register_subs(self.SUBS_DICT)
        else:
            self.config_topic = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = self.SRVS_DICT,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                                                )
   

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()    
    
        nepi_sdk.start_timer_process(1.0, self._publishTransformCb)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

  
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the Transform3DIF.

        Returns:
            bool: True if initialization completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the Transform3DIF is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace used by this Transform3DIF.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SRVS_DICT is not None:
                        for service_name in self.SRVS_DICT.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)


    def get_zero_3d_transform(self):
        """Return the zero/identity 3D transform list.

        Returns:
            list: Seven-element list of zeros [x, y, z, roll, pitch, yaw, heading].
        """
        return copy.deepcopy(self.ZERO_TRANSFORM)

    def get_3d_transform(self):
        """Return the current 3D transform as a seven-element list.

        If a get_3d_transform_function was provided at construction, it is called
        first. Falls back to the internally stored transform if the function returns
        None.

        Returns:
            list: [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, heading_deg].
        """
        transform = None
        if self.get_3d_transform_function is not None:
            transform = self.get_3d_transform_function()
        if transform is None:
            transform = self.transform
        return transform

    def get_3d_transform_msg(self):
        """Build and return the current 3D transform as a ROS Transform message.

        Returns:
            nepi_interfaces.msg.Transform: Transform message populated with the
                current transform values and source/end reference descriptions.
        """
        transform = self.get_3d_transform()
        transform_msg = nepi_nav.convert_transform_list2msg(transform,
                source_ref_description = self.source,
                end_ref_description = self.end)
        return transform_msg

    def get_3d_transform_dict(self):
        """Return the current 3D transform as a nepi_nav transform dict.

        Suitable for passing to nepi_nav.transform_navpose_dict() to apply the
        transform to a navpose dict.

        Returns:
            dict: BLANK_TRANSFORM_DICT-shaped dict populated from the current transform.
        """
        return nepi_nav.convert_transform_list2dict(self.get_3d_transform())

    def set_3d_transform(self,transform_list):
        """Set the 3D transform from a seven-element list and publish the update.

        No-ops if updates are not supported (i.e., a get_3d_transform_function was
        supplied at construction) or if the list does not have exactly seven elements.

        Args:
            transform_list (list): [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg,
                heading_deg].
        """
        if self.supports_updates == True:
            if len(transform_list) == 7:
                self.transform = transform_list
                self.publish_transform()
                if self.node_if is not None:
                    self.node_if.set_param(self.node_if_prefix + 'transform',transform_list)

    def clear_3d_transform(self):
        """Reset the 3D transform to the zero/identity transform and publish the update.

        No-ops if updates are not supported (i.e., a get_3d_transform_function was
        supplied at construction).
        """
        if self.supports_updates == True:
            self.transform = copy.deepcopy(self.ZERO_TRANSFORM)
            self.publish_transform()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix + 'transform',self.transform)

    def set_has_transform(self,has_transform):
        """Set whether this interface reports having a valid transform.

        Args:
            has_transform (bool): True if a valid transform is available.
        """
        self.has_trasform = has_transform

    def get_has_transform(self):
        """Return whether this interface reports having a valid transform.

        Returns:
            bool: True if a valid transform is available, False otherwise.
        """
        return self.has_transform


    def get_source_description(self):
        """Return the source reference frame description string.

        Returns:
            str: Human-readable description of the source reference frame.
        """
        return self.source

    def set_source_description(self,source):
        """Set the source reference frame description and republish the transform.

        Args:
            source (str): Human-readable description of the source reference frame.
        """
        self.source = source
        self.publish_transform()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'source',source)

    def get_end_description(self):
        """Return the end (target) reference frame description string.

        Returns:
            str: Human-readable description of the end reference frame.
        """
        return self.end

    def set_end_description(self,end):
        """Set the end (target) reference frame description and republish the transform.

        Args:
            end (str): Human-readable description of the end reference frame.
        """
        self.end = end
        self.publish_transform()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'end',end)


    def publish_transform(self):
        """Build and publish the current transform as a ROS Transform message.

        Converts the current seven-element transform list to a ROS message and
        publishes it on the 'transform_pub' topic. No-ops if node_if is None.
        """
        transform = self.get_3d_transform()
        transform_msg = nepi_nav.convert_transform_list2msg(transform,
                                                source_ref_description = self.source,
                                                end_ref_description = self.end)
        if self.node_if is not None:
            self.node_if.publish_pub(self.node_if_prefix +  'transform_pub',transform_msg)

    def publish_status(self):
        """Publish the current TransformStatus message on the 'transform_status_pub' topic.

        No-ops if node_if is None.
        """
        if self.node_if is not None:
            self.status_msg.config_topic = self.config_topic
            self.status_msg.has_transform = self.has_transform
            self.node_if.publish_pub('transform_status_pub',self.status_msg)


    def init(self, do_updates = True):
        """Load transform, source, and end parameters from the ROS param server.

        Reads the persisted values and publishes the updated status.

        Args:
            do_updates (bool, optional): Reserved for future use; has no additional
                effect currently. Defaults to True.
        """
        if self.node_if is not None:
            # Prefixed keys, matching how the params are registered in PARAMS_DICT and
            # how set_3d_transform()/set_source_description()/set_end_description()
            # write them back. get_param() returns None for a name it does not know,
            # so the unprefixed names wiped the transform on every config init, reset
            # and factory reset -- saved 3D transforms never restored.
            transform = self.node_if.get_param(self.node_if_prefix + 'transform')
            if transform is not None:
                self.transform = transform
            source = self.node_if.get_param(self.node_if_prefix + 'source')
            if source is not None:
                self.source = source
            end = self.node_if.get_param(self.node_if_prefix + 'end')
            if end is not None:
                self.end = end
            #self.msg_if.pub_debug("Setting init values to param server values: " + str(self.init_settings_dict), log_name_list = self.log_name_list)
            if do_updates:
                pass
            self.publish_status()


    def reset(self):
        """Reset transform parameters to their last-saved values and reinitialize.

        Calls node_if.reset_params() to reload the user configuration tier, then
        reinitializes from the param server.
        """
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset transform parameters to factory defaults and reinitialize.

        Calls node_if.factory_reset_params() to restore factory values, then
        reinitializes from the param server.
        """
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Factory resetting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)


    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)


    def _setFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Received Frame Transform update message: " + str(msg))
        transform_msg = msg
        # The stored transform is a 7-element signed list; fold the per-axis
        # invert flags into the sign so the stored/republished value is unambiguous.
        x = -transform_msg.x_m if transform_msg.x_invert else transform_msg.x_m
        y = -transform_msg.y_m if transform_msg.y_invert else transform_msg.y_m
        z = -transform_msg.z_m if transform_msg.z_invert else transform_msg.z_m
        roll = -transform_msg.roll_deg if transform_msg.roll_invert else transform_msg.roll_deg
        pitch = -transform_msg.pitch_deg if transform_msg.pitch_invert else transform_msg.pitch_deg
        yaw = -transform_msg.yaw_deg if transform_msg.yaw_invert else transform_msg.yaw_deg
        heading = -transform_msg.heading_deg if transform_msg.heading_invert else transform_msg.heading_deg
        transform = [x, y, z, roll, pitch, yaw, heading]
        self.set_3d_transform(transform)


    def _clearFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Recived Clear 3D Transform update message: ")
        self.clear_3d_transform()

    def _setSourceRefCb(self, msg):
        self.set_source_description(msg.data)

    def _publishTransformCb(self, timer):
        self.publish_transform()

    def _publishStatusCb(self, timer):
        self.publish_status()



#######################################
# SettingsIF


# The device-side settings contract, unchanged by the move onto nepi_controls.
# A driver (or system_mgr, for the system config) describes its settings as
# string-valued dicts and hands SettingsIF two functions to read and write them.
# SettingsIF converts that form to and from a nepi_controls controls dict, which
# is what it publishes -- so the capability report that used to require a
# SettingsCapabilitiesQuery round trip now rides the status message.


def SET_NONE_SETTINGS_FUNCTION(setting):
    return False, "No settings update function available"

def GET_NONE_SETTINGS_FUNCTION():
    return dict()


class SettingsIF:
    """Publishes a node's settings as a nepi_settings settings set.

    A device's settings are settings: named, typed, bounded values with a
    factory, default and set tier. This interface keeps the string-valued
    settings dict contract that drivers implement, holds the live state as a
    nepi_settings settings dict, and publishes it as a SettingsStatus message
    on '<namespace>/settings/status'. The capability information that the
    retired SettingsCapabilitiesQuery service used to return (type, options,
    bounds, default) is carried in that status message.

    ROS interface, all under '<namespace>/<settings_name>':
        status              (SettingsStatus, latched)  the settings and their capabilities
        update_setting_value      (UpdateSetting)            change one setting
        reset_settings      (Empty)                    restore last-saved values
    """

    # Class Vars ####################

    msg_if = None
    ready = False
    namespace = '~'

    node_if = None
    node_if_shared = False
    config_topic = ''
    # Registry keys are prefixed so they stay domain-unique on a shared
    # node_if. The ROS wire name comes from namespace+topic, not the key, so
    # these are wire-safe -- EXCEPT the param key, where the wire name IS
    # namespace+key. The param stays 'settings' (see SETTINGS_PARAM_KEY below), which is
    # what deployed config files hold.
    node_if_prefix = ''
    SETTINGS_PARAM_KEY = 'settings'

    settings_dict_values = None
    getCapSettingsFunction = None
    getSettingsFunction = None
    setSettingFunction = None
    callback_arg = None

    settings_dict = dict()
    settings_status_msg = None

    settings_name = 'settings'

    enable_list = []
    disable_list = []

    save_params = True

    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
                settings_name = 'settings',
                getSettingsFunction=None, 
                setSettingFunction=None, 
                callback_arg = None,
                enable_list = [],
                disable_list = [],
                save_params = True,
                use_nodename_prefix = True,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
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
        self.msg_if.pub_info("Starting Settings IF Initialization Processes", log_name_list = self.log_name_list)
        

        #############################

        self.callback_arg = callback_arg

        self.enable_list = enable_list
        self.disable_list = disable_list

        # Create Namespace
        settings_name = nepi_utils.get_clean_name(settings_name)
        if settings_name is None or settings_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(settings_name)) 
            return
        self.settings_name = settings_name
        self.msg_if.pub_info("Using Settings Name: " + settings_name)
        # An explicit namespace lets a caller place this interface somewhere other than
        # its own node namespace. system_mgr passes the base namespace so the system
        # config settings own the global '<base>/settings' namespace the RUI subscribes
        # to, and the device IFs pass their device namespace ('<node>/idx', '<node>/ptx',
        # ...) so each device type's settings sit under its own device namespace.
        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.create_namespace(namespace,settings_name)

        if use_nodename_prefix == True:
            self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_' 
        else:
            self.node_if_prefix = settings_name  + '_' 
        SETTINGS_PARAM_KEY = self.node_if_prefix

        self.save_params = save_params


        if getSettingsFunction is None:
            self.getSettingsFunction = GET_NONE_SETTINGS_FUNCTION
        else:
            self.getSettingsFunction = getSettingsFunction


        if setSettingFunction is None:
            self.setSettingFunction = SET_NONE_SETTINGS_FUNCTION
        else:
            self.setSettingFunction = setSettingFunction
        
        # Build the settings dict. The factory and default tiers come from the
        # device's factory settings (or a cap setting's own declared default);
        # the set tier is seeded from whatever the device currently reports.

        if self.callback_arg is None:
            self.settings_dict = self.getSettingsFunction()
        else:
            self.settings_dict = self.getSettingsFunction(self.callback_arg)

      
        self.settings_status_msg = nepi_controls.create_status_msg(
                                    self.settings_name, 'Settings', 'Device Settings',
                                    show_controls = True, has_show_control = False)

        ##############################  
        # Create NodeClassIF Class  

        # Configs Config Dict ####################
        # The persisted value stays the string-valued settings dict it has
        # always been, under the param key it has always used. The controls
        # dict is derived state, so no deployed config file needs migrating.
        if self.save_params == True:
            self.CONFIGS_DICT = {
                'init_callback': self._initCb,
                'reset_callback': self._resetCb,
                'factory_reset_callback': self._factoryResetCb,
                'init_configs': True,
                'namespace': self.namespace
            }
            self.PARAMS_DICT = {
                self.SETTINGS_PARAM_KEY: {
                    'namespace': self.namespace,
                    'factory_val': self.settings_dict_values
                }
            }
        else:
            self.CONFIGS_DICT = None
            self.PARAMS_DICT = None

        # Services Config Dict ####################
        # No capabilities service. Everything _capabilitiesHandler used to
        # return is published in the status message.
        self.SRVS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'status_pub': {
                'namespace': self.namespace,
                'msg': SettingsStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            self.node_if_prefix + 'update_setting': {
                'msg': UpdateControl,
                'namespace': self.namespace,
                'topic': 'update_setting',
                'qsize': 5,
                'callback': self._updateSettingCb
            },
            self.node_if_prefix + 'reset_settings': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'qsize': 5,
                'callback': self._resetSettingsCb,
                'callback_args': None
            }
        }

        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if_shared = True
            self.node_if = node_if
            if self.PARAMS_DICT is not None:
               self.node_if.add_params(self.PARAMS_DICT) 
            self.node_if.register_pubs(self.PUBS_DICT)
            self.node_if.register_subs(self.SUBS_DICT)
        else:
            self.config_topic = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = self.SRVS_DICT,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                                                )

   

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status() 
    
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

  
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the SettingsIF.

        Returns:
            bool: True if initialization completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the SettingsIF is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace used by this SettingsIF.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace
    
    def unregister(self):
        """Shut down this settings interface and release its ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)
                self.PUBS_DICT = None


    def get_settings_dict(self):
        """Return the current settings in the string-valued device settings form.

        Returns:
            dict: setting name -> {'name', 'type', 'value'}.
        """
        return nepi_controls.get_controls_values_dict(self.getSettingsDict())

    def publish_status(self):
        """Build and publish the settings SettingsStatus message.

        Refreshes the settings dict from the device (its live values, and its
        capability report if it provides a getCapSettingsFunction), then
        publishes. No-ops if node_if is None.
        """
        if self.node_if is None:
            return
        self.settings_status_msg.config_topic = self.config_topic
        settings_dict = self.getSettingsDict()

        self.settings_status_msg = nepi_controls.update_status_msg(self.settings_status_msg, settings_dict)
        self.node_if.publish_pub(self.node_if_prefix + 'status_pub', self.settings_status_msg)



    def update_setting_value(self,setting_name, setting_value, do_updates = True, update_param = True):
        """Apply a single setting update using the registered setSettingFunction.

        Compares the incoming value against the value currently held in the
        settings dict and only calls down to the device if it has changed.
        Optionally persists the settings dict to the ROS param server and
        publishes an updated status message.

        Args:
            setting (dict): Setting dictionary with 'name', 'type', and 'value' keys.
            do_updates (bool, optional): If True, publish status after a successful
                update. Defaults to True.
            update_param (bool, optional): If True, write the updated settings to the
                ROS param server. Defaults to True.

        Returns:
            bool: True if the setting was successfully applied, False otherwise.
        """
        success = False
        
        if self.setSettingFunction is None:
            self.msg_if.pub_debug("Settings updates ignored. No settings update function defined ", log_name_list = self.log_name_list)
            return success

        settings_dict = self.getSettingsDict()
        if setting_name not in settings_dict.keys():
            self.msg_if.pub_warn("Ignoring update for unknown setting: " + str(setting_name), log_name_list = self.log_name_list)
            return

        if nepi_controls.get_clean_value(settings_dict, setting_name, setting_value) is None:
            self.msg_if.pub_warn("Setting update rejected as invalid: " + str([setting_name, setting_value, index]), log_name_list = self.log_name_list)

        current_value = nepi_controls.get_control_value(self.settings_dict, setting_name)
        if current_value == setting_value:
            #self.msg_if.pub_warn("Setting allready set: " + str([current_value, setting_value]), log_name_list = self.log_name_list)
            return True

        self.msg_if.pub_info("Updating setting : " + str([setting_name,setting_value]), log_name_list = self.log_name_list)
        try:
            if self.callback_arg is None:
                [success, msg, self.settings_dict] = self.setSettingFunction(setting_name, setting_value)
            else:
                [success, msg, self.settings_dict] = self.setSettingFunction(setting_name, setting_value, self.callback_arg)
        except Exception as e:
            self.msg_if.pub_warn("setSettingFunction callback failed: " + str(e), log_name_list = self.log_name_list)
            success = False
            msg = str(e)
        #self.msg_if.pub_warn("setSettingFunction returned: " + str(self.settings_dict), log_name_list = self.log_name_list)
        if do_updates == True:
            self.publish_status()
        if success == True:
            self.msg_if.pub_info("Setting Updated: " + str([setting_name, setting_value]), log_name_list = self.log_name_list)
            if update_param == True:
                self.saveSettingsParam()
        else:
            self.msg_if.pub_warn("Setting update failed: " + str([setting_name, setting_value]) + " : " + str(msg), log_name_list = self.log_name_list)
        return success


    def init(self, do_updates = True):
        """Load settings from the ROS param server and optionally apply them.

        Reads the persisted string-valued settings dict from the param server,
        seeds the settings dict default and set tiers from it, and if
        do_updates is True pushes each stored value down to the device before a
        final status publish.

        Args:
            do_updates (bool, optional): If True, apply all stored settings to the
                hardware after loading. Defaults to True.
        """
        init_settings_values_dict = nepi_controls.get_controls_values_dict(self.settings_dict)
        if self.node_if is not None and self.save_params == True:
            init_settings_values_dict = self.node_if.get_param(self.SETTINGS_PARAM_KEY)            

        if self.callback_arg is None:
            init_settings_dict = self.getSettingsFunction()
        else:
            init_settings_dict = self.getSettingsFunction(self.callback_arg)

        if type(init_settings_dict) == dict:
            if type(init_settings_values_dict) == dict:
                for setting_name in init_settings_values_dict.keys():
                    if setting_name in init_settings_dict.keys():
                        setting_value = init_settings_values_dict[setting_name]
                        init_settings_dict = nepi_controls.set_control_value(init_settings_dict, setting_name, setting_value, setting_value)
        else:
            init_settings_dict = dict()

        self.settings_dict = init_settings_dict
        settings_values_dict = nepi_controls.get_controls_values_dict(init_settings_dict)
        if init_settings_values_dict != settings_values_dict:
            self.node_if.set_param(self.SETTINGS_PARAM_KEY, init_settings_values_dict)


        if do_updates == True:
            settings_dict = self.getSettingsDict()
            settings_values_dict = nepi_controls.get_controls_values_dict(settings_dict)
            self.msg_if.pub_info("Applying Init Settings: " + str(settings_values_dict), log_name_list = self.log_name_list)
            for setting_name in settings_dict.keys():
                setting_value = nepi_controls.get_control_value(settings_dict, setting_name)
                try:
                    if self.callback_arg is None:
                        [success, msg, self.settings_dict] = self.setSettingFunction(setting_name, setting_value)
                    else:
                        [success, msg, self.settings_dict] = self.setSettingFunction(setting_name, setting_value, self.callback_arg)
                except Exception as e:
                    self.msg_if.pub_warn("setSettingFunction callback failed: " + str(e), log_name_list = self.log_name_list)
                    success = False
                    msg = str(e)
            self.saveSettingsParam()
        self.publish_status()

    def reset(self):
        """Reset settings to their last-saved (user) values and reinitialize.

        Restores the default tier of every setting, reloads the user
        configuration tier via node_if.reset_params(), then reapplies.
        """
        self.settings_dict = nepi_controls.reset_control_values(self.settings_dict)
        if self.node_if is not None and self.save_params == True and self.node_if_shared == False:
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset settings to factory defaults and reinitialize.

        Restores the factory tier of every setting (factory -> default -> set),
        restores the factory param values via node_if.factory_reset_params(),
        then reapplies.
        """
        self.settings_dict = nepi_controls.reset_control_values(self.settings_dict)
        if self.node_if is not None and self.save_params == True and self.node_if_shared == False:
            self.node_if.factory_reset_params()
        self.init(do_updates = True)


    ###############################
    # Class Private Methods
    ###############################

    def getSettingsDict(self):
        return copy.deepcopy(self.settings_dict)


    def saveSettingsParam(self):
        if self.node_if is not None and self.save_params == True:
            settings_values_dict = nepi_controls.get_controls_values_dict(self.settings_dict)
            self.node_if.set_param(self.SETTINGS_PARAM_KEY, settings_values_dict)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.reset()

    def _factoryResetCb(self, do_updates = True):
        self.factory_reset()

    def _resetSettingsCb(self, msg):
        self.reset()

    def _publishStatusCb(self, timer):
        self.publish_status()


    def _updateSettingCb(self,msg):
            #self.msg_if.pub_info("Received setting update msg: " + str(msg), log_name_list = self.log_name_list)
            setting_name = msg.name
            settings_dict = copy.deepcopy(self.settings_dict)
            cur_value = nepi_controls.get_control_value(settings_dict, setting_name )
            settings_dict = nepi_controls.apply_update_control_msg(settings_dict, msg)
            setting_value = nepi_controls.get_control_value(settings_dict, setting_name )
            #self.msg_if.pub_info("Sending Updated Val from/to: " + str([cur_value, setting_value]), log_name_list = self.log_name_list)
            self.update_setting_value(setting_name, setting_value, do_updates = True, update_param = True)















############################################
# StatesIF


STATE_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]

EXAMPLE_STATES_DICT = {
                    "state_name": {
                        "value": False
                    }
}


class StatesIF:

    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'states_'
    

    ready = False
    msg_if = None
    namespace = '~'

    get_states_dict_function = None



    #######################
    ### IF Initialization
    def __init__(self,
                states_name = 'states',
                get_states_dict_function = None,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
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
        self.msg_if.pub_info("Starting States IF Initialization Processes", log_name_list = self.log_name_list)
        
        #############################
        # Initialize Class Variables

        if get_states_dict_function is None:
            self.msg_if.pub_warn("get_states_dict_function can not be None") 
            return
        self.get_states_dict_function = get_states_dict_function

        # Create Namespace
        states_name = nepi_utils.get_clean_name(states_name)
        if states_name is None or states_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(states_name)) 
            return
        self.msg_if.pub_info("Using States Name: " + states_name)
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,states_name)

        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            self.node_if_prefix + 'states_query': {
                'namespace': self.namespace,
                'topic': 'system_states_query',
                'srv': SystemStatesQuery,
                'req': SystemStatesQueryRequest(),
                'resp': SystemStatesQueryResponse(),                
                'callback': self._statesQueryHandler
            }
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'states_status_pub': {
                'namespace': self.namespace,
                'msg': SystemStatesStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            self.node_if.register_services(self.SRVS_DICT)

        else:
            self.config_topic = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            services_dict = self.SRVS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                                                )


        success = nepi_sdk.wait()
        states_dict = self.get_states_dict_function()
        status_msg = SystemStatesStatus()
        status_msg.state_names = list(states_dict.keys())
        self.node_if.publish_pub(self.node_if_prefix + 'states_status_pub', status_msg)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the StatesIF.

        Returns:
            bool: True if initialization completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the StatesIF is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready


    def get_namespace(self):
        """Return the ROS namespace used by this StatesIF.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SRVS_DICT is not None:
                        for service_name in self.SRVS_DICT.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                # if self.node_if is not None:
                #     if self.PUBS_DICT is not None:
                #         for pub_name in self.PUBS_DICT.keys():
                #             self.node_if.unregister_pub(pub_name)

    ###############################
    # Class Private Methods
    ###############################

    def _statesQueryHandler(self, req):
        resp = SystemStatesQueryResponse()
        try:
            states_dict = self.get_states_dict_function()
            resp = nepi_states.create_states_msg(states_dict)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create resp msg: " + str(e), log_name_list = self.log_name_list)
        return resp





EXAMPLE_TRIGGER_DICT = {
                    "name":"None",
                    "node_name": '~',
                    "description": "None",
                    "data_str_list":["None"],
                    "time":nepi_utils.get_time() 
}


EXAMPLE_TRIGGERS_DICT = {
                "trigger_name": {
                    "name":"None",
                    "node_name": '~',
                    "description": "None",
                    "data_str_list":["None"],
                    "time":nepi_utils.get_time() 
                    }

}



class TriggersIF:

    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'triggers_'

    msg_if = None
    ready = False
    namespace = '~'

    triggers_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                triggers_dict = None,
                triggers_name = 'triggers',
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
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
        self.msg_if.pub_info("Starting Triggers IF Initialization Processes", log_name_list = self.log_name_list)
        
        #############################
        # Initialize Class Variables

        if triggers_dict is None:
            self.triggers_dict = dict()
        else:
            self.triggers_dict = triggers_dict

        # Create Namespace
        triggers_name = nepi_utils.get_clean_name(triggers_name)
        if triggers_name is None or triggers_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(triggers_name)) 
            return
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'
        ##############################  
        # Create NodeClassIF Class  


        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'trigger_pub': {
                'msg': SystemTrigger,
                'namespace': self.base_namespace,
                'topic': 'system_triggers',
                'qsize': 1,
                'latch': False
            }
        }

        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            self.node_if.register_services(self.SRVS_DICT)
            self.node_if.register_pubs(self.PUBS_DICT)
        else:
            self.config_topic = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            pubs_dict = self.PUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                                                )

        success = nepi_sdk.wait()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################



    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the TriggersIF.

        Returns:
            bool: True if initialization completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the TriggersIF is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace used by this TriggersIF.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SRVS_DICT is not None:
                        for service_name in self.SRVS_DICT.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)



    def publish_trigger(self, trigger_dict):
        """Build and publish a system trigger message on the 'system_triggers' topic.

        Converts the trigger dictionary to a SystemTrigger ROS message and publishes
        it via the node_if. No-ops if node_if is None.

        Args:
            trigger_dict (dict): Trigger event dictionary with keys 'name',
                'node_name', 'description', 'data_str_list', and 'time'.
        """
        trig_msg = nepi_triggers.create_trigger_msg(self.namespace, trigger_dict)
        if self.node_if is not None:
            self.node_if.publish_pub(self.node_if_prefix +  'trigger_pub',trig_msg)


    ###############################
    # Class Private Methods
    ###############################



