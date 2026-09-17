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
import math
import cv2



from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped


from nepi_interfaces.msg import NavPose, NavPoses, NavPoseStatus, NavPosesStatus
from nepi_interfaces.msg import ColorBGR, ImageStatus, MgrSystemStatus
from nepi_interfaces.msg import DepthMapStatus
from nepi_interfaces.msg import IntensityMapStatus
from nepi_interfaces.msg import PointcloudStatus

from nepi_interfaces.msg import NavPoseTrack
from nepi_interfaces.msg import NavPoseLocation, NavPoseHeading
from nepi_interfaces.msg import NavPoseOrientation, NavPosePosition
from nepi_interfaces.msg import NavPoseAltitude, NavPoseDepth
from nepi_interfaces.msg import NavPosePanTilt
from nepi_interfaces.srv import NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryRequest, NavPoseCapabilitiesQueryResponse



from nepi_interfaces.msg import StringArray, UpdateBool, UpdateFloat, ImageWindow, RangeWindow, ImagePixel, ImageMouseEvent, ImageCrosshair, ImageTarget
from nepi_interfaces.srv import ImageCapabilitiesQuery, ImageCapabilitiesQueryRequest, ImageCapabilitiesQueryResponse

from nepi_interfaces.msg import RangeWindow
from nepi_interfaces.msg import ImageSize

from sensor_msgs.msg import PointCloud2

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, Transform3DIF
from nepi_api.connect_data_if import ConnectNavPoseIF


SYSTEM_ALL_TOPIC = 'all'

##################################################


API_LIB_FOLDER = "/opt/nepi/nepi_engine/lib/nepi_api"

SYSTEM_DATA_FOLDER = 'mnt/nepi_storage/data'

SUPPORTED_DICT_FILE_TYPES = ['yaml']
SUPPORTED_IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']  
SUPPORTED_POINTCLOUD_FILE_TYPES = ['pcd']
SUPPORTED_VID_FILE_TYPES = ['avi','AVI']

SUPPORTED_DATA_TYPES = ['dict','cv2_image','o3d_pc']


EXAMPLE_FILENAME_DICT = {
        'prefix': "", 
        'add_timestamp': True, 
        'use_utc_tz': True,
        'add_ms': True,
        'add_us': False,
        'add_tz': True,
        'add_node_name': True
        }



from nepi_sdk import nepi_data
from nepi_interfaces.msg import Datum, DataStatus

from nepi_interfaces.msg import UpdateOrder, UpdateFloat, UpdateFloats, UpdateInt, UpdateInts, UpdateBool, UpdateBools, UpdateString, UpdateStringArray, UpdateTrigger



#########################################
# Data IF Class
#########################################


class DataIF:

    msg_if = None
    node_if = None
    node_if_shared = False
    node_if_prefix = 'data_'

    data_name = 'data'
    namespace = ''
    data_display_name = ''
    data_description = ''
    data_dict = dict()
    data_hidden = False
    data_status_msg = DataStatus()

    data_node_pubs_dict = None
    data_node_subs_dict = None
    data_ready = False

    active_nodes = []
    active_topics = []
    active_topic_types =  []
    active_services =  []

    status_has_published = False

    data_updated_callback = None # if not None: Calls function with datum_name after a datum is written and status is published
    data_updater_max_rate = 1 # set to -1 to disable the updater thread
    data_updater_callback = None # if not None: Calls function at the begining of each updater loop

    #######################
    ### IF Initialization
    def __init__(self,
                data_name = 'data',
                data_display_name = 'Data',
                data_description = 'Data',
                data_init_dict = dict(),
                data_updated_callback = None, # if not None: Calls function with datum_name after a datum is written and status is published
                data_updater_max_rate = 1, # set to -1 to disable updater thread
                data_updater_callback = None, # if not None: Calls function at the begining of each loop
                show_data = True,
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
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        # Create Namespace
        self.data_name = nepi_utils.get_clean_name(data_name)
        if self.data_name is None or self.data_name == '':
            self.msg_if.pub_warn("Data Name Not Valid: " + str(data_name))
            return
        self.msg_if.pub_info("Using Data Name: " + self.data_name)
        # get_node_namespace() is already fully resolved, so this is an absolute
        # namespace. The RUI subscribes to <namespace>/status, so it must
        # never be a relative path.
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,self.data_name)

        # Registry keys are prefixed with the data domain so a shared node_if
        # cannot have a sibling IF silently overwrite this IF's entries
        # (see the 2026-07 registry-key decision in the top-level CLAUDE.md).
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        ##############################
        # Initialize Class Variables

        self.data_display_name = str(data_display_name)
        self.data_description = str(data_description)
        self.data_dict = nepi_data.create_data_dict(data_init_dict)
        self.data_status_msg = nepi_data.create_status_msg(self.data_name, self.data_display_name, self.data_description,
                                                                    show_data, has_show_control)
        self.data_hidden = hidden

        self.data_updated_callback = data_updated_callback
        self.data_updater_max_rate = data_updater_max_rate
        self.data_updater_callback = data_updater_callback

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
        # Persist the data dict under the data namespace so display
        # configuration (display name, description, hidden, order) survives node
        # restarts via the config manager. Passing a params_dict is what enables
        # config management on NodeClassIF.
        PARAMS_DICT = {
            self.node_if_prefix + 'data_dict': {
                'namespace': self.namespace,
                'factory_val': self.data_dict
            }
        }

        # Publishers Config Dict ####################
        self.data_node_pubs_dict = {
             self.node_if_prefix + 'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DataStatus,
                'qsize': 1,
                'latch': True
            }
        }



        # Subscribers Config Dict ####################
        # These are how the owning node (or another node) pushes a datum in. The
        # RUI never publishes to them -- Nepi_IF_Data.js is display only.
        self.data_node_subs_dict = {
            #####################
            # Data Subs
            ####################
             self.node_if_prefix + 'set_bool_datum_value': {
                'msg': UpdateBool,
                'namespace': self.namespace,
                'topic': 'set_bool_datum_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_bools_datum_value': {
                'msg': UpdateBools,
                'namespace': self.namespace,
                'topic': 'set_bools_datum_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_string_datum_value': {
                'msg': UpdateString,
                'namespace': self.namespace,
                'topic': 'set_string_datum_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_strings_datum_value': {
                'msg': UpdateStringArray,
                'namespace': self.namespace,
                'topic': 'set_strings_datum_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_int_datum_value': {
                'msg': UpdateInt,
                'namespace': self.namespace,
                'topic': 'set_int_datum_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_ints_datum_value': {
                'msg': UpdateInts,
                'namespace': self.namespace,
                'topic': 'set_ints_datum_value',
                'qsize': 5,
                'callback': self._setValueCb
            },

             self.node_if_prefix + 'set_float_datum_value': {
                'msg': UpdateFloat,
                'namespace': self.namespace,
                'topic': 'set_float_datum_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             # UpdateFloats, not UpdateFloat -- a Floats datum carries an array.
             # Note UpdateFloats names its payload field 'values', unlike every
             # other Update* msg; _setValueCb handles both spellings.
             self.node_if_prefix + 'set_floats_datum_value': {
                'msg': UpdateFloats,
                'namespace': self.namespace,
                'topic': 'set_floats_datum_value',
                'qsize': 5,
                'callback': self._setValueCb
            },

            #####################
            # Display Subs
            #####################
             self.node_if_prefix + 'set_datum_hidden': {
                'msg': UpdateBool,
                'namespace': self.namespace,
                'topic': 'set_datum_hidden',
                'qsize': 5,
                'callback': self._setHiddenValueCb
            },
             self.node_if_prefix + 'set_data_hidden': {
                'msg': UpdateBool,
                'namespace': self.namespace,
                'topic': 'set_data_hidden',
                'qsize': 5,
                'callback': self._setDataHiddenCb
            },
             self.node_if_prefix + 'set_datum_order': {
                'msg': UpdateInt,
                'namespace': self.namespace,
                'topic': 'set_datum_order',
                'qsize': 5,
                'callback': self._setOrderValueCb
            },
             self.node_if_prefix + 'set_datum_up': {
                'msg': UpdateTrigger,
                'namespace': self.namespace,
                'topic': 'set_datum_up',
                'qsize': 5,
                'callback': self._setOrderUpCb
            },
             self.node_if_prefix + 'set_datum_down': {
                'msg': UpdateTrigger,
                'namespace': self.namespace,
                'topic': 'set_datum_down',
                'qsize': 5,
                'callback': self._setOrderDownCb
            },
             self.node_if_prefix + 'set_datum_top': {
                'msg': UpdateTrigger,
                'namespace': self.namespace,
                'topic': 'set_datum_top',
                'qsize': 5,
                'callback': self._setOrderTopCb
            },
             self.node_if_prefix + 'set_datum_bottom': {
                'msg': UpdateTrigger,
                'namespace': self.namespace,
                'topic': 'set_datum_bottom',
                'qsize': 5,
                'callback': self._setOrderBottomCb
            },

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
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.data_node_pubs_dict,
                            subs_dict = self.data_node_subs_dict,
                            log_name_list = [],
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()
        else:
            self.node_if_shared = True
            try:
                self.node_if = node_if
                self.node_if.register_pubs(self.data_node_pubs_dict)
                self.node_if.register_subs(self.data_node_subs_dict)
                # Register the persisted data dict on the shared node_if too,
                # under the same prefixed key init() reads back.
                self.node_if.add_param(self.node_if_prefix + 'data_dict', self.namespace, self.data_dict)
                nepi_sdk.sleep(1)
            except Exception as e:
                self.msg_if.pub_info("Failed to register pubs and subs: " + str(e))
                return


        ##############################
        # Start updater and status publisher
        if self.data_updater_max_rate != -1:
            nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

        ##############################
        # Complete Initialization
        self.data_ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete")
        ###############################


    #######################
    # Class Public Methods
    #######################


    def get_data_ready_state(self):
        """Return the ready state of the data interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.data_ready

    def wait_for_data_ready(self, timeout = float('inf') ):
        """Block until the data interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.data_ready is not None:
            self.msg_if.pub_info("Waiting for data interface ready")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.data_ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.data_ready == False:
                self.msg_if.pub_info("Data interface wait timed out")
            else:
                self.msg_if.pub_info("Data interface ready")
        return self.data_ready

    def get_namespace(self):
        """Return the fully-resolved ROS namespace this data set publishes under.

        The namespace is built from the owning node's namespace, so the status
        topic is <namespace>/status and is directly subscribable by the RUI.

        Returns:
            str: The fully-qualified data namespace.
        """
        return self.namespace

    def unregister(self):
        """Unregister every publisher and subscriber this interface registered.

        Returns:
            bool: True if the interface unwound cleanly, False otherwise.
        """
        success = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.sleep(1)
            else:
                # Shared node_if: unwind only the entries this IF added, leaving
                # the owning node's and sibling IFs' entries alone.
                if self.data_node_subs_dict is not None:
                    for sub_name in self.data_node_subs_dict.keys():
                        self.node_if.unregister_sub(sub_name)
                self.data_node_subs_dict = None

                if self.data_node_pubs_dict is not None:
                    for pub_name in self.data_node_pubs_dict.keys():
                        self.node_if.unregister_pub(pub_name)
                self.data_node_pubs_dict = None

        time.sleep(1)
        try:
            self.node_if = None
            self.data_ready = False
            success = True
        except Exception as e:
            self.msg_if.pub_warn("Failed to unregister:  " + str(e))
        return success


    ##################
    # Data Functions

    def get_data_dict(self):
        """Return a copy of the full data dict, keyed by datum name.

        Returns:
            dict: A deep copy of the data dict.
        """
        data_dict = copy.deepcopy(self.data_dict)
        return data_dict

    def get_datum_value(self, datum_name):
        """Return the current value of one datum, read from its type-correct field.

        Args:
            datum_name (str): The datum key name.

        Returns:
            The datum value, or None if the datum is not registered.
        """
        data_dict = copy.deepcopy(self.data_dict)
        value = nepi_data.get_datum_value(data_dict, datum_name)
        return value

    def set_datum_value(self, datum_name, update_value, timestamp = None):
        """Write one datum value, stamp its timestamp, and publish status.

        The node that owns this interface is the only writer of record; the RUI
        has no publish path to this method.

        Args:
            datum_name (str): The datum key name.
            update_value: The new value. Coerced to the datum's declared type.
            timestamp (float, optional): Write time. Defaults to now.
        """
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.set_datum_value(data_dict, datum_name, update_value, timestamp = timestamp)
        self.data_dict = data_dict
        self.publish_status()
        if self.data_updated_callback is not None:
            self.data_updated_callback(datum_name)

    def get_datum_timestamp(self, datum_name):
        """Return the time one datum's value was last written.

        Args:
            datum_name (str): The datum key name.

        Returns:
            float: The datum timestamp in seconds, or 0.0 if not registered.
        """
        data_dict = copy.deepcopy(self.data_dict)
        timestamp = nepi_data.get_datum_timestamp(data_dict, datum_name)
        return timestamp

    ##################
    # Display Functions

    def get_datum_display_name(self, datum_name):
        """Return the RUI display name for one datum.

        Args:
            datum_name (str): The datum key name.

        Returns:
            str: The display name, or '' if not registered.
        """
        data_dict = copy.deepcopy(self.data_dict)
        display_name = nepi_data.get_datum_display_name(data_dict, datum_name)
        return display_name

    def set_datum_display_name(self, datum_name, display_name):
        """Set the RUI display name for one datum.

        Args:
            datum_name (str): The datum key name.
            display_name (str): The name shown in the RUI.
        """
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.set_datum_display_name(data_dict, datum_name, display_name)
        self.data_dict = data_dict
        self._saveDataDict()


    def get_datum_description(self, datum_name):
        """Return the description text for one datum.

        Args:
            datum_name (str): The datum key name.

        Returns:
            str: The description, or '' if not registered.
        """
        data_dict = copy.deepcopy(self.data_dict)
        description = nepi_data.get_datum_description(data_dict, datum_name)
        return description

    def set_datum_description(self, datum_name, description):
        """Set the description text for one datum.

        Args:
            datum_name (str): The datum key name.
            description (str): The description shown in the RUI.
        """
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.set_datum_description(data_dict, datum_name, description)
        self.data_dict = data_dict
        self._saveDataDict()

    def get_datum_hidden(self, datum_name):
        """Return whether one datum is hidden in the RUI.

        Args:
            datum_name (str): The datum key name.

        Returns:
            bool: True if the datum is hidden.
        """
        data_dict = copy.deepcopy(self.data_dict)
        hidden = nepi_data.get_datum_hidden(data_dict, datum_name)
        return hidden

    def set_datum_hidden(self, datum_name, hidden):
        """Hide or show one datum in the RUI.

        Args:
            datum_name (str): The datum key name.
            hidden (bool): True to hide the datum.
        """
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.set_datum_hidden(data_dict, datum_name, hidden)
        self.data_dict = data_dict
        self._saveDataDict()

    def get_data_hidden(self):
        """Return whether the whole data set is hidden in the RUI.

        Returns:
            bool: True if the data set is hidden.
        """
        return self.data_hidden

    def set_data_hidden(self, hidden):
        """Hide or show the whole data set in the RUI.

        Args:
            hidden (bool): True to hide the data set.
        """
        self.data_hidden = bool(hidden)

    def get_datum_display_order(self, datum_name):
        """Return the display position of one datum.

        Args:
            datum_name (str): The datum key name.

        Returns:
            int: Zero-based display index, or -1 if not registered.
        """
        data_dict = copy.deepcopy(self.data_dict)
        order = nepi_data.get_datum_display_order(data_dict, datum_name)
        return order

    def set_datum_display_order(self, datum_name, update_order = 0):
        """Move one datum to an absolute display position.

        Args:
            datum_name (str): The datum key name.
            update_order (int, optional): Zero-based target index. Defaults to 0.
        """
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.set_datum_display_order(data_dict, datum_name, update_order)
        self.data_dict = data_dict
        self._saveDataDict()

    def move_datum_display_top(self, datum_name):
        """Move one datum to the top of the display order.

        Args:
            datum_name (str): The datum key name.
        """
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.move_datum_display_top(data_dict, datum_name)
        self.data_dict = data_dict
        self._saveDataDict()

    def move_datum_display_bottom(self, datum_name):
        """Move one datum to the bottom of the display order.

        Args:
            datum_name (str): The datum key name.
        """
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.move_datum_display_bottom(data_dict, datum_name)
        self.data_dict = data_dict
        self._saveDataDict()

    def move_datum_display_up(self, datum_name):
        """Move one datum one position in the display order.

        Args:
            datum_name (str): The datum key name.
        """
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.move_datum_display_up(data_dict, datum_name)
        self.data_dict = data_dict
        self._saveDataDict()

    def move_datum_display_down(self, datum_name):
        """Move one datum one position in the display order.

        Args:
            datum_name (str): The datum key name.
        """
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.move_datum_display_down(data_dict, datum_name)
        self.data_dict = data_dict
        self._saveDataDict()


    ##################
    # Misc Functions

    def publish_status(self):
        """Rebuild the DataStatus message from the data dict and publish it."""
        ###########
        data_dict = copy.deepcopy(self.data_dict)
        self.data_status_msg = nepi_data.update_status_msg(self.data_status_msg, data_dict, self.data_hidden)
        if self.node_if is not None:
            if self.status_has_published == False:
                self.msg_if.pub_info("Publishing first Data Status on: " + str(self.namespace) + "/status")
                self.status_has_published = True
            self.node_if.publish_pub(self.node_if_prefix + 'status_pub', self.data_status_msg)

    def init(self, do_updates = False):
        """Initialize or re-initialize the data dict from the parameter server and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            # Prefixed key, matching how the param is registered and how the
            # display setters write it back. get_param(self.node_if_prefix +  ) returns None for a name
            # it does not know, so an unprefixed name wiped the data dict on
            # every config init, reset and factory reset, leaving DataStatus with
            # empty data lists and the RUI with an empty data box. The None guard
            # below means a missing param can never overwrite a live dict.
            param_name = self.node_if_prefix + 'data_dict'
            data_dict = self.node_if.get_param(param_name)
            if data_dict is not None:
                self.data_dict = data_dict

        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the data interface to its initialized state."""
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        nepi_sdk.sleep(1)     
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset the data interface to factory defaults."""
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Factory resetting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)


    ###############################
    # Class Private Methods
    ###############################

    # Persist the data dict. Called by the display setters only. Datum *values*
    # deliberately do not write here: a datum is live telemetry that can update
    # many times a second, and pushing the whole dict to the param server at
    # that rate would be pure overhead. What is worth persisting across a node
    # restart is the display configuration -- display name, description, hidden
    # and order.
    def _saveDataDict(self):
        if self.node_if is not None:
            param_name = self.node_if_prefix + 'data_dict'
            self.node_if.set_param(param_name, self.data_dict)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    # ROS callback for the system status msg. Populates the active node/topic/
    # service lists available to the owning node.
    def _systemStatusCb(self,msg):
            self.active_nodes = msg.active_nodes
            self.active_topics = msg.active_topics
            self.active_topic_types = msg.active_topic_types
            self.active_services = msg.active_services


    # Updater timer. Gives the owning node one call per loop to refresh its data,
    # then publishes status if the callback reports a change. Rearms itself.
    def _updaterCb(self,timer):
        needs_publish = False
        start_time = nepi_utils.get_time()
        ##############
        if self.data_updater_callback is not None:
            needs_publish = self.data_updater_callback()
        ##################
        if needs_publish == True:
          self.publish_status()

        ##################
        # Setup Next Update
        delay_time = float(1) / self.data_updater_max_rate
        update_time = nepi_utils.get_time() - start_time
        next_time = delay_time - update_time
        if next_time < 0.01:
            next_time = 0.01
        nepi_sdk.start_timer_process(next_time, self._updaterCb, oneshot = True)


    def _setValueCb(self,msg):
            datum_name = msg.name
            # The value setters share this single callback. Most Update* msgs
            # carry a 'value' field; UpdateFloats carries 'values'.
            if hasattr(msg, 'value'):
                datum_value = msg.value
            elif hasattr(msg, 'values'):
                datum_value = msg.values
            else:
                return
            self.set_datum_value(datum_name, datum_value)

    def _setHiddenValueCb(self,msg):
            self.set_datum_hidden(msg.name, msg.value)

    def _setDataHiddenCb(self,msg):
            self.set_data_hidden(msg.value)

    def _setOrderValueCb(self,msg):
            self.set_datum_display_order(msg.name, msg.value)

    def _setOrderUpCb(self,msg):
            self.move_datum_display_up(msg.name)

    def _setOrderDownCb(self,msg):
            self.move_datum_display_down(msg.name)

    def _setOrderTopCb(self,msg):
            self.move_datum_display_top(msg.name)

    def _setOrderBottomCb(self,msg):
            self.move_datum_display_bottom(msg.name)

    def _publishStatusCb(self,timer):
            self.publish_status()





##################################################
## NavPoseIF

EXAMPLE_NAVPOSE_DATA_DICT = {
    'navpose_frame': 'nepi_frame',
    'frame_nav': 'ENU',
    'frame_altitude': 'WGS84',
    'frame_depth': 'MSL',

    'geoid_height_meters': 0,


    'has_location': True,
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'latitude': 47.080909,
    'longitude': -120.8787889,
    # Speed over ground, meters per second
    'location_m_per_sec': 0.0,

    'has_heading': True,
    'time_heading': nepi_utils.get_time(),
    # Heading should be provided in Degrees True North
    'heading_deg': 120.50,
    # Ground speed in the heading direction, meters per second
    'heading_m_per_sec': 0.0,

    'has_position': True,
    'time_position': nepi_utils.get_time(),
    # Position should be provided in Meters in specified 3d frame (x,y,z) with x forward, y right/left, and z up/down
    'x_m': 1.234,
    'y_m': 1.234,
    'z_m': 1.234,
    # Linear velocity in Meters per second in specified 3d frame
    'x_m_per_sec': 0.0,
    'y_m_per_sec': 0.0,
    'z_m_per_sec': 0.0,

    'has_orientation': True,
    'time_orientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees in specified 3d frame
    'roll_deg': 30.51,
    'pitch_deg': 30.51,
    'yaw_deg': 30.51,
    # Angular rates in Degrees per second in specified 3d frame
    'roll_deg_per_sec': 0.0,
    'pitch_deg_per_sec': 0.0,
    'yaw_deg_per_sec': 0.0,

    'has_altitude': True,
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters in specified altitude_m frame
    'altitude_m': 12.321,
    # Vertical rate (altitude change), meters per second
    'altitude_m_per_sec': 0.0,

    'has_depth': False,
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive meters
    'depth_m': 0.0,
    # Depth rate (depth change), meters per second
    'depth_m_per_sec': 0.0,

    'has_pan_tilt': False,
    'time_pan_tilt': nepi_utils.get_time(),
    # Pan Tilt should be provided in positive degs
    'pan_deg': 0.0,
    'tilt_deg': 0.0
}




class NavPoseIF:


    NAVPOSE_NAV_FRAME_OPTIONS = ['ENU','NED','UKNOWN']
    NAVPOSE_ALT_FRAME_OPTIONS = ['WGS84','AMSL','UKNOWN'] # ['WGS84','AMSL','AGL','MSL','HAE','BAROMETER','UKNOWN']
    NAVPOSE_DEPTH_FRAME_OPTIONS = ['DEPTH','UKNOWN'] # ['MSL','TOC','DF','KB','DEPTH','UKNOWN']


    DEFAULT_CALLBACK_DICT = dict(
        frame_updated_callback = None
    )
    callback_dict = copy.deepcopy(DEFAULT_CALLBACK_DICT)

    ready = False
    namespace = '~/navpose'

    node_if = None
    node_if_shared = True


    save_data_if = None
    save_data_enabled = True

    status_msg = NavPoseStatus()

    data_product = 'navpose'
    needs_data = False
    
    last_pub_time = None
    time_list = [0,0,0,0,0,0,0,0,0,0]


    frame_nav = 'ENU'
    frame_altitude = 'WGS84'
    frame_depth = 'MSL'



    navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    navpose_settings_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_INFO_DICT)
    navpose_frame = 'None'

    caps_report = NavPoseCapabilitiesQueryResponse()

    pubs_dict = dict()
    subs_dict = dict()

    def __init__(self, namespace = None,
                data_product = None,
                data_source_description = 'sensor',
                data_ref_description = 'sensor',
                pub_navpose = True,
                pub_location = False, pub_heading = False,
                pub_orientation = False, pub_position = False,
                pub_altitude = False, pub_depth = False,
                pub_pan_tilt = False,
                save_data_if = None,
                save_data_enabled = True,
                transform_namespace = '',
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
            log_name = nepi_utils.get_clean_name(log_name)
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################    
        # Initialize Class Variables


        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product

        if namespace is not None:
            self.namespace = namespace
        if os.path.basename(namespace) != self.data_product:
            namespace = nepi_sdk.create_namespace(namespace,self.data_product)
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        if data_source_description is None:
            data_source_description = self.data_source_description
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.data_ref_description
        self.data_ref_description = data_ref_description



        self.pub_navpose = pub_navpose
        self.pub_location = pub_location
        self.pub_heading = pub_heading
        self.pub_orientation = pub_orientation
        self.pub_position = pub_position
        self.pub_altitude = pub_altitude
        self.pub_depth = pub_depth
        self.pub_pan_tilt = pub_pan_tilt

        # Create Capabilities Report

        self.caps_report.has_navpose_pub = self.pub_navpose
        self.caps_report.has_location_pub = self.pub_location
        self.caps_report.has_heading_pub = self.pub_heading
        self.caps_report.has_position_pub = self.pub_position
        self.caps_report.has_orientation_pub = self.pub_orientation
        self.caps_report.has_depth_pub = self.pub_depth
        self.caps_report.has_pan_tilt = self.pub_pan_tilt


        #########################
        # Initialize status message
        self.status_msg.node_name = self.node_name
        self.status_msg.navpose_topic = self.namespace
        self.status_msg.transform_topic = transform_namespace if transform_namespace is not None else ''

        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description


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

        # Services Config Dict ####################     
        self.SRVS_DICT = {
            'navpose_caps_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': NavPoseCapabilitiesQuery,
                'req': NavPoseCapabilitiesQueryRequest(),
                'resp': NavPoseCapabilitiesQueryResponse(),
                'callback': self._provideCapabilities
            }
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None



        # Pubs Config Dict ####################
        self.PUBS_DICT = dict()

        
        if self.pub_navpose == True:

            self.PUBS_DICT[self.node_if_prefix + 'navpose_status_pub'] = {
                    'msg': NavPoseStatus,
                    'namespace': self.namespace,
                    'topic': 'status',
                    'qsize': 1,
                    'latch': True
                }
            self.PUBS_DICT[self.node_if_prefix + 'navpose_pub'] = {
                    'msg': NavPose,
                    'namespace': self.namespace,
                    'topic': '',
                    'qsize': 1,
                    'latch': False
                }
      
        if self.pub_location == True:
            self.PUBS_DICT[self.node_if_prefix + 'location_pub'] = {
                'msg': NavPoseLocation,
                'namespace': self.namespace,
                'topic': 'location',
                'qsize': 1,
                'latch': False
            }
            
        if self.pub_orientation == True:
            self.PUBS_DICT[self.node_if_prefix + 'orientation_pub'] = {
                'msg': NavPoseOrientation,
                'namespace': self.namespace,
                'topic': 'orientation',
                'qsize': 1,
                'latch': False
            }

        if self.pub_position == True:
            self.PUBS_DICT[self.node_if_prefix + 'position_pub'] = {
                'msg': NavPosePosition,
                'namespace': self.namespace,
                'topic': 'position',
                'qsize': 1,
                'latch': False
            }

        if self.pub_heading == True:
            self.PUBS_DICT[self.node_if_prefix + 'heading_pub'] = {
                'msg': NavPoseHeading,
                'namespace': self.namespace,
                'topic': 'heading',
                'qsize': 1,
                'latch': False
            }

        if self.pub_altitude == True:
            self.PUBS_DICT[self.node_if_prefix + 'altitude_pub'] = {
                'msg': NavPoseAltitude,
                'namespace': self.namespace,
                'topic': 'altitude',
                'qsize': 1,
                'latch': False
            }

        if self.pub_depth == True:
            self.PUBS_DICT[self.node_if_prefix + 'depth_pub'] = {
                'msg': NavPoseDepth,
                'namespace': self.namespace,
                'topic': 'depth',
                'qsize': 1,
                'latch': False
            }

        if self.pub_pan_tilt == True:
            self.PUBS_DICT[self.node_if_prefix + 'pan_tilt_pub'] = {
                'msg': NavPosePanTilt,
                'namespace': self.namespace,
                'topic': 'pan_tilt',
                'qsize': 1,
                'latch': False
            }


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            self.node_if_prefix + 'navpose_reset': {
                'namespace': self.namespace,
                'topic': 'reset',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetCb, 
                'callback_args': ()
            }
        }


        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            if self.PARAMS_DICT is not None:
               self.node_if.add_params(self.PARAMS_DICT) 
            self.node_if.register_services(self.SRVS_DICT)
            self.node_if.register_pubs(self.PUBS_DICT)
            self.node_if.register_subs(self.SUBS_DICT)

        else:
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

        ####################
        self.save_data_enabled = save_data_enabled

        if self.save_data_enabled == True:
            self.msg_if.pub_info("####################", log_name_list = self.log_name_list)
            self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
            if save_data_if is not None and save_data_if != 'None':
                self.save_data_if = save_data_if
                data_products = self.save_data_if.get_data_products()
                if self.data_product not in data_products:
                    self.save_data_if.register_data_product(self.data_product)
            elif save_data_if != 'None':
                
                # Setup Save Data IF Class 
                self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
                factory_data_rates= dict()
                factory_data_rates[self.data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

                factory_filename_dict = {
                    'prefix': "", 
                    'add_timestamp': True, 
                    'add_ms': True,
                    'add_us': False,
                    'suffix': "",
                    'add_node_name': True
                    }

                sd_namespace = self.node_namespace
                self.save_data_if = SaveDataIF(namespace = sd_namespace,
                                        data_products = [self.data_product],
                                        factory_rate_dict = factory_data_rates,
                                        factory_filename_dict = factory_filename_dict,
                                        log_name_list = self.log_name_list,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if)
                nepi_sdk.sleep(1)

            if self.save_data_if is not None:
                self.status_msg.save_data_topic = self.save_data_if.get_namespace()
                self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic), log_name_list = self.log_name_list)





        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

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
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
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
        """Return the ROS namespace for this nav pose interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace

    def get_frame_nav_options(self):
        """Return the list of supported navigation frame identifiers.

        Returns:
            list: Supported nav frame strings (e.g. 'ENU', 'NED').
        """
        return self.NAVPOSE_NAV_FRAME_OPTIONS

    def get_frame_altitude_options(self):
        """Return the list of supported altitude frame identifiers.

        Returns:
            list: Supported altitude frame strings (e.g. 'WGS84', 'AMSL').
        """
        return self.NAVPOSE_ALT_FRAME_OPTIONS

    def get_frame_depth_options(self):
        """Return the list of supported depth frame identifiers.

        Returns:
            list: Supported depth frame strings (e.g. 'DEPTH').
        """
        return self.NAVPOSE_DEPTH_FRAME_OPTIONS

    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'navpose').
        """
        return self.data_product

    def get_blank_navpose_dict(self):
        """Return a deep copy of the blank nav pose dictionary template.

        Returns:
            dict: A blank nav pose dictionary with all fields at default values.
        """
        blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return blank_navpose_dict

    def get_navpose_dict(self):
        """Return a deep copy of the most recently published nav pose dictionary.

        Returns:
            dict: The last nav pose data dictionary.
        """
        navpose_dict =  copy.deepcopy(self.navpose_dict)
        return navpose_dict

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict


    def needs_data_check(self):
        """Return whether downstream consumers currently need nav pose data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        needs_data = copy.deepcopy(self.needs_data)
        # self.msg_if.pub_debug("Returning: " + self.namespace + " " "needs data: " + str(needs_data), log_name_list = self.log_name_list, throttle_s = 5.0)
        return needs_data

    # Update System Status
    def publish_navpose(self,navpose_dict,
                        timestamp = None,
                        transform = None,
                        ):
        """Publish a nav pose data dictionary to all configured ROS topics.

        Converts the input dictionary to NEPI standard frames (ENU / WGS84),
        publishes individual component messages (location, heading, orientation,
        position, altitude, depth, pan/tilt) if enabled, publishes the combined
        NavPose message, optionally applies a 3-D transform, and triggers data
        saving if a SaveDataIF is registered.

        Args:
            navpose_dict (dict): Nav pose data following the NEPI navpose dict schema.
            timestamp (float or rospy.Time, optional): Acquisition timestamp in seconds
                or as a ROS Time object. Defaults to current time if None.
            transform (object, optional): Transform to apply to the nav pose after
                publishing individual components. Defaults to None.

        Returns:
            dict: The processed nav pose dictionary in NEPI standard frames.
        """
        np_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        if navpose_dict is None and self.status_msg is not None:
            return np_dict
        else:
            # Initialize np_dict here so it's available in both branches
            for key in np_dict.keys():
                if key in navpose_dict.keys():
                    np_dict[key] = navpose_dict[key]
            
            self.msg_if.pub_debug("Start Navpose data dict: " + str(np_dict), log_name_list = self.log_name_list, throttle_s = 5.0)

            if timestamp == None:
                timestamp = nepi_utils.get_time()
            else:
                timestamp = nepi_sdk.sec_from_timestamp(timestamp)

            current_time = nepi_utils.get_time()
            get_latency = (current_time - timestamp)
            self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(get_latency), log_name_list = self.log_name_list, throttle_s = 5.0)

            # Start Img Pub Process
            start_time = nepi_utils.get_time()   


            # Transform navpose data frames to nepi standard frames
            if np_dict['frame_nav'] != 'ENU':
                if np_dict['frame_nav'] == 'NED':
                    nepi_nav.convert_navpose_ned2enu(np_dict)
            if np_dict['frame_altitude'] != 'WGS84':
                if np_dict['frame_altitude'] == 'AMSL':
                    nepi_nav.convert_navpose_amsl2wgs84(np_dict)
            if np_dict['frame_depth'] != 'MSL':
                if np_dict['frame_depth'] == 'DEPTH':
                    pass # need to add conversions                 

            self.status_msg.frame_nav = np_dict['frame_nav']
            self.status_msg.frame_altitude = np_dict['frame_altitude']
            self.status_msg.frame_depth = np_dict['frame_depth']

            # Publish nav pose subs
            if self.pub_location == True:
                pub_name = self.node_if_prefix + 'location_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_location']
                msg.latitude = np_dict['latitude']
                msg.longitude = np_dict['longitude']
                msg.location_m_per_sec = np_dict['location_m_per_sec']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_heading == True:
                pub_name = self.node_if_prefix + 'heading_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_heading']
                msg.heading_deg = np_dict['heading_deg']
                msg.heading_m_per_sec = np_dict['heading_m_per_sec']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_orientation == True:
                pub_name = self.node_if_prefix + 'orientation_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_orientation']
                msg.roll_deg = np_dict['roll_deg']
                msg.pitch_deg = np_dict['pitch_deg']
                msg.yaw_deg = np_dict['yaw_deg']
                msg.roll_deg_per_sec = np_dict['roll_deg_per_sec']
                msg.pitch_deg_per_sec = np_dict['pitch_deg_per_sec']
                msg.yaw_deg_per_sec = np_dict['yaw_deg_per_sec']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_position == True:
                pub_name = self.node_if_prefix + 'position_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_position']
                msg.x_m = np_dict['x_m']
                msg.y_m = np_dict['y_m']
                msg.z_m = np_dict['z_m']
                msg.x_m_per_sec = np_dict['x_m_per_sec']
                msg.y_m_per_sec = np_dict['y_m_per_sec']
                msg.z_m_per_sec = np_dict['z_m_per_sec']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_altitude == True:
                pub_name = self.node_if_prefix + 'altitude_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_altitude']
                msg.altitude_m = np_dict['altitude_m']
                msg.altitude_m_per_sec = np_dict['altitude_m_per_sec']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_depth == True:
                pub_name = self.node_if_prefix + 'depth_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_depth']
                msg.depth_m = np_dict['depth_m']
                msg.depth_m_per_sec = np_dict['depth_m_per_sec']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_pan_tilt == True:
                pub_name = self.node_if_prefix + 'pan_tilt_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_depth']
                msg.pan_deg = np_dict['pan_deg']
                msg.tilt_deg = np_dict['tilt_deg']
                self.node_if.publish_pub(pub_name,msg)

            # Transform navpose in ENU and WSG84 frames
            if transform is not None:
                np_dict = nepi_nav.transform_navpose_dict(np_dict,transform)

            self.navpose_dict  = np_dict
            # Transform navpose data frames to system set frames
            frame_nav = self.navpose_settings_dict['frame_nav']
            frame_alt = self.navpose_settings_dict['frame_alt']
            frame_depth = self.navpose_settings_dict['frame_depth']
            
            if np_dict['frame_nav'] != frame_nav:
                if np_dict['frame_nav'] == 'NED' and frame_nav == 'ENU':
                    nepi_nav.convert_navpose_ned2enu(np_dict)
                elif np_dict['frame_nav'] == 'ENU' and frame_nav == 'NED':
                    nepi_nav.convert_navpose_enu2ned(np_dict)
            if np_dict['frame_altitude'] != frame_alt:
                if np_dict['frame_altitude'] == 'AMSL' and frame_alt ==  'WGS84':
                    nepi_nav.convert_navpose_amsl2wgs84(np_dict)
                elif np_dict['frame_altitude'] == 'WGS84' and frame_alt ==  'AMSL':
                    nepi_nav.convert_navpose_wgs842amsl(np_dict)
            #if np_dict['frame_depth'] != 'MSL':
            #    if np_dict['frame_depth'] == 'DEPTH':
            #        pass # need to add conversions

            self.status_msg.frame_name = np_dict['navpose_frame']
            self.status_msg.frame_nav = np_dict['frame_nav']
            self.status_msg.frame_altitude = np_dict['frame_altitude']
            self.status_msg.frame_depth = np_dict['frame_depth']

            if self.pub_navpose == True:
                data_msg = None
                try:
                    data_msg = nepi_nav.convert_navpose_dict2msg(np_dict)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to convert navpose data to msg: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
                    success = False

                if data_msg is not None:
                    try:
                        self.node_if.publish_pub(self.node_if_prefix + 'navpose_pub', data_msg)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to publish navpose data msg: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
                        success = False


            current_time = nepi_utils.get_time()
            pub_latency = (current_time - timestamp)
            process_time = (current_time - start_time)
            self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(pub_latency), log_name_list = self.log_name_list, throttle_s = 5.0)

            # Update Pub Stats
            if self.last_pub_time is None:
                pub_time_sec = 1.0
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time


            self.time_list.pop(0)
            self.time_list.append(pub_time_sec)

            if self.save_data_if is not None:
                save_enabled = self.save_data_if.data_product_save_enabled('navpose') == True
                should_save = self.save_data_if.data_product_should_save('navpose') == True
                snapshot_enabled = self.save_data_if.data_product_snapshot_enabled('navpose') == True
                save_navpose = should_save or snapshot_enabled
                time_ns = nepi_utils.get_time()
                key_name = int(math.floor(time_ns * 1000))
                navposes_save_dict = {key_name: np_dict}
                if self.save_data_if is not None and len(list(navposes_save_dict.keys())) > 0 and save_navpose == True:
                    filename = self.save_data_if.save('navposes', navposes_save_dict, timestamp = time_ns, filename = self.save_filename, key_name = key_name)
                    if save_enabled == False:
                        filename = None
                    self.save_filename = filename



        return np_dict

    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this depth map interface."""          
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)

    def register_pubs(self):
        """Re-register all ROS publishers managed by this interface."""
        if self.node_if is not None:
            self.node_if.register_pubs()

    def unsubscribe(self):
        """Shut down this interface, unregister all ROS resources, and clear state."""
        self.ready = False
        if self.node_if is not None:
            self.node_if.unregister_class()
        time.sleep(1)
        self.namespace = None
        self.status_msg = NavPoseStatus()

    def publish_status(self):
        """Compute the current average publish rate and publish the status message."""
        if self.node_if is not None and self.status_msg is not None:

            avg_rate = 0
            if len(self.time_list) > 0:
                avg_time = sum(self.time_list) / len(self.time_list)
                if avg_time > .01:
                    avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate

            self.node_if.publish_pub(self.node_if_prefix + 'navpose_status_pub', self.status_msg)

    def init(self, do_updates = False):
        """Initialize or re-initialize interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the interface to its initialized state."""
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        nepi_sdk.sleep(1)     
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset the interface to factory defaults."""
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


    def _needsDataCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers(self.node_if_prefix + 'navpose_pub')
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
            needs_data = has_subs or needs_save or needs_snapshot
        else:
            needs_data = has_subs
        self.needs_data = needs_data
        #self.msg_if.pub_warn("Needs Data Check End: " + self.namespace + " : " + str([has_subs,needs_save, needs_snapshot]), log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        self.publish_status()

    def _provideCapabilities(self, _):
        return self.caps_report





##################################################
# BaseImageIF

IMAGE_ALL_TOPIC = 'images'

SUPPORTED_DATA_PRODUCTS = ['image','color_image','bw_image',
                            'intensity_map','depth_map','pointcloud']
ENCODING_OPTIONS = ["mono8",'rgb8','bgr8','32FC1','passthrough']

PERSPECTIVE_OPTIONS = ['pov','top']

EXAMPLE_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = False,
        has_flip_horz = False,
        has_flip_vert = False,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_zoom_3d = False,
        has_rotate_3d = False,
        has_tilt_3d = False,
        has_camera_3d = False
    )

EXAMPLE_FILTERS_DICT = dict(
    # Low_Light = {
    #     'enabled': False,
    #     'function': nepi_img.low_light_filter,
    #     'ratio': 0.5
    # }
)

EXAMPLE_CONTROLS_DICT = dict( 
    resolution_ratio = 1.0,
    auto_adjust_enabled = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    start_range_ratio = 0.0,
    stop_range_ratio = 1.0,
    window_ratios = [0,1,0,1],
    rotate_3d_ratio = 0.5,
    tilt_3d_ratio = 0.5       
    )



class BaseImageIF:

    DEFUALT_IMG_WIDTH_PX = 700
    DEFUALT_IMG_HEIGHT_PX = 400

    DEFAULT_WIDTH_DEG = 100
    DEFAULT_HEIGHT_DEG = 70

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = False,
        has_flip_horz = False,
        has_flip_vert = False,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_zoom_3d = False,
        has_rotate_3d = False,
        has_tilt_3d = False,
        has_camera_3d = False
        )

    DEFAULT_FILTERS_DICT = dict()

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        rotate_2d_deg = 0,
        rotate_2d_keep_size = False,
        rotate_2d_swap_box = False,
        flip_horz = False,
        flip_vert = False,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        zoom_3d_ratio = 0,
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5,
        cam_fov = 60,
        cam_view = [3, 0, 0],
        cam_pos = [-5, 0, 0],
        cam_rot = [0, 0, 1]
        )


    DEFAULT_CALLBACK_DICT = dict(
        needs_update_callback = None,
        mouse_event_callback = None,
        click_pixel_callback = None,
        click_angle_callback = None,
        drag_callback = None,
        window_callback = None,
        scroll_callback = None,
        frame_updated_callback = None
    )

    BLANK_CROSSHAIR_DICT = dict(
        x_deg_offset = 0,
        y_deg_offset = 0,
        color_rgb = (0,255,0),
        msg_str = ''
    )

    BLANK_TARGET_DICT = dict(
        x_deg_offset = 0,
        y_deg_offset = 0,
        color_rgb = (0,255,0),
        msg_str = ''
    )

    DEFAULT_OVERLAYS_DICT = dict(
            overlay_text_enabled = False,
            
            overlay_text_size_ratio = 0.5,
            overlay_text_horz_ratio = 0.0,
            overlay_text_vert_ratio = 0.0,
            overlay_text_transparency_ratio = 0.0,
            overlay_text_color_rgb = (0,255,0),
            overlay_text_img_name = False,
            overlay_text_date_time = False,
            overlay_text_nav = False,
            overlay_text_pose = False, 
            init_overlay_text_list = [],
            add_overlay_text_list = [],

            crosshairs_enabled = False,
            crosshairs_size_ratio = 0.5,
            crosshairs_thickness_ratio = 0.5,
            crosshairs_text_ratio = 0.5,
            crosshairs_transparency_ratio = 0.0,
            crosshairs_color_rgb = (0,255,0),
            overlay_crosshair_names = False,
            overlay_crosshair_pixels = False,
            overlay_crosshair_degrees = False,
            overlay_crosshair_messages = False,
            crosshairs_dict = dict(),
            
            targets_enabled = False,
            targets_size_ratio = 0.5,
            targets_thickness_ratio = 0.5,
            targets_text_ratio = 0.5,
            targets_transparency_ratio = 0.0,
            targets_color_rgb = (0,255,0),
            overlay_target_names = False,
            overlay_target_pixels = False,
            overlay_target_degrees = False,
            overlay_target_messages = False,
            targets_dict = dict(),
    )

    callback_dict = copy.deepcopy(DEFAULT_CALLBACK_DICT)

    ready = False
    namespace = '~'

    # Enable state for 3D render controls (mouse drag/window interaction)
    render_3d_controls_enabled = False

    node_if = None
    node_if_shared = True
    save_data_if = None

    status_msg = ImageStatus()
    
    last_width = DEFUALT_IMG_WIDTH_PX
    last_height = DEFUALT_IMG_HEIGHT_PX

    min_range_m = 0
    max_range_m = 0

    perspective = 'pov'

    blank_img = nepi_img.create_cv2_blank_img(last_width, last_height, color_rgb = (0, 0, 0) )

    last_pub_time = None

    needs_data = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    depth_map_topic = ""
    pointcloud_topic = ""

    caps_dict = DEFAULT_CAPS_DICT
    controls_dict = DEFAULT_CONTROLS_DICT
    init_controls_dict = controls_dict

    caps_report = ImageCapabilitiesQueryResponse()


    click_text_enabled = False
    click_crosshair_enabled = False
    click_target_enabled = False
    overlays_dict = copy.deepcopy(DEFAULT_OVERLAYS_DICT) 

    auto_adjust_controls = []
    filter_dict = dict()
    has_filters = False
    filter_options = []
    sel_filters = []

    data_source_description = 'imaging_sensor'
    data_ref_description = 'sensor'

    data_product = 'image'

    save_data_if = None

    perspective = 'pov'

    add_pubs_dict = dict()

    pub_count = 0


    pixel = None
    window = None


    drag_pixel = None
    drag_window = None

    save_config = False


    x_offset = 0
    y_offset = 0
    x_scaler = 1
    y_scaler = 1

    height_org = 0
    width_org = 0

    height_proc = 0
    width_proc = 0



    width_start_deg = 0
    width_stop_deg = DEFAULT_WIDTH_DEG
    height_start_deg = 0
    height_stop_deg = DEFAULT_HEIGHT_DEG
    width_deg = DEFAULT_WIDTH_DEG
    height_deg = DEFAULT_HEIGHT_DEG

    zoom_ratio = 1
    x_ratio = 0.5
    y_ratio = 0.5
    window_ratios = [0,1,0,1]

    publishing = False

    navpose_if = None
    save_data_if = None

    pubs_dict = dict()
    subs_dict = dict()

    active_topics = []
    active_topic_types = []
    active_services = []

    aspect_adjustment_disabled = False
    aspect_adjust_enabled = False
    aspect_ratio_set = 1.78 # 16:9
    aspect_ratio = -1 # Not Known Yet

    live_adjustments_disabled = False
    live_adjust_enabled = True
    live_adjust_dict = dict(
        live_adjust_enabled = True,
        live_adjust_rotate_ratio = 0.5,
        live_adjust_x_ratio = 0.5,
        live_adjust_y_ratio = 0.5
    )

    stream_compression_enabled = False
    stream_compression_ratio = 0.5
    

    def __init__(self, 
                namespace , 
                data_product,
                data_source_description,
                data_ref_description,
                perspective,
                caps_dict,
                controls_dict, 
                filter_dict,
                params_dict,
                services_dict,
                pubs_dict,
                subs_dict,
                save_data_if,
                navpose_if,
                navpose_namespace,
                transform_namespace,
                init_overlay_text_list,
                live_adjustments_disabled,
                aspect_adjustment_disabled,
                log_name,
                log_name_list,
                msg_if,
                node_if
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
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)


        ##############################  
        # Setup All Namespace

        # Collective controls publish on the shared images all_namespace, which fans a
        # single command out to every IDX image.
        all_namespace = nepi_sdk.create_namespace(self.base_namespace, SYSTEM_ALL_TOPIC)
        all_namespace = nepi_sdk.create_namespace(all_namespace, IMAGE_ALL_TOPIC)
        self.all_namespace = all_namespace
        self.msg_if.pub_warn("Using connect all_namespace: " + str(self.all_namespace) )



        ##############################    
        # Initialize Class Variables
        
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product

        if namespace is not None:
            self.namespace = namespace
        if os.path.basename(namespace) != self.data_product:
            namespace = nepi_sdk.create_namespace(namespace,self.data_product)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

          
        if perspective is not None:
            self.perspective = perspective

        # Setup filter dict
        if filter_dict is not None:
            self.filter_dict = filter_dict
            self.filter_options = list(self.filter_dict.keys())
            if len(self.filter_options) > 0:
                self.has_filters = True
        else:
            self.filter_dict = dict()


        self.live_adjustments_disabled = live_adjustments_disabled == True
        #self.msg_if.pub_warn(self.data_product + " Got Live Adjust Disabled: " + str(self.live_adjustments_disabled ) )
        self.aspect_adjustment_disabled = aspect_adjustment_disabled == True
        #self.msg_if.pub_warn(self.data_product + " Got Aspect Adjust Disabled: " + str(self.aspect_adjustment_disabled ) )
        # Create and update capabilities dictionary
        if caps_dict is not None:
            for cap in self.caps_dict.keys():
                if caps_dict.get(cap) != None:
                    self.caps_dict[cap] = caps_dict[cap]





        self.caps_report.has_resolution = self.caps_dict['has_resolution']
        self.caps_report.has_contrast = self.caps_dict['has_contrast']
        self.caps_report.has_brightness = self.caps_dict['has_brightness']
        self.caps_report.has_threshold = self.caps_dict['has_threshold']
        self.caps_report.has_rotate_2d = self.caps_dict['has_rotate_2d']
        self.caps_report.has_flip_horz = self.caps_dict['has_flip_horz']
        self.caps_report.has_flip_vert = self.caps_dict['has_flip_vert']
        self.caps_report.has_range = self.caps_dict['has_range']
        self.caps_report.has_auto_adjust = self.caps_dict['has_auto_adjust']
        self.caps_report.has_zoom = self.caps_dict['has_zoom']
        self.caps_report.has_pan = self.caps_dict['has_pan']
        self.caps_report.has_window = self.caps_dict['has_window']
        self.caps_report.has_zoom_3d = self.caps_dict['has_zoom_3d']
        self.caps_report.has_rotate_3d = self.caps_dict['has_rotate_3d']
        self.caps_report.has_tilt_3d = self.caps_dict['has_tilt_3d']
        self.caps_report.has_camera_3d = self.caps_dict['has_camera_3d']

        self.caps_report.has_filters = self.has_filters
        self.caps_report.filter_options = self.filter_options






        # Give this instance its own controls store before anything writes to it.
        # The class attributes controls_dict and init_controls_dict are both plain
        # references to BaseImageIF.DEFAULT_CONTROLS_DICT, so without this every
        # image IF in the process mutates one shared dict (color, depth map and
        # pointcloud controls bleed into each other), init_controls_dict tracks the
        # live values instead of the baseline (which makes every reset_* a no-op),
        # and the class-level defaults the pointcloud renderer falls back to get
        # overwritten as controls change. Base is the floor because the subclass
        # dicts omit keys Base code reads: PointcloudImageIF has no rotate_2d_deg
        # or flip_horz/flip_vert, so seeding from the subclass alone would KeyError.
        merged_controls_dict = copy.deepcopy(BaseImageIF.DEFAULT_CONTROLS_DICT)
        merged_controls_dict.update(copy.deepcopy(self.DEFAULT_CONTROLS_DICT))
        self.controls_dict = merged_controls_dict

        # Create and update controls dictionary
        if controls_dict is not None:
            for control in self.controls_dict.keys():
                if controls_dict.get(control) != None:
                    self.controls_dict[control] = controls_dict[control]
        self.init_controls_dict = copy.deepcopy(self.controls_dict)

 
        self.overlays_dict['init_overlay_text_list'] = init_overlay_text_list

 
        # Initialize Status Msg.  Updated on each publish

        if data_source_description is None:
            data_source_description = self.data_source_description
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.data_ref_description
        self.data_ref_description = data_ref_description

        self.status_msg.node_name = self.node_name
        self.status_msg.navpose_topic = navpose_namespace if navpose_namespace is not None else ''
        self.status_msg.transform_topic = transform_namespace if transform_namespace is not None else ''

        # Published Data Info
        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description
        self.status_msg.encoding = 'bgr8'
        self.status_msg.width_px = 0
        self.status_msg.height_px = 0

        self.status_msg.width_start_deg = 0
        self.status_msg.width_stop_deg = 0
        self.status_msg.height_start_deg = 0
        self.status_msg.height_stop_deg = 0
        self.status_msg.width_deg = 0
        self.status_msg.height_deg = 0


        self.status_msg.perspective = self.perspective
        self.status_msg.auto_adjust_controls = self.auto_adjust_controls
        self.status_msg.get_latency_time = 0
        self.status_msg.pub_latency_time = 0
        self.status_msg.process_time = 0



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
        self.PARAMS_DICT = {

            self.node_if_prefix + 'resolution_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["resolution_ratio"]
            },
            self.node_if_prefix + 'auto_adjust_enabled': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["auto_adjust_enabled"]
            },
            self.node_if_prefix + 'auto_adjust_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["auto_adjust_ratio"]
            },
            self.node_if_prefix + 'brightness_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["brightness_ratio"]
            },
            self.node_if_prefix + 'contrast_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["contrast_ratio"]
            },
            self.node_if_prefix + 'threshold_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["threshold_ratio"]
            },
            self.node_if_prefix + 'rotate_2d_deg': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["rotate_2d_deg"]
            },
            self.node_if_prefix + 'rotate_2d_swap_box': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["rotate_2d_swap_box"]
            },
            self.node_if_prefix + 'flip_horz': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["flip_horz"]
            },
            self.node_if_prefix + 'flip_vert': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["flip_vert"]
            },
            self.node_if_prefix + 'start_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["start_range_ratio"]
            },
            self.node_if_prefix + 'cam_fov': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict['cam_fov']
            },
            self.node_if_prefix + 'cam_view': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict['cam_view']
            },
            self.node_if_prefix + 'cam_pos': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict['cam_pos']
            },
            self.node_if_prefix + 'cam_rot': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict['cam_rot']
            },
           self.node_if_prefix +   'stop_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["stop_range_ratio"]
            },
            # The three 3-D render ratios and the render background sit beside
            # cam_fov/cam_view/cam_pos/cam_rot in controls_dict and are resolved by
            # the same renderer, but had no keys here, so set_zoom_3d_ratio,
            # set_rotate_3d_ratio, set_tilt_3d_ratio and setWhiteBgCb all wrote into
            # a name node_if did not know and the writes were dropped silently.
            # use_wbg is only in PointcloudImageIF's DEFAULT_CONTROLS_DICT, hence the
            # get() default here.
            self.node_if_prefix + 'zoom_3d_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict['zoom_3d_ratio']
            },
            self.node_if_prefix + 'rotate_3d_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict['rotate_3d_ratio']
            },
            self.node_if_prefix + 'tilt_3d_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict['tilt_3d_ratio']
            },
            self.node_if_prefix + 'use_wbg': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict.get('use_wbg', False)
            },
            self.node_if_prefix + 'live_adjust_enabled': {
                'namespace': self.namespace,
                'factory_val': self.live_adjust_enabled
            },
            self.node_if_prefix + 'aspect_adjust_enabled': {
                'namespace': self.namespace,
                'factory_val': self.aspect_adjust_enabled
            },
            self.node_if_prefix +  'aspect_ratio_set': {
                'namespace': self.namespace,
                'factory_val': self.aspect_ratio_set
            },
            self.node_if_prefix + 'filter_dict': {
                'namespace': self.namespace,
                'factory_val': self.filter_dict
            },
            self.node_if_prefix + 'overlays_dict': {
                'namespace': self.namespace,
                'factory_val': self.overlays_dict
            },
            self.node_if_prefix + 'stream_compression_enabled': {
                'namespace': self.namespace,
                'factory_val': self.stream_compression_enabled
            },
            self.node_if_prefix + 'stream_compression_ratio': {
                'namespace': self.namespace,
                'factory_val': self.stream_compression_ratio
            },

        }

        if params_dict is not None:
            self.PARAMS_DICT = params_dict | self.PARAMS_DICT
        else:
            self.PARAMS_DICT = self.PARAMS_DICT

        # Services Config Dict ####################     
        self.SRVS_DICT = {
            'image_caps_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': ImageCapabilitiesQuery,
                'req': ImageCapabilitiesQueryRequest(),
                'resp': ImageCapabilitiesQueryResponse(),
                'callback': self._provideCapabilities
            }
        }

        if services_dict is not None:
            self.SRVS_DICT = services_dict | self.SRVS_DICT
        else:
            self.SRVS_DICT = self.SRVS_DICT
        

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'data_pub': {
                'msg': Image,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            # NOTE: self.node_if_prefix + 'data_pub'/'status_pub' keys are generic and identical across every
            # image-IF instance. Safe only while this IF owns its own node_if. If a shared
            # node_if is ever passed in, make these keys namespace-unique first or coexisting
            # instances (and the parent device) will clobber each other (see CLAUDE.md decision log).
            self.node_if_prefix + 'status_pub': {
                'msg': ImageStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

    

        if pubs_dict is not None:
            self.PUBS_DICT = pubs_dict | self.PUBS_DICT
        else:
            self.PUBS_DICT = self.PUBS_DICT        

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            self.node_if_prefix + 'reset_all': {
                'namespace': self.namespace,
                'topic': 'reset',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetControlsCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'reset_filters': {
                'namespace': self.namespace,
                'topic': 'reset_filters',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetFiltersCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'reset_overlays': {
                'namespace': self.namespace,
                'topic': 'reset_overlays',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetOverlaysCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'reset_settings': {
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetSettingsCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'reset_renders': {
                'namespace': self.namespace,
                'topic': 'reset_renders',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetRendersCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'mouse_event': {
                'namespace': self.namespace,
                'topic': 'mouse_event',
                'msg': ImageMouseEvent,
                'qsize': 5,
                'callback': self._mouseEventCb,
                'callback_args': ()
            },
        
            self.node_if_prefix + 'render_3d_controls': {
                'namespace': self.namespace,
                'topic': 'render_3d_controls',
                'msg': Bool,
                'qsize': 10,
                'callback': self.render3dControlsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'reset_render_3d_controls': {
                'namespace': self.namespace,
                'topic': 'reset_render_3d_controls',
                'msg': Empty,
                'qsize': 10,
                'callback': self.resetRender3dControlsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'reset_render_3d_position': {
                'namespace': self.namespace,
                'topic': 'reset_render_3d_position',
                'msg': Empty,
                'qsize': 10,
                'callback': self.resetRender3dPositionCb,
                'callback_args': ()
            },

            ######################
            self.node_if_prefix + 'overlay_text_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_enable',
                'qsize': 5,
                'callback': self._setrOverlayTextEnableCb
            },
            self.node_if_prefix + 'click_text_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'click_text_enable',
                'qsize': 5,
                'callback': self._clickTextEnableCb
            },
            self.node_if_prefix + 'overlay_text_size_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_size_ratio',
                'qsize': 5,
                'callback': self._setOverlaySizeCb
            },
            self.node_if_prefix + 'overlay_text_vert_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_vert_ratio',
                'qsize': 5,
                'callback': self._setOverlayVertCb
            },
            self.node_if_prefix + 'overlay_text_horz_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_horz_ratio',
                'qsize': 5,
                'callback': self._setOverlayHorzCb
            },
            self.node_if_prefix + 'overlay_text_transparency_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_transparency_ratio',
                'qsize': 5,
                'callback': self._setOverlayTransparencyCb
            },
            self.node_if_prefix + 'overlay_text_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_color_rgb',
                'qsize': 5,
                'callback': self._setOverlayColorRGBCb
            },
            self.node_if_prefix + 'overlay_text_img_name': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_source_name',
                'qsize': 5,
                'callback': self._setOverlayImgNameCb
            },
            self.node_if_prefix + 'overlay_text_date_time': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_date_time',
                'qsize': 5,
                'callback': self._setOverlayDateTimeCb
            },
            self.node_if_prefix + 'overlay_text_nav': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_nav',
                'qsize': 5,
                'callback': self._setOverlayNavCb
            },
            self.node_if_prefix + 'overlay_text_pose': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_pose',
                'qsize': 5,
                'callback': self._setOverlayPoseCb
            },
            self.node_if_prefix + 'add_overlay_text': {
                'msg': String,
                'namespace': self.namespace,
                'topic': 'add_overlay_text',
                'qsize': 5,
                'callback': self._setOverlayTextCb
            },
            self.node_if_prefix + 'set_overlay_text_list': {
                'msg': StringArray,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_list',
                'qsize': 5,
                'callback': self._setOverlayListCb
            },
            self.node_if_prefix + 'overlay_text_clear': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'clear_overlay_text_list',
                'qsize': 5,
                'callback': self._clearOverlayListCb
            },

            #######################################
            self.node_if_prefix + 'crosshairs_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'crosshairs_enable',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairsCb
            },
            self.node_if_prefix + 'crosshairs_size_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_crosshairs_size_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsSizeRatioCb
            },
            self.node_if_prefix + 'crosshairs_thickness_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_crosshairs_thickness_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsThicknessRatioCb
            },
            self.node_if_prefix + 'crosshairs_text_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_crosshairs_text_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsTextRatioCb
            },
            self.node_if_prefix + 'crosshairs_transparency_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_crosshairs_transparency_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsTransparencyRatioCb
            },
            self.node_if_prefix + 'crosshairs_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.namespace,
                'topic': 'set_crosshairs_color_rgb',
                'qsize': 5,
                'callback': self._setCrosshairsColorRGBCb
            },

            self.node_if_prefix + 'overlay_crosshair_names': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_crosshair_names',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairNamesCb
            },
            self.node_if_prefix + 'overlay_crosshair_pixels': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_crosshair_pixels',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairPixelsCb
            },
            self.node_if_prefix + 'overlay_crosshair_degrees': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_crosshair_degrees',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairDegreesCb
            },
            self.node_if_prefix + 'overlay_crosshair_messages': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_crosshair_messages',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairMessagesCb
            },
            self.node_if_prefix + 'click_crosshair_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'click_crosshair_enable',
                'qsize': 5,
                'callback': self._clickCrosshairEnableCb
            },
            self.node_if_prefix + 'add_crosshair_pixel': {
                'msg': ImageCrosshair,
                'namespace': self.namespace,
                'topic': 'add_crosshair_pixel',
                'qsize': 5,
                'callback': self._addCrosshairPixelCb
            },
            self.node_if_prefix + 'add_crosshair_ratios': {
                'msg': ImageCrosshair,
                'namespace': self.namespace,
                'topic': 'add_crosshair_ratios',
                'qsize': 5,
                'callback': self._addCrosshairRatiosCb
            },
            self.node_if_prefix + 'add_crosshair_degree_offsets': {
                'msg': ImageCrosshair,
                'namespace': self.namespace,
                'topic': 'add_crosshair_degree_offsets',
                'qsize': 5,
                'callback': self._addCrosshairDegreesCb
            },
            self.node_if_prefix + 'remove_crosshair': {
                'msg': String,
                'namespace': self.namespace,
                'topic': 'remove_crosshair',
                'qsize': 5,
                'callback': self._removeCrosshairCb
            },
            self.node_if_prefix + 'clear_crosshairs': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'clear_crosshairs',
                'qsize': 5,
                'callback': self._clearCrosshairsCb
            },
            self.node_if_prefix + 'set_live_adjust_rotate_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_rotate_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustRotateRatioCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_rotate_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_rotate_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustRotateDegCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_x_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXRatioCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_x_pixel': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_pixel',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXPixelCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_x_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXDegCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_y_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYRatioCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_y_pixel': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_pixel',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYPixelCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_y_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYDegCb,
                'callback_args': ()
            },

            ############################

            self.node_if_prefix + 'targets_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'targets_enable',
                'qsize': 5,
                'callback': self._setrOverlayTargetsCb
            },
            self.node_if_prefix + 'targets_size_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_targets_size_ratio',
                'qsize': 5,
                'callback': self._setTargetsSizeRatioCb
            },
            self.node_if_prefix + 'targets_thickness_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_targets_thickness_ratio',
                'qsize': 5,
                'callback': self._setTargetsThicknessRatioCb
            },
            self.node_if_prefix + 'targets_text_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_targets_text_ratio',
                'qsize': 5,
                'callback': self._setTargetsTextRatioCb
            },
            self.node_if_prefix + 'targets_transparency_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_targets_transparency_ratio',
                'qsize': 5,
                'callback': self._setTargetsTransparencyRatioCb
            },
            self.node_if_prefix + 'targets_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.namespace,
                'topic': 'set_targets_color_rgb',
                'qsize': 5,
                'callback': self._setTargetsColorRGBCb
            },

            self.node_if_prefix + 'overlay_target_names': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_target_names',
                'qsize': 5,
                'callback': self._setrOverlayTargetNamesCb
            },
            self.node_if_prefix + 'overlay_target_pixels': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_target_pixels',
                'qsize': 5,
                'callback': self._setrOverlayTargetPixelsCb
            },
            self.node_if_prefix + 'overlay_target_degrees': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_target_degrees',
                'qsize': 5,
                'callback': self._setrOverlayTargetDegreesCb
            },
            self.node_if_prefix + 'overlay_target_messages': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_target_messages',
                'qsize': 5,
                'callback': self._setrOverlayTargetMessagesCb
            },
            self.node_if_prefix + 'click_target_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'click_target_enable',
                'qsize': 5,
                'callback': self._clickTargetEnableCb
            },
            self.node_if_prefix + 'add_target_pixel': {
                'msg': ImageTarget,
                'namespace': self.namespace,
                'topic': 'add_target_pixel',
                'qsize': 5,
                'callback': self._addTargetPixelCb
            },
            self.node_if_prefix + 'add_target_ratios': {
                'msg': ImageTarget,
                'namespace': self.namespace,
                'topic': 'add_target_ratios',
                'qsize': 5,
                'callback': self._addTargetRatiosCb
            },
            self.node_if_prefix + 'add_target_degree_offsets': {
                'msg': ImageTarget,
                'namespace': self.namespace,
                'topic': 'add_target_degree_offsets',
                'qsize': 5,
                'callback': self._addTargetDegreesCb
            },
            self.node_if_prefix + 'remove_target': {
                'msg': String,
                'namespace': self.namespace,
                'topic': 'remove_target',
                'qsize': 5,
                'callback': self._removeTargetCb
            },
            self.node_if_prefix + 'clear_targets': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'clear_targets',
                'qsize': 5,
                'callback': self._clearTargetsCb
            },

            ####################################
            self.node_if_prefix + 'set_aspect_adjust_enable': {
                'namespace': self.namespace,
                'topic': 'set_aspect_adjust_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setAspectAdjustEnableCb,
                'callback_args': ()
            },
              self.node_if_prefix + 'set_aspect_adjust_ratio': {
                'namespace': self.namespace,
                'topic': 'set_aspect_adjust_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setAspectAdjustRatioCb,
                'callback_args': ()
            },
              self.node_if_prefix + 'set_aspect_adjust_by_ratio': {
                'namespace': self.namespace,
                'topic': 'set_aspect_adjust_by_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setAspectAdjustByRatioCb,
                'callback_args': ()
            },
            ####################################
            self.node_if_prefix + 'set_stream_compression_enable': {
                'namespace': self.namespace,
                'topic': 'set_stream_compression_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setStreamCompressionEnableCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_stream_compression_ratio': {
                'namespace': self.namespace,
                'topic': 'set_stream_compression_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setStreamCompressionRatioCb,
                'callback_args': ()
            },
            #########################
            self.node_if_prefix + 'set_live_adjust_enable': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setLiveAdjustEnableCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_rotate_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_rotate_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustRotateRatioCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_rotate_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_rotate_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustRotateDegCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_x_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXRatioCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_x_pixel': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_pixel',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXPixelCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_x_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXDegCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_y_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYRatioCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_y_pixel': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_pixel',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYPixelCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_live_adjust_y_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYDegCb,
                'callback_args': ()
            },
            ############################
            self.node_if_prefix + 'all_overlay_text_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_enable',
                'qsize': 5,
                'callback': self._setrOverlayTextEnableCb
            },
            self.node_if_prefix + 'all_click_text_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'click_text_enable',
                'qsize': 5,
                'callback': self._clickTextEnableCb
            },

            self.node_if_prefix + 'all_overlay_text_size_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_size_ratio',
                'qsize': 5,
                'callback': self._setOverlaySizeCb
            },
            self.node_if_prefix + 'all_overlay_text_vert_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_vert_ratio',
                'qsize': 5,
                'callback': self._setOverlayVertCb
            },
            self.node_if_prefix + 'all_overlay_text_horz_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_horz_ratio',
                'qsize': 5,
                'callback': self._setOverlayHorzCb
            },
            self.node_if_prefix + 'all_overlay_text_transparency_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_transparency_ratio',
                'qsize': 5,
                'callback': self._setOverlayTransparencyCb
            },
            self.node_if_prefix + 'all_overlay_text_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_color_rgb',
                'qsize': 5,
                'callback': self._setOverlayColorRGBCb
            },
            self.node_if_prefix + 'all_overlay_text_img_name': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_source_name',
                'qsize': 5,
                'callback': self._setOverlayImgNameCb
            },
            self.node_if_prefix + 'all_overlay_text_date_time': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_date_time',
                'qsize': 5,
                'callback': self._setOverlayDateTimeCb
            },
            self.node_if_prefix + 'all_overlay_text_nav': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_nav',
                'qsize': 5,
                'callback': self._setOverlayNavCb
            },
            self.node_if_prefix + 'all_overlay_text_pose': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_pose',
                'qsize': 5,
                'callback': self._setOverlayPoseCb
            },
            self.node_if_prefix + 'all_add_overlay_text': {
                'msg': String,
                'namespace': self.all_namespace,
                'topic': 'add_overlay_text',
                'qsize': 5,
                'callback': self._setOverlayTextCb
            },
            self.node_if_prefix + 'all_set_overlay_text_list': {
                'msg': StringArray,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_list',
                'qsize': 5,
                'callback': self._setOverlayListCb
            },
            self.node_if_prefix + 'all_overlay_text_clear': {
                'msg': Empty,
                'namespace': self.all_namespace,
                'topic': 'clear_overlay_text_list',
                'qsize': 5,
                'callback': self._clearOverlayListCb
            },
            #############################
            self.node_if_prefix + 'all_crosshairs_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'crosshairs_enable',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairsCb
            },
            self.node_if_prefix + 'all_crosshairs_size_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_size_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsSizeRatioCb
            },
            self.node_if_prefix + 'all_crosshairs_thickness_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_thickness_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsThicknessRatioCb
            },
            self.node_if_prefix + 'all_crosshairs_text_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_text_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsTextRatioCb
            },
            self.node_if_prefix + 'all_crosshairs_transparency_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_transparency_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsTransparencyRatioCb
            },
            self.node_if_prefix + 'all_crosshairs_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_color_rgb',
                'qsize': 5,
                'callback': self._setCrosshairsColorRGBCb
            },
            self.node_if_prefix + 'all_overlay_crosshair_names': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_names',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairNamesCb
            },
            self.node_if_prefix + 'all_overlay_crosshair_pixels': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_pixels',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairPixelsCb
            },
            self.node_if_prefix + 'all_overlay_crosshair_degrees': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_degrees',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairDegreesCb
            },
            self.node_if_prefix + 'all_overlay_crosshair_messages': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_messages',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairMessagesCb
            },
            self.node_if_prefix + 'all_click_crosshair_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'click_crosshair_enable',
                'qsize': 5,
                'callback': self._clickCrosshairEnableCb
            },
            self.node_if_prefix + 'all_add_crosshair_pixel': {
                'msg': ImageCrosshair,
                'namespace': self.all_namespace,
                'topic': 'add_crosshair_pixel',
                'qsize': 5,
                'callback': self._addCrosshairPixelCb
            },
            self.node_if_prefix + 'all_add_crosshair_ratios': {
                'msg': ImageCrosshair,
                'namespace': self.all_namespace,
                'topic': 'add_crosshair_ratios',
                'qsize': 5,
                'callback': self._addCrosshairRatiosCb
            },
            self.node_if_prefix + 'all_add_crosshair_degree_offsets': {
                'msg': ImageCrosshair,
                'namespace': self.all_namespace,
                'topic': 'add_crosshair_degree_offsets',
                'qsize': 5,
                'callback': self._addCrosshairDegreesCb
            },
            self.node_if_prefix + 'all_remove_crosshair': {
                'msg': String,
                'namespace': self.all_namespace,
                'topic': 'remove_crosshair',
                'qsize': 5,
                'callback': self._removeCrosshairCb
            },
            self.node_if_prefix + 'all_clear_crosshairs': {
                'msg': Empty,
                'namespace': self.all_namespace,
                'topic': 'clear_crosshairs',
                'qsize': 5,
                'callback': self._clearCrosshairsCb
            },
            #############################
            self.node_if_prefix + 'all_targets_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'targets_enable',
                'qsize': 5,
                'callback': self._setrOverlayTargetsCb
            },
            self.node_if_prefix + 'all_targets_size_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_targets_size_ratio',
                'qsize': 5,
                'callback': self._setTargetsSizeRatioCb
            },
            self.node_if_prefix + 'all_targets_thickness_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_targets_thickness_ratio',
                'qsize': 5,
                'callback': self._setTargetsThicknessRatioCb
            },
            self.node_if_prefix + 'all_targets_text_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_targets_text_ratio',
                'qsize': 5,
                'callback': self._setTargetsTextRatioCb
            },
            self.node_if_prefix + 'all_targets_transparency_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_targets_transparency_ratio',
                'qsize': 5,
                'callback': self._setTargetsTransparencyRatioCb
            },
            self.node_if_prefix + 'all_targets_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.all_namespace,
                'topic': 'set_targets_color_rgb',
                'qsize': 5,
                'callback': self._setTargetsColorRGBCb
            },
            self.node_if_prefix + 'all_overlay_target_names': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_target_names',
                'qsize': 5,
                'callback': self._setrOverlayTargetNamesCb
            },
            self.node_if_prefix + 'all_overlay_target_pixels': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_target_pixels',
                'qsize': 5,
                'callback': self._setrOverlayTargetPixelsCb
            },
            self.node_if_prefix + 'all_overlay_target_degrees': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_target_degrees',
                'qsize': 5,
                'callback': self._setrOverlayTargetDegreesCb
            },
            self.node_if_prefix + 'all_overlay_target_messages': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_target_messages',
                'qsize': 5,
                'callback': self._setrOverlayTargetMessagesCb
            },
            self.node_if_prefix + 'all_click_target_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'click_target_enable',
                'qsize': 5,
                'callback': self._clickTargetEnableCb
            },
            self.node_if_prefix + 'all_add_target_pixel': {
                'msg': ImageTarget,
                'namespace': self.all_namespace,
                'topic': 'add_target_pixel',
                'qsize': 5,
                'callback': self._addTargetPixelCb
            },
            self.node_if_prefix + 'all_add_target_ratios': {
                'msg': ImageTarget,
                'namespace': self.all_namespace,
                'topic': 'add_target_ratios',
                'qsize': 5,
                'callback': self._addTargetRatiosCb
            },
            self.node_if_prefix + 'all_add_target_degree_offsets': {
                'msg': ImageTarget,
                'namespace': self.all_namespace,
                'topic': 'add_target_degree_offsets',
                'qsize': 5,
                'callback': self._addTargetDegreesCb
            },
            self.node_if_prefix + 'all_remove_target': {
                'msg': String,
                'namespace': self.all_namespace,
                'topic': 'remove_target',
                'qsize': 5,
                'callback': self._removeTargetCb
            },
            self.node_if_prefix + 'all_clear_targets': {
                'msg': Empty,
                'namespace': self.all_namespace,
                'topic': 'clear_targets',
                'qsize': 5,
                'callback': self._clearTargetsCb
            },
            ####################################
            self.node_if_prefix + 'all_set_aspect_adjust_enable': {
                'namespace': self.all_namespace,
                'topic': 'set_aspect_adjust_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setAspectAdjustEnableCb,
                'callback_args': ()
            },
              self.node_if_prefix + 'all_set_aspect_adjust_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_aspect_adjust_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setAspectAdjustRatioCb,
                'callback_args': ()
            },
              self.node_if_prefix + 'all_set_aspect_adjust_by_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_aspect_adjust_by_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setAspectAdjustByRatioCb,
                'callback_args': ()
            },
            ####################################
            self.node_if_prefix + 'all_set_stream_compression_enable': {
                'namespace': self.all_namespace,
                'topic': 'set_stream_compression_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setStreamCompressionEnableCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_stream_compression_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_stream_compression_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setStreamCompressionRatioCb,
                'callback_args': ()
            },
            ####################################
            self.node_if_prefix + 'all_set_live_adjust_enable': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setLiveAdjustEnableCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_live_adjust_rotate_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_rotate_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustRotateRatioCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_live_adjust_rotate_deg': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_rotate_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustRotateDegCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_live_adjust_x_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_x_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXRatioCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_live_adjust_x_pixel': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_x_pixel',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXPixelCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_live_adjust_x_deg': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_x_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXDegCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_live_adjust_y_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_y_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYRatioCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_live_adjust_y_pixel': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_y_pixel',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYPixelCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_live_adjust_y_deg': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_y_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYDegCb,
                'callback_args': ()
            },

            self.node_if_prefix + 'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self._systemStatusCb
            }
        }



        # Create subs if required
        if caps_dict['has_resolution'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_resolution'] = {
                'namespace': self.namespace,
                'topic': 'set_resolution_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setResolutionRatioCb, 
                'callback_args': ()
            }

        if caps_dict['has_auto_adjust'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_auto_adjust'] = {
                'namespace': self.namespace,
                'topic': 'set_auto_adjust_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setAutoAdjustCb, 
                'callback_args': ()
            }
            self.SUBS_DICT[self.node_if_prefix + 'set_auto_adjust_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_auto_adjust_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setAutoAdjustRatioCb, 
                'callback_args': ()
            }
        if caps_dict['has_brightness'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_brightness'] = {
                'namespace': self.namespace,
                'topic': 'set_brightness_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setBrightnessCb, 
                'callback_args': ()
            }
        if caps_dict['has_contrast'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_contrast'] = {
                'namespace': self.namespace,
                'topic': 'set_contrast_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setContrastCb, 
                'callback_args': ()
            }
        if caps_dict['has_threshold'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_thresholding'] = {
                'namespace': self.namespace,
                'topic': 'set_threshold_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setThresholdingCb, 
                'callback_args': ()
            }
        if caps_dict['has_rotate_2d'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'rotate_2d'] = {
                'namespace': self.namespace,
                'topic': 'rotate_2d',
                'msg': Empty,
                'qsize': 5,
                'callback': self._setRotate2dCb,
                'callback_args': ()
            }
            self.SUBS_DICT[self.node_if_prefix + 'set_rotate_2d_deg'] = {
                'namespace': self.namespace,
                'topic': 'set_rotate_2d_deg',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setRotate2dDegCb,
                'callback_args': ()
            }
            self.SUBS_DICT[self.node_if_prefix + 'set_rotate_2d_swap_box'] = {
                'namespace': self.namespace,
                'topic': 'set_rotate_2d_swap_box',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setRotate2dSwapBoxCb,
                'callback_args': ()
            }
        if caps_dict['has_flip_horz'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_flip_horz'] = {
                'namespace': self.namespace,
                'topic': 'set_flip_horz',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setFlipHorzCb, 
                'callback_args': ()
            }
        if caps_dict['has_flip_vert'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_flip_vert'] = {
                'namespace': self.namespace,
                'topic': 'set_flip_vert',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setFlipVertCb, 
                'callback_args': ()
            }
        if caps_dict['has_range'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_range'] = {
                'namespace': self.namespace,
                'topic': 'set_range_ratios',
                'msg': RangeWindow,
                'qsize': 5,
                'callback': self._setRangeCb, 
                'callback_args': ()
            }
            # self.SUBS_DICT[self.node_if_prefix + 'set_range_ratios'] = {
            #     'namespace': self.node_namespace,
            #     'topic': 'set_range_ratios',
            #     'msg': RangeWindow,
            #     'qsize': 10,
            #     'callback': self.setRangeRatiosCb,
            #     'callback_args': ()setRangeRatiosCb
            # },
        if caps_dict['has_zoom'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_zoom'] = {
                'namespace': self.namespace,
                'topic': 'set_zoom_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setZoomCb, 
                'callback_args': ()
            }
        if caps_dict['has_pan'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_pan_x_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_pan_x_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setPanXCb, 
                'callback_args': ()
            }
            self.SUBS_DICT[self.node_if_prefix + 'set_pan_y_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_pan_y_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setPanYCb, 
                'callback_args': ()
            }

        if caps_dict['has_zoom_3d'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_zoom_3d'] = {
                'namespace': self.namespace,
                'topic': 'set_zoom_3d_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setZoom3DCb, 
                'callback_args': ()
            }

        if caps_dict['has_rotate_3d'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_rotate_3d'] = {
                'namespace': self.namespace,
                'topic': 'set_rotate_3d_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setRotate3DCb, 
                'callback_args': ()
            }

        if caps_dict['has_tilt_3d'] == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_tilt_3d'] = {
                'namespace': self.namespace,
                'topic': 'set_tilt_3d_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setTilt3DCb, 
                'callback_args': ()
            }




            if caps_dict['has_tilt_3d'] == True:
                self.SUBS_DICT[self.node_if_prefix + 'set_camera_fov'] = {
                    'namespace': self.namespace,
                    'topic': 'set_camera_fov',
                    'msg': Int32,
                    'qsize': 10,
                    'callback': self.setCamFovCb,
                    'callback_args': ()
                }
                self.SUBS_DICT[self.node_if_prefix + 'set_camera_view'] = {
                    'namespace': self.namespace,
                    'topic': 'set_camera_view',
                    'msg': Vector3,
                    'qsize': 10,
                    'callback': self.setCamViewCb,
                    'callback_args': ()
                }
                self.SUBS_DICT[self.node_if_prefix + 'set_camera_position'] = {
                    'namespace': self.namespace,
                    'topic': 'set_camera_position',
                    'msg': Vector3,
                    'qsize': 10,
                    'callback': self.setCamPositionCb,
                    'callback_args': ()
                }
                self.SUBS_DICT[self.node_if_prefix + 'set_camera_rotation'] = {
                    'namespace': self.namespace,
                    'topic': 'set_camera_rotation',
                    'msg': Vector3,
                    'qsize': 10,
                    'callback': self.setCamRotationCb,
                    'callback_args': ()
                }
                self.SUBS_DICT[self.node_if_prefix + 'set_white_bg_enable'] = {
                    'namespace': self.namespace,
                    'topic': 'set_white_bg_enable',
                    'msg': Bool,
                    'qsize': 10,
                    'callback': self.setWhiteBgCb,
                    'callback_args': ()
                }

        if self.has_filters == True:
            self.SUBS_DICT[self.node_if_prefix + 'set_filter_enable'] = {
                'namespace': self.namespace,
                'topic': 'set_filter_enable',
                'msg': UpdateBool,
                'qsize': 5,
                'callback': self._setFilterEnableCb, 
                'callback_args': ()
            }
            self.SUBS_DICT[self.node_if_prefix + 'set_filter_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_filter_ratio',
                'msg': UpdateFloat,
                'qsize': 5,
                'callback': self._setFilterRatioCb, 
                'callback_args': ()
            }



        if subs_dict is not None:
            self.SUBS_DICT = subs_dict | self.SUBS_DICT
        else:
            self.SUBS_DICT = self.SUBS_DICT     

        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            if self.PARAMS_DICT is not None:
               self.node_if.add_params(self.PARAMS_DICT) 
            self.node_if.register_services(self.SRVS_DICT)
            self.node_if.register_pubs(self.PUBS_DICT)
            self.node_if.register_subs(self.SUBS_DICT)

        else:
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

        ####################
        self.msg_if.pub_info("####################", log_name_list = self.log_name_list)
        self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
        if save_data_if is not None and save_data_if != 'None':
            self.save_data_if = save_data_if
            data_products = self.save_data_if.get_data_products()
            if self.data_product not in data_products:
                self.save_data_if.register_data_product(self.data_product)
                if self.data_product == 'color_image':
                    self.save_data_if.set_save_rate(self.data_product,1)
            
        elif save_data_if != 'None':
            
            # Setup Save Data IF Class 
            self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
            factory_data_rates= dict()
            factory_data_rates[self.data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
            if self.data_product == 'color_image':
                factory_data_rates[self.data_product] = [1.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
            factory_filename_dict = {
                'prefix': "", 
                'add_timestamp': True, 
                'add_ms': True,
                'add_us': False,
                'suffix': "",
                'add_node_name': True
                }

            sd_namespace = self.node_namespace
            self.save_data_if = SaveDataIF(namespace = sd_namespace,
                                    data_products = [self.data_product],
                                    factory_rate_dict = factory_data_rates,
                                    factory_filename_dict = factory_filename_dict,
                                    log_name_list = self.log_name_list,
                                    msg_if = self.msg_if,
                                        node_if = self.node_if
                                    )
            nepi_sdk.sleep(1)

        if self.save_data_if is not None:
            self.status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic), log_name_list = self.log_name_list)


        ####################
        if navpose_if is not None:
            self.navpose_if = navpose_if
        else:
            # Setup NavPose Connect IF Class
            self.msg_if.pub_info("Starting NavPose IF Initialization")
            np_namespace = self.namespace + '/navpose'
            if navpose_namespace is not None:
                np_namespace = navpose_namespace

            self.navpose_if = ConnectNavPoseIF(namespace = np_namespace,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if)

        if self.navpose_if is not None:
            navpose_topic = self.navpose_if.get_namespace()
            navpose_namespace = navpose_topic
            self.status_msg.navpose_topic = navpose_topic
            self.msg_if.pub_info("Using navpose namespace: " + str(navpose_topic))

        ##############################
        # Start Node Processes
        self.msg_if.pub_warn("Staring subscribers check process", log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)
        self.msg_if.pub_warn("Staring updater process", log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
        self.msg_if.pub_warn("Starting status publisher process", log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)



        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the image interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
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
        """Return the ROS namespace for this image interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace

    def add_pub_namespace(self,namespace):
        """Register an additional image publisher under the given namespace.

        Creates a new image publisher and a companion status publisher under
        a sub-namespace derived from the supplied namespace and the data product
        name. No-ops if the namespace is already registered or equals the
        primary namespace.

        Args:
            namespace (str): The base ROS namespace to add a publisher for.

        Returns:
            bool: True if the publisher was successfully added, False otherwise.
        """
        success = False
        if namespace is None:
            return success
        namespace = nepi_sdk.get_full_namespace(namespace)
        img_ns = nepi_sdk.create_namespace(namespace,self.data_product)
        if img_ns == self.namespace or namespace in self.add_pubs_dict.keys():
            self.msg_if.pub_warn("Image pub namespace allready registered: " + str(namespace), log_name_list = self.log_name_list)
            return success
        else:
            self.msg_if.pub_warn("Adding image pub namespace: " + str(self.namespace), log_name_list = self.log_name_list)
            img_pub_dict ={
                    'msg': Image,
                    'namespace': img_ns,
                    'topic': '',
                    'qsize': 1,
                    'latch': False
                }
            self.node_if.register_pub(img_ns,img_pub_dict)

            status_ns = nepi_sdk.create_namespace(img_ns,'status')
            status_pub_dict ={
                    'msg': ImageStatus,
                    'namespace': status_ns,
                    'topic': '',
                    'qsize': 1,
                    'latch': False
                }
            self.node_if.register_pub(status_ns,status_pub_dict)

        return success

    def remove_pub_namespace(self,namespace):
        """Unregister an additional image publisher that was previously added.

        Removes the image, status, and nav publishers associated with the given
        namespace. Cannot remove the primary namespace.

        Args:
            namespace (str): The base ROS namespace to remove.

        Returns:
            bool: True if the publisher was successfully removed, False otherwise.
        """
        success = False
        if namespace is None:
            return success
        namespace = nepi_sdk.get_full_namespace(namespace)
        img_ns = nepi_sdk.create_namespace(namespace,self.data_product)
        if img_ns == self.namespace:
            self.msg_if.pub_warn("Can't remove base namespace: " + str(namespace), log_name_list = self.log_name_list)
            return success
        elif namespace in self.add_pubs_dict.keys():
            self.msg_if.pub_warn("Removing image pub namespace: " + str(self.namespace), log_name_list = self.log_name_list)
            [img_ns,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
            del self.add_pubs_dict[namespace]
            nepi_sdk.sleep(1)
            self.node_if.unregister_pub(img_ns)
            self.node_if.unregister_pub(status_ns)
            self.node_if.unregister_pub(nav_ns)
            success = True
        return success


    def get_blank_navpose_dict(self):
        """Return a deep copy of the blank nav pose dictionary template.

        Returns:
            dict: A blank nav pose dictionary with all fields at default values.
        """
        blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return blank_navpose_dict

    def get_navpose_dict(self):
        """Return the most recent nav pose dictionary from the associated NavPoseIF.

        Returns:
            dict: The current nav pose data dictionary, or a blank dict if no
            NavPoseIF is attached.
        """
        navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        # if self.navpose_if is not None:
        #     navpose_dict = self.navpose_if.get_navpose_dict()
        return navpose_dict

    def get_data_source_description(self):
        """Return the human-readable data source description string.

        Returns:
            str: Description of the data source (e.g. 'imaging_sensor').
        """
        return self.data_source_description

    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'color_image').
        """
        return self.data_product

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def needs_data_check(self):
        """Return whether downstream consumers currently need image data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        return self.needs_data

    def get_image_callback_options(self):
        """Return the list of supported image callback names.

        Returns:
            list: Callback name strings registered in the callback dictionary.
        """
        return list(self.callback_dict.keys())

    def set_image_callback(self,name,function):
        """Register a callable for the named image callback slot.

        Args:
            name (str): Name of the callback slot (must be in the callback dict).
            function (callable): Function to call when the event fires.
        """
        self.msg_if.pub_warn("Got set callback for: " + str(name), log_name_list = self.log_name_list)
        if name in self.callback_dict.keys():
            self.msg_if.pub_warn("Callback set for: " + str(name), log_name_list = self.log_name_list)
            self.callback_dict[name] = function
        #self.msg_if.pub_info("Updated callback dict: " + str(self.callback_dict), log_name_list = self.log_name_list)

    def clear_image_callback(self,name):
        """Clear (un-register) the callable for the named image callback slot.

        Args:
            name (str): Name of the callback slot to clear.
        """
        self.msg_if.pub_warn("Got clear callback for: " + str(name), log_name_list = self.log_name_list)
        if name in self.callback_dict.keys():
            self.callback_dict[name] = None

    def get_navpose_callback_options(self):
        """Return the list of supported nav pose callback names.

        Returns:
            list: Callback name strings from the attached NavPoseIF, or an empty
            list if no NavPoseIF is attached.
        """
        if self.navpose_if is not None:
            return self.navpose_if.get_navpose_callback_options()
        else:
            return []

    def set_navpose_callback(self,name,function):
        """Register a callable for the named nav pose callback slot.

        Delegates to the attached NavPoseIF if one is present.

        Args:
            name (str): Name of the nav pose callback slot.
            function (callable): Function to call when the event fires.
        """
        if self.navpose_if is not None:
            self.msg_if.pub_warn("Got set navpose callback for: " + str(name), log_name_list = self.log_name_list)
            if name in self.navpose_if.get_image_callback_options():
                self.msg_if.pub_warn("navpose Callback set for: " + str(name), log_name_list = self.log_name_list)
                self.navpose_if.set_image_callback(name,function)
        #self.msg_if.pub_info("Updated callback dict: " + str(self.callback_dict), log_name_list = self.log_name_list)

    def clear_navpose_callback(self,name):
        """Clear the callable for the named nav pose callback slot.

        Delegates to the attached NavPoseIF if one is present.

        Args:
            name (str): Name of the nav pose callback slot to clear.
        """
        if self.navpose_if is not None:
            self.msg_if.pub_warn("Got clear navpose callback for: " + str(name), log_name_list = self.log_name_list)
            if name in self.navpose_if.get_image_callback_options():
               self.navpose_if.callback_dict[name] = None

    def process_cv2_img(self, cv2_img):
        """Apply any image processing pipeline to a org OpenCV image.

        Base implementation is a pass-through. Subclasses override this to apply
        resolution, orientation, filter, and adjustment controls.

        Args:
            cv2_img (numpy.ndarray): Input OpenCV image array.

        Returns:
            numpy.ndarray: The processed image (same object in the base class).
        """
        return cv2_img

    def publish_cv2_img(self, cv2_img,
                        encoding = "bgr8",
                        timestamp = None,
                        width_deg = 100,
                        height_deg = 70,
                        min_range_m = 0,
                        max_range_m = 0,
                        add_overlay_text_list = [],
                        process_data = True,
                        pub_twice = False,
                        add_pubs = []
                        ):
        """Process and publish an OpenCV image to all configured ROS topics.

        Applies the image processing pipeline (if process_data is True), renders
        configured text overlays, publishes the result as a ROS Image message,
        optionally publishes to additional namespaces, saves to disk if a
        SaveDataIF is registered, and updates publication statistics.

        Args:
            cv2_img (numpy.ndarray): OpenCV image to publish.
            encoding (str, optional): ROS image encoding string. Defaults to 'bgr8'.
            timestamp (float or rospy.Time, optional): Acquisition timestamp.
                Defaults to current time if None.
            width_deg (float, optional): Horizontal field of view in degrees.
                Defaults to 100.
            height_deg (float, optional): Vertical field of view in degrees.
                Defaults to 70.
            min_range_m (float, optional): Minimum sensor range in meters.
                Defaults to 0.
            max_range_m (float, optional): Maximum sensor range in meters.
                Defaults to 0.
            add_overlay_text_list (list, optional): Additional text strings to overlay
                on the image. Defaults to [].
            process_data (bool, optional): Whether to run process_cv2_img before
                publishing. Defaults to True.
            pub_twice (bool, optional): Publish the image a second time after a
                brief sleep to work around subscriber latching issues.
                Defaults to False.
            add_pubs (list, optional): Additional namespace strings to publish to.
                Defaults to [].

        Returns:
            numpy.ndarray: The processed image, or the original image if processing
            was skipped or an error occurred.
        """
        

        if self.navpose_if is not None:
            navpose_dict = self.navpose_if.get_navpose_dict()
        else:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        # get_navpose_dict() consumes and clears its data, so it returns None
        # until the next navpose arrives; fall back to the blank dict so the
        # unguarded navpose_dict['navpose_frame'] lookup below can't crash.
        if navpose_dict is None:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)


        if self.publishing == False:
            self.publishing = True



            #self.msg_if.pub_debug("Got Image to Publish", log_name_list = self.log_name_list, throttle_s = 5.0)
            success = False
            if cv2_img is None and self.status_msg is not None:
                self.msg_if.pub_warn("Can't publish None image", log_name_list = self.log_name_list)
                # Clear the re-entrancy latch before the early return, or this IF
                # never publishes another frame.
                self.publishing = False
                return cv2_img

            # Process
            try: # Catch for lost camera in middle of send
                
        

                self.status_msg.encoding = encoding
                #self.msg_if.pub_warn("Got timestamp: " + str(timestamp), log_name_list = self.log_name_list)
                if timestamp == None:
                    timestamp = nepi_utils.get_time()
                else:
                    timestamp = nepi_sdk.sec_from_timestamp(timestamp)
                #self.msg_if.pub_warn("Using timestamp: " + str(timestamp), log_name_list = self.log_name_list)


                current_time = nepi_utils.get_time()
                latency = (current_time - timestamp)
                self.status_msg.get_latency_time = latency
                #self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(latency), log_name_list = self.log_name_list, throttle_s = 5.0)

                # Start Img Pub Process
                start_time = nepi_utils.get_time()   


                self.width_deg = width_deg
                self.height_deg = height_deg
                self.status_msg.width_deg = width_deg * (1 - self.zoom_ratio)
                self.status_msg.height_deg = height_deg * (1 - self.zoom_ratio)

                if (min_range_m is not None and max_range_m is not None):
                    self._updateRangesM(min_range_m,max_range_m)
                    self.status_msg.min_range_m = self.min_range_m
                    self.status_msg.max_range_m = self.max_range_m
                else:
                    self.status_msg.min_range_m = 0
                    self.status_msg.max_range_m = 1

                start_range_ratio = self.controls_dict['start_range_ratio']
                stop_range_ratio = self.controls_dict['stop_range_ratio']

                delta_range = self.max_range_m - self.min_range_m
                self.status_msg.min_range_m_adj = self.min_range_m + delta_range * start_range_ratio
                self.status_msg.max_range_m_adj = self.min_range_m + delta_range * stop_range_ratio

                [height,width] = cv2_img.shape[0:2]
                [self.height_org,self.width_org] = [height,width]
                #self.msg_if.pub_warn("Got Image size: " + str([height,width]), log_name_list = self.log_name_list)


                if process_data == True and  cv2_img is not None:
                    cv2_img = self.process_cv2_img(cv2_img)
                if cv2_img is not None:
                    
                    

                    [height,width] = cv2_img.shape[0:2]
                    [self.height_proc,self.width_proc] = [height,width]


                    last_width = self.status_msg.width_px
                    last_height = self.status_msg.height_px
                    self.status_msg.width_px = width
                    self.status_msg.height_px = height
                    res_str = str(width) + ":" + str(height)
                    self.status_msg.resolution_current = res_str

                    if height > 5 and width > 5:
 
                        ######################             
                        crosshairs_dict = self.overlays_dict['crosshairs_dict']
                        crosshair_len = len(list(crosshairs_dict.keys()))
                        crosshairs_enabled = self.overlays_dict['crosshairs_enabled']
                        overlay_crosshair_names = self.overlays_dict['overlay_crosshair_names']
                        overlay_crosshair_pixels = self.overlays_dict['overlay_crosshair_pixels']
                        overlay_crosshair_degrees = self.overlays_dict['overlay_crosshair_degrees']
                        overlay_crosshair_messages = self.overlays_dict['overlay_crosshair_messages']
                        if crosshair_len > 0 and crosshairs_enabled == True:
                            
                            for crosshair_name in crosshairs_dict.keys():
                                crosshair_dict = crosshairs_dict[crosshair_name]
                                #self.msg_if.pub_warn("Rendering image with crosshair_dict: " + str(crosshair_dict) , log_name_list = self.log_name_list)
        


                                x_deg_offset = round(crosshair_dict['x_deg_offset'] + 0.02, 1)
                                x_ratio = ((self.width_deg/2) - x_deg_offset)/self.width_deg
                                x_offset_ratio = (x_ratio - 0.5)
                                x_scale = (self.width_proc/self.width_org)
                                x_offset_pixel = int((x_offset_ratio * self.width_org))
                                x_pixel = int((self.width_proc/2) + x_offset_pixel)
                                if x_pixel < 0:
                                    x_pixel = 0
                                if x_pixel > self.width_proc:
                                    x_pixel = self.width_proc
                                #self.msg_if.pub_warn("Rendering target x: " + str([x_deg_offset,x_ratio,x_offset_ratio,x_scale,x_offset_pixel,x_pixel]) , log_name_list = self.log_name_list, throttle_s = 5)


                                y_deg_offset = round(crosshair_dict['y_deg_offset'] + 0.02, 1)
                                y_ratio = ((self.height_deg/2) + y_deg_offset)/self.height_deg
                                y_offset_ratio = -1 * (0.5 - y_ratio)
                                y_scale = (self.height_proc/self.height_org)
                                y_offset_pixel = int((y_offset_ratio * self.height_org))
                                y_pixel = int((self.height_proc/2) + y_offset_pixel)
                                if y_pixel < 0:
                                    y_pixel = 0
                                if y_pixel > self.height_proc:
                                    y_pixel = self.height_proc
                                #self.msg_if.pub_warn("Rendering target y: " + str([y_deg_offset,y_ratio,y_offset_ratio,y_scale,y_offset_pixel,y_pixel]) , log_name_list = self.log_name_list, throttle_s = 5)


                                crosshair_rbg = crosshair_dict['color_rgb']
                                crosshair_msg = crosshair_dict['msg_str']

                                overlay_text = []
                                if overlay_crosshair_names == True:
                                    overlay_text.append(crosshair_name)
                                if overlay_crosshair_pixels == True:
                                    overlay_text.append(str(x_pixel) + ',' + str(y_pixel))
                                if overlay_crosshair_degrees == True:
                                    overlay_text.append(str(x_deg_offset) + ',' + str(y_deg_offset))
                                if overlay_crosshair_messages == True and len(crosshair_msg) > 0:
                                    overlay_text.append(str(crosshair_msg))

                                crosshairs_size_ratio = self.overlays_dict['crosshairs_size_ratio']
                                crosshairs_thickness_ratio = self.overlays_dict['crosshairs_thickness_ratio']
                                crosshairs_text_ratio = self.overlays_dict['crosshairs_text_ratio']
                                #self.msg_if.pub_warn("Rendering image crosshair: " + str([x_pixel,y_pixel]) , log_name_list = self.log_name_list)
                                cv2_img = nepi_img.overlay_crosshair(cv2_img, 
                                                        x_px = x_pixel , y_px = y_pixel, 
                                                        color_rgb = crosshair_rbg, 
                                                        size_ratio =  crosshairs_size_ratio,
                                                        thickness_ratio = crosshairs_thickness_ratio,
                                                        text_list = overlay_text,
                                                        text_ratio = crosshairs_text_ratio)


                        ######################             
                        targets_dict = self.overlays_dict['targets_dict']
                        target_len = len(list(targets_dict.keys()))
                        targets_enabled = self.overlays_dict['targets_enabled']
                        overlay_target_names = self.overlays_dict['overlay_target_names']
                        overlay_target_pixels = self.overlays_dict['overlay_target_pixels']
                        overlay_target_degrees = self.overlays_dict['overlay_target_degrees']
                        overlay_target_messages = self.overlays_dict['overlay_target_messages']
                        if target_len > 0 and targets_enabled == True:
                            
                            for target_name in targets_dict.keys():
                                target_dict = targets_dict[target_name]
                                #self.msg_if.pub_warn("Rendering image with target_dict: " + str(target_dict) , log_name_list = self.log_name_list)
        

                                x_deg_offset = round(target_dict['x_deg_offset'] + 0.02, 1) 
                                x_ratio = ((self.width_deg/2) - x_deg_offset)/self.width_deg
                                x_offset_ratio = (x_ratio - 0.5)
                                x_scale = (self.width_proc/self.width_org)
                                x_offset_pixel = int((x_offset_ratio * self.width_org))
                                x_pixel = int((self.width_proc/2) + x_offset_pixel)
                                #self.msg_if.pub_warn("Rendering target x: " + str([x_deg_offset,x_ratio,x_offset_ratio,x_scale,x_offset_pixel,x_pixel]) , log_name_list = self.log_name_list, throttle_s = 5)


                                y_deg_offset = round(target_dict['y_deg_offset'] + 0.02, 1)
                                y_ratio = ((self.height_deg/2) + y_deg_offset)/self.height_deg
                                y_offset_ratio = -1 * (0.5 - y_ratio)
                                y_scale = (self.height_proc/self.height_org)
                                y_offset_pixel = int((y_offset_ratio * self.height_org))
                                y_pixel = int((self.height_proc/2) + y_offset_pixel)

                                #self.msg_if.pub_warn("Rendering target y: " + str([y_deg_offset,y_ratio,y_offset_ratio,y_scale,y_offset_pixel,y_pixel]) , log_name_list = self.log_name_list, throttle_s = 5)


                                target_rbg = target_dict['color_rgb']
                                target_msg = target_dict['msg_str']

                                overlay_text = []
                                if overlay_target_names == True:
                                    overlay_text.append(target_name)
                                if overlay_target_pixels == True:
                                    overlay_text.append(str(x_pixel) + ',' + str(y_pixel))
                                if overlay_target_degrees == True:
                                    overlay_text.append(str(x_deg_offset) + ',' + str(y_deg_offset))
                                if overlay_target_messages == True and len(target_msg) > 0:
                                    overlay_text.append(str(target_msg))

                                targets_size_ratio = self.overlays_dict['targets_size_ratio']
                                targets_thickness_ratio = self.overlays_dict['targets_thickness_ratio']
                                targets_text_ratio = self.overlays_dict['targets_text_ratio']
                                #self.msg_if.pub_warn("Rendering image target: " + str([x_pixel,y_pixel]) , log_name_list = self.log_name_list)
                                cv2_img = nepi_img.overlay_target(cv2_img, 
                                                        x_px = x_pixel , y_px = y_pixel, 
                                                        color_rgb = target_rbg, 
                                                        size_ratio =  targets_size_ratio,
                                                        thickness_ratio = targets_thickness_ratio,
                                                        text_list = overlay_text,
                                                        text_ratio = targets_text_ratio)
                                


                        if process_data == True and  cv2_img is not None:
                             cv2_img = self._zoomAdjust(cv2_img)

                        if process_data == True and  cv2_img is not None:
                            cv2_img = self._liveAdjust(cv2_img)       

                        if cv2_img is not None:
                            [height,width] = cv2_img.shape[0:2]
                            if height > 5 and width > 5:
                                #self.msg_if.pub_debug("Got Processed size: " + str([height,width]), log_name_list = self.log_name_list)



                                
                                # Apply Text Overlays
                                overlay_text_list = []
                                overlay_text_enabled = self.overlays_dict['overlay_text_enabled']
                                overlay_text_size_ratio = self.overlays_dict['overlay_text_size_ratio']
                                overlay_text_vert_ratio = self.overlays_dict['overlay_text_vert_ratio']
                                overlay_text_horz_ratio = self.overlays_dict['overlay_text_horz_ratio']
                                overlay_text_color_rgb = self.overlays_dict['overlay_text_color_rgb']
                                overlay_text_img_name = self.overlays_dict['overlay_text_img_name']
                                overlay_text_date_time = self.overlays_dict['overlay_text_date_time']
                                overlay_text_nav = self.overlays_dict['overlay_text_nav']
                                overlay_text_pose = self.overlays_dict['overlay_text_pose']
                                if overlay_text_enabled == True:
                                    if overlay_text_img_name == True:
                                        overlay = nepi_img.getImgShortName(self.namespace)
                                        overlay_text_list.append(overlay)
                                    
                                    if overlay_text_date_time == True:
                                        overlay = nepi_utils.get_datetime_str_from_timestamp(timestamp)
                                        overlay = overlay.replace('D','')
                                        overlay = overlay.replace('T',' T: ')
                                        overlay_text_list.append(overlay)

                                    if overlay_text_nav == True or overlay_text_pose == True:
                                        if navpose_dict is not None:
                                            if overlay_text_nav == True and navpose_dict is not None:
                                                overlay = 'Lat: ' +  str(round(navpose_dict['latitude'],6)) + ' Long: ' +  str(round(navpose_dict['longitude'],6)) + ' Head: ' +  str(round(navpose_dict['heading_deg'],0))
                                                overlay_text_list.append(overlay)

                                            if overlay_text_pose == True and navpose_dict is not None:
                                                overlay = 'Roll: ' +  str(round(navpose_dict['roll_deg'],0)) + ' Pitch: ' +  str(round(navpose_dict['pitch_deg'],0)) + ' Yaw: ' +  str(round(navpose_dict['yaw_deg'],0))
                                                overlay_text_list.append(overlay)

                                    overlay_text_list = overlay_text_list + self.overlays_dict['init_overlay_text_list'] + self.overlays_dict['add_overlay_text_list'] + add_overlay_text_list

                                    if len(overlay_text_list) > 0:
                                        start_x = (width * 0.01) + ((width * 0.99) * overlay_text_horz_ratio)
                                        start_y = (height * 0.01) + ((height * 0.90) * overlay_text_vert_ratio)
                                        cv2_img = nepi_img.overlay_text_list(cv2_img, 
                                                                text_list = overlay_text_list, 
                                                                x_px = start_x , y_px = start_y, 
                                                                color_rgb = overlay_text_color_rgb, 
                                                                apply_shadow = True, 
                                                                size_ratio = overlay_text_size_ratio )



                        if self.node_if is not None and self.needs_data == True and cv2_img is not None:
                            #self.msg_if.pub_warn("Publishing once")
                            #Convert to ros Image message
                            ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding=encoding)
                            sec = nepi_sdk.sec_from_timestamp(timestamp)
                            header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = navpose_dict['navpose_frame'])
                            #self.msg_if.pub_warn("Publishing image with header: " + str(header))
                            ros_img.header = header
                            self.node_if.publish_pub(self.node_if_prefix + 'data_pub', ros_img)


                            for namespace in add_pubs:
                                if namespace in self.add_pubs_dict.keys():
                                    [img_ns,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
                                    #self.msg_if.pub_warn("Publishing Add Image on namespace: " + str(img_ns), log_name_list = self.log_name_list, throttle_s = 5.0)
                                    self.node_if.publish_pub(img_ns, ros_img)
                                    
                            if pub_twice == True:
                                #self.msg_if.pub_warn("Publishing twice: " + str(pub_twice))
                                nepi_sdk.sleep(0.01)
                                self.node_if.publish_pub(self.node_if_prefix + 'data_pub', ros_img)
                                for namespace in add_pubs:
                                    if namespace in self.add_pubs_dict.keys():
                                        [img_ns,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
                                        self.node_if.publish_pub(img_ns, ros_img)

                        # Save Data
                        if self.save_data_if is not None:
                            self.save_data_if.save(self.data_product,cv2_img,timestamp)
                        
                        # Update stats
                        process_time = round( (nepi_utils.get_time() - start_time) , 3)
                        self.status_msg.process_time = process_time
                        latency = (current_time - timestamp)
                        self.status_msg.pub_latency_time = latency
                        

                        if self.last_pub_time is None:
                            self.last_pub_time = nepi_utils.get_time()
                        else:
                            cur_time = nepi_utils.get_time()
                            pub_time_sec = cur_time - self.last_pub_time
                            self.last_pub_time = cur_time
                            self.status_msg.last_pub_sec = pub_time_sec

                            self.time_list.pop(0)
                            self.time_list.append(pub_time_sec)

                        # Update blank image if needed
                        if last_width != self.status_msg.width_px or last_height != self.status_msg.height_px:
                            self.blank_img = nepi_img.create_cv2_blank_img(width, height, color_rgb = (0, 0, 0) )


   
            except Exception as e:
                self.msg_if.pub_warn("Failed to publish image: " + str(e), log_name_list = self.log_name_list)
        self.publishing = False
        return cv2_img
        

    def publish_msg_img(self, msg_text, timestamp = None, navpose_frame = 'None'):
        """Publish a text message rendered onto the blank image template.

        Renders the message string over the cached blank image and publishes it
        as a ROS Image message on the primary data publisher.

        Args:
            msg_text (str): Text to render on the image.
            timestamp (float, optional): Timestamp in seconds. Defaults to current time.
            navpose_frame (str, optional): TF frame ID for the image header.
                Defaults to 'None'.
        """
        cv2_img = nepi_img.overlay_text_autoscale(self.blank_img, msg_text)

        if timestamp == None:
            timestamp = nepi_utils.get_time()

        #Convert to ros Image message
        ros_img = nepi_img.cv2img_to_rosimg(cv2_img)
        sec = nepi_sdk.sec_from_timestamp(timestamp)
        ros_img.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = navpose_frame)
        if self.node_if is not None:
            self.node_if.publish_pub(self.node_if_prefix + 'data_pub', ros_img)

    


    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this depth map interface."""          
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)

    def register_pubs(self):
        """Re-register all ROS publishers managed by this image interface."""
        if self.node_if is not None:
            self.node_if.register_pubs()

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



    ########################
    # Filter Functions

    def set_auto_adjust_enable(self, enabled):
        """Enable or disable automatic image adjustment.

        Args:
            enabled (bool): True to enable auto-adjust, False to disable.
        """
        if enabled:
            self.msg_if.pub_info("Enabling Auto Adjust", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Disabling Auto Adjust", log_name_list = self.log_name_list)
        self.controls_dict['auto_adjust_enabled'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'auto_adjust_enabled', enabled)

    def set_auto_adjust_ratio(self, ratio):
        """Set the auto-adjust strength ratio, clamped to [0.0, 1.0].

        Args:
            ratio (float): Auto-adjust strength where 0.0 is no adjustment and
                1.0 is maximum adjustment.
        """
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['auto_adjust_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'auto_adjust_ratio', ratio)

    def set_brightness_ratio(self, ratio):
        """Set the image brightness ratio, clamped to [0.0, 1.0].

        Args:
            ratio (float): Brightness level where 0.0 is darkest and 1.0 is brightest.
        """
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['brightness_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'brightness_ratio', ratio)

    def set_contrast_ratio(self, ratio):
        """Set the image contrast ratio, clamped to [0.0, 1.0].

        Args:
            ratio (float): Contrast level where 0.0 is minimum and 1.0 is maximum.
        """
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['contrast_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'contrast_ratio', ratio)

    def set_threshold_ratio(self, ratio):
        """Set the image threshold (sharpness) ratio, clamped to [0.0, 1.0].

        Args:
            ratio (float): Sharpness level where 0.0 is no sharpening and
                1.0 is maximum sharpening.
        """
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['threshold_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'threshold_ratio', ratio)

    def set_filter_enable(self,name, enabled):
        """Enable or disable a named image filter.

        Args:
            name (str): Name of the filter as registered in the filter dictionary.
            enabled (bool): True to enable the filter, False to disable it.
        """
        if self.filter_dict is not None:
            if name in self.filter_dict.keys():
                was_enabled = self.filter_dict[name]['enabled']
                if was_enabled != enabled:
                    if enabled == True:
                        self.msg_if.pub_info("Enabling Filter: " + name, log_name_list = self.log_name_list)
                    else:
                        self.msg_if.pub_info("Disabling Filter: " + name, log_name_list = self.log_name_list)
                    self.filter_dict[name]['enabled'] = enabled
                    self.publish_status()
                    self.needs_update()
                    if self.node_if is not None:
                        self.node_if.set_param(self.node_if_prefix +  'filter_dict', self.filter_dict)

    def set_filter_ratio(self,name, ratio):
        """Set the intensity ratio for a named image filter, clamped to [0.0, 1.0].

        Args:
            name (str): Name of the filter as registered in the filter dictionary.
            ratio (float): Filter intensity where 0.0 is no effect and 1.0 is
                maximum effect.
        """
        if self.filter_dict is not None:
            if ratio < 0:
                ratio = 0
            if ratio > 1.0:
                ratio = 1.0
            if name in self.filter_dict.keys():
                self.msg_if.pub_info("Setting Filter Ratio: " + name + " : " + str(ratio), log_name_list = self.log_name_list)
                self.filter_dict[name]['ratio'] = ratio
                self.publish_status()
                self.needs_update()
                self.node_if.set_param(self.node_if_prefix +  'filter_dict', self.filter_dict)

    ########################
    # Res and Orientation Functions

    def set_resolution_ratio(self, ratio):
        """Set the output image resolution ratio, clamped to [0.2, 1.0].

        Args:
            ratio (float): Fraction of full resolution, where 1.0 is full resolution
                and 0.2 is the minimum allowed.
        """
        if (ratio < 0.2):
            ratio = 0.2
        if (ratio > 1.0):
            ratio = 1.0
        self.controls_dict['resolution_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'resolution_ratio', ratio)

    def set_rotate_2d_deg(self, deg):
        """Set the 2-D rotation angle, rounded to the nearest integer degree.

        Args:
            deg (float): Rotation angle in degrees.
        """
        deg_int = int(round(deg,0))
        self.controls_dict['rotate_2d_deg'] = deg_int
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'rotate_2d_deg', deg_int)

    def set_rotate_2d_swap_box(self, enabled):
        """Enable or disable swapping the Free Cam output box dimensions (W/H).

        Swaps only the output box aspect (e.g. landscape <-> portrait); the image
        content orientation is unaffected and stays rotated by the current angle.

        Args:
            enabled (bool): True to swap the output box aspect, False for the original box.
        """
        self.controls_dict['rotate_2d_swap_box'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'rotate_2d_swap_box', enabled)

    def set_flip_horz(self, enabled):
        """Enable or disable horizontal image flip.

        Args:
            enabled (bool): True to flip horizontally, False to disable.
        """
        self.controls_dict['flip_horz'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'flip_horz', enabled)

    def set_flip_vert(self, enabled):
        """Enable or disable vertical image flip.

        Args:
            enabled (bool): True to flip vertically, False to disable.
        """
        self.controls_dict['flip_vert'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'flip_vert', enabled)

    ########################
    # Render Functions

    def set_range_ratios(self, start_ratio, stop_ratio):
        """Set the normalized depth range window for image rendering.

        Both ratios must be in [0, 1] and start_ratio must be less than or
        equal to stop_ratio.

        Args:
            start_ratio (float): Start of the displayed depth range as a fraction
                of the total sensor range.
            stop_ratio (float): End of the displayed depth range as a fraction of
                the total sensor range.
        """
        if (start_ratio < 0 or stop_ratio > 1 or stop_ratio < start_ratio):
            self.msg_if.pub_error("Range values out of bounds", log_name_list = self.log_name_list)
            self.publish_status() # No change
            return

        self.controls_dict['start_range_ratio'] = start_ratio
        self.controls_dict['stop_range_ratio'] = stop_ratio

        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'start_range_ratio', start_ratio)
            self.node_if.set_param(self.node_if_prefix +  'stop_range_ratio', stop_ratio)


    def set_zoom_ratio(self, ratio):
        """Set the zoom level, recalculating the image crop window accordingly.

        A zoom of 0.0 shows the full image and 1.0 crops to the maximum zoom
        centered on the current pan position.

        Args:
            ratio (float): Zoom level in [0.0, 1.0].
        """
        self.drag_pixel = None
        self.drag_window = None

        # Update Ratios

        wrs = copy.deepcopy(self.controls_dict['window_ratios'])
        xr_ratio = wrs[0] + (wrs[1] - wrs[0]) / 2
        yr_ratio = wrs[2] + (wrs[3] - wrs[2]) / 2

        xlen_max_r = (1 - abs(xr_ratio - 0.)) * 2
        ylen_max_r = (1 - abs(yr_ratio - 0.5)) * 2
        len_min_r = 0.05
        len_max_r = min(xlen_max_r, ylen_max_r)

        len_zoom = 1 - ratio * len_max_r
        if len_zoom < len_min_r:
            len_zoom = len_min_r

        xr_min = xr_ratio - len_zoom / 2
        xr_max = xr_ratio + len_zoom / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = len_zoom
        if xr_max > 1:
            xr_min = 1 - len_zoom
            xr_max = 1
    
        yr_min = yr_ratio - len_zoom / 2
        yr_max = yr_ratio + len_zoom / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = len_zoom
        if yr_max > 1:
            yr_min = 1 - len_zoom
            yr_max = 1

        self.msg_if.pub_info("Zoom Image Window: " + str([xr_min, xr_max, yr_min, yr_max]), log_name_list = self.log_name_list)
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = ratio


        self.publish_status() 
        self.needs_update()


    def set_pixel(self, pixel, color_bgr = [0,0,0,0]):
        """Center the image crop window on a selected pixel coordinate.

        Recenters the current zoom window so that the given pixel becomes the
        center of the view, preserving the existing window size.

        Args:
            pixel (list): [x, y] pixel coordinates in the org image space.
            color_bgr (list, optional): BGRA color of the selected pixel.
                Defaults to [0, 0, 0, 0].
        """
        self.drag_pixel = None
        self.drag_window = None

        if self.width_org > 10 and self.height_org > 10:
            # Update Ratios
            xr_ratio = pixel[0] / self.width_org
            yr_ratio = pixel[1] / self.height_org
            wrs = copy.deepcopy(self.controls_dict['window_ratios'])

            xr_len = wrs[1] - wrs[0]
            xr_min = xr_ratio - xr_len / 2
            xr_max = xr_ratio + xr_len / 2
            if xr_min < 0:
                xr_min = 0
                xr_max = xr_len
            if xr_max > 1:
                xr_min = 1 - xr_len
                xr_max = 1
        
            yr_len = wrs[3] - wrs[2]
            yr_min = yr_ratio - yr_len / 2
            yr_max = yr_ratio + yr_len / 2
            if yr_min < 0:
                yr_min = 0
                yr_max = yr_len
            if yr_max > 1:
                yr_min = 1 - yr_len
                yr_max = 1

            self.msg_if.pub_info("Pixel Image Window: " + str([xr_min, xr_max, yr_min, yr_max]), log_name_list = self.log_name_list)
            self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
            self.x_ratio = xr_min + (xr_max - xr_min) / 2
            self.y_ratio = yr_min + (yr_max - yr_min) / 2
            self.zoom_ratio = 1 - max(xr_len, yr_len)

            self.publish_status()  
            self.needs_update()    



    def set_x_ratio(self, ratio):
        """Pan the image crop window horizontally.

        Moves the horizontal center of the crop window to the position
        corresponding to the given ratio within the pannable range.

        Args:
            ratio (float): Horizontal pan position in [0.0, 1.0], where 0.0 is
                full left and 1.0 is full right.
        """
        self.drag_pixel = None
        self.drag_window = None


        wrs = copy.deepcopy(self.controls_dict['window_ratios'])
        xr_len = wrs[1] - wrs[0]
        yr_len = wrs[3] - wrs[2]

        r_min = xr_len / 2
        r_max = 1 - xr_len / 2
        xr_r = r_min + (ratio * (r_max - r_min))

        xr_min = xr_r - xr_len / 2
        xr_max = xr_r + xr_len / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = xr_len
        if xr_max > 1:
            xr_min = 1 - xr_len
            xr_max = 1
    
        yr_r = wrs[2] + yr_len / 2
        yr_min = yr_r - xr_len / 2
        yr_max = yr_r + xr_len / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = xr_len
        if yr_max > 1:
            yr_min = 1 - xr_len
            yr_max = 1

        self.msg_if.pub_info("X Ratio Image Window: " + str([xr_min, xr_max, yr_min, yr_max]), log_name_list = self.log_name_list)
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = 1 - xr_len

        self.publish_status() 
        self.needs_update()

    def set_y_ratio(self, ratio):
        """Pan the image crop window vertically.

        Moves the vertical center of the crop window to the position
        corresponding to the given ratio within the pannable range.

        Args:
            ratio (float): Vertical pan position in [0.0, 1.0], where 0.0 is
                top and 1.0 is bottom.
        """
        self.drag_pixel = None
        self.drag_window = None


        wrs = copy.deepcopy(self.controls_dict['window_ratios'])
        xr_len = wrs[1] - wrs[0]
        yr_len = wrs[3] - wrs[2]

        r_min = yr_len / 2
        r_max = 1 - yr_len / 2
        yr_r = r_min + (ratio * (r_max - r_min))

        yr_min = yr_r - yr_len / 2
        yr_max = yr_r + yr_len / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = yr_len
        if yr_max > 1:
            yr_min = 1 - yr_len
            yr_max = 1
    
        xr_r = wrs[0] + xr_len / 2
        xr_min = xr_r - yr_len / 2
        xr_max = xr_r + yr_len / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = yr_len
        if xr_max > 1:
            xr_min = 1 - yr_len
            xr_max = 1

        self.msg_if.pub_info("Y Ratio Image Window: " + str([xr_min, xr_max, yr_min, yr_max]), log_name_list = self.log_name_list)
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = 1 - yr_len

        self.publish_status() 
        self.needs_update()


    def set_window(self, window):
        """Set the image crop window from absolute pixel coordinates.

        Converts the pixel-space window into normalized ratio coordinates and
        updates the crop window. Requires the org image dimensions to be known.

        Args:
            window (list): [x_min, x_max, y_min, y_max] in org image pixel space.
        """
        self.drag_pixel = None
        self.drag_window = None
 
        if self.width_org > 10 and self.height_org > 10:
            # Update Ratios
            xr_len = (window[1] - window[0]) / self.width_org
            yr_len = (window[3] - window[2]) / self.height_org
            xr_ratio = window[0] / self.width_org + (xr_len / 2) 
            yr_ratio = window[2] / self.height_org + (yr_len / 2)

            r_len_max = max(xr_len, yr_len)

            xr_min = xr_ratio - r_len_max / 2
            xr_max = xr_ratio + r_len_max / 2
            if xr_min < 0:
                xr_min = 0
                xr_max = r_len_max
            if xr_max > 1:
                xr_min = 1 - r_len_max
                xr_max = 1
        

            yr_min = yr_ratio - r_len_max / 2
            yr_max = yr_ratio + r_len_max / 2
            if yr_min < 0:
                yr_min = 0
                yr_max = r_len_max
            if yr_max > 1:
                yr_min = 1 - r_len_max
                yr_max = 1

            self.msg_if.pub_warn("Window Image Window set to: " + str([xr_min, xr_max, yr_min, yr_max]), log_name_list = self.log_name_list)
            self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
            self.x_ratio = xr_min + (xr_max - xr_min) / 2
            self.y_ratio = yr_min + (yr_max - yr_min) / 2
            self.zoom_ratio = 1 - r_len_max 

            self.publish_status()  
            self.needs_update()    
            self.publish_status()  
            self.needs_update()



    def update_window_ratios(self):
        """Republish status and request a new frame after a window ratio change."""
        self.window_ratios = copy.deepcopy(self.controls_dict['window_ratios'])
        self.publish_status()
        self.needs_update()

    def set_zoom_3d_ratio(self, ratio):
        """Set the 3-D rotation ratio, clamped to a valid ratio range.

        Args:
            ratio (float): Rotation position in [0.0, 1.0].
        """
        self.controls_dict['zoom_3d_ratio'] = nepi_utils.check_ratio(ratio)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'zoom_3d_ratio', self.controls_dict['zoom_3d_ratio'])

    def set_rotate_3d_ratio(self, ratio):
        """Set the 3-D rotation ratio, clamped to a valid ratio range.

        Args:
            ratio (float): Rotation position in [0.0, 1.0].
        """
        self.controls_dict['rotate_3d_ratio'] = nepi_utils.check_ratio(ratio)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'rotate_3d_ratio', self.controls_dict['rotate_3d_ratio'])

    def set_tilt_3d_ratio(self, ratio):
        """Set the 3-D tilt ratio, clamped to a valid ratio range.

        Args:
            ratio (float): Tilt position in [0.0, 1.0].
        """
        self.controls_dict['tilt_3d_ratio'] = nepi_utils.check_ratio(ratio)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'tilt_3d_ratio', self.controls_dict['tilt_3d_ratio'])

    ########################

    def set_aspect_adjust_enable(self,enabled):
        self.aspect_adjust_enabled = enabled and self.aspect_adjustment_disabled == False
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'aspect_adjust_enabled', self.aspect_adjust_enabled)

    def set_aspect_adjust_ratio(self,aspect_ratio):
        if aspect_ratio is not None:
            if aspect_ratio >= 0.5 and aspect_ratio <= 2.5:
                self.aspect_ratio_set = aspect_ratio #nepi_img.get_aspect_ratio_clean(aspect_ratio)
                if self.node_if is not None:
                    self.node_if.set_param(self.node_if_prefix +  'aspect_ratio_set', self.aspect_ratio_set)

    def set_aspect_adjust_by_ratio(self,ratio):
        ratio = nepi_utils.check_ratio(ratio)
        aspect_ratio = 1 + ratio
        self.set_aspect_adjust_ratio(aspect_ratio)
           

        

    def set_stream_compression_enable(self,enabled):
        self.stream_compression_enabled = enabled
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'stream_compression_enabled', self.stream_compression_enabled)

    def set_stream_compression_ratio(self,ratio):
        self.stream_compression_ratio = nepi_utils.check_ratio(ratio)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'stream_compression_ratio', self.stream_compression_ratio)



    ########################
    # Overlay Functions
    def set_overlay_text_enable(self,enabled):
        """Enable or disable the text overlays.

        Args:
            enabled (bool): True to show text data, False to hide it.
        """
        self.overlays_dict['overlay_text_enabled'] = enabled
        if enabled == False:
            self.click_text_enabled = False
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)



    def set_click_text(self,enabled):
        """Enable or disable the click text enable.

        Args:
            enabled (bool): True to show text data, False to hide it.
        """
        self.click_text_enabled = enabled
        self.click_crosshair_enabled = False
        self.click_target_enabled = False
        self.publish_status()

    def set_overlay_text_size_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['overlay_text_size_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_vert_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['overlay_text_vert_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_horz_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['overlay_text_horz_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_transparency_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['overlay_text_transparency_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_color_rgb(self, r = 0, g = 255, b = 0):
        self.overlays_dict['overlay_text_color_rgb'] = (r,g,b)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_image_name(self,enabled):
        """Enable or disable the image name text overlay.

        Args:
            enabled (bool): True to show the image source name, False to hide it.
        """
        self.overlays_dict['overlay_text_img_name'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_date_time(self,enabled):
        """Enable or disable the date/time text overlay.

        Args:
            enabled (bool): True to show the timestamp, False to hide it.
        """
        self.overlays_dict['overlay_text_date_time'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_nav(self,enabled):
        """Enable or disable the GPS navigation (lat/lon/heading) text overlay.

        Args:
            enabled (bool): True to show navigation data, False to hide it.
        """
        self.overlays_dict['overlay_text_nav'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_pose(self,enabled):
        """Enable or disable the roll/pitch/yaw pose text overlay.

        Args:
            enabled (bool): True to show pose data, False to hide it.
        """
        self.overlays_dict['overlay_text_pose'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_list(self,overlay_text_list):
        """Replace the additional text overlay list with the provided list.

        Args:
            overlay_text_list (list): List of text strings to display as overlays.
        """
        self.overlays_dict['add_overlay_text_list'] = overlay_text_list
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text(self,overlay_text):
        """Append a text string to the additional overlay list.

        Args:
            overlay_text_text (str): Text string to add to the overlay list.
        """
        overlay_text_list = self.overlays_dict['add_overlay_text_list']
        overlay_text_list.append(overlay_text)
        self.overlays_dict['add_overlay_text_list'] = overlay_text_list
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def clear_overlay_text_list(self):
        """Clear all entries from the additional text overlay list."""
        self.overlays_dict['add_overlay_text_list'] = []
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    ###################################
    def set_crosshairs_enable(self,enabled):
        """Enable or disable the crosshair overlays.

        Args:
            enabled (bool): True to show crosshairs data, False to hide it.
        """
        was_enabled = copy.deepcopy(self.overlays_dict['crosshairs_enabled'])
        self.overlays_dict['crosshairs_enabled'] = enabled
        if enabled == False:
            self.click_crosshair_enabled = False
        if was_enabled != enabled:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_click_crosshair(self,enabled):
        """Enable or disable the click crosshair enable.

        Args:
            enabled (bool): True to show crosshairs data, False to hide it.
        """
        self.msg_if.pub_info("Setting Click Crosshair Enable" + str(enabled), log_name_list = self.log_name_list)
        self.click_text_enabled = False
        self.click_crosshair_enabled = enabled
        self.click_target_enabled = False
        self.publish_status()

    def set_crosshairs_size_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['crosshairs_size_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['crosshairs_size_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_crosshairs_thickness_ratio(self, ratio):
        """Set the relative thickness of crosshairs overlays on the image.

        Args:
            ratio (float): Text thickness ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['crosshairs_thickness_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['crosshairs_thickness_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_crosshairs_text_ratio(self, ratio):
        """Set the relative text of crosshairs overlays on the image.

        Args:
            ratio (float): Text text ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['crosshairs_text_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['crosshairs_text_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_crosshairs_transparency_ratio(self, ratio):
        """Set the relative size of crosshairs overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['targets_thickness_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['crosshairs_transparency_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    def set_crosshairs_color_rgb(self, r = 0, g = 255, b = 0):
        self.overlays_dict['crosshairs_color_rgb'] = (r,g,b)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)
      



    def set_overlay_crosshair_names(self,enabled):
        """Enable or disable the crosshair names overlays.

        Args:
            enabled (bool): True to show crosshairs name data, False to hide it.
        """
        self.overlays_dict['overlay_crosshair_names'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_crosshair_pixels(self,enabled):
        """Enable or disable the crosshair pixels overlays.

        Args:
            enabled (bool): True to show crosshairs pixel data, False to hide it.
        """
        self.overlays_dict['overlay_crosshair_pixels'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_crosshair_degrees(self,enabled):
        """Enable or disable the crosshair degrees overlays.

        Args:
            enabled (bool): True to show crosshairs degrees data, False to hide it.
        """
        self.overlays_dict['overlay_crosshair_degrees'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_crosshair_messages(self,enabled):
        """Enable or disable the crosshair messages overlays.

        Args:
            enabled (bool): True to show crosshairs messages, False to hide it.
        """
        self.overlays_dict['overlay_crosshair_messages'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    def add_crosshair_pixels(self, x_pixel, y_pixel, name = None, color_rgb = None, msg_str = ''):
        """Append a crosshair overlay at pixel location.

        Args:
            x pixel, y pixel, crosshair name str.
        """
        x_ratio = float(x_pixel / self.width_org) 
        y_ratio = float(y_pixel / self.height_org)
        x_deg =  -1 * round(  ((x_ratio - 0.5) * self.width_deg),1)
        y_deg = round(  ((y_ratio - 0.5) * self.height_deg),1)  
        self.add_crosshair_degs(x_deg, y_deg, name, color_rgb, msg_str)


    def add_crosshair_ratios(self, x_ratio, y_ratio, name = None, color_rgb = None, msg_str = ''):
        """Append a crosshair overlay at pixel location.

        Args:
            x pixel, y pixel, crosshair name str.
        """
        x_ratio = nepi_utils.check_ratio(x_ratio)
        y_ratio = nepi_utils.check_ratio(y_ratio)
        x_deg = -1 * round(  ((x_ratio - 0.5) * self.width_deg),1)
        y_deg = round(  ((y_ratio - 0.5) * self.height_deg),1)  
        # self.msg_if.pub_info("Adding Crosshair ratio to degs: " + str([x_ratio,x_deg,y_ratio,y_deg]), log_name_list = self.log_name_list)
        self.add_crosshair_degs(x_deg, y_deg, name, color_rgb, msg_str)


    def add_crosshair_degs(self, x_deg, y_deg, name = None, color_rgb = None, msg_str = ''):
        """Append a crosshair overlay at degs offset location.

        Args:
            x deg, y deg, crosshair name str.
        """
        crosshairs_dict = self.overlays_dict['crosshairs_dict']
        crosshair_names = list(crosshairs_dict.keys())
        num_crosshairs = len(crosshair_names)
        ch_name = str(num_crosshairs + 1)
        if name is not None:
            if name != '':
                ch_name = name
        if color_rgb is None:
            color_rgb = self.overlays_dict['crosshairs_color_rgb']
        crosshair_dict = copy.deepcopy(self.BLANK_CROSSHAIR_DICT)
        crosshair_dict['x_deg_offset'] = round(x_deg, 2)
        crosshair_dict['y_deg_offset'] = round(y_deg, 2)
        crosshair_dict['color_rgb'] = color_rgb
        crosshair_dict['msg_str'] = msg_str
        #self.msg_if.pub_info("Adding Crosshair: " + str(crosshair_dict), log_name_list = self.log_name_list)
        needs_update = True
        if ch_name in self.overlays_dict['crosshairs_dict'].keys():
            if self.overlays_dict['crosshairs_dict'][ch_name] == crosshair_dict:
                needs_update = False
        if needs_update == True:
            self.overlays_dict['crosshairs_dict'][ch_name] = crosshair_dict
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def remove_crosshair(self, name):
        """Remove entry from crosshairs overlay dict."""
        try:
            del self.overlays_dict['crosshairs_dict'][name]
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)
        except:
            pass
        
        

    def clear_crosshairs(self):
        """Clear all entries from crosshairs overlay dict."""
        if len(list(self.overlays_dict['crosshairs_dict'].keys())) > 0:
            self.overlays_dict['crosshairs_dict'] = dict()
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    ###################################
    def set_targets_enable(self,enabled):
        """Enable or disable the target overlays.

        Args:
            enabled (bool): True to show targets data, False to hide it.
        """
        was_enabled = copy.deepcopy(self.overlays_dict['targets_enabled'])
        self.overlays_dict['targets_enabled'] = enabled
        if enabled == False:
            self.click_target_enabled = False
        if was_enabled != enabled:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_click_target(self,enabled):
        """Enable or disable the click target enable.

        Args:
            enabled (bool): True to show targets data, False to hide it.
        """
        self.click_text_enabled = False
        self.click_crosshair_enabled = False
        self.click_target_enabled = enabled
        self.publish_status()

    def set_targets_size_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['targets_size_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['targets_size_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_targets_thickness_ratio(self, ratio):
        """Set the relative thickness of targets overlays on the image.

        Args:
            ratio (float): Text thickness ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['targets_thickness_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['targets_thickness_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_targets_text_ratio(self, ratio):
        """Set the relative text of targets overlays on the image.

        Args:
            ratio (float): Text text ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['targets_text_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['targets_text_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_targets_transparency_ratio(self, ratio):
        """Set the relative size of targets overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['targets_transparency_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    def set_targets_color_rgb(self, r = 0, g = 255, b = 0):
        self.overlays_dict['targets_color_rgb'] = (r,g,b)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)
      



    def set_overlay_target_names(self,enabled):
        """Enable or disable the target names overlays.

        Args:
            enabled (bool): True to show targets name data, False to hide it.
        """
        self.overlays_dict['overlay_target_names'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_target_pixels(self,enabled):
        """Enable or disable the target pixels overlays.

        Args:
            enabled (bool): True to show targets pixel data, False to hide it.
        """
        self.overlays_dict['overlay_target_pixels'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_target_degrees(self,enabled):
        """Enable or disable the target degrees overlays.

        Args:
            enabled (bool): True to show targets degrees data, False to hide it.
        """
        self.overlays_dict['overlay_target_degrees'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_target_messages(self,enabled):
        """Enable or disable the target messages overlays.

        Args:
            enabled (bool): True to show targets messages, False to hide it.
        """
        self.overlays_dict['overlay_target_messages'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    def add_target_pixels(self, x_pixel, y_pixel, name = None, color_rgb = None, msg_str = ''):
        """Append a target overlay at pixel location.

        Args:
            x pixel, y pixel, target name str.
        """
        x_ratio = float(x_pixel / self.width_org) 
        y_ratio = float(y_pixel / self.height_org)
        x_deg =  -1 * round(  ((x_ratio - 0.5) * self.width_deg),1)
        y_deg = round(  ((y_ratio - 0.5) * self.height_deg),1)  
        self.add_target_degs(x_deg, y_deg, name, color_rgb, msg_str)


    def add_target_ratios(self, x_ratio, y_ratio, name = None, color_rgb = None, msg_str = ''):
        """Append a target overlay at pixel location.

        Args:
            x pixel, y pixel, target name str.
        """
        x_ratio = nepi_utils.check_ratio(x_ratio)
        y_ratio = nepi_utils.check_ratio(y_ratio)
        x_deg =  -1 * round(  ((x_ratio - 0.5) * self.width_deg),1)
        y_deg = round(  ((y_ratio - 0.5) * self.height_deg),1)  
        self.add_target_degs(x_deg, y_deg, name, color_rgb, msg_str)


    def add_target_degs(self, x_deg, y_deg, name = None, color_rgb = None, msg_str = ''):
        """Append a target overlay at degs offset location.

        Args:
            x deg, y deg, target name str.
        """
        targets_dict = self.overlays_dict['targets_dict']
        target_names = list(targets_dict.keys())
        num_targets = len(target_names)
        ch_name = str(num_targets + 1)
        if name is not None:
            if name != '':
                ch_name = name
        if color_rgb is None:
            color_rgb = self.overlays_dict['targets_color_rgb']
        target_dict = copy.deepcopy(self.BLANK_CROSSHAIR_DICT)
        target_dict['x_deg_offset'] = round(x_deg)
        target_dict['y_deg_offset'] = round(y_deg)
        target_dict['color_rgb'] = color_rgb
        target_dict['msg_str'] = msg_str
        #self.msg_if.pub_info("Adding Target: " + str(target_dict), log_name_list = self.log_name_list)
        needs_update = True
        if ch_name in self.overlays_dict['targets_dict'].keys():
            if self.overlays_dict['targets_dict'][ch_name] == target_dict:
                needs_update = False
        if needs_update == True:
            self.overlays_dict['targets_dict'][ch_name] = target_dict
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def remove_target(self, name):
        """Remove entry from targets overlay dict."""
        try:
            del self.overlays_dict['targets_dict'][name]
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)
        except:
            pass
        


    def clear_targets(self):
        """Clear all entries from targets overlay dict."""
        if len(list(self.overlays_dict['targets_dict'].keys())) > 0:
            self.overlays_dict['targets_dict'] = dict()
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)




    #############################

    def set_live_adjust_enable(self,enabled):
        self.live_adjust_dict['live_adjust_enabled'] = enabled and self.live_adjustments_disabled == False
        self.live_adjust_enabled = self.live_adjust_dict['live_adjust_enabled']
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'live_adjust_enabled', self.live_adjust_enabled)

    def set_live_adjust_rotate_ratio(self,ratio):
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            self.live_adjust_dict['live_adjust_rotate_ratio'] = nepi_utils.check_ratio(ratio)

    def set_live_adjust_rotate_deg(self,deg):
        #self.msg_if.pub_info("Received Live Adjust Rotate Deg: " + str(deg), log_name_list = self.log_name_list)
        if abs(deg) > 180:
                deg = np.sign(deg) * 180
        #self.msg_if.pub_info("Updated Live Adjust Rotate Deg: " + str(deg), log_name_list = self.log_name_list)
        ratio = nepi_utils.check_ratio(0.5 + (deg / 180)/2)
        #self.msg_if.pub_info("Received Live Adjust Rotate Ratio: " + str(ratio), log_name_list = self.log_name_list)
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            self.live_adjust_dict['live_adjust_rotate_ratio'] = nepi_utils.check_ratio(ratio)

    def set_live_adjust_x_ratio(self,ratio):
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            self.live_adjust_dict['live_adjust_x_ratio'] = nepi_utils.check_ratio(ratio)

    def set_live_adjust_x_pixel(self,pixel):
        if abs(pixel) > self.width_org:
            pixel = np.sign(pixel) * self.width_org
        ratio = round(0.5 + (pixel / self.width_org)/2,4)
        ratio = nepi_utils.check_ratio(ratio)
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            self.live_adjust_dict['live_adjust_x_ratio'] = nepi_utils.check_ratio(ratio)

    def set_live_adjust_x_deg(self,deg):

        if abs(deg) > self.width_deg:
            deg = np.sign(deg) * self.width_deg
        ratio = round(0.5 - (deg / self.width_deg)/2,4) 
        ratio = nepi_utils.check_ratio(ratio)
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            #self.msg_if.pub_info("Updating X Rotate Deg to Ratio: " + str(deg) + ":" + str(ratio), log_name_list = self.log_name_list, throttle_s = 5)   
            self.live_adjust_dict['live_adjust_x_ratio'] = nepi_utils.check_ratio(ratio)

    def set_live_adjust_y_ratio(self,ratio):
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            ratio = nepi_utils.check_ratio(ratio)
            self.live_adjust_dict['live_adjust_y_ratio'] = nepi_utils.check_ratio(ratio)

    def set_live_adjust_y_pixel(self,pixel):
        if abs(pixel) > self.height_org:
            pixel = np.sign(pixel) * self.height_org
        ratio = round(0.5 + (pixel / self.height_org)/2,4)
        ratio = nepi_utils.check_ratio(ratio)
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            self.live_adjust_dict['live_adjust_y_ratio'] = nepi_utils.check_ratio(ratio)

    def set_live_adjust_y_deg(self,deg):
        if abs(deg) > self.height_deg:
            deg = np.sign(deg) * self.height_deg
        ratio = round(0.5 + (deg / self.height_deg)/2,4)    
        ratio = nepi_utils.check_ratio(ratio)
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            self.live_adjust_dict['live_adjust_y_ratio'] = nepi_utils.check_ratio(ratio)


    def reset_filters(self):
        """Reset all filter and adjustment controls to factory defaults."""
        # First reset controls to init dict to capture non param managed settings
        self.controls_dict = copy.deepcopy(self.init_controls_dict)

        self.node_if.factory_reset_param('auto_adjust_enabled')
        self.node_if.factory_reset_param('auto_adjust_ratio')
        self.node_if.factory_reset_param('brightness_ratio')
        self.node_if.factory_reset_param('contrast_ratio')
        self.node_if.factory_reset_param('threshold_ratio')
        self.node_if.factory_reset_param('filter_dict')

        self.controls_dict['auto_adjust_enabled'] = self.node_if.get_param(self.node_if_prefix +  'auto_adjust_enabled')
        self.controls_dict['auto_adjust_ratio'] = self.node_if.get_param(self.node_if_prefix +  'auto_adjust_ratio')
        self.controls_dict['brightness_ratio'] = self.node_if.get_param(self.node_if_prefix +  'brightness_ratio')
        self.controls_dict['contrast_ratio'] = self.node_if.get_param(self.node_if_prefix +  'contrast_ratio')
        self.controls_dict['threshold_ratio'] = self.node_if.get_param(self.node_if_prefix +  'threshold_ratio')
        filter_dict = self.node_if.get_param(self.node_if_prefix +  'filter_dict')
        if filter_dict is not None:
            self.filter_dict = filter_dict

        self.publish_status()  
        self.needs_update()


    def reset_overlays(self):
        """Reset all overlay settings to factory defaults."""
        self.node_if.factory_reset_param('overlays_dict')
        self.overlays_dict = self.node_if.get_param(self.node_if_prefix +  'overlays_dict')
       
        
        self.publish_status()  
        self.needs_update()


    def reset_settings(self):
        """Reset resolution, rotation, and flip settings to factory defaults."""
        # First reset controls to init dict to capture non param managed settings
        self.controls_dict = copy.deepcopy(self.init_controls_dict)

        self.node_if.factory_reset_param('resolution_ratio')
        self.node_if.factory_reset_param('rotate_2d_deg')
        self.node_if.factory_reset_param('rotate_2d_swap_box')
        self.node_if.factory_reset_param('flip_horz')
        self.node_if.factory_reset_param('flip_vert')


        self.controls_dict['resolution_ratio'] = self.node_if.get_param(self.node_if_prefix +  'resolution_ratio')
        self.controls_dict['rotate_2d_deg'] = self.node_if.get_param(self.node_if_prefix +  'rotate_2d_deg')
        self.controls_dict['rotate_2d_swap_box'] = self.node_if.get_param(self.node_if_prefix +  'rotate_2d_swap_box')
        self.controls_dict['flip_horz'] = self.node_if.get_param(self.node_if_prefix +  'flip_horz')
        self.controls_dict['flip_vert'] = self.node_if.get_param(self.node_if_prefix +  'flip_vert')

        filter_dict = self.node_if.get_param(self.node_if_prefix +  'filter_dict')
        if filter_dict is not None:
            self.filter_dict = filter_dict

        self.publish_status()  
        self.needs_update()

    def reset_renders(self):
        """Reset all render controls (zoom, pan, window, range, 3-D rotation) to defaults."""
        self.msg_if.pub_warn("Reseting render values", log_name_list = self.log_name_list)
        self.drag_pixel = None
        self.drag_window = None
        self.zoom_ratio = 0
        self.x_ratio = 0.5
        self.y_ratio = 0.5
        self.x_offset = 0
        self.y_offset = 0
        self.x_scaler = 1
        self.y_scaler = 1
        self.controls_dict = copy.deepcopy(self.init_controls_dict)


        # self.controls_dict['start_range_ratio'] = 0
        # self.controls_dict['stop_range_ratio'] = 1
        # self.controls_dict['window_ratios'] = [0,1,0,1]

        # self.window_ratios = [0,1,0,1]

        # self.controls_dict['rotate_2d_ratio'] = 0.5
        # self.controls_dict['rotate_3d_ratio'] = 0.5
        # self.controls_dict['tilt_3d_ratio'] = 0.5

        # self.node_if.factory_reset_param('start_range_ratio')
        # self.node_if.factory_reset_param('stop_range_ratio')

        # self.controls_dict['start_range_ratio'] = self.node_if.get_param(self.node_if_prefix +  'start_range_ratio')
        # self.controls_dict['stop_range_ratio'] = self.node_if.get_param(self.node_if_prefix +  'stop_range_ratio')

        self.live_adjust_dict['live_adjust_rotate_ratio'] = 0.5
        self.live_adjust_dict['live_adjust_x_ratio'] = 0.5
        self.live_adjust_dict['live_adjust_y_ratio'] = 0.5


        self.publish_status()  
        self.needs_update()


    def publish_status(self):
        """Populate the status message from current controls and publish it."""
        if self.node_if is not None and self.status_msg is not None:

            self.status_msg.auto_adjust_enabled = self.controls_dict['auto_adjust_enabled']
            self.status_msg.auto_adjust_ratio = self.controls_dict['auto_adjust_ratio']
            self.status_msg.contrast_ratio = self.controls_dict['contrast_ratio']
            self.status_msg.brightness_ratio = self.controls_dict['brightness_ratio']
            self.status_msg.threshold_ratio = self.controls_dict['threshold_ratio']
            filter_options = []
            filter_states = []
            filter_ratios = []
            if self.filter_dict is not None:
                for name in self.filter_dict.keys():
                    filter_dict = self.filter_dict[name]
                    filter_options.append(name)
                    filter_states.append(filter_dict['enabled'])
                    filter_ratios.append(filter_dict['ratio'])
            self.status_msg.filter_options = filter_options
            self.status_msg.filter_states = filter_states
            self.status_msg.filter_ratios = filter_ratios

            self.status_msg.aspect_adjustment_disabled = self.aspect_adjustment_disabled
            #self.msg_if.pub_warn(self.data_product + " Publishing Aspect Adjust Disabled: " + str(self.aspect_adjustment_disabled ) )
            self.status_msg.aspect_adjust_enabled = self.aspect_adjust_enabled and self.aspect_adjustment_disabled == False
            self.status_msg.aspect_ratio_set = self.aspect_ratio_set

            self.status_msg.aspect_ratio = self.aspect_ratio
            aspect_ratio_str = nepi_img.get_aspect_ratio_str(self.aspect_ratio)
            self.status_msg.aspect_ratio_str = aspect_ratio_str

            self.status_msg.resolution_ratio = self.controls_dict['resolution_ratio']
            self.status_msg.rotate_2d_deg = self.controls_dict['rotate_2d_deg']
            self.status_msg.flip_horz = self.controls_dict['flip_horz'] 
            self.status_msg.flip_vert = self.controls_dict['flip_vert'] 

            self.status_msg.range_ratios.start_range = self.controls_dict['start_range_ratio']
            self.status_msg.range_ratios.stop_range = self.controls_dict['stop_range_ratio']


            self.status_msg.zoom_ratio = self.zoom_ratio
            if self.zoom_ratio > 0.01:
                self.click_crosshair_enabled = False
                self.click_target_enabled = False
            self.status_msg.pan_x_ratio = self.x_ratio
            self.status_msg.pan_y_ratio = self.y_ratio
            self.status_msg.window_x_ratios.start_range = self.controls_dict['window_ratios'][0]
            self.status_msg.window_x_ratios.stop_range = self.controls_dict['window_ratios'][1]
            self.status_msg.window_y_ratios.start_range = self.controls_dict['window_ratios'][2]
            self.status_msg.window_y_ratios.stop_range = self.controls_dict['window_ratios'][3]
            self.status_msg.zoom_3d_ratio = self.controls_dict['zoom_3d_ratio']
            self.status_msg.rotate_3d_ratio = self.controls_dict['rotate_3d_ratio']
            self.status_msg.tilt_3d_ratio = self.controls_dict['tilt_3d_ratio']
            self.status_msg.render_3d_controls_enabled = self.render_3d_controls_enabled

            self.status_msg.camera_fov = self.controls_dict['cam_fov']

            view = self.controls_dict['cam_view']
            cam_view = Vector3()
            cam_view.x = view[0]
            cam_view.y = view[1]
            cam_view.z = view[2]
            self.status_msg.camera_view = cam_view

            pos = self.controls_dict['cam_pos']
            cam_pos = Vector3()
            cam_pos.x = pos[0]
            cam_pos.y = pos[1]
            cam_pos.z = pos[2]
            self.status_msg.camera_position = cam_pos

            rot = self.controls_dict['cam_rot']
            cam_rot = Vector3()
            cam_rot.x = rot[0]
            cam_rot.y = rot[1]
            cam_rot.z = rot[2]
            self.status_msg.camera_rotation = cam_rot


            live_adjust_dict = copy.deepcopy(self.live_adjust_dict)
            live_adjust_enabled = live_adjust_dict['live_adjust_enabled']
            live_adjust_rotate_ratio = live_adjust_dict['live_adjust_rotate_ratio']
            rotate_deg = ((live_adjust_rotate_ratio - 0.5) * 2) * 180
            if abs(rotate_deg) > 180:
                rotate_deg = np.sign(rotate_deg) * 180
            live_adjust_rotate_deg = rotate_deg

            live_adjust_x_ratio = live_adjust_dict['live_adjust_x_ratio']
            shift_x_scaler = (live_adjust_x_ratio - 0.5) * 2
            live_adjust_x_pixels = math.floor((shift_x_scaler * self.width_org))
            live_adjust_x_degs = round(shift_x_scaler * self.width_deg,1)

            live_adjust_y_ratio = live_adjust_dict['live_adjust_y_ratio']
            shift_y_scaler = (live_adjust_y_ratio - 0.5) * 2
            live_adjust_y_pixels = math.floor((shift_y_scaler * self.height_org))
            live_adjust_y_degs = round(shift_y_scaler * self.height_deg,1)

            self.status_msg.live_adjustments_disabled = self.live_adjustments_disabled
            #self.msg_if.pub_warn(self.data_product + " Publishing Live Adjust Disabled: " + str(self.live_adjustments_disabled ) , throttle_s = 5)
            self.status_msg.live_adjust_enabled = live_adjust_enabled
            self.status_msg.live_adjust_rotate_ratio = live_adjust_rotate_ratio
            self.status_msg.live_adjust_rotate_deg = live_adjust_rotate_deg
            self.status_msg.live_adjust_x_ratio = live_adjust_x_ratio
            self.status_msg.live_adjust_x_pixels = live_adjust_x_pixels
            self.status_msg.live_adjust_x_degs = live_adjust_x_degs
            self.status_msg.live_adjust_y_ratio = live_adjust_y_ratio
            self.status_msg.live_adjust_y_pixels = live_adjust_y_pixels
            self.status_msg.live_adjust_y_degs = live_adjust_y_degs


            self.status_msg.overlay_text_enabled = self.overlays_dict['overlay_text_enabled']
            self.status_msg.click_text_enabled = self.click_text_enabled
            self.status_msg.overlay_text_size_ratio = self.overlays_dict['overlay_text_size_ratio']
            self.status_msg.overlay_text_vert_ratio = self.overlays_dict['overlay_text_vert_ratio']
            self.status_msg.overlay_text_horz_ratio = self.overlays_dict['overlay_text_horz_ratio']
            self.status_msg.overlay_text_transparency_ratio = self.overlays_dict['overlay_text_transparency_ratio']
            overlay_text_color_rgb = self.overlays_dict['overlay_text_color_rgb']
            self.status_msg.overlay_text_color_r = overlay_text_color_rgb[0]
            self.status_msg.overlay_text_color_g = overlay_text_color_rgb[1]
            self.status_msg.overlay_text_color_b = overlay_text_color_rgb[2]
            self.status_msg.overlay_text_source_name = self.overlays_dict['overlay_text_img_name']
            self.status_msg.overlay_text_date_time =  self.overlays_dict['overlay_text_date_time']
            self.status_msg.overlay_text_nav = self.overlays_dict['overlay_text_nav']
            self.status_msg.overlay_text_pose = self.overlays_dict['overlay_text_pose']
            self.status_msg.base_overlay_text_list = self.overlays_dict['init_overlay_text_list']
            self.status_msg.add_overlay_text_list = self.overlays_dict['add_overlay_text_list']


            ################
            crosshairs_dict = self.overlays_dict['crosshairs_dict']
            #self.msg_if.pub_info("Publishing crosshairs_dict: " + str(crosshairs_dict), log_name_list = self.log_name_list)
            crosshairs_msg_list = []
            for crosshair_name in crosshairs_dict.keys():
                crosshair_msg = ImageCrosshair()
                crosshair_msg.name = crosshair_name
                crosshair_dict = crosshairs_dict[crosshair_name]
                try:

                    x_deg_offset = crosshair_dict['x_deg_offset'] #round( -1 * ((x_ratio - 0.5) * self.width_deg),1)
                    x_ratio = ((self.width_deg/2) - x_deg_offset)/self.width_deg
                    x_offset_ratio = (0.5 - x_ratio)
                    x_scale = (self.width_proc/self.width_org)
                    x_offset_pixel = int((x_offset_ratio * self.width_org))
                    x_pixel = int((self.width_proc/2) + x_offset_pixel)
                    #self.msg_if.pub_warn("Rendering target x: " + str([x_ratio,x_offset_ratio,x_scale,x_offset_pixel,x_pixel,x_deg_offset]) , log_name_list = self.log_name_list, throttle_s = 5)


                    y_deg_offset = crosshair_dict['y_deg_offset'] #round( -1 * ((y_ratio - 0.5) * self.height_deg),1)
                    y_ratio = (self.height_deg/2) + y_deg_offset
                    y_offset_ratio = -1 * (0.5 - y_ratio)
                    y_scale = (self.height_proc/self.height_org)
                    y_offset_pixel = int((y_offset_ratio * self.height_org))
                    y_pixel = int((self.height_proc/2) + y_offset_pixel)
                    #self.msg_if.pub_warn("Rendering target y: " + str([y_ratio,y_offset_ratio,y_scale,y_offset_pixel,y_pixel,y_deg_offset]) , log_name_list = self.log_name_list, throttle_s = 5)

                    crosshair_msg.x_ratio = x_ratio
                    crosshair_msg.x_offset_deg = x_deg_offset
                    crosshair_msg.x_offset_pixel = x_offset_pixel
                    crosshair_msg.x_pixel = x_pixel

                    crosshair_msg.y_ratio = y_ratio
                    crosshair_msg.y_offset_deg = y_deg_offset
                    crosshair_msg.y_offset_pixel = y_offset_pixel
                    crosshair_msg.y_pixel = y_pixel

                    crosshair_msg.r = crosshair_dict['color_rgb'][0]
                    crosshair_msg.g = crosshair_dict['color_rgb'][1]
                    crosshair_msg.b = crosshair_dict['color_rgb'][2]


                    crosshair_msg.msg_str = str(crosshair_dict['msg_str'])
                    crosshairs_msg_list.append(crosshair_msg)
                except:
                    pass
                    

            self.status_msg.crosshairs_enabled = self.overlays_dict['crosshairs_enabled']
            self.status_msg.click_crosshair_enabled = self.click_crosshair_enabled
            self.status_msg.crosshairs_size_ratio = self.overlays_dict['crosshairs_size_ratio']
            self.status_msg.crosshairs_thickness_ratio = self.overlays_dict['crosshairs_thickness_ratio']
            self.status_msg.crosshairs_text_ratio = self.overlays_dict['crosshairs_text_ratio']
            self.status_msg.crosshairs_transparency_ratio = self.overlays_dict['crosshairs_transparency_ratio']
            crosshairs_color_rgb = self.overlays_dict['crosshairs_color_rgb']
            self.status_msg.crosshairs_color_r = crosshairs_color_rgb[0]
            self.status_msg.crosshairs_color_g = crosshairs_color_rgb[1]
            self.status_msg.crosshairs_color_b = crosshairs_color_rgb[2]
            self.status_msg.overlay_crosshair_names = self.overlays_dict['overlay_crosshair_names']
            self.status_msg.overlay_crosshair_pixels = self.overlays_dict['overlay_crosshair_pixels']
            self.status_msg.overlay_crosshair_degrees = self.overlays_dict['overlay_crosshair_degrees']
            self.status_msg.overlay_crosshair_messages = self.overlays_dict['overlay_crosshair_messages']
            self.status_msg.num_crosshairs = len(list(crosshairs_dict.keys()))
            self.status_msg.crosshairs = crosshairs_msg_list


            ################
            targets_dict = self.overlays_dict['targets_dict']
            #self.msg_if.pub_info("Publishing targets_dict: " + str(targets_dict), log_name_list = self.log_name_list)
            targets_msg_list = []
            for target_name in targets_dict.keys():
                target_msg = ImageTarget()
                target_msg.name = target_name
                target_dict = targets_dict[target_name]
                try:

                    x_deg_offset = target_dict['x_deg_offset'] #round( -1 * ((x_ratio - 0.5) * self.width_deg),1)
                    x_ratio = ((self.width_deg/2) - x_deg_offset)/self.width_deg
                    x_offset_ratio = (0.5 - x_ratio)
                    x_scale = (self.width_proc/self.width_org)
                    x_offset_pixel = int((x_offset_ratio * self.width_org))
                    x_pixel = int((self.width_proc/2) + x_offset_pixel)
                    #self.msg_if.pub_warn("Rendering target x: " + str([x_ratio,x_offset_ratio,x_scale,x_offset_pixel,x_pixel,x_deg_offset]) , log_name_list = self.log_name_list, throttle_s = 5)


                    y_deg_offset = target_dict['y_deg_offset'] #round( -1 * ((y_ratio - 0.5) * self.height_deg),1)
                    y_ratio = ((self.height_deg/2) + y_deg_offset)/self.height_deg
                    y_offset_ratio = -1 * (0.5 - y_ratio)
                    y_scale = (self.height_proc/self.height_org)
                    y_offset_pixel = int((y_offset_ratio * self.height_org))
                    y_pixel = int((self.height_proc/2) + y_offset_pixel)
                    #self.msg_if.pub_warn("Rendering target y: " + str([y_ratio,y_offset_ratio,y_scale,y_offset_pixel,y_pixel,y_deg_offset]) , log_name_list = self.log_name_list, throttle_s = 5)

                    target_msg.x_ratio = x_ratio
                    target_msg.x_offset_deg = x_deg_offset
                    target_msg.x_offset_pixel = x_offset_pixel
                    target_msg.x_pixel = x_pixel

                    target_msg.y_ratio = y_ratio
                    target_msg.y_offset_deg = y_deg_offset
                    target_msg.y_offset_pixel = y_offset_pixel
                    target_msg.y_pixel = y_pixel

                    target_msg.r = target_dict['color_rgb'][0]
                    target_msg.g = target_dict['color_rgb'][1]
                    target_msg.b = target_dict['color_rgb'][2]


                    target_msg.msg_str = str(target_dict['msg_str'])
                    targets_msg_list.append(target_msg)
                except:
                    pass
                    

            self.status_msg.targets_enabled = self.overlays_dict['targets_enabled']
            self.status_msg.click_target_enabled = self.click_target_enabled
            self.status_msg.targets_size_ratio = self.overlays_dict['targets_size_ratio']
            self.status_msg.targets_thickness_ratio = self.overlays_dict['targets_thickness_ratio']
            self.status_msg.targets_text_ratio = self.overlays_dict['targets_text_ratio']
            self.status_msg.targets_transparency_ratio = self.overlays_dict['targets_transparency_ratio']
            targets_color_rgb = self.overlays_dict['targets_color_rgb']
            self.status_msg.targets_color_r = targets_color_rgb[0]
            self.status_msg.targets_color_g = targets_color_rgb[1]
            self.status_msg.targets_color_b = targets_color_rgb[2]
            self.status_msg.overlay_target_names = self.overlays_dict['overlay_target_names']
            self.status_msg.overlay_target_pixels = self.overlays_dict['overlay_target_pixels']
            self.status_msg.overlay_target_degrees = self.overlays_dict['overlay_target_degrees']
            self.status_msg.overlay_target_messages = self.overlays_dict['overlay_target_messages']
            self.status_msg.num_targets = len(list(targets_dict.keys()))
            self.status_msg.targets = targets_msg_list


            self.status_msg.stream_compression_enabled  = self.stream_compression_enabled
            stream_compression_ratio = 0
            if self.stream_compression_enabled == True:
                stream_compression_ratio = self.stream_compression_ratio
            self.status_msg.stream_compression_ratio = stream_compression_ratio

            self.status_msg.publishing = self.needs_data

            avg_rate = 0
            if len(self.time_list) > 0:
                avg_time = sum(self.time_list) / len(self.time_list)
                if avg_time > .01:
                    avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate
            
            if self.node_if is not None:
                # if self.data_product == 'pointcloud_image':
                #     self.msg_if.pub_info("Publishing Status Msg: " + str(self.status_msg), log_name_list = self.log_name_list, throttle_s = 10)
                self.node_if.publish_pub(self.node_if_prefix + 'status_pub',self.status_msg)




    def init(self, do_updates = False):
        """Initialize or re-initialize controls from the parameter server and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            self.controls_dict['resolution_ratio'] = self.node_if.get_param(self.node_if_prefix +  'resolution_ratio')
            self.controls_dict['auto_adjust_enabled'] = self.node_if.get_param(self.node_if_prefix +  'auto_adjust_enabled')
            self.controls_dict['auto_adjust_ratio'] = self.node_if.get_param(self.node_if_prefix +  'auto_adjust_ratio')
            self.controls_dict['brightness_ratio'] = self.node_if.get_param(self.node_if_prefix +  'brightness_ratio')
            self.controls_dict['contrast_ratio'] = self.node_if.get_param(self.node_if_prefix +  'contrast_ratio')
            self.controls_dict['threshold_ratio'] = self.node_if.get_param(self.node_if_prefix +  'threshold_ratio')

            self.controls_dict['rotate_2d_deg'] = self.node_if.get_param(self.node_if_prefix +  'rotate_2d_deg')
            self.controls_dict['rotate_2d_swap_box'] = self.node_if.get_param(self.node_if_prefix +  'rotate_2d_swap_box')
            self.controls_dict['flip_horz'] = self.node_if.get_param(self.node_if_prefix +  'flip_horz')
            self.controls_dict['flip_vert'] = self.node_if.get_param(self.node_if_prefix +  'flip_vert')


            self.controls_dict['window_ratios'] = [0,1,0,1]
            # Read the range window back rather than hardcoding 0/1. Both keys are
            # registered and both write paths persist them, so hardcoding here threw
            # away a correctly saved range window on every restart.
            self.controls_dict['start_range_ratio'] = self.node_if.get_param(self.node_if_prefix +  'start_range_ratio')
            self.controls_dict['stop_range_ratio'] = self.node_if.get_param(self.node_if_prefix +  'stop_range_ratio')


            self.controls_dict['cam_fov'] = self.node_if.get_param(self.node_if_prefix +  'cam_fov')


            # get_param() has no default-value fallback and can legitimately
            # return None if this param hasn't been written back to the ROS
            # param server yet (a startup-timing race with NodeParamsIF's own
            # async init) -- confirmed live: ArdupilotNode crashed here every
            # single time it started, 'NoneType' object is not subscriptable,
            # since publish_status() below indexes cam_view/cam_pos/cam_rot
            # as [x, y, z] unconditionally. Fall back to this same IF's own
            # init_controls_dict value (this class's own class default,
            # never None) rather than letting a transient None propagate.
            cam_view = self.node_if.get_param(self.node_if_prefix +  'cam_view')
            self.controls_dict['cam_view'] = cam_view if cam_view is not None else self.init_controls_dict['cam_view']
            cam_pos = self.node_if.get_param(self.node_if_prefix +  'cam_pos')
            self.controls_dict['cam_pos'] = cam_pos if cam_pos is not None else self.init_controls_dict['cam_pos']
            cam_rot = self.node_if.get_param(self.node_if_prefix +  'cam_rot')
            self.controls_dict['cam_rot'] = cam_rot if cam_rot is not None else self.init_controls_dict['cam_rot']

            self.controls_dict['zoom_3d_ratio'] = self.node_if.get_param(self.node_if_prefix +  'zoom_3d_ratio')
            self.controls_dict['rotate_3d_ratio'] = self.node_if.get_param(self.node_if_prefix +  'rotate_3d_ratio')
            self.controls_dict['tilt_3d_ratio'] = self.node_if.get_param(self.node_if_prefix +  'tilt_3d_ratio')
            self.controls_dict['use_wbg'] = self.node_if.get_param(self.node_if_prefix +  'use_wbg')

            self.status_msg.camera_fov = self.controls_dict['cam_fov']



            self.live_adjust_enabled  = self.node_if.get_param(self.node_if_prefix +  'live_adjust_enabled') and self.live_adjustments_disabled == False
            self.live_adjust_dict['live_adjust_enabled'] = self.live_adjust_enabled

            aspect_adjust_enabled  = self.node_if.get_param(self.node_if_prefix +  'aspect_adjust_enabled')
            self.set_aspect_adjust_enable(aspect_adjust_enabled)
            aspect_ratio_set = self.node_if.get_param(self.node_if_prefix +  'aspect_ratio_set')
            self.set_aspect_adjust_ratio(aspect_ratio_set)

            self.stream_compression_enabled  = self.node_if.get_param(self.node_if_prefix +  'stream_compression_enabled')
            self.stream_compression_ratio = self.node_if.get_param(self.node_if_prefix +  'stream_compression_ratio')


            filter_dict = self.node_if.get_param(self.node_if_prefix +  'filter_dict')
            if filter_dict is not None:
                self.filter_dict = filter_dict
            else:
                self.filter_dict = dict()

            overlays_dict = self.node_if.get_param(self.node_if_prefix +  'overlays_dict')
            if overlays_dict is not None:

                crosshairs_dict = dict()
                if 'crosshairs_dict' in overlays_dict.keys():
                    for name in overlays_dict['crosshairs_dict'].keys():
                        if name != 'click':
                            try:
                                crosshair_dict = overlays_dict['crosshairs_dict'][name]
                                for key in self.BLANK_CROSSHAIR_DICT.keys():
                                    if key not in crosshair_dict.keys():
                                        crosshair_dict[key] = self.BLANK_CROSSHAIR_DICT[key]
                                crosshairs_dict[name] = crosshair_dict
                            except:
                                pass
                overlays_dict['crosshairs_dict'] = crosshairs_dict

                targets_dict = dict()
                if 'targets_dict' in overlays_dict.keys():
                    for name in overlays_dict['targets_dict'].keys():
                        if name != 'click':
                            try:
                                target_dict = overlays_dict['targets_dict'][name]
                                for key in self.BLANK_TARGET_DICT.keys():
                                    if key not in target_dict.keys():
                                        target_dict[key] = self.BLANK_TARGET_DICT[key]
                                targets_dict[name] = target_dict
                            except:
                                pass
                overlays_dict['targets_dict'] = targets_dict

                if overlays_dict is not None:
                    for key in self.overlays_dict.keys():
                        if key in overlays_dict.keys():
                            self.overlays_dict[key] = overlays_dict[key]

        if do_updates == True:
            pass
        self.zoom_ratio = 0
        self.x_ratio = 0.5
        self.y_ratio = 0.5
        self.x_offset = 0
        self.y_offset = 0
        self.x_scaler = 1
        self.y_scaler = 1

        self.publish_status()

    def reset(self):
        """Reset the image interface to its initialized state."""
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        nepi_sdk.sleep(1)     
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset the image interface to factory defaults."""
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

    def _provideCapabilities(self, _):
        return self.caps_report

    def _updaterCb(self, timer):

        # Check for other topics
        image_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),self.data_product)
        depth_map_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'depth_map')
        pointcloud_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'pointcloud')
        found_topics = self.active_topics
        for topic in found_topics:
            if image_ns == topic:
                self.status_msg.image_topic = image_ns
                #self.msg_if.pub_warn("Found depth map topic: " + str(topic), log_name_list = self.log_name_list, throttle_s = 5)
            if depth_map_ns == topic:
                self.status_msg.depth_map_topic = depth_map_ns
            if pointcloud_ns == topic:
                self.status_msg.pointcloud_topic = pointcloud_ns
       
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
        

    def _needsDataCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers(self.node_if_prefix + 'data_pub')
        needs_save = False
        needs_snapshot = False
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
        needs_data = has_subs or needs_save or needs_snapshot
        if needs_data == False and self.status_msg is not None:
            self.status_msg.publishing = False
        self.needs_data = needs_data
        #self.msg_if.pub_warn("Needs Data Check End: " + self.namespace + " : " + str([has_subs,needs_save, needs_snapshot]), log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)

    def _publishStatusCb(self,timer):
        self.publish_status()
        if self.save_config == True and self.node_if is not None:
            self.save_config = False
            self.node_if.save_config()

    def needs_update(self):
        """Signal that a parameter change requires a new frame to be captured or processed.

        Sets the save-config flag and fires the registered needs_update_callback
        if one has been set.
        """
        self.save_config = True
        if self.callback_dict['needs_update_callback'] is not None:
            self.callback_dict['needs_update_callback']()

    def _updateRangesM(self, min_m, max_m):
        if min_m < 0:
            min_m = 0
        if min_m <= max_m:
          self.min_range_m = min_m
          self.max_range_m = max_m
        else:
          self.msg_if.pub_warn("Invalid ranges supplied: " + str([min_m,max_m]), log_name_list = self.log_name_list)



    def _zoomAdjust(self,cv2_img):
        #####################
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1]
        img_height = cv2_shape[0]
        ratio = img_width / img_height

        #####################
        # Apply render controls 
        [xr_min,xr_max,yr_min,yr_max] = copy.deepcopy(self.controls_dict['window_ratios'])
        x_min = int(max(0, img_width * xr_min )) 
        x_max = int(min(img_width, img_width * xr_max))
        y_min = int(max(0, img_height * yr_min))
        y_max = int(min(img_height, img_height * yr_max))

        #self.msg_if.pub_warn("Got Image Window: " + str([x_min,x_max,y_min,y_max]), log_name_list = self.log_name_list)
        cv2_img = cv2_img[y_min:y_max, x_min:x_max]

        self.x_offset = x_min 
        self.y_offset = y_min
        #self.msg_if.pub_info("Image Render: " + str(cv2_img.shape), log_name_list = self.log_name_list)



        ##########
        # Show Drag Box if Needed
        drag_window = copy.deepcopy(self.drag_window)

        #self.msg_if.pub_info("Processing drag_window" + str(drag_window), log_name_list = self.log_name_list)
        if drag_window is not None:
            #self.msg_if.pub_info("Processing drag_window" + str(drag_window), log_name_list = self.log_name_list)
            # Define the rectangle parameters
            x1 = min(drag_window[0], drag_window[1])
            x2 = max(drag_window[0], drag_window[1])
            y1 = min(drag_window[2], drag_window[3])
            y2 = max(drag_window[2], drag_window[3])


            color = (0, 200, 0) # Green color in BGR
            alpha = 0.4 # Transparency factor (0.0 for fully transparent, 1.0 for fully opaque)

            # Dorg a filled rectangle on the overlay copy
            cv2_img = nepi_img.overlay_rectangle(cv2_img, (x1, y1), (x2, y2), color = color, alpha = alpha)
        
        return cv2_img


    def _liveAdjust(self, cv2_img):
        """
        Translates an OpenCV image by a given number of ratio in x and y directions.
        
        Positive shift_x_ratio: moves image right
        Negative shift_x_ratio: moves image left
        Positive shift_y_ratio: moves image down
        Negative shift_y_ratio: moves image up
        """
        # Get adjustment in pixels)
        height, width = cv2_img.shape[:2]


        ########### Live Adjust
        ###### Rotation Update
        live_adjust_dict = copy.deepcopy(self.live_adjust_dict)
        live_adjust_enabled = live_adjust_dict['live_adjust_enabled'] and self.live_adjustments_disabled == False
        if live_adjust_enabled:
            live_adjust_rotate_ratio = live_adjust_dict['live_adjust_rotate_ratio']
            live_adjust_x_ratio = live_adjust_dict['live_adjust_x_ratio']
            live_adjust_y_ratio = live_adjust_dict['live_adjust_y_ratio']

            rotate_deg = ((live_adjust_rotate_ratio - 0.5) * 2) * 180
            if abs(rotate_deg) > 180:
                rotate_deg = np.sign(rotate_deg) * 180
            if abs(rotate_deg) > 0.01:
                #self.msg_if.pub_warn("Live Adjusting Rotate with ratio and degrees: " + str([rotate_ratio,rotate_deg]), log_name_list = self.log_name_list, throttle_s = 5)
                cv2_img = nepi_img.rotate_degrees(cv2_img,rotate_deg)
        
            ###### Translation Update
            shift_x_ratio = nepi_utils.check_ratio(live_adjust_x_ratio)
            shift_x_scaler = (shift_x_ratio - 0.5) * 2
            shift_x_pixels = math.floor((shift_x_scaler * width))
            shift_y_ratio = nepi_utils.check_ratio(live_adjust_y_ratio)
            shift_y_scaler = (shift_y_ratio - 0.5) * 2
            shift_y_pixels = math.floor((shift_y_scaler * height))



            if abs(shift_x_pixels) > 0 or abs(shift_y_pixels) > 0:
                cv2_img = nepi_img.translate_pixels(cv2_img,shift_x_pixels,shift_y_pixels)
 
        
        return cv2_img



    def _mouseEventCb(self,msg):
        #self.msg_if.pub_info("Received mouse event message: " + str(msg), log_name_list = self.log_name_list)
        if self.callback_dict['mouse_event_callback'] is not None:
            try:
                self.callback_dict['mouse_event_callback'](msg)
            except Exception as e:
                self.msg_if.pub_warn("Failed to call mouse mouse_event_callback: " + str(e), log_name_list = self.log_name_list)

        if msg.click_event == True:
            pixel = [int(msg.click.x   + self.x_offset), int(msg.click.y   + self.y_offset)]
            color_bgr = (msg.click.b,msg.click.g,msg.click.r,msg.click.a)
            click_count = msg.click_count
            image_width = self.status_msg.width_px
            image_height = self.status_msg.height_px
            image_fov_horz = self.status_msg.width_deg
            image_fov_vert = self.status_msg.height_deg
            pixel_vert_angle_deg = 0
            pixel_horz_angle_deg = 0
            object_loc_x_ratio_from_center = 0
            object_loc_y_ratio_from_center = 0
            if image_width > 10 and image_height > 10 and image_fov_horz > 10 and image_fov_vert > 10:
                object_loc_x_ratio_from_center = float(pixel[0] - image_width/2) / float(image_width/2)
                object_loc_y_ratio_from_center = float(pixel[0] - image_height/2) / float(image_height/2)
                pixel_vert_angle_deg = (object_loc_y_ratio_from_center * float(image_fov_vert/2))
                pixel_horz_angle_deg = - (object_loc_x_ratio_from_center * float(image_fov_horz/2))
            angles = [pixel_horz_angle_deg,pixel_vert_angle_deg]
            #self.msg_if.pub_warn("Received Click event message: " + str(msg) + " with click crosshair set to: " + str(self.click_crosshair_enabled), log_name_list = self.log_name_list)
            
            x_ratio = float(pixel[0] / self.width_org) 
            y_ratio = float(pixel[1] / self.height_org)
            self.msg_if.pub_warn("Got mouse click pixel", log_name_list = self.log_name_list)
            self.msg_if.pub_warn("Click Pixels: " + str([msg.click.x,msg.click.y]), log_name_list = self.log_name_list)
            self.msg_if.pub_warn("Pixel Offsets: " + str([self.x_offset,self.y_offset]), log_name_list = self.log_name_list)
            self.msg_if.pub_warn("Orig H/W: " + str([self.width_org,self.height_org]), log_name_list = self.log_name_list)
            self.msg_if.pub_warn("Proc H/W: " + str([self.width_proc,self.height_proc]), log_name_list = self.log_name_list)
            self.msg_if.pub_warn("Pixel Ratios: " + str([x_ratio,y_ratio]), log_name_list = self.log_name_list)
            if self.callback_dict['click_pixel_callback'] is not None:
                    try:
                        self.callback_dict['click_pixel_callback'](pixel,color_bgr,click_count,angles)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to call mouse click_pixel_callback: " + str(e), log_name_list = self.log_name_list)
            elif self.click_text_enabled == True and click_count == 1:
                            self.set_overlay_text_horz_ratio(x_ratio)
                            self.set_overlay_text_vert_ratio(y_ratio)
            elif self.click_crosshair_enabled == True and click_count == 1:
                            if self.zoom_ratio < 0.01:
                                click_color_rgb = self.overlays_dict['crosshairs_color_rgb']
                                click_name = 'click'
                                self.add_crosshair_ratios(x_ratio,y_ratio, color_rgb = click_color_rgb, name = click_name)
            elif self.click_target_enabled == True and click_count == 1:
                            if self.zoom_ratio < 0.01:
                                click_color_rgb = self.overlays_dict['targets_color_rgb']
                                click_name = 'click'
                                self.add_target_ratios(x_ratio,y_ratio, color_rgb = click_color_rgb, name = click_name)
            else:
                    if click_count == 1:
                        #self.msg_if.pub_info("Single Click setting pixel value: " + str(pixel), log_name_list = self.log_name_list)
                        self.set_pixel(pixel,color_bgr)
                    else:
                        #self.msg_if.pub_info("Double Click resetting render controls", log_name_list = self.log_name_list)
                        self.reset_renders()

        if msg.drag_event == True:
            self.last_click_time = None
            start_pixel = [int(msg.drag_start.x), int(msg.drag_start.y)]
            start_color_bgr = (msg.drag_start.b,msg.drag_start.g,msg.drag_start.r,msg.drag_start.a)
            stop_pixel = [int(msg.drag_stop.x), int(msg.drag_stop.y)]
            stop_color_bgr = (msg.drag_stop.b,msg.drag_stop.g,msg.drag_stop.r,msg.drag_stop.a)
            #self.msg_if.pub_info("Using drag pixel: " + str(pixel), log_name_list = self.log_name_list)
            if self.callback_dict['drag_callback'] is not None:
                try:
                    self.callback_dict['drag_callback'](start_pixel, start_color_bgr, stop_pixel, stop_color_bgr )
                except Exception as e:
                    self.msg_if.pub_warn("Failed to call mouse drag_callback: " + str(e), log_name_list = self.log_name_list)
            else: #if self.zoom_ratio < 0.01:
                    self.drag_window = [start_pixel[0], stop_pixel[0],start_pixel[1], stop_pixel[1]]
                    self.msg_if.pub_warn("Drag Window Updated: " + str(self.drag_window), log_name_list = self.log_name_list, throttle_s = 1)
                    self.needs_update()




        if msg.window_event == True:
            window = [int(msg.window.x_min   + self.x_offset) , 
                    int(msg.window.x_max   + self.x_offset), 
                    int(msg.window.y_min   + self.y_offset), 
                    int(msg.window.y_max  + self.y_offset)]
            if msg.window.x_min > msg.window.x_max:
                window[0] = msg.window.x_max  + self.x_offset
                window[1] = msg.window.x_min  + self.x_offset
            if msg.window.y_min > msg.window.y_max:
                window[2] = msg.window.y_max   + self.y_offset
                window[3] = msg.window.y_min   + self.y_offset

            if self.callback_dict['window_callback'] is not None:
                try:
                    self.callback_dict['window_callback'](window)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to call mouse window_callback: " + str(e), log_name_list = self.log_name_list)
            else:
                self.set_window(window)
                self.needs_update()
            self.drag_window = None

        if msg.scroll_event == True:
            scroll_pixel = [int(msg.scroll.x), int(msg.scroll.y)]
            scroll_color_bgr = (msg.scroll.b, msg.scroll.g, msg.scroll.r, msg.scroll.a)
            scroll_amount = msg.scroll_amount
            if self.callback_dict['scroll_callback'] is not None:
                try:
                    self.callback_dict['scroll_callback'](scroll_pixel, scroll_color_bgr, scroll_amount)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to call mouse scroll_callback: " + str(e), log_name_list = self.log_name_list)





    ########################
    # Filter Callbacks

    def _setFilterEnableCb(self, msg):
        self.msg_if.pub_info("Received Enable Enhacement message: " + str(msg), log_name_list = self.log_name_list)
        name = msg.name
        enabled = msg.value
        self.set_filter_enable(name,enabled) 

    def _setFilterRatioCb(self, msg):
        self.msg_if.pub_info("Received Ehnacement Ratio update message: " + str(msg), log_name_list = self.log_name_list)
        name = msg.name
        ratio = msg.value
        self.set_filter_ratio(name,ratio) 


    def _setAutoAdjustCb(self, msg):
        self.msg_if.pub_info("Received Auto Adjust Enable update message: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data
        self.set_auto_adjust_enable(enabled)

    def _setAutoAdjustRatioCb(self, msg):
        self.msg_if.pub_info("Received Auto Adjust Ratio update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_auto_adjust_ratio(ratio)

    def _setBrightnessCb(self, msg):
        self.msg_if.pub_info("Received Brightness update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_brightness_ratio(ratio)


    def _setContrastCb(self, msg):
        self.msg_if.pub_info("Received Contrast update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_contrast_ratio(ratio)
        


    def _setThresholdingCb(self, msg):
        self.msg_if.pub_info("Received Threshold update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_threshold_ratio(ratio)



    ########################
    # Res and Orientation Callbacks

    def _setResolutionRatioCb(self, msg):
        self.msg_if.pub_info("Received Resolution update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_resolution_ratio(ratio)


    def _setRotate2dCb(self, msg):
        self.msg_if.pub_info("Received Rotate 2d Deg update message: " + str(msg), log_name_list = self.log_name_list)
        # 90-deg button path: resize-box mode (output box grows to fit).
        self.controls_dict['rotate_2d_keep_size'] = False
        cur_angle = self.controls_dict['rotate_2d_deg']
        new_angle = cur_angle + 90
        if new_angle >= 360:
            new_angle = 0
        new_angle = int(round(new_angle/90.0,0) * 90)
        self.set_rotate_2d_deg(new_angle)

    def _setRotate2dDegCb(self, msg):
        self.msg_if.pub_info("Received Set Rotate 2d Deg message: " + str(msg), log_name_list = self.log_name_list)
        # Free Cam path: fixed-box mode (box holds shape, black fill).
        self.controls_dict['rotate_2d_keep_size'] = True
        new_angle = int(msg.data) % 360
        self.set_rotate_2d_deg(new_angle)

    def _setRotate2dSwapBoxCb(self, msg):
        self.msg_if.pub_info("Received Set Rotate 2d Swap Box message: " + str(msg), log_name_list = self.log_name_list)
        enable = msg.data
        self.set_rotate_2d_swap_box(enable)

    def _setFlipHorzCb(self, msg):
        self.msg_if.pub_info("Received Flip Horz update message: " + str(msg), log_name_list = self.log_name_list)
        enable = msg.data
        self.set_flip_horz(enable)

    def _setFlipVertCb(self, msg):
        self.msg_if.pub_info("Received Flip Vert update message: " + str(msg), log_name_list = self.log_name_list)
        enable = msg.data
        self.set_flip_vert(enable)

    ########################
    # Render Callbacks

    def _setRangeCb(self, msg):
        self.msg_if.pub_info("Received Range update message: " + str(msg), log_name_list = self.log_name_list)
        start_ratio = msg.start_range
        stop_ratio = msg.stop_range
        self.set_range_ratios(start_ratio,stop_ratio)
      

    def _setZoomCb(self, msg):
        self.msg_if.pub_info("Received Zoom update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_zoom_ratio(ratio)

    def _setPanXCb(self, msg):
        self.msg_if.pub_info("Received Pan Left Right update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_x_ratio(ratio)


    def _setPanYCb(self, msg):
        self.msg_if.pub_info("Received Pan Up Down update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_y_ratio(ratio)


    def _setZoom3DCb(self, msg):
        self.msg_if.pub_info("Received Zoom 3D update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_zoom_3d_ratio(ratio) 

    def _setRotate3DCb(self, msg):
        self.msg_if.pub_info("Received Rotate 3D update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_rotate_3d_ratio(ratio) 

    def _setTilt3DCb(self, msg):
        ratio = msg.data
        self.set_tilt_3d_ratio(ratio)

    def render3dControlsCb(self, msg):
        self.msg_if.pub_info("Received Render 3D Controls Enable message: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data
        self.render_3d_controls_enabled = enabled

        if enabled == True:
            self.msg_if.pub_info("Enabling 3D render controls (mouse drag/window)", log_name_list = self.log_name_list)
            if hasattr(self, 'render3dDragHandler'):
                self.set_image_callback('drag_callback', self.render3dDragHandler)
            if hasattr(self, 'render3dWindowHandler'):
                self.set_image_callback('window_callback', self.render3dWindowHandler)
            if hasattr(self, 'render3dScrollHandler'):
                self.set_image_callback('scroll_callback', self.render3dScrollHandler)
            if hasattr(self, 'render3dClickHandler'):
                self.set_image_callback('click_pixel_callback', self.render3dClickHandler)
        else:
            self.msg_if.pub_info("Disabling 3D render controls (mouse drag/window)", log_name_list = self.log_name_list)
            self.clear_image_callback('drag_callback')
            self.clear_image_callback('window_callback')
            self.clear_image_callback('scroll_callback')
            self.clear_image_callback('click_pixel_callback')
        self.publish_status()



    def setCamFovCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        new_val = msg.data
        if new_val > 100:
            new_val = 100
        if new_val < 30:
            new_val = 30
        self.controls_dict['cam_fov'] = new_val
        self.publish_status()
        self.node_if.set_param(self.node_if_prefix +  'cam_fov',new_val)

    def setCamViewCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        new_array = []
        new_array.append(msg.x)
        new_array.append(msg.y)
        new_array.append(msg.z)
        self.controls_dict['cam_view'] = new_array
        self.publish_status()
        self.node_if.set_param(self.node_if_prefix +  'cam_view',new_array)

    def setCamPositionCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        new_array = []
        new_array.append(msg.x)
        new_array.append(msg.y)
        new_array.append(msg.z)
        self.controls_dict['cam_pos'] = new_array
        self.publish_status()
        self.node_if.set_param(self.node_if_prefix +  'cam_pos',new_array)


    def setCamRotationCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        new_array = []
        new_array.append(msg.x)
        new_array.append(msg.y)
        new_array.append(msg.z)
        self.controls_dict['cam_rot'] = new_array
        self.publish_status()
        self.node_if.set_param(self.node_if_prefix +  'cam_rot',new_array)

    def setRangeRatiosCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        min_ratio = msg.start_range
        max_ratio = msg.stop_range
        if min_ratio < max_ratio and min_ratio >= 0 and max_ratio <= 1:
            # controls_dict is what publish_status reports and what the renderer
            # resolves from; a param write alone left the slider snapping back to the
            # old value in the status published on the next line, and the image never
            # re-ranged. Same shape as the setWhiteBgCb note below.
            self.controls_dict['start_range_ratio'] = min_ratio
            self.controls_dict['stop_range_ratio'] = max_ratio
            self.node_if.set_param(self.node_if_prefix +  'start_range_ratio', min_ratio)
            self.node_if.set_param(self.node_if_prefix +  'stop_range_ratio', max_ratio)
            self.publish_status()
            self.needs_update()

    def setWhiteBgCb(self,msg):
        enable = msg.data
        # controls_dict is what the renderer resolves from; a param write alone
        # left this toggle inert until the next node restart.
        self.controls_dict['use_wbg'] = enable
        self.node_if.set_param(self.node_if_prefix +  'use_wbg', enable)
        self.publish_status()
        self.needs_update()

    def resetRender3dControlsCb(self, msg):
        self.msg_if.pub_info("Got reset 3D render orientation msg", log_name_list = self.log_name_list)
        if hasattr(self, 'reset_render_3d_orientation'):
            self.reset_render_3d_orientation()

    def resetRender3dPositionCb(self, msg):
        self.msg_if.pub_info("Got reset 3D render position msg", log_name_list = self.log_name_list)
        if hasattr(self, 'reset_render_3d_position'):
            self.reset_render_3d_position()



    ########################
    # Overlay Callbacks
    def _setrOverlayTextEnableCb(self,msg):
        enabled = msg.data
        self.set_overlay_text_enable(enabled)

    def _setOverlaySizeCb(self,msg):
        ratio = msg.data
        self.set_overlay_text_size_ratio(ratio)


    def _setOverlayVertCb(self,msg):
        ratio = msg.data
        self.set_overlay_text_vert_ratio(ratio)


    def _setOverlayHorzCb(self,msg):
        ratio = msg.data
        self.set_overlay_text_horz_ratio(ratio)

    def _setOverlayTransparencyCb(self,msg):
        ratio = msg.data
        self.set_overlay_text_transparency_ratio(ratio)


    def _setOverlayColorRGBCb(self,msg):
        r = msg.r
        g = msg.g
        b = msg.b
        self.set_overlay_text_color_rgb(r,g,b)

    def _setOverlayImgNameCb(self,msg):
        enabled = msg.data
        self.set_overlay_text_image_name(enabled)

    def _setOverlayDateTimeCb(self,msg):
        enabled = msg.data
        self.set_overlay_text_date_time(enabled)

    def _setOverlayNavCb(self,msg):
        enabled = msg.data
        self.set_overlay_text_nav(enabled)

    def _setOverlayPoseCb(self,msg):
        enabled = msg.data
        self.set_overlay_text_pose(enabled)

    def _setOverlayListCb(self,msg):
        overlay_text_list = msg.array
        self.set_overlay_text_list(overlay_text_list)


    def _setOverlayTextCb(self,msg):
        overlay_text = msg.data
        self.set_overlay_text(overlay_text)


    def _clearOverlayListCb(self,msg):
        self.clear_overlay_text_list()

    ##############################

    def _clickTextEnableCb(self,msg):
        enabled = msg.data
        self.set_click_text(enabled)

    def _clickCrosshairEnableCb(self,msg):
        self.msg_if.pub_info("Got Click Crosshair Enable msg", log_name_list = self.log_name_list)
        enbled = msg.data
        self.set_click_crosshair(enbled)


    def _setCrosshairsSizeRatioCb(self,msg):
        ratio = msg.data
        self.set_crosshairs_size_ratio(ratio)

    def _setCrosshairsThicknessRatioCb(self,msg):
        ratio = msg.data
        self.set_crosshairs_thickness_ratio(ratio)

    def _setCrosshairsTextRatioCb(self,msg):
        ratio = msg.data
        self.set_crosshairs_text_ratio(ratio)

    def _setCrosshairsTransparencyRatioCb(self,msg):
        ratio = msg.data
        self.set_crosshairs_transparency_ratio(ratio)

    def _setCrosshairsColorRGBCb(self,msg):
        r = msg.r
        g = msg.g
        b = msg.b
        self.set_crosshairs_color_rgb(r,g,b)

    def _setrOverlayCrosshairsCb(self,msg):
        enabled = msg.data
        self.set_crosshairs_enable(enabled)


    def _setrOverlayCrosshairNamesCb(self,msg):
        enabled = msg.data
        self.set_overlay_crosshair_names(enabled)

    def _setrOverlayCrosshairPixelsCb(self,msg):
        enabled = msg.data
        self.set_overlay_crosshair_pixels(enabled)
        

    def _setrOverlayCrosshairDegreesCb(self,msg):
        enabled = msg.data
        self.set_overlay_crosshair_degrees(enabled)

    def _setrOverlayCrosshairMessagesCb(self,msg):
        enabled = msg.data
        self.set_overlay_crosshair_messages(enabled)


    def _addCrosshairPixelCb(self,msg):
        name = msg.name
        if name != '':
            name = 'Crosshair'
        x_px = msg.x_pixel
        x_ratio = nepi_utils.check_ratio(x_px/self.width_org)
        y_px = msg.y_pixel
        y_ratio = nepi_utils.check_ratio(y_px/self.height_org)
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_crosshair_enabled = False
        self.add_crosshair_ratios(x_ratio, y_ratio, name = name, color_rgb = (r,g,b), msg_str = msg_str)

    def _addCrosshairRatiosCb(self,msg):
        name = msg.name
        if name != '':
            name = 'Crosshair'
        x_ratio = msg.x_ratio
        y_ratio = msg.y_ratio
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_crosshair_enabled = False
        self.add_crosshair_ratios(x_ratio, y_ratio, name = name, color_rgb = (r,g,b), msg_str = msg_str)


    def _addCrosshairDegreesCb(self,msg):
        self.msg_if.pub_info("Got Add Crosshair Msg: " + str(msg), log_name_list = self.log_name_list)
        name = msg.name
        if name != '':
            name = 'Crosshair'
        x_deg = msg.x_offset_deg
        y_deg = msg.y_offset_deg
       
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_crosshair_enabled = False
        self.add_crosshair_degs(x_deg, y_deg, name = name, color_rgb = (r,g,b), msg_str = msg_str)

    def _removeCrosshairCb(self,msg):
        name = msg.data
        #self.click_crosshair_enabled = False
        self.remove_crosshair(name)


    def _clearCrosshairsCb(self,msg):
        self.click_crosshair_enabled = False
        self.clear_crosshairs()

    ########################################


    def _clickTargetEnableCb(self,msg):
        enbled = msg.data
        self.set_click_target(enbled)


    def _setTargetsSizeRatioCb(self,msg):
        ratio = msg.data
        self.set_targets_size_ratio(ratio)

    def _setTargetsThicknessRatioCb(self,msg):
        ratio = msg.data
        self.set_targets_thickness_ratio(ratio)

    def _setTargetsTextRatioCb(self,msg):
        ratio = msg.data
        self.set_targets_text_ratio(ratio)

    def _setTargetsTransparencyRatioCb(self,msg):
        ratio = msg.data
        self.set_targets_transparency_ratio(ratio)

    def _setTargetsColorRGBCb(self,msg):
        r = msg.r
        g = msg.g
        b = msg.b
        self.set_targets_color_rgb(r,g,b)

    def _setrOverlayTargetsCb(self,msg):
        enabled = msg.data
        self.set_targets_enable(enabled)


    def _setrOverlayTargetNamesCb(self,msg):
        enabled = msg.data
        self.set_overlay_target_names(enabled)

    def _setrOverlayTargetPixelsCb(self,msg):
        enabled = msg.data
        self.set_overlay_target_pixels(enabled)
        

    def _setrOverlayTargetDegreesCb(self,msg):
        enabled = msg.data
        self.set_overlay_target_degrees(enabled)

    def _setrOverlayTargetMessagesCb(self,msg):
        enabled = msg.data
        self.set_overlay_target_messages(enabled)


    def _addTargetPixelCb(self,msg):
        name = msg.name
        if name != '':
            name = 'Target'
        x_px = msg.x_pixel
        x_ratio = nepi_utils.check_ratio(x_px/self.width_org)
        y_px = msg.y_pixel
        y_ratio = nepi_utils.check_ratio(y_px/self.height_org)
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_target_enabled = False
        self.add_target_ratios(x_ratio, y_ratio, name = name, color_rgb = (r,g,b), msg_str = msg_str)

    def _addTargetRatiosCb(self,msg):
        name = msg.name
        if name != '':
            name = 'Target'
        x_ratio = msg.x_ratio
        y_ratio = msg.y_ratio
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_target_enabled = False
        self.add_target_ratios(x_ratio, y_ratio, name = name, color_rgb = (r,g,b), msg_str = msg_str)


    def _addTargetDegreesCb(self,msg):
        self.msg_if.pub_info("Got Add Target Msg: " + str(msg), log_name_list = self.log_name_list)
        name = msg.name
        if name != '':
            name = 'Target'
        x_deg = msg.x_offset_deg
        y_deg = msg.y_offset_deg
       
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_target_enabled = False
        self.add_target_degs(x_deg, y_deg, name = name, color_rgb = (r,g,b), msg_str = msg_str)

    def _removeTargetCb(self,msg):
        name = msg.data
        #self.click_target_enabled = False
        self.remove_target(name)


    def _clearTargetsCb(self,msg):
        self.click_target_enabled = False
        self.clear_targets()

    ########################################
    def _setAspectAdjustEnableCb(self,msg):
        enabled = msg.data
        self.set_aspect_adjust_enable(enabled)

    def _setAspectAdjustRatioCb(self,msg):
        ratio = msg.data
        self.set_aspect_adjust_ratio(ratio)

    def _setAspectAdjustByRatioCb(self,msg):
        ratio = msg.data
        self.set_aspect_adjust_by_ratio(ratio)


    ########################################
    def _setStreamCompressionEnableCb(self,msg):
        enabled = msg.data
        self.set_stream_compression_enable(enabled)

    def _setStreamCompressionRatioCb(self,msg):
        ratio = msg.data
        self.set_stream_compression_ratio(ratio)


    ########################################
    def _setLiveAdjustEnableCb(self,msg):
        enabled = msg.data
        self.set_live_adjust_enable(enabled)

    def _setLiveAdjustRotateRatioCb(self,msg):
        ratio = msg.data
        self.set_live_adjust_rotate_ratio(ratio)

    def _setLiveAdjustRotateDegCb(self,msg):
        deg = msg.data
        self.set_live_adjust_rotate_deg(deg)

    def _setLiveAdjustTranXRatioCb(self,msg):
        ratio = msg.data
        self.set_live_adjust_x_ratio(ratio)

    def _setLiveAdjustTranXPixelCb(self,msg):
        pixel = msg.data
        self.set_live_adjust_x_pixel(pixel)

    def _setLiveAdjustTranXDegCb(self,msg):
        deg = msg.data
        self.set_live_adjust_x_deg(deg)

    def _setLiveAdjustTranYRatioCb(self,msg):
        ratio = msg.data
        self.set_live_adjust_y_ratio(ratio)

    def _setLiveAdjustTranYPixelCb(self,msg):
        pixel = msg.data
        self.set_live_adjust_y_pixel(pixel)

    def _setLiveAdjustTranYDegCb(self,msg):
        deg = msg.data
        self.set_live_adjust_y_deg(deg)

    def _resetControlsCb(self,msg):
        self.reset_filters()
        self.reset_overlays()
        self.reset_settings()
        self.reset_renders()

    def _resetFiltersCb(self,msg):
        self.reset_filters()

    def _resetOverlaysCb(self,msg):
        self.reset_overlays()

    def _resetSettingsCb(self,msg):
        self.reset_settings()
    
    def _resetRendersCb(self,msg):
        self.msg_if.pub_warn("Received reset renders message", log_name_list = self.log_name_list)
        self.reset_renders()

    def _systemStatusCb(self,msg):
        self.active_topics = msg.active_topics
        self.active_topic_types = msg.active_topic_types
        self.active_services = msg.active_services

##################################################
# ImageIF

class ImageIF(BaseImageIF):

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = False,
        has_flip_horz = False,
        has_flip_vert = False,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_zoom_3d = False,
        has_rotate_3d = False,
        has_tilt_3d = False,
        has_camera_3d = False
        )

    DEFAULT_FILTERS_DICT = dict(
    )

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5,
        )

    params_dict = None
    services_dict = None
    pubs_dict = None
    subs_dict = None

    data_product = 'image'


    auto_adjust_controls = []


    
    def __init__(self, namespace = None , 
                data_product = None,
                data_source_description = 'image',
                data_ref_description = 'image',
                perspective = 'pov',
                init_overlay_text_list = [],
                navpose_if = None,
                navpose_namespace = None,
                transform_namespace = None,
                save_data_if = None,
                live_adjustments_disabled = False,
                aspect_adjustment_disabled = False,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
                ):

        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product
    
        self.save_data_if = save_data_if
        self.navpose_if = navpose_if
        # Call the parent class constructor
        super().__init__(namespace , 
                self.data_product,
                data_source_description,
                data_ref_description,
                perspective,
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_FILTERS_DICT, 
                self.params_dict,
                self.services_dict,
                self.pubs_dict,
                self.subs_dict,
                self.save_data_if,
                self.navpose_if,
                navpose_namespace,
                transform_namespace,
                init_overlay_text_list,
                live_adjustments_disabled,
                aspect_adjustment_disabled,
                log_name,
                log_name_list,
                msg_if,
                node_if
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def process_cv2_img(self, cv2_img):
        """Pass the image through unchanged (no processing applied).

        Args:
            cv2_img (numpy.ndarray): Input OpenCV image array.

        Returns:
            numpy.ndarray: The unmodified input image.
        """
        return cv2_img


    ###############################
    # Class Private Methods
    ###############################





##################################################
# ColorImageIF

class ColorImageIF(BaseImageIF):

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = True,
        has_auto_adjust = True,
        has_contrast = True,
        has_brightness = True,
        has_threshold = True,
        has_rotate_2d = True,
        has_flip_horz = True,
        has_flip_vert = True,
        has_range = False,
        has_zoom = True,
        has_pan = True,
        has_window = True,
        has_zoom_3d = False,
        has_rotate_3d = False,
        has_tilt_3d = False,
        has_camera_3d = False
        )

    DEFAULT_FILTERS_DICT = dict(
        # Low_Light = {
        #     'enabled': False,
        #     'function': nepi_img.low_light_filter,
        #     'ratio': 0.5
        # }
    )

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5,
        )

    params_dict = None
    services_dict = None
    pubs_dict = None
    subs_dict = None

    data_product = 'color_image'


    auto_adjust_controls = ['brightness','contrast','threshold']


    
    def __init__(self, namespace = None ,
                data_product = None,
                data_source_description = 'imaging_sensor',
                data_ref_description = 'sensor',
                perspective = 'pov',
                init_overlay_text_list = [],
                navpose_if = None,
                navpose_namespace = None,
                transform_namespace = None,
                save_data_if = None,
                live_adjustments_disabled = False,
                aspect_adjustment_disabled = False,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
                ):

        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product
        self.save_data_if = save_data_if
        self.navpose_if = navpose_if
        self.node_if = node_if
        # Call the parent class constructor
        super().__init__(namespace ,
                self.data_product,
                data_source_description,
                data_ref_description,
                perspective,
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_FILTERS_DICT,
                self.params_dict,
                self.services_dict,
                self.pubs_dict,
                self.subs_dict,
                self.save_data_if,
                self.navpose_if,
                navpose_namespace,
                transform_namespace,
                init_overlay_text_list,
                live_adjustments_disabled,
                aspect_adjustment_disabled,
                log_name,
                log_name_list,
                msg_if,
                node_if
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def process_cv2_img(self, cv2_img):
        """Apply the full color image processing pipeline to an OpenCV image.

        Applies, in order: resolution scaling, 2-D rotation and flips, crop
        window, drag-selection overlay, user-defined filters, and then either
        manual brightness/contrast/sharpness adjustments or auto-adjustment.

        Args:
            cv2_img (numpy.ndarray): Input BGR OpenCV image array.

        Returns:
            numpy.ndarray: The fully-processed BGR image.
        """
        ##########
        # Apply Aspect Controls
        aspect_ratio_set = self.aspect_ratio_set
        if self.aspect_adjust_enabled == True and self.aspect_adjustment_disabled == False:
            try:
                cv2_img = nepi_img.adjust_aspect_ratio(cv2_img, aspect_ratio_set)
            except:
                pass
        cv2_shape_ar = cv2_img.shape
        img_width_ar = cv2_shape_ar[1]
        img_height_ar = cv2_shape_ar[0]
        self.aspect_ratio = (img_width_ar / img_height_ar)
        


        # if res_ratio < 0.9:
        #     [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)


        ##########
        # Apply Resolution Controls
        res_ratio = self.controls_dict['resolution_ratio']
        # cv2_shape = cv2_img.shape
        # img_width1 = cv2_shape[1]
        # img_height1 = cv2_shape[0]
        if res_ratio < 0.9:
            [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)



        ##########
        # Apply Oreantation Controls
        degrees = self.controls_dict['rotate_2d_deg']
        keep_size = self.controls_dict['rotate_2d_keep_size']
        swap_box = self.controls_dict['rotate_2d_swap_box']
        fliph = self.controls_dict['flip_horz']
        flipv = self.controls_dict['flip_vert']

        # Also rotate at angle 0 when the Free Cam box is swapped, so the upright
        # image is re-canvased (centered, black-filled) into the swapped box.
        if degrees != 0 or (keep_size == True and swap_box == True):
           cv2_img = nepi_img.rotate_degrees(cv2_img, degrees) #, keep_size=keep_size, swap_dims=swap_box)

        if fliph == True:
            cv2_img = nepi_img.flip_horz(cv2_img)

        if flipv == True:
            cv2_img = nepi_img.flip_vert(cv2_img)






        ###################
        # Apply Filters
        if self.filter_dict is not None:
            for filter_name in self.filter_dict.keys():
                enabled = self.filter_dict[filter_name]['enabled']
                if enabled == True:
                    ratio = self.filter_dict[filter_name]['ratio']
                    if ratio > 0.05:
                        function = self.filter_dict[filter_name]['function']
                        cv2_img = function(cv2_img,ratio)


        ##########
        # Apply Adjustment Controls
        auto = self.controls_dict['auto_adjust_enabled']
        auto_ratio = self.controls_dict['auto_adjust_ratio']
        brightness = self.controls_dict['contrast_ratio']
        contrast = self.controls_dict['brightness_ratio']
        threshold = self.controls_dict['threshold_ratio']

        if auto is False:
            cv2_img = nepi_img.adjust_brightness(cv2_img, brightness)
            cv2_img = nepi_img.adjust_contrast(cv2_img, contrast)
            cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold)
        else:
            cv2_img = nepi_img.adjust_auto(cv2_img,auto_ratio)

        #self.msg_if.pub_info("Image Filter: " + str(cv2_img.shape), log_name_list = self.log_name_list)


        return cv2_img
        




    ###############################
    # Class Private Methods
    ###############################




##################################################
# DepthMapIF

class DepthMapIF:

    DEFUALT_IMG_WIDTH_PX = 700
    DEFUALT_IMG_HEIGHT_PX = 400

    DEFAULT_WIDTH_DEG = 100
    DEFAULT_HEIGHT_DEG = 70

    DEFAULT_CALLBACK_DICT = dict(
        needs_update_callback = None,
        mouse_event_callback = None,
        click_pixel_callback = None,
        click_angle_callback = None,
        drag_callback = None,
        window_callback = None,
        frame_updated_callback = None
    )

    callback_dict = copy.deepcopy(DEFAULT_CALLBACK_DICT)

    ready = False
    namespace = '~'

    node_if = None
    node_if_shared = True

    status_msg = DepthMapStatus()

    save_config = False

    last_width = DEFUALT_IMG_WIDTH_PX
    last_height = DEFUALT_IMG_HEIGHT_PX

    blank_img = nepi_img.create_cv2_blank_img(DEFUALT_IMG_WIDTH_PX, DEFUALT_IMG_HEIGHT_PX, color_rgb = (0, 0, 0) )

    last_pub_time = None

    needs_data = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    #img_pub_file = 'nepi_depth_map_img_pub_node.py'

    min_range_m = 0.0
    max_range_m = 1.0

    data_source_description = 'depth_map_sensor'
    data_ref_description = 'sensor'

    perspective = 'pov'
    data_product = 'depth_map'

    save_data_if = None
    navpose_if = None
    image_if = None

    pubs_dict = dict()
    subs_dict = dict()

    publishing = False

    active_topics = []
    active_topic_types = []
    active_services = []
    live_adjustments_disabled = False
    aspect_adjustment_disabled = False

    def __init__(self, namespace = None,
                data_product = None,
                data_source_description = 'depth_map_sensor',
                data_ref_description = 'sensor',
                perspective = 'pov',
                pub_image = True,
                save_data_if = None,
                navpose_if = None,
                navpose_namespace = None,
                init_overlay_text_list = [],
                live_adjustments_disabled = False,
                aspect_adjustment_disabled = False,
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
            log_name = nepi_utils.get_clean_name(log_name)
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################    
        # Initialize Class Variables
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product

        if namespace is not None:
            self.namespace = namespace
        if os.path.basename(namespace) != self.data_product:
            namespace = nepi_sdk.create_namespace(namespace,self.data_product)
        self.namespace = nepi_sdk.get_full_namespace(namespace)


        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        '''
                default_min_meters = 0.0,
                default_max_meters = 20.0,
        self._updateRangesM(default_min_meters,default_max_meters)
        '''

        # Initialize Status Msg.  Updated on each publish
        self.perspective = perspective

        self.live_adjustments_disabled = live_adjustments_disabled
        self.aspect_adjustment_disabled = aspect_adjustment_disabled

        if data_source_description is None:
            data_source_description = self.data_source_description
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.data_ref_description
        self.data_ref_description = data_ref_description

        self.status_msg.node_name = self.node_name

        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description



        self.status_msg.publishing = False
        self.status_msg.encoding = '32FC1'
        self.status_msg.width_px = 0
        self.status_msg.height_px = 0
        self.status_msg.get_latency_time = 0
        self.status_msg.pub_latency_time = 0
        self.status_msg.process_time = 0
        self.status_msg.img_pub_enabled = pub_image
    


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
        self.PARAMS_DICT = None

        # Srvs Config Dict ####################
        # DepthMapIF provides no services; None is handled by the node_if setup
        # below and guarded elsewhere (see the 'if self.SRVS_DICT is not None' check).
        self.SRVS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'data_pub': {
                'msg': Image,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            # NOTE: self.node_if_prefix + 'data_pub'/self.node_if_prefix + 'status_pub' keys are generic and identical across every
            # image-IF instance. Safe only while this IF owns its own node_if. If a shared
            # node_if is ever passed in, make these keys namespace-unique first or coexisting
            # instances (and the parent device) will clobber each other (see CLAUDE.md decision log).
            self.node_if_prefix + 'status_pub': {
                'msg': DepthMapStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            self.node_if_prefix + 'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self._systemStatusCb
            }
        }


        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            if self.PARAMS_DICT is not None:
               self.node_if.add_params(self.PARAMS_DICT) 
            self.node_if.register_services(self.SRVS_DICT)
            self.node_if.register_pubs(self.PUBS_DICT)
            self.node_if.register_subs(self.SUBS_DICT)

        else:
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

        self.init(do_updates = True)

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)


        ###############################
        #Setup Image Pub if needed


        ####################
        self.msg_if.pub_info("####################", log_name_list = self.log_name_list)
        self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
        if save_data_if is not None and save_data_if != 'None':
            self.save_data_if = save_data_if
            data_products = self.save_data_if.get_data_products()
            if self.data_product not in data_products:
                self.save_data_if.register_data_product(self.data_product)
        elif save_data_if != 'None':
            
            # Setup Save Data IF Class 
            self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
            factory_data_rates= dict()
            factory_data_rates[self.data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

            factory_filename_dict = {
                'prefix': "", 
                'add_timestamp': True, 
                'add_ms': True,
                'add_us': False,
                'suffix': "",
                'add_node_name': True
                }

            sd_namespace = self.node_namespace
            self.save_data_if = SaveDataIF(namespace = sd_namespace,
                                    data_products = [self.data_product],
                                    factory_rate_dict = factory_data_rates,
                                    factory_filename_dict = factory_filename_dict,
                                    log_name_list = self.log_name_list,
                                    msg_if = self.msg_if,
                                        node_if = self.node_if)
            nepi_sdk.sleep(1)

        if self.save_data_if is not None:
            self.status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.msg_if.pub_warn("Using save_data namespace: " + str(self.status_msg.save_data_topic), log_name_list = self.log_name_list)


        ####################
        ####################
        if navpose_if is not None:
            self.navpose_if = navpose_if
        else:
            # Setup NavPose Connect IF Class
            self.msg_if.pub_info("Starting NavPose IF Initialization")
            np_namespace = self.namespace + '/navpose'
            if navpose_namespace is not None:
                np_namespace = navpose_namespace

            self.navpose_if = ConnectNavPoseIF(namespace = np_namespace,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if)

        if self.navpose_if is not None:
            navpose_topic = self.navpose_if.get_namespace()
            navpose_namespace = navpose_topic
            self.status_msg.navpose_topic = navpose_topic
            self.msg_if.pub_info("Using navpose namespace: " + str(navpose_topic))

            

        ################
        if pub_image == True:
            self.image_if = DepthMapImageIF(namespace = self.namespace, 
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        perspective = self.perspective,
                        init_overlay_text_list = init_overlay_text_list,
                        save_data_if = self.save_data_if,
                        navpose_if = self.navpose_if,
                        navpose_namespace = navpose_namespace,
                        live_adjustments_disabled = self.live_adjustments_disabled,
                        aspect_adjustment_disabled = self.aspect_adjustment_disabled,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if,
                        # NOTE: intentionally NOT sharing self.node_if. DepthMapImageIF
                        # (via BaseImageIF) registers the generic self.node_if_prefix + 'data_pub'/self.node_if_prefix + 'status_pub'
                        # keys, which would clobber DepthMapIF's own entries on a shared
                        # node_if and cross-publish raw 32FC1 (grayscale) and jet-colorized
                        # bgr8 frames onto the same topic (the depth_map flashing bug).
                        # Passing None makes it build its own node_if. See CLAUDE.md decision log.
                        node_if = None
                        )

        self.msg_if.pub_warn("Staring updater process", log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the depth map interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
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
        """Return the ROS namespace for this depth map interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace

    def get_blank_navpose_dict(self):
        """Return a deep copy of the blank nav pose dictionary template.

        Returns:
            dict: A blank nav pose dictionary with all fields at default values.
        """
        blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return blank_navpose_dict

    def get_navpose_dict(self):
        """Return the most recent nav pose dictionary from the associated NavPoseIF.

        Returns:
            dict: The current nav pose data dictionary, or a blank dict if no
            NavPoseIF is attached.
        """
        if self.navpose_if is not None:
            navpose_dict = self.navpose_if.get_navpose_dict()
        else:
            blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return navpose_dict

    def get_data_source_description(self):
        """Return the human-readable data source description string.

        Returns:
            str: Description of the data source (e.g. 'depth_map_sensor').
        """
        return self.data_source_description

    def get_depth_map_callback_options(self):
        """Return the list of supported depth map callback names.

        Returns:
            list: Callback name strings registered in the callback dictionary.
        """
        return list(self.callback_dict.keys())

    def set_depth_map_callback(self,name,function):
        """Register a callable for the named depth map callback slot.

        Args:
            name (str): Name of the callback slot (must be in the callback dict).
            function (callable): Function to call when the event fires.
        """
        self.msg_if.pub_warn("Got set callback for: " + str(name), log_name_list = self.log_name_list)
        if name in self.callback_dict.keys():
            self.msg_if.pub_warn("Callback set for: " + str(name), log_name_list = self.log_name_list)
            self.callback_dict[name] = function
        #self.msg_if.pub_info("Updated callback dict: " + str(self.callback_dict), log_name_list = self.log_name_list)

    def clear_depth_map_callback(self,name):
        """Clear (un-register) the callable for the named depth map callback slot.

        Args:
            name (str): Name of the callback slot to clear.
        """
        self.msg_if.pub_warn("Got clear callback for: " + str(name), log_name_list = self.log_name_list)
        if name in self.callback_dict.keys():
            self.callback_dict[name] = None

    def get_image_callback_options(self):
        """Return the list of supported image callback names from the image sub-interface.

        Returns:
            list: Callback name strings from the DepthMapImageIF, or an empty list
            if no image interface is attached.
        """
        if self.image_if is not None:
            return self.image_if.get_image_callback_options()
        else:
            return []

    def set_image_callback(self,name,function):
        """Register a callable for a named callback slot in the image sub-interface.

        Args:
            name (str): Name of the image callback slot.
            function (callable): Function to call when the event fires.
        """
        if self.image_if is not None:
            self.msg_if.pub_warn("Got set image callback for: " + str(name), log_name_list = self.log_name_list)
            if name in self.image_if.get_image_callback_options():
                self.msg_if.pub_warn("Image Callback set for: " + str(name), log_name_list = self.log_name_list)
                self.image_if.set_callback(name,function)
        #self.msg_if.pub_info("Updated callback dict: " + str(self.callback_dict), log_name_list = self.log_name_list)

    def clear_image_callback(self,name):
        """Clear the callable for a named callback slot in the image sub-interface.

        Args:
            name (str): Name of the image callback slot to clear.
        """
        if self.image_if is not None:
            self.msg_if.pub_warn("Got clear image callback for: " + str(name), log_name_list = self.log_name_list)
            if name in self.image_if.get_image_callback_options():
               self.image_if.callback_dict[name] = None

    def set_image_pub_enabled(self, enabled = True):
        """Declare whether a depth map image is being published for this depth map.

        For producers that were constructed with pub_image = False because they
        publish their own depth map image rather than letting this interface
        colorize the raw array -- a file-backed publisher serving a pre-rendered
        image, for example. Such a producer still owes consumers the standard
        contract: the image published at <namespace>/depth_map_image and
        DepthMapStatus.img_pub_enabled reporting True, which is the flag every
        consumer gates its depth map image topic on.

        This only sets the reported flag. It does not create, destroy or
        reconfigure the internal DepthMapImageIF, and it is not needed by
        producers constructed with pub_image = True, which already report True.
        A producer calling this is responsible for publishing the image at
        <namespace>/depth_map_image itself.

        Note that status_msg is declared at class scope, so co-resident
        DepthMapIF instances in one process share it -- as they already do for
        every other status field this class writes.

        Args:
            enabled (bool, optional): True while a depth map image is being
                published for this depth map, False when none is. Defaults to True.
        """
        self.status_msg.img_pub_enabled = enabled

    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'depth_map').
        """
        return self.data_product

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def needs_data_check(self):
        """Return whether downstream consumers currently need depth map data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        return self.needs_data

    def publish_np_depth_map(self, np_depth_map,
                            encoding = '32FC1',
                            width_deg = 100,
                            height_deg = 70,
                             min_range_m = 0,
                             max_range_m = 100,
                             timestamp = None,
                             pub_twice = False
                            ):
        """Publish a NumPy depth map array as a ROS Image message.

        Also forwards the depth map to the DepthMapImageIF for colorized image
        generatio (if pub_image was True at construction) and saves to disk via
        SaveDataIF if registered.

        Args:
            np_depth_map (numpy.ndarray): 2-D float32 depth map array in meters.
            encoding (str, optional): ROS image encoding. Defaults to '32FC1'.
            width_deg (float, optional): Horizontal field of view in degrees.
                Defaults to 100.
            height_deg (float, optional): Vertical field of view in degrees.
                Defaults to 70.
            min_range_m (float, optional): Minimum valid depth in meters.
                Defaults to 0.
            max_range_m (float, optional): Maximum valid depth in meters.
                Defaults to 100.
            timestamp (float or rospy.Time, optional): Acquisition timestamp.
                Defaults to current time if None.
            pub_twice (bool, optional): Publish the image message twice with a
                brief delay to work around subscriber latching issues.
                Defaults to False.

        Returns:
            numpy.ndarray: The original depth map array unchanged.
        """
        
        if self.navpose_if is not None:
            navpose_dict = self.navpose_if.get_navpose_dict()
        else:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

        #print('Pub Min Max Depths: ' + str([np.nanmin(np_depth_map),np.nanmax(np_depth_map)]) )
        success = False

        if self.publishing == False:
            self.publishing = True

            if np_depth_map is None and self.status_msg is not None:
                self.msg_if.pub_info("Can't publish None image", log_name_list = self.log_name_list)
                return np_depth_map


            self.status_msg.encoding = encoding

            if timestamp == None:
                timestamp = nepi_utils.get_time()
            else:
                timestamp = nepi_sdk.sec_from_timestamp(timestamp)


            current_time = nepi_utils.get_time()
            latency = (current_time - timestamp)
            self.status_msg.get_latency_time = latency
            self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(latency), log_name_list = self.log_name_list, throttle_s = 5.0)

            # Start Img Pub Process
            start_time = nepi_utils.get_time()   

            # Publish and Save org Image Data if Required  
            [height,width] = np_depth_map.shape[0:2]
            last_width = self.status_msg.width_px
            last_height = self.status_msg.height_px
            self.status_msg.width_px = width
            self.status_msg.height_px = height

            self.status_msg.width_deg = width_deg
            self.status_msg.height_deg = height_deg
            #self.msg_if.pub_info('Pub Depth Got Min Max Depths: ' + str([min_range_m, max_range_m]) )
            if (min_range_m is not None and max_range_m is not None):
                self._updateRangesM(min_range_m,max_range_m)
            else:
                self._updateRangesM(0,1)

            #self.msg_if.pub_info('Pub Depth Adj Min Max Depths: ' + str([self.min_range_m, self.max_range_m]) )

            self.status_msg.publishing = True

            if self.node_if is not None and self.needs_data == True:
                #Convert to ros Image message
                try:
                    ros_img = nepi_img.cv2img_to_rosimg(np_depth_map, encoding=encoding)
                    sec = nepi_sdk.sec_from_timestamp(timestamp)
                    ros_img.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = 'None')
                    #self.msg_if.pub_debug("Publishing Image with header: " + str(ros_img.header), log_name_list = self.log_name_list, throttle_s = 5.0)
                    self.node_if.publish_pub(self.node_if_prefix + 'data_pub', ros_img)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to publish Depth Map: " + str(e) , throttle = 5)

            process_time = round( (nepi_utils.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
            latency = (current_time - timestamp)
            self.status_msg.pub_latency_time = latency

            
            
            if self.image_if is not None:
                needs_img = self.image_if.needs_data_check()
                if needs_img == True:
                    self.image_if.publish_depth_map_img(np_depth_map,
                                        min_range_m = self.min_range_m, 
                                        max_range_m = self.max_range_m,
                                        width_deg = width_deg,
                                        height_deg = height_deg,
                                        timestamp = timestamp,
                                        pub_twice = pub_twice
                                    )

            # Save Data
            if self.save_data_if is not None:
                self.save_data_if.save(self.data_product,np_depth_map,timestamp)

            if self.last_pub_time is None:
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time
                self.status_msg.last_pub_sec = pub_time_sec

                self.time_list.pop(0)
                self.time_list.append(pub_time_sec)
            
            self.publishing = False

        return np_depth_map

    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this depth map interface."""          
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)

    def register_pubs(self):
        """Re-register all ROS publishers managed by this depth map interface."""
        if self.node_if is not None:
           self.node_if.register_pubs(self.PUBS_DICT)

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

    def publish_status(self, do_updates = True):
        """Compute the current average publish rate and publish the depth map status message.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to True.
        """
        if self.node_if is not None:

            self.status_msg.min_range_m = self.min_range_m
            self.status_msg.max_range_m = self.max_range_m
            avg_rate = 0
            if len(self.time_list) > 0:
                avg_time = sum(self.time_list) / len(self.time_list)
                if avg_time > .01:
                    avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate

            self.node_if.publish_pub(self.node_if_prefix + 'status_pub',self.status_msg)



    def init(self, do_updates = False):
        """Initialize or re-initialize depth map interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the depth map interface to its initialized state."""
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        nepi_sdk.sleep(1)     
        self.init(do_updates = True)
        self.init()

    def factory_reset(self):
        """Reset the depth map interface to factory defaults."""
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

    def _updaterCb(self, timer):

        # Check for other topics
        image_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'color_image')
        depth_map_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'depth_map')
        pointcloud_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'pointcloud')
        found_topics = self.active_topics
        for topic in found_topics:
            if image_ns == topic:
                self.status_msg.image_topic = image_ns
                #self.msg_if.pub_warn("Found depth map topic: " + str(topic), log_name_list = self.log_name_list, throttle_s = 5)
            if depth_map_ns == topic:
                self.status_msg.depth_map_topic = depth_map_ns
            if pointcloud_ns == topic:
                self.status_msg.pointcloud_topic = pointcloud_ns

        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)


    def _needsDataCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers(self.node_if_prefix + 'data_pub')
        needs_save = False
        needs_snapshot = False
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
        needs_data = has_subs or needs_save or needs_snapshot
        img_needs_data = None
        if self.image_if is not None:
            img_needs_data = self.image_if.needs_data_check()
            needs_data = needs_data or img_needs_data
        if needs_data == False and self.status_msg is not None:
            self.status_msg.publishing = False
        self.needs_data = needs_data
        #self.msg_if.pub_warn("Needs Data Check End: " + self.namespace + " : " + str([has_subs,needs_save, needs_snapshot,img_needs_data]), log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        self.publish_status()
        if self.save_config == True and self.node_if is not None:
            self.save_config = False
            self.node_if.save_config()

    def needs_update(self):
        """Signal that a parameter change requires a new depth map frame.

        Sets the save-config flag and fires the registered needs_update_callback
        if one has been set.
        """
        self.save_config = True
        if self.callback_dict['needs_update_callback'] is not None:
            self.callback_dict['needs_update_callback']()

    def _updateRangesM(self, min_m, max_m):
        if min_m < 0:
            min_m = 0
        if min_m <= max_m:
          self.min_range_m = min_m
          self.max_range_m = max_m
        else:
          self.msg_if.pub_warn("Invalid ranges supplied: " + str([min_m,max_m]), log_name_list = self.log_name_list)


    def _systemStatusCb(self,msg):
        self.active_topics = msg.active_topics
        self.active_topic_types = msg.active_topic_types
        self.active_services = msg.active_services

##################################################
# DepthMapImageIF

class DepthMapImageIF(BaseImageIF):

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = True,
        has_flip_horz = True,
        has_flip_vert = True,
        has_range = True,
        has_zoom = True,
        has_pan = False,
        has_window = False,
        has_zoom_3d = False,
        has_rotate_3d = False,
        has_tilt_3d = False,
        has_camera_3d = False
        )

    DEFAULT_FILTERS_DICT = dict()

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5,
        )

    params_dict = None
    services_dict = None
    pubs_dict = None
    subs_dict = None

    data_product = 'depth_map_image'

    auto_adjust_controls = []



    def __init__(self, namespace = None , 
                data_product = None,
                data_source_description = 'sensor',
                data_ref_description = 'sensor',
                perspective = 'pov',
                init_overlay_text_list = [],
                save_data_if = None,
                navpose_if = None,
                navpose_namespace = None,
                transform_namespace = None,
                live_adjustments_disabled = False,
                aspect_adjustment_disabled = False,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
                ):

        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product
        if log_name is not None:
            log_name = nepi_utils.get_clean_name(log_name)

        self.save_data_if = save_data_if
        self.navpose_if = navpose_if
        # Call the parent class constructor
        super().__init__(namespace , 
                self.data_product,
                data_source_description,
                data_ref_description,
                perspective,
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_FILTERS_DICT, 
                self.params_dict,
                self.services_dict,
                self.pubs_dict,
                self.subs_dict,
                self.save_data_if,
                self.navpose_if,
                navpose_namespace,
                transform_namespace,
                init_overlay_text_list,
                live_adjustments_disabled,
                aspect_adjustment_disabled,
                log_name,
                log_name_list,
                msg_if,
                node_if
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################



    ###############################
    # Class Public Methods
    ###############################

    def publish_depth_map_img(self,np_depth_map,
                            min_range_m = 0,
                            max_range_m = 100,
                            width_deg = 100,
                            height_deg = 70,
                            timestamp = None,
                            pub_twice = False
                            ):
        """Convert a depth map to a colorized image and publish it.

        Applies the configured range ratios to clip the depth map, converts it
        to a BGR colorized image using a jet colormap, then calls publish_cv2_img
        to apply orientation controls and publish the result.

        Args:
            np_depth_map (numpy.ndarray): 2-D float32 depth map in meters.
            min_range_m (float, optional): Minimum depth value for colorization.
                Defaults to 0.
            max_range_m (float, optional): Maximum depth value for colorization.
                Defaults to 100.
            width_deg (float, optional): Horizontal field of view in degrees.
                Defaults to 100.
            height_deg (float, optional): Vertical field of view in degrees.
                Defaults to 70.
            timestamp (float or rospy.Time, optional): Acquisition timestamp.
                Defaults to current time if None.
            pub_twice (bool, optional): Publish the image twice to work around
                subscriber latching issues. Defaults to False.

        Returns:
            numpy.ndarray: The colorized BGR image, or None if input was None.
        """
        if self.navpose_if is not None:
            navpose_dict = self.navpose_if.get_navpose_dict()
        else:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

        cv2_img = None
        if np_depth_map is not None:
            start_range_ratio = self.controls_dict['start_range_ratio']
            stop_range_ratio = self.controls_dict['stop_range_ratio']

            #self.msg_if.pub_info('Pub Img Got Min Max Depths: ' + str([min_range_m, max_range_m]), log_name_list = self.log_name_list)
            cv2_img = nepi_img.npDepthMap_to_cv2ColorImg(np_depth_map, min_range_m = min_range_m, max_range_m = max_range_m,min_ratio = start_range_ratio, max_ratio = stop_range_ratio)      
            if cv2_img is not None:
                self.publish_cv2_img(cv2_img,
                                    encoding = 'bgr8',
                                    width_deg = width_deg,
                                    height_deg = height_deg,
                                    min_range_m = min_range_m, 
                                    max_range_m = max_range_m,
                                    timestamp = timestamp,
                                    pub_twice = pub_twice
                                 )
            
            return cv2_img


    ###############################
    # Class Private Methods
    ###############################



    def process_cv2_img(self, cv2_img):
        """Apply the depth map image processing pipeline to a colorized depth image.

        Applies, in order: resolution scaling, 2-D rotation and flips, crop
        window, drag-selection overlay, user-defined filters, and then either
        manual brightness/contrast/sharpness adjustments or auto-adjustment.

        Args:
            cv2_img (numpy.ndarray): Input BGR colorized depth image array.

        Returns:
            numpy.ndarray: The fully-processed BGR image.
        """

        ##########
        # Apply Resolution Controls
        res_ratio = self.controls_dict['resolution_ratio']
        # cv2_shape = cv2_img.shape
        # img_width1 = cv2_shape[1]
        # img_height1 = cv2_shape[0]
        if res_ratio < 0.9:
            [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)



        ##########
        # Apply Oreantation Controls
        degrees = self.controls_dict['rotate_2d_deg']
        keep_size = self.controls_dict['rotate_2d_keep_size']
        swap_box = self.controls_dict['rotate_2d_swap_box']
        fliph = self.controls_dict['flip_horz']
        flipv = self.controls_dict['flip_vert']

        # Also rotate at angle 0 when the Free Cam box is swapped, so the upright
        # image is re-canvased (centered, black-filled) into the swapped box.
        if degrees != 0 or (keep_size == True and swap_box == True):
           cv2_img = nepi_img.rotate_degrees(cv2_img, degrees) #, keep_size=keep_size, swap_dims=swap_box)

        if fliph == True:
            cv2_img = nepi_img.flip_horz(cv2_img)

        if flipv == True:
            cv2_img = nepi_img.flip_vert(cv2_img) 


        #####################
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1] 
        img_height = cv2_shape[0] 
        ratio = img_width / img_height

        #####################
        # Apply render controls 
        [xr_min,xr_max,yr_min,yr_max] = copy.deepcopy(self.controls_dict['window_ratios'])
        x_min = int(max(0, img_width * xr_min )) 
        x_max = int(min(img_width, img_width * xr_max))
        y_min = int(max(0, img_height * yr_min))
        y_max = int(min(img_height, img_height * yr_max))

        #self.msg_if.pub_warn("Got Image Window: " + str([x_min,x_max,y_min,y_max]), log_name_list = self.log_name_list)
        cv2_img = cv2_img[y_min:y_max, x_min:x_max]

        self.x_offset = x_min 
        self.y_offset = y_min
        #self.msg_if.pub_info("Image Render: " + str(cv2_img.shape), log_name_list = self.log_name_list)



        ##########
        # Show Drag Box if Needed
        drag_window = copy.deepcopy(self.drag_window)

        #self.msg_if.pub_info("Processing drag_window" + str(drag_window), log_name_list = self.log_name_list)
        if drag_window is not None:
            #self.msg_if.pub_info("Processing drag_window" + str(drag_window), log_name_list = self.log_name_list)
            # Define the rectangle parameters
            x1 = min(drag_window[0], drag_window[1])
            x2 = max(drag_window[0], drag_window[1])
            y1 = min(drag_window[2], drag_window[3])
            y2 = max(drag_window[2], drag_window[3])


            color = (0, 200, 0) # Green color in BGR
            alpha = 0.4 # Transparency factor (0.0 for fully transparent, 1.0 for fully opaque)

            # Dorg a filled rectangle on the overlay copy
            cv2_img = nepi_img.overlay_rectangle(cv2_img, (x1, y1), (x2, y2), color = color, alpha = alpha)



        ###################
        # Apply Filters
        if self.filter_dict is not None:
            for filter_name in self.filter_dict.keys():
                enabled = self.filter_dict[filter_name]['enabled']
                if enabled == True:
                    ratio = self.filter_dict[filter_name]['ratio']
                    if ratio > 0.05:
                        function = self.filter_dict[filter_name]['function']
                        cv2_img = function(cv2_img,ratio)


        ##########
        # Apply Adjustment Controls
        auto = self.controls_dict['auto_adjust_enabled']
        auto_ratio = self.controls_dict['auto_adjust_ratio']
        brightness = self.controls_dict['contrast_ratio']
        contrast = self.controls_dict['brightness_ratio']
        threshold = self.controls_dict['threshold_ratio']

        if auto is False:
            cv2_img = nepi_img.adjust_brightness(cv2_img, brightness)
            cv2_img = nepi_img.adjust_contrast(cv2_img, contrast)
            cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold)
        else:
            cv2_img = nepi_img.adjust_auto(cv2_img,auto_ratio)

        #self.msg_if.pub_info("Image Filter: " + str(cv2_img.shape), log_name_list = self.log_name_list)




        return cv2_img




##################################################
# PointcloudIF



class PointcloudIF:

    DEFAULT_WIDTH_DEG = 100
    DEFAULT_HEIGHT_DEG = 70

    Factory_Image_Width = 955
    Factory_Image_Height = 600
    Factory_Zoom_Ratio = .5
    Factory_Rotate_3D_Ratio = .5
    Factory_Tilt_3D_Ratio = .5
    Factory_Cam_FOV = 60
    Factory_Cam_View = [3, 0, 0]
    Factory_Cam_Pos = [-5, 0, 0]
    Factory_Cam_Rot = [0, 0, 1]

    #Default Control Values (single merged dict holding all process + render controls)
    DEFAULT_CONTROLS_DICT = dict(
        clip_enabled = True,
        clip_selection = 'Range',
        clip_min_range_m = -20,
        clip_max_range_m = 20,
        voxel_downsample_size = 0.0, # Zero value skips process
        uniform_downsample_k_points = 0, # Zero value skips process
        outlier_removal_num_neighbors = 0, # Zero value skips process
        outlier_removal_std_ratio = 1.0 # Only applied when outlier_removal_num_neighbors > 0
        )


    ready = False
    namespace = '~'

    node_if = None
    node_if_shared = True

    api_lib_folder = '/opt/nepi/nepi_engine/lib/nepi_api'

    bounding_box3d_topic = "NONE"
    bounding_box3d_sub = None

    launch_node_process = None
    pub_img_node_name = ""
    pub_img_namepace = ""

    status_msg =  PointcloudStatus()

    save_config = False

    last_pub_time = None

    needs_data = False
    needs_data_last_logged = None

    time_list = [0,0,0,0,0,0,0,0,0,0]

    #img_pub_file = 'nepi_pointcloud_img_pub_node.py'

    min_range_m = 0
    max_range_m = 100

    data_source_description = 'pointcloud_sensor'
    data_ref_description = 'sensor'
    perspective = 'pov'
    data_product = 'pointcloud'


    voxel = [0,0,0]

    save_data_if = None
    navpose_if = None
    image_if = None

    pubs_dict = dict()
    subs_dict = dict()

    publishing = False

    active_topics = []
    active_topic_types = []
    active_services = []


    def __init__(self, namespace = None,
                data_product = None,
                data_source_description = 'sensor',
                data_ref_description = 'sensor',
                perspective = 'pov',
                pub_image = True,
                save_data_if = None,
                navpose_if = None,
                navpose_namespace = None,
                init_overlay_text_list = [],
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
            log_name = nepi_utils.get_clean_name(log_name)
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)


        ##############################
        # Wait for System Folders
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
        # Initialize Class Variables
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product

        if namespace is not None:
            self.namespace = namespace
        if os.path.basename(namespace) != self.data_product:
            namespace = nepi_sdk.create_namespace(namespace,self.data_product)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        self.init_overlay_text_list = init_overlay_text_list


        # Initialize Status Msg.  Updated on each publish
        if data_source_description is None:
            data_source_description = self.data_source_description
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.data_ref_description
        self.data_ref_description = data_ref_description


        self.perspective = perspective


        self.status_msg.node_name = self.node_name

        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description


        self.status_msg.publishing = False
        self.status_msg.has_rgb = False
        self.status_msg.has_intensity = False
        self.status_msg.point_count = 0
        self.status_msg.get_latency_time = 0.0
        self.status_msg.pub_latency_time = 0.0
        self.status_msg.process_time = 0.0
        self.status_msg.img_pub_enabled = pub_image

        self.clip_options = ['Range', 'BoundingBox']
        self.frame3d_list = nepi_nav.NAVPOSE_3D_FRAME_OPTIONS


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
        self.PARAMS_DICT = {

            self.node_if_prefix + 'render_enable': {
                'namespace': self.namespace,
                'factory_val': True
            },
            self.node_if_prefix + 'use_wbg': {
                'namespace': self.namespace,
                'factory_val': False
            },
            # setRotateRatioCb/setTiltRatioCb are wired to the live set_rotate_ratio
            # and set_tilt_ratio topics below and PointcloudStatus declares matching
            # fields, but neither key was registered: the writes were dropped, the
            # factory_reset_param calls in resetRenderControls were no-ops, and the
            # status fields never left 0.0.
            self.node_if_prefix + 'rotate_ratio': {
                'namespace': self.namespace,
                'factory_val': 0.5
            },
            self.node_if_prefix + 'tilt_ratio': {
                'namespace': self.namespace,
                'factory_val': 0.5
            },
            self.node_if_prefix + 'frame_3d': {
                'namespace': self.namespace,
                'factory_val': self.frame3d_list[0]
            },
            self.node_if_prefix + 'clip_enabled': {
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['clip_enabled']
            },
            self.node_if_prefix + 'clip_selection': {
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['clip_selection']
            },
            self.node_if_prefix + 'range_min_m': {
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['clip_min_range_m']
            },
            self.node_if_prefix + 'range_max_m': {
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['clip_max_range_m']
            },
            self.node_if_prefix + 'voxel_downsample_size': {
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['voxel_downsample_size']
            },
            self.node_if_prefix + 'uniform_downsample_k_points': {
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['uniform_downsample_k_points']
            },
            self.node_if_prefix + 'outlier_removal_num_neighbors': {
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['outlier_removal_num_neighbors']
            },
            self.node_if_prefix + 'outlier_removal_std_ratio': {
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['outlier_removal_std_ratio']
            }
        }



        # Srvs Config Dict ####################
        # PointcloudIF provides no services; None is handled by the node_if setup
        # below and guarded elsewhere (see the 'if self.SRVS_DICT is not None' check).
        self.SRVS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'data_pub': {
                'msg': PointCloud2,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            # NOTE: self.node_if_prefix + 'data_pub'/self.node_if_prefix + 'status_pub' keys are generic and identical across every
            # image-IF instance. Safe only while this IF owns its own node_if. If a shared
            # node_if is ever passed in, make these keys namespace-unique first or coexisting
            # instances (and the parent device) will clobber each other (see CLAUDE.md decision log).
            self.node_if_prefix + 'status_pub': {
                'msg':  PointcloudStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }


        # Subs Config Dict ###########
        self.SUBS_DICT = {
            self.node_if_prefix + 'reset_controls': {
                'namespace': self.namespace,
                'topic': 'reset_controls',
                'msg': Empty,
                'qsize': 10,
                'callback': self.resetProcessControlsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_clip_enable': {
                'namespace': self.namespace,
                'topic': 'set_clip_enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.clipEnableCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'clip_selection': {
                'namespace': self.namespace,
                'topic': 'set_clip_selection',
                'msg': String,
                'qsize': 10,
                'callback': self.setClipSelectionCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'range_clip_m': {
                'namespace': self.namespace,
                'topic': 'set_range_clip_m',
                'msg': RangeWindow,
                'qsize': 10,
                'callback': self.setRangeMetersCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'clip_bounding_box3d_topic': {
                'namespace': self.namespace,
                'topic': 'set_clip_bounding_box3d_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.setClipBoxTopicCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'voxel_downsample_size': {
                'namespace': self.namespace,
                'topic': 'set_voxel_downsample_size',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setVoxelSizeCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'downsample_k_points': {
                'namespace': self.namespace,
                'topic': 'uniform_downsample_k_points',
                'msg': Int32,
                'qsize': 10,
                'callback': self.setUniformPointsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'outlier_removal': {
                'namespace': self.namespace,
                'topic': 'outlier_removal_num_neighbors',
                'msg': Int32,
                'qsize': 10,
                'callback': self.setOutlierNumCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_rotate_ratio': {
                'namespace': self.namespace,
                'topic': 'set_rotate_ratio',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setRotateRatioCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_tilt_ratio': {
                'namespace': self.namespace,
                'topic': 'set_tilt_ratio',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setTiltRatioCb,
                'callback_args': ()
            },

            self.node_if_prefix + 'set_render_enable': {
                'namespace': self.namespace,
                'topic': 'set_render_enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setRenderEnableCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self._systemStatusCb
            }
        }





        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            if self.PARAMS_DICT is not None:
               self.node_if.add_params(self.PARAMS_DICT) 
            self.node_if.register_services(self.SRVS_DICT)
            self.node_if.register_pubs(self.PUBS_DICT)
            self.node_if.register_subs(self.SUBS_DICT)

        else:
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

        self.init(do_updates = True)

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)


        ####################
        self.msg_if.pub_info("####################", log_name_list = self.log_name_list)
        self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
        if save_data_if is not None and save_data_if != 'None':
            self.save_data_if = save_data_if
            data_products = self.save_data_if.get_data_products()
            if self.data_product not in data_products:
                self.save_data_if.register_data_product(self.data_product)
            if 'pointcloud_image' not in data_products and pub_image == True:
                self.save_data_if.register_data_product('pointcloud_image')
        elif save_data_if != 'None':
            
            # Setup Save Data IF Class 
            self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
            factory_data_rates= dict()
            factory_data_rates[self.data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
            if pub_image == True:
                factory_data_rates['pointcloud_image'] = [1.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
            

            factory_filename_dict = {
                'prefix': "", 
                'add_timestamp': True, 
                'add_ms': True,
                'add_us': False,
                'suffix': "",
                'add_node_name': True
                }

            sd_namespace = self.node_namespace
            self.save_data_if = SaveDataIF(namespace = sd_namespace,
                                    data_products = [self.data_product],
                                    factory_rate_dict = factory_data_rates,
                                    factory_filename_dict = factory_filename_dict,
                                    log_name_list = self.log_name_list,
                                    msg_if = self.msg_if,
                                    node_if = self.node_if)
            nepi_sdk.sleep(1)

        if self.save_data_if is not None:
            self.status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic), log_name_list = self.log_name_list)

        ####################
        ####################
        if navpose_if is not None:
            self.navpose_if = navpose_if
        else:
            # Setup NavPose Connect IF Class
            self.msg_if.pub_info("Starting NavPose IF Initialization")
            np_namespace = self.namespace + '/navpose'
            if navpose_namespace is not None:
                np_namespace = navpose_namespace

            self.navpose_if = ConnectNavPoseIF(namespace = np_namespace,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if)

        if self.navpose_if is not None:
            navpose_topic = self.navpose_if.get_namespace()
            navpose_namespace = navpose_topic
            self.status_msg.navpose_topic = navpose_topic
            self.msg_if.pub_info("Using navpose namespace: " + str(navpose_topic))

        ####################
        self.pub_image = pub_image
        if pub_image == True:
            self.msg_if.pub_warn("Launching Image Pub Node")
            self.launch_image_pub_node()

        ####################
        self.msg_if.pub_warn("Staring updater process", log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    def launch_image_pub_node(self):
        """Launch the pointcloud image publisher node as a subprocess.

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
        node_file_name = 'nepi_pointcloud_img_pub_node.py'

        ###############################
        # Launch Node
        node_file_path = os.path.join(node_file_folder, node_file_name)
        if self.launch_node_process is not None:
            self.msg_if.pub_warn("Node Already Launched: " + node_name)
        elif os.path.exists(node_file_path) == False or self.pub_image == False:
            self.msg_if.pub_warn("Could not find Node File at: " + node_file_path)
        else:
            #Try and launch node
            self.msg_if.pub_warn("Launching Pointcloud Img Node with settings " + str([pkg_name, node_file_name, node_name]))
            self.msg_if.pub_warn("Launching Node: " + node_name)

            param_ns = nepi_sdk.create_namespace(node_namespace, 'data_products')
            nepi_sdk.set_param(param_ns, [self.data_product])

            param_ns = nepi_sdk.create_namespace(node_namespace, 'pointcloud_namespace')
            nepi_sdk.set_param(param_ns, self.namespace)

            param_ns = nepi_sdk.create_namespace(node_namespace, 'navpose_namespace')
            nepi_sdk.set_param(param_ns, self.status_msg.navpose_topic)

            [success, msg, sub_process] = nepi_sdk.launch_node(pkg_name, node_file_name, node_name, namespace=launch_namespace)
            if success == True:
                self.launch_node_process = sub_process
                self.pub_img_node_name = node_name
                self.pub_img_namepace = node_namespace
            self.msg_if.pub_warn("Node launch return msg: " + str(msg))

    def kill_image_pub_node(self):
        """Terminate the running pointcloud image publisher node.

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


    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the pointcloud interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
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
        """Return the ROS namespace for this pointcloud interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace

    def get_blank_navpose_dict(self):
        """Return a deep copy of the blank nav pose dictionary template.

        Returns:
            dict: A blank nav pose dictionary with all fields at default values.
        """
        blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return blank_navpose_dict

    def get_navpose_dict(self):
        """Return the most recent nav pose dictionary from the associated NavPoseIF.

        Returns:
            dict: The current nav pose data dictionary, or a blank dict if no
            NavPoseIF is attached.
        """
        if self.navpose_if is not None:
            navpose_dict = self.navpose_if.get_navpose_dict()
        else:
            blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return navpose_dict

    def get_data_source_description(self):
        """Return the human-readable data source description string.

        Returns:
            str: Description of the data source (e.g. 'pointcloud_sensor').
        """
        return self.data_source_description


    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'pointcloud').
        """
        return self.data_product

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def needs_data_check(self):
        """Return whether downstream consumers currently need pointcloud data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        needs_data = copy.deepcopy(self.needs_data)
        if needs_data != self.needs_data_last_logged:
            self.msg_if.pub_warn("Returning: " + self.namespace + " " "needs data: " + str(needs_data), log_name_list = self.log_name_list)
            self.needs_data_last_logged = needs_data
        return needs_data

    def _setVoxelCb(self,msg):
        self.msg_if.pub_info("Received set voxel message: " + str(msg), log_name_list = self.log_name_list)
        self.voxel = [msg.x,msg.y,msg.z]
        if self.callback_dict['voxel_callback'] is not None:
            self.callback_dict['voxel_callback'](self.voxel)

    def publish_o3d_pc(self,o3d_pc,
                        timestamp = None,
                        width_deg = 100,
                        height_deg = 70,
                        min_range_m = None,
                        max_range_m = None,
                        process_data = True,
                        pub_twice = False,
                        frame_id = 'sensor',
                        add_pubs = []):
        """Publish an Open3D pointcloud as a ROS PointCloud2 message.

        Also forwards the pointcloud to the PointcloudImageIF for image generatio
        (if pub_image was True at construction) and saves to disk via SaveDataIF
        if registered.

        Args:
            o3d_pc (open3d.geometry.PointCloud): The pointcloud to publish.
            timestamp (float or rospy.Time, optional): Acquisition timestamp.
                Defaults to current time if None.
            width_deg (float, optional): Horizontal field of view in degrees.
                Defaults to 100.
            height_deg (float, optional): Vertical field of view in degrees.
                Defaults to 70.
            min_range_m (float, optional): Minimum valid range in meters. When
                None, range status is set to 0/1 defaults. Defaults to None.
            max_range_m (float, optional): Maximum valid range in meters.
                Defaults to None.
            process_data (bool, optional): Reserved for future use. Defaults to True.
            pub_twice (bool, optional): Publish the pointcloud twice with a brief
                delay. Defaults to False.
            frame_id (str, optional): TF frame ID for the PointCloud2 header.
                Defaults to 'sensor'.
            add_pubs (list, optional): Additional namespace strings to publish to.
                Defaults to [].

        Returns:
            open3d.geometry.PointCloud: The original pointcloud unchanged.
        """
    


        if self.navpose_if is not None:
            navpose_dict = self.navpose_if.get_navpose_dict()
        else:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

        if self.publishing == False:
            self.publishing = True

            if o3d_pc is None:
                self.msg_if.pub_info("Can't publish None image", log_name_list = self.log_name_list)
                return False

            if timestamp == None:
                timestamp = nepi_utils.get_time()
            else:
                timestamp = nepi_sdk.sec_from_timestamp(timestamp)

            self.status_msg.has_rgb = o3d_pc.has_colors()
            self.status_msg.has_intensity = False # Need to add
            #self.status_msg.point_count = o3d_pc.point["colors"].shape[0]

            self.status_msg.width_deg = width_deg
            self.status_msg.height_deg = height_deg

            if (min_range_m is not None and max_range_m is not None):
                self._updateRangesM(min_range_m,max_range_m)
                self.status_msg.range_min_max_m.start_range = self.min_range_m
                self.status_msg.range_min_max_m.stop_range = self.max_range_m
                o3d_pc = nepi_pc.range_clip_spherical(o3d_pc, self.min_range_m, self.max_range_m)

            else:
                self.status_msg.range_min_max_m.start_range = 0
                self.status_msg.range_min_max_m.stop_range = 1



            current_time = nepi_utils.get_time()
            latency = (current_time - timestamp)
            self.status_msg.get_latency_time = latency
            #self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(latency), log_name_list = self.log_name_list, throttle_s = 5.0)

            # Start Pub Process. Timed from here so process_time covers the process controls
            # below as well as the message conversion and publish.
            start_time = nepi_utils.get_time()

            ##########
            # Apply Process Controls
            # Order is voxel, then uniform, then outlier removal. Outlier removal runs last on
            # the already-reduced cloud even though its neighbor statistics are more meaningful
            # at native density: it builds a KD-tree and runs a per-point k-nearest-neighbor
            # query, by far the most expensive of the three, and is not viable at frame rate on
            # a full-resolution stereo cloud of ~2M points. Decimate first, then filter.
            # Each stage is a no-op at its factory value, so a device that never sets these
            # controls sees the pointcloud it saw before.
            pre_process_pc = o3d_pc

            # Read the controls once per publish call, never per point
            voxel_size_m = None
            uniform_k_points = None
            outlier_num_neighbors = None
            outlier_std_ratio = None
            if self.node_if is not None:
                voxel_size_m = self.node_if.get_param(self.node_if_prefix +  'voxel_downsample_size')
                uniform_k_points = self.node_if.get_param(self.node_if_prefix +  'uniform_downsample_k_points')
                outlier_num_neighbors = self.node_if.get_param(self.node_if_prefix +  'outlier_removal_num_neighbors')
                outlier_std_ratio = self.node_if.get_param(self.node_if_prefix +  'outlier_removal_std_ratio')

            if voxel_size_m is not None and voxel_size_m > 0:
                try:
                    o3d_pc = nepi_pc.voxel_down_sampling(o3d_pc, voxel_size_m)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to voxel downsample pointcloud with size: " + str(voxel_size_m) + " " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)

            # k of zero is invalid input to Open3D and k of one returns every point
            if uniform_k_points is not None and uniform_k_points > 1:
                try:
                    o3d_pc = nepi_pc.uniform_down_sampling(o3d_pc, uniform_k_points)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to uniform downsample pointcloud with k points: " + str(uniform_k_points) + " " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)

            if outlier_num_neighbors is not None and outlier_num_neighbors > 0:
                if outlier_std_ratio is None:
                    outlier_std_ratio = self.DEFAULT_CONTROLS_DICT['outlier_removal_std_ratio']
                try:
                    o3d_pc = nepi_pc.statistical_outlier_removal(o3d_pc, outlier_num_neighbors, outlier_std_ratio)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to remove pointcloud outliers with neighbors and std ratio: " + str([outlier_num_neighbors,outlier_std_ratio]) + " " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)

            # Don't let an over-aggressive control setting blank the data product silently
            if o3d_pc is None or len(o3d_pc.points) == 0:
                self.msg_if.pub_warn("Process controls returned an empty pointcloud, publishing unprocessed data. Controls: " + str([voxel_size_m,uniform_k_points,outlier_num_neighbors]), log_name_list = self.log_name_list, throttle_s = 5.0)
                o3d_pc = pre_process_pc

            self.status_msg.point_count = len(o3d_pc.points)


            if self.node_if is not None and self.needs_data == True:
                #Convert to ros Image message
                ros_pc = nepi_pc.o3dpc_to_rospc(o3d_pc)
                sec = nepi_sdk.sec_from_timestamp(timestamp)
                ros_pc.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = 'sensor')
                #self.msg_if.pub_debug("Publishing Pointcloud with header: " + str(ros_pc.header), log_name_list = self.log_name_list, throttle_s = 5.0)
                self.node_if.publish_pub(self.node_if_prefix + 'data_pub', ros_pc)

            process_time = round( (nepi_utils.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
            latency = (current_time - timestamp)
            self.status_msg.pub_latency_time = latency

            # Save Data
            if self.save_data_if is not None:
                self.save_data_if.save(self.data_product,o3d_pc,timestamp)


            if self.last_pub_time is None:
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time
                self.status_msg.last_pub_sec = pub_time_sec

                self.time_list.pop(0)
                self.time_list.append(pub_time_sec)

            self.publishing = False
            
        return o3d_pc
    
    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this depth map interface."""          
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)

    def register_pubs(self):
        """Re-register all ROS publishers managed by this pointcloud interface."""
        if self.node_if is not None:
            self.node_if.register_pubs()

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


    def publish_status(self, do_updates = True):
        """Read current parameters and publish the pointcloud status message.

        Args:
            do_updates (bool, optional): When True, re-reads image size, range,
                zoom, 3-D camera, and white-background parameters from the
                parameter server before publishing. Defaults to True.
        """
        if self.node_if is not None:

            if do_updates == True:
                self.status_msg.clip_enabled = self.node_if.get_param(self.node_if_prefix +  'clip_enabled')
                self.status_msg.clip_options = self.clip_options
                self.status_msg.clip_selection = self.node_if.get_param(self.node_if_prefix +  'clip_selection')

                clip_meters = RangeWindow()
                clip_meters.start_range =   float(self.node_if.get_param(self.node_if_prefix +  'range_min_m'))
                clip_meters.stop_range =   float(self.node_if.get_param(self.node_if_prefix +  'range_max_m'))
                self.status_msg.clip_meters = clip_meters

                self.status_msg.clip_target_topic = self.bounding_box3d_topic


                self.status_msg.voxel_downsample_size_m = self.node_if.get_param(self.node_if_prefix +  'voxel_downsample_size')
                self.status_msg.uniform_downsample_points = self.node_if.get_param(self.node_if_prefix +  'uniform_downsample_k_points')
                self.status_msg.outlier_k_points = self.node_if.get_param(self.node_if_prefix +  'outlier_removal_num_neighbors')

                self.status_msg.render_enable = self.node_if.get_param(self.node_if_prefix +  'render_enable')

                # Degrees use the same conversion the renderer applies to its own
                # rotate_3d_ratio/tilt_3d_ratio, so the two agree on what 0.5 means.
                rotate_ratio = self.node_if.get_param(self.node_if_prefix +  'rotate_ratio')
                self.status_msg.rotate_ratio = rotate_ratio
                self.status_msg.rotate_deg = (0.5 - rotate_ratio) * 2 * 180
                tilt_ratio = self.node_if.get_param(self.node_if_prefix +  'tilt_ratio')
                self.status_msg.tilt_ratio = tilt_ratio
                self.status_msg.tilt_deg = (0.5 - tilt_ratio) * 2 * 180

                self.status_msg.range_min_max_m.start_range = self.min_range_m
                self.status_msg.range_min_max_m.stop_range = self.max_range_m

            avg_rate = 0
            if len(self.time_list) > 0:
                avg_time = sum(self.time_list) / len(self.time_list)
                if avg_time > .01:
                    avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate

            self.node_if.publish_pub(self.node_if_prefix + 'status_pub',self.status_msg)


    def init(self, do_updates = False):
        """Initialize or re-initialize pointcloud interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the pointcloud interface to its initialized state."""
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        nepi_sdk.sleep(1)     
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset the pointcloud interface to factory defaults."""
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

    def _updaterCb(self, timer):

        # Check for other topics
        image_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'color_image')
        depth_map_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'depth_map')
        pointcloud_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'pointcloud')
        found_topics = self.active_topics
        for topic in found_topics:
            if image_ns == topic:
                self.status_msg.image_topic = image_ns
                #self.msg_if.pub_warn("Found depth map topic: " + str(topic), log_name_list = self.log_name_list, throttle_s = 5)
            if depth_map_ns == topic:
                self.status_msg.depth_map_topic = depth_map_ns
            if pointcloud_ns == topic:
                self.status_msg.pointcloud_topic = pointcloud_ns



        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)

    def _needsDataCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers(self.node_if_prefix + 'data_pub')
        needs_save = False
        needs_snapshot = False
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
        needs_data = has_subs or needs_save or needs_snapshot
        img_needs_data = None
        if self.image_if is not None:
            img_needs_data = self.image_if.needs_data_check()
            needs_data = needs_data or img_needs_data
        if needs_data == False and self.status_msg is not None:
            self.status_msg.publishing = False
        self.needs_data = needs_data
        #self.msg_if.pub_warn("Needs Data Check End: " + self.namespace + " : " + str([has_subs,needs_save, needs_snapshot,img_needs_data]), log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        self.publish_status()
        if self.save_config == True and self.node_if is not None:
            self.save_config = False
            self.node_if.save_config()

    def needs_update(self):
        """Signal that a parameter change requires a new pointcloud frame.

        Sets the save-config flag and fires the registered needs_update_callback
        if one has been set.
        """
        self.save_config = True
        if self.callback_dict['needs_update_callback'] is not None:
            self.callback_dict['needs_update_callback']()

    def _updateRangesM(self, min_m, max_m):
        if min_m < 0:
            min_m = 0
        if min_m <= max_m:
          self.min_range_m = min_m
          self.max_range_m = max_m
        else:
          self.msg_if.pub_warn("Invalid ranges supplied: " + str([min_m,max_m]), log_name_list = self.log_name_list)

    def _systemStatusCb(self,msg):
        self.active_topics = msg.active_topics
        self.active_topic_types = msg.active_topic_types
        self.active_services = msg.active_services

    ###################
    ## Process Control Public API
    #
    # These exist so a host node (e.g. a driver exposing these as SettingsIF
    # cap_settings entries) can drive the process pipeline directly instead of
    # constructing a ROS message to hand to the subscriber callbacks below.
    # Each accepts a value that str()s to a number, so a SettingsIF string value
    # can be passed through unparsed, and returns whether the value was applied.

    def set_voxel_downsample_size(self, size_m):
        """Set the voxel downsample size for the pointcloud process pipeline.

        Args:
            size_m (float): Voxel edge length in meters. Zero disables the voxel
                downsample stage. Negative values are rejected.

        Returns:
            bool: True if the value was applied, False if it was rejected.
        """
        success = False
        try:
            val = float(size_m)
        except (TypeError, ValueError):
            val = None
        if val is not None and val >= 0:
            self.node_if.set_param(self.node_if_prefix +  'voxel_downsample_size', val)
            success = True
        self.publish_status()
        return success

    def set_uniform_downsample_k_points(self, k_points):
        """Set the uniform downsample sampling rate for the process pipeline.

        Args:
            k_points (int): Keep every k-th point. Zero or one disables the
                uniform downsample stage. Negative values are rejected.

        Returns:
            bool: True if the value was applied, False if it was rejected.
        """
        success = False
        try:
            val = int(k_points)
        except (TypeError, ValueError):
            val = None
        if val is not None and val >= 0:
            self.node_if.set_param(self.node_if_prefix +  'uniform_downsample_k_points', val)
            success = True
        self.publish_status()
        return success

    def set_outlier_removal_num_neighbors(self, num_neighbors):
        """Set the neighbor count for the statistical outlier removal stage.

        Args:
            num_neighbors (int): Neighbors considered per point. Zero disables
                the outlier removal stage. Negative values are rejected.

        Returns:
            bool: True if the value was applied, False if it was rejected.
        """
        success = False
        try:
            val = int(num_neighbors)
        except (TypeError, ValueError):
            val = None
        if val is not None and val >= 0:
            self.node_if.set_param(self.node_if_prefix +  'outlier_removal_num_neighbors', val)
            success = True
        self.publish_status()
        return success

    def set_outlier_removal_std_ratio(self, std_ratio):
        """Set the standard deviation ratio for statistical outlier removal.

        Lower values cull more aggressively. Has no effect unless the outlier
        removal neighbor count is greater than zero. 

        Args:
            std_ratio (float): Standard deviation ratio. Must be greater than
                zero; zero and negative values are rejected.

        Returns:
            bool: True if the value was applied, False if it was rejected.
        """
        success = False
        try:
            val = float(std_ratio)
        except (TypeError, ValueError):
            val = None
        if val is not None and val > 0:
            self.node_if.set_param(self.node_if_prefix +  'outlier_removal_std_ratio', val)
            success = True
        self.publish_status()
        return success

    ###################
    ## Process Callbacks
    def resetProcessControlsCb(self,msg):
        self.resetProcessControls()

    def resetProcessControls(self,do_updates = True):
        self.node_if.factory_reset_param('clip_enabled')
        self.node_if.factory_reset_param('clip_selection')
        self.node_if.factory_reset_param('range_min_m')
        self.node_if.factory_reset_param('range_max_m')
        self.bounding_box3d_topic = "NONE"
        self.node_if.factory_reset_param('voxel_downsample_size')
        self.node_if.factory_reset_param('uniform_downsample_k_points')
        self.node_if.factory_reset_param('outlier_removal_num_neighbors')
        self.node_if.factory_reset_param('outlier_removal_std_ratio')

        if do_updates:
            self.publish_status()

    def clipEnableCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        new_enable = msg.data
        self.node_if.set_param(self.node_if_prefix +  'clip_enabled', new_enable)
        self.publish_status()

    def setClipSelectionCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        sel = msg.data
        if sel in self.clip_options:
            self.node_if.set_param(self.node_if_prefix +  'clip_selection', sel )
        self.publish_status()

    def setClipBoxTopicCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        self.bounding_box3d_topic = msg.data
        self.publish_status()

    def setRangeMetersCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        range_min_m = msg.start_range
        range_max_m = msg.stop_range
        if range_min_m < range_max_m:
            self.node_if.set_param(self.node_if_prefix +  'range_min_m', range_min_m)
            self.node_if.set_param(self.node_if_prefix +  'range_max_m', range_max_m)
        self.publish_status()

    def setVoxelSizeCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        self.set_voxel_downsample_size(msg.data)

    def setUniformPointsCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        self.set_uniform_downsample_k_points(msg.data)

    def setOutlierNumCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        self.set_outlier_removal_num_neighbors(msg.data)

    def setFrame3dCb(self, msg):
        #self.msg_if.pub_info(str(msg))
        frame_3d = msg.data
        frame3d_list = self.frame3d_list
        if frame_3d in frame3d_list:
            self.node_if.set_param(self.node_if_prefix +  'frame_3d',frame_3d)
        self.publish_status()

    ###################
    ## Render Callbacks
    def resetRenderControlsCb(self,msg):
        self.resetRenderControls()

    def resetRenderControls(self,do_updates = True):
        self.node_if.factory_reset_param('rotate_ratio')
        self.node_if.factory_reset_param('tilt_ratio')
        self.node_if.factory_reset_param('render_enable')

        if do_updates:
            self.publish_status()

    def setImageSizeCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        width = msg.image_width
        height = msg.image_height
        if width > 100 and width < 5000 and height > 100 and height < 5000:
            self.node_if.set_param(self.node_if_prefix +  'image_width',  width)
            self.node_if.set_param(self.node_if_prefix +  'image_height', height)
            self.publish_status()

    def setImageSizeIndCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        index = msg.data
        if index < len(nepi_img.STANDARD_IMAGE_SIZES):
            size_str = nepi_img.STANDARD_IMAGE_SIZES[index]
            size__split = size_str.split(" ")
            width = float(size__split[0])
            height = float(size__split[2])
            if width > 100 and width < 5000 and height > 100 and height < 5000:
                self.node_if.set_param(self.node_if_prefix +  'image_width',  width)
                self.node_if.set_param(self.node_if_prefix +  'image_height', height)
                self.publish_status()

    def setZoomRatioCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        new_val = msg.data
        if new_val >= 0 and new_val <= 1 :
            self.node_if.set_param(self.node_if_prefix +  'zoom_ratio',new_val)
            self.publish_status()

    def setRotateRatioCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        new_val = msg.data
        if new_val >= 0 and new_val <= 1 :
            self.node_if.set_param(self.node_if_prefix +  'rotate_ratio',new_val)
            self.publish_status()

    def setTiltRatioCb(self,msg):
        #self.msg_if.pub_info(str(msg))
        new_val = msg.data
        if new_val >= 0 and new_val <= 1 :
            self.node_if.set_param(self.node_if_prefix +  'tilt_ratio',new_val)
            self.publish_status()


    def setRenderEnableCb(self,msg):
        render_enable = msg.data
        self.node_if.set_param(self.node_if_prefix +  'render_enable', render_enable)
        self.publish_status()

##################################################
# PointcloudImageIF

class PointcloudImageIF(BaseImageIF):

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = True,
        has_flip_horz = True,
        has_flip_vert = True,
        has_range = True,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_zoom_3d = True,
        has_rotate_3d = True,
        has_tilt_3d = True,
        has_camera_3d = True
        )

    DEFAULT_FILTERS_DICT = dict()

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 

        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0,
        start_range_ratio = 0,
        # 1.0 = no range clip. A 0 here makes the renderer clip to
        # [min_range_m, min_range_m] and hand back an empty cloud, so every
        # rendered frame comes out blank. BaseImageIF uses 1.0 for the same key.
        stop_range_ratio = 1.0,
        # [x_min, x_max, y_min, y_max] as ratios: [0,1,0,1] is the full frame. A
        # [0,0,0,0] here is a zero-area window, and the crop step then divides by
        # (x_max - x_min) == 0. init() force-corrects it at startup, so the bad value
        # only ever surfaced through reset_renders, which restores the constructor
        # baseline captured before init() ran. Every other image IF uses [0,1,0,1].
        window_ratios = [0,1,0,1],
        # Camera distance, and the mapping is inverted: for a negative cam_pos x the
        # renderer uses (1 - zoom_3d_ratio) as the multiplier on cam_pos, so 0 is the
        # FARTHEST view (full -5 m) and larger values move the camera in. A 0 here
        # started every pointcloud pinned to the far end of its range, which reads as
        # "zoomed way out". 0.5 is the neutral midpoint, matching how every other
        # ratio control in this file defaults, and puts the camera at -2.5 m with
        # equal headroom to scroll in or out.
        zoom_3d_ratio = 0.5,
        # 0.5 = no rotation. The renderer maps ratio to angle as
        # (0.5 - ratio) * 360, so a 0 here is a 180 deg flip: rotate about Z and
        # tilt about Y both invert, and the cloud renders upside down. Every other
        # image IF uses 0.5 for these two keys.
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5,
        cam_fov = 60,
        cam_view = [3, 0, 0],
        cam_pos = [-5, 0, 0],
        cam_rot = [0, 0, 1],
        # Render background. Present here so the renderer resolves it from
        # controls_dict like every other render control.
        use_wbg = False
        )


    # #Default Control Values 
    # DEFAULT_OFFSETS_DICT = dict( 

    #     resolution_ratio = 0,
    #     auto_adjust_enabled = False,
    #     auto_adjust_ratio = 0,
    #     brightness_ratio = 0,
    #     contrast_ratio =  0,
    #     threshold_ratio =  0,
    #     start_range_ratio = 0,
    #     stop_range_ratio = 0,
    #     zoom_3d_ratio = 0,
    #     window_ratios = [0,0,0,0],
    #     rotate_3d_ratio = 0,
    #     tilt_3d_ratio = 0,
    #     cam_fov = 0,
    #     cam_view = [0, 0, 0],
    #     cam_pos = [0, 0, 0],
    #     cam_rot = [0, 0, 0]
    #     )

    # # Working copy of the render offsets; drag handlers mutate this, the
    # # reset callback restores it to DEFAULT_OFFSETS_DICT (start orientation).
    #offsets_dict = copy.deepcopy(DEFAULT_OFFSETS_DICT)

    # Anchor state for interactive 3D drags (start pixel is the fixed drag anchor)
    render_3d_drag_anchor = None
    render_3d_drag_last = None

    params_dict = None
    services_dict = None
    pubs_dict = None
    subs_dict = None

    data_product = 'pointcloud_image'

    auto_adjust_controls = []

    # Pointcloud renderer state (lazily created on first render)
    img_renderer = None
    img_renderer_mtl = None
    last_img_width = None
    last_img_height = None
    last_fov = None
    last_bg_white = None

    is_processing = False
    def __init__(self, namespace = None ,
                data_product = None,
                data_source_description = 'sensor',
                data_ref_description = 'sensor',
                perspective = 'pov',
                init_overlay_text_list = [],
                save_data_if = None,
                navpose_if = None,
                navpose_namespace = None,
                transform_namespace = None,
                live_adjustments_disabled = False,
                aspect_adjustment_disabled = False,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
                ):

        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product
        if log_name is not None:
            log_name = nepi_utils.get_clean_name(log_name)
        self.save_data_if = save_data_if
        self.navpose_if = navpose_if

        # Call the parent class constructor
        super().__init__(namespace , 
                self.data_product,
                data_source_description,
                data_ref_description,
                perspective,
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_FILTERS_DICT, 
                self.params_dict,
                self.services_dict,
                self.pubs_dict,
                self.subs_dict,
                self.save_data_if,
                self.navpose_if,
                navpose_namespace,
                transform_namespace,
                init_overlay_text_list,
                live_adjustments_disabled,
                aspect_adjustment_disabled,
                log_name,
                log_name_list,
                msg_if,
                node_if
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################



    ###############################
    # Class Public Methods
    ###############################

    def publish_pointcloud_img(self,o3d_pc,
                            width_deg = 100,
                            height_deg = 70,
                            render_dict = None,
                            min_range_m = None,
                            max_range_m = None,
                            img_width = None,
                            img_height = None,
                            timestamp = None,
                            frame_id = 'sensor',
                            pub_twice = False
                            ):
        """Render an Open3D pointcloud to an image and publish it.

        Applies the supplied render controls (image size, range clip, zoom,
        rotate, tilt, and camera pose), renders the pointcloud through an
        Open3D offscreen renderer, and publishes the resulting image via the
        BaseImageIF publish pipeline.

        Args:
            o3d_pc (open3d.geometry.PointCloud): The pointcloud to visualize.
            width_deg (float, optional): Horizontal field of view in degrees.
                Defaults to 100.
            height_deg (float, optional): Vertical field of view in degrees.
                Defaults to 70.
            render_dict (dict, optional): Render control values (image_width,
                image_height, start_range_ratio, stop_range_ratio, zoom_3d_ratio,
                rotate_ratio, tilt_ratio, cam_fov, cam_view, cam_pos, cam_rot,
                use_wbg). Falls back to interface defaults when None.
            timestamp (float or rospy.Time, optional): Acquisition timestamp.
                Defaults to current time if None.
            frame_id (str, optional): TF frame ID for the published image.
                Defaults to 'sensor'.
            pub_twice (bool, optional): Publish the resulting image twice.
                Defaults to False.

        Returns:
            tuple: (o3d_pc, cv2_img) where cv2_img is the rendered image, or
            None if rendering produced no image.
        """
        cv2_img = None
        if self.is_processing == True:
            self.is_processing = False
            return o3d_pc, cv2_img
        self.is_processing = True

        if self.navpose_if is not None:
            navpose_dict = self.navpose_if.get_navpose_dict()
        else:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

        
        if o3d_pc is None:
            self.is_processing = False
            return o3d_pc, cv2_img

        if timestamp is None:
            timestamp = nepi_utils.get_time()
        else:
            timestamp = nepi_sdk.sec_from_timestamp(timestamp)

        # Resolve render controls from self.controls_dict. That is the single store
        # every render control writes: the ROS/RUI setters (set_zoom_3d_ratio,
        # set_rotate_3d_ratio, set_tilt_3d_ratio, setCamFovCb, setCamViewCb,
        # setCamPositionCb, setCamRotationCb, setWhiteBgCb) and the interactive
        # drag/scroll/click handlers all update it, and publish_status reports it.
        # Rendering from anything else is what made the ROS and RUI controls inert.
        # A caller-supplied render_dict overrides for this frame only and is not
        # persisted, so it cannot become a competing source of truth.
        resolved = copy.deepcopy(self.controls_dict)
        if render_dict is not None:
            resolved.update(render_dict)

        # Explicit img_width/img_height args override the status-driven size
        if img_width is None:
            img_width = resolved.get('image_width', 955)
        if img_height is None:
            img_height = resolved.get('image_height', 600)
        img_width = int(img_width)
        img_height = int(img_height)
        start_range_ratio = resolved.get('start_range_ratio', 0.0)
        stop_range_ratio = resolved.get('stop_range_ratio', 1.0)
        zoom_ratio = nepi_utils.check_ratio(resolved.get('zoom_3d_ratio', 0))
        rotate_ratio = resolved.get('rotate_3d_ratio', 0.5)
        tilt_ratio = resolved.get('tilt_3d_ratio', 0.5)
        cam_fov = resolved.get('cam_fov', self.DEFAULT_CONTROLS_DICT['cam_fov'])
        # Read each camera vector once. These previously read the same key twice as
        # a "base" plus an "offset" and summed them, which doubled every value
        # (cam_pos [-5,0,0] rendered as [-10,0,0]) and made a camera position set
        # over ROS arrive at twice what was asked for. The offsets_dict those three
        # lines were written against does not exist.
        cam_view = list(resolved.get('cam_view', self.DEFAULT_CONTROLS_DICT['cam_view']))
        cam_pos = list(resolved.get('cam_pos', self.DEFAULT_CONTROLS_DICT['cam_pos']))
        cam_rot = list(resolved.get('cam_rot', self.DEFAULT_CONTROLS_DICT['cam_rot']))
        use_wbg = resolved.get('use_wbg', False)

        # Range clip
        if min_range_m is None:
            min_range_m = nepi_pc.get_min_range(o3d_pc)
        if max_range_m is None:
            max_range_m = nepi_pc.get_max_range(o3d_pc)
        delta_range_m = max_range_m - min_range_m
        clip_min_range_m = min_range_m + start_range_ratio * delta_range_m
        clip_max_range_m = min_range_m + stop_range_ratio * delta_range_m
        if start_range_ratio > 0 or stop_range_ratio < 1:
            o3d_pc = nepi_pc.range_clip_spherical(o3d_pc, clip_min_range_m, clip_max_range_m)

        # Apply zoom via camera position
        if cam_pos[0] < 0:
            zoom_ratio = 1 - zoom_ratio
        cam_pos[0] = cam_pos[0] * zoom_ratio

        # Apply rotate and tilt
        rotate_angle = (0.5 - rotate_ratio) * 2 * 180
        o3d_pc = nepi_pc.rotate_pc(o3d_pc, [0, 0, rotate_angle])
        tilt_angle = (0.5 - tilt_ratio) * 2 * 180
        o3d_pc = nepi_pc.rotate_pc(o3d_pc, [0, tilt_angle, 0])

        bg_color = [1, 1, 1, 1] if use_wbg else [0, 0, 0, 0]

        # (Re)create the renderer when geometry-independent params change
        update_renderer = (self.img_renderer is None or self.img_renderer_mtl is None
            or self.last_img_width != img_width or self.last_img_height != img_height
            or self.last_fov != cam_fov or self.last_bg_white != use_wbg)
        if update_renderer:
            self.img_renderer = None
            self.msg_if.pub_warn("Creating new pointcloud renderer", log_name_list = self.log_name_list)
            self.img_renderer = nepi_pc.create_img_renderer(img_width=img_width, img_height=img_height, fov=cam_fov, background=bg_color)
            self.img_renderer_mtl = nepi_pc.create_img_renderer_mtl(shader="defaultUnlit")
            self.last_img_width = img_width
            self.last_img_height = img_height
            self.last_fov = cam_fov
            self.last_bg_white = use_wbg

        # Render the frame
        self.img_renderer = nepi_pc.remove_img_renderer_geometry(self.img_renderer)
        self.img_renderer = nepi_pc.add_img_renderer_geometry(o3d_pc, self.img_renderer, self.img_renderer_mtl)
        o3d_img = nepi_pc.render_img(self.img_renderer, cam_view, cam_pos, cam_rot)
        self.img_renderer = nepi_pc.remove_img_renderer_geometry(self.img_renderer)

        if o3d_img is not None:
            cv2_img = nepi_pc.o3dimg_to_cv2img(o3d_img)

        # Publish and Save Pointcloud Image Data
        if cv2_img is not None:
            self.publish_cv2_img(cv2_img,
                                encoding = 'rgb8',
                                width_deg = width_deg,
                                height_deg = height_deg,
                                min_range_m = min_range_m,
                                max_range_m = max_range_m,
                                timestamp = timestamp,
                                pub_twice = pub_twice
                                )
        self.is_processing = False
        return o3d_pc, cv2_img


    ###############################
    # Class Private Methods
    ###############################

    def render3dDragHandler(self, start_pixel, start_color_bgr, stop_pixel, stop_color_bgr):
        #self.msg_if.pub_info("3D render drag handler fired - start: " + str(start_pixel) + " stop: " + str(stop_pixel), log_name_list = self.log_name_list)
        img_width = self.status_msg.width_px
        img_height = self.status_msg.height_px
        if img_width <= 0 or img_height <= 0:
            return
        # A drag streams many events sharing the same fixed anchor (start) pixel and a
        # moving stop pixel; accumulate the offset from the previous streamed position
        # so a continuous drag maps 1:1 and successive drags keep adding.
        if start_pixel != self.render_3d_drag_anchor:
            self.render_3d_drag_anchor = list(start_pixel)
            self.render_3d_drag_last = list(start_pixel)
        rotate_change = float(stop_pixel[0] - self.render_3d_drag_last[0]) / float(img_width)
        tilt_change = float(stop_pixel[1] - self.render_3d_drag_last[1]) / float(img_height)
        self.render_3d_drag_last = list(stop_pixel)
        rotate_offset = min(1.0, max(-1.0, self.controls_dict['rotate_3d_ratio'] + rotate_change))
        tilt_offset = min(1.0, max(-1.0, self.controls_dict['tilt_3d_ratio'] + tilt_change))
        self.controls_dict['rotate_3d_ratio'] = rotate_offset
        self.controls_dict['tilt_3d_ratio'] = tilt_offset
        self.publish_status()
        self.needs_update()

    def render3dWindowHandler(self, window = None):
        self.msg_if.pub_info("3D render window handler fired - window: " + str(window), log_name_list = self.log_name_list)
        self.publish_status()

    def render3dScrollHandler(self, scroll_pixel, scroll_color_bgr, scroll_amount):
        self.msg_if.pub_info("3D render scroll handler fired - amount: " + str(scroll_amount), log_name_list = self.log_name_list)
        # scroll_amount is a normalized wheel step (+ = zoom in, - = zoom out); accumulate
        # into the zoom offset applied on top of the status-driven zoom in publish.
        zoom_step = float(scroll_amount) * 0.05
        zoom_offset = min(1.0, max(-1.0, self.controls_dict['zoom_3d_ratio'] + zoom_step))
        self.controls_dict['zoom_3d_ratio'] = zoom_offset
        self.publish_status()
        self.needs_update()

    def render3dClickHandler(self, pixel, color_bgr, click_count, angles):
        self.msg_if.pub_info("3D render click handler fired - pixel: " + str(pixel), log_name_list = self.log_name_list)
        img_width = self.status_msg.width_px
        img_height = self.status_msg.height_px
        if img_width <= 0 or img_height <= 0:
            return
        # Click position as a ratio from image center, [-1, 1]
        x_ratio = float(pixel[0] - img_width / 2.0) / float(img_width / 2.0)
        y_ratio = float(pixel[1] - img_height / 2.0) / float(img_height / 2.0)
        # Pan the view by shifting both the camera eye (cam_pos) and look-at target
        # (cam_view) by the same vector so the view direction stays fixed: horizontal
        # click pans sideways (camera y), vertical click pans up/down (camera z).
        # pixel y increases downward, so invert it for up/down.
        pan_scale = 1.0
        pan_y = x_ratio * pan_scale
        pan_z = -y_ratio * pan_scale
        cam_view = list(self.controls_dict['cam_view'])
        cam_pos = list(self.controls_dict['cam_pos'])
        cam_view[1] = cam_view[1] + pan_y
        cam_pos[1] = cam_pos[1] + pan_y
        cam_view[2] = cam_view[2] + pan_z
        cam_pos[2] = cam_pos[2] + pan_z
        self.controls_dict['cam_view'] = cam_view
        self.controls_dict['cam_pos'] = cam_pos
        self.publish_status()
        self.needs_update()

    def reset_render_3d_orientation(self):
        """Reset the interactive 3D orientation offsets (rotate, tilt, zoom) to start."""
        defaults = copy.deepcopy(self.DEFAULT_CONTROLS_DICT)
        self.controls_dict['rotate_3d_ratio'] = defaults['rotate_3d_ratio']
        self.controls_dict['tilt_3d_ratio'] = defaults['tilt_3d_ratio']
        self.controls_dict['zoom_3d_ratio'] = defaults['zoom_3d_ratio']
        self.render_3d_drag_anchor = None
        self.render_3d_drag_last = None
        self.publish_status()
        self.needs_update()

    def reset_render_3d_position(self):
        """Reset the interactive 3D position offsets (pan / camera translation) to start."""
        defaults = copy.deepcopy(self.DEFAULT_CONTROLS_DICT)
        self.controls_dict['cam_view'] = defaults['cam_view']
        self.controls_dict['cam_pos'] = defaults['cam_pos']
        self.publish_status()
        self.needs_update()




