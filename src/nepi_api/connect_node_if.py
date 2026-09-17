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
from nepi_sdk import nepi_utils


from std_msgs.msg import String, Empty

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyRequest as EmptySrvRequest
from std_srvs.srv import EmptyResponse as EmptySrvResponse

from nepi_interfaces.msg import MgrSystemStatus, ConnectIFStatus

from nepi_interfaces.msg import Reset

from nepi_api.messages_if import MsgIF

# Reuse the full node config/param machinery so ConnectNodeClassIF can persist
# params (e.g. the connect selected_topic) via the config manager, exactly the
# way NodeClassIF does. node_if.py does not import this module, so there is no
# circular import.
from nepi_api.node_if import NodeConfigsIF, NodeParamsIF






#########################################
# Connect Node IF Class
#########################################


CONNECTED_TIMEOUT = 2

# Interface tokens that appear between a node namespace and its data products,
# i.e. the namespace each device_if_*.py class creates under its node namespace
# ('idx', 'ptx', 'lsx', 'npx', 'rbx', 'motors') plus the AI namespace. These are
# structural, not something an operator chooses between, so
# get_available_path_names() drops them from selector labels. The RUI strips the
# same set in Utilities.js createShortValuesFromNamespaces/
# createShortImagesFromNamespaces. Deliberately not nepi_system's
# NEPI_ALL_CONFIG_IDS -- that list identifies config domains, omits 'motors' and
# 'ai', and would silently change these labels if a config domain were added.
IF_NAMESPACE_TOKENS = ['idx','ptx','lsx','npx','rbx','motors','ai']

class ConnectNodeIF:
    
    msg_if = None
    node_if = None
    node_if_shared = False
    connect_if = None
    connect_ready = False
    namespace = '~'

    active_nodes = []
    active_topics = []
    active_topic_types =  []
    active_services =  []  
    
    connect_namespace = ''
    connect_id = None
    connect_status_msg = None
    connect_name = None

    filter_topics_list = []
    available_topics = []
    available_names = []
    auto_select_enabled = True
    # Distinguishes "the operator explicitly picked None" from "nothing has
    # ever been selected" -- both look identical as plain selected_topic ==
    # "None", which is exactly why picking None from the RUI selector used to
    # "not work": the very next status build (or updateConnectsHandler cycle)
    # saw an empty selection with candidates available and auto-selected one
    # right back, one publish cycle later. set_selected_topic() sets this
    # whenever the operator asks for "None" specifically, and clears it the
    # moment they pick a real topic again; both auto-select sites below skip
    # re-selecting while this is set.
    user_cleared_selection = False

    selected_topic_param = 'None'
    selected_topic = "None"
    connecting = False
    connected = False
    connected_topic = 'None'
    connect_msg = 'Not Selected'

    # Last topic whose connection was announced at warn level by
    # _announceConnected(). See that method for why a remembered topic rather than
    # a bare once-only flag.
    announced_connected_topic = None

    show_selector = True
    show_controls = True
    show_data = True

    connect_node_pubs_dict = None
    connect_node_subs_dict = None

    status_has_published = False

    #######################
    ### IF Initialization
    def __init__(self, 
                connect_id = None,
                connect_status_msg = None,
                connect_name = None,
                filter_topics_list = [],
                selected_topic = "None",
                auto_select_enabled = True,
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  

        
        if msg_if is None:
            self.msg_if = MsgIF(log_name = self.class_name)
        else:
            self.msg_if = msg_if
        self.msg_if.pub_info("Starting " + str(self.class_name) + " Initialization Processes")


        # Check ID Class
        if connect_id is None:
            self.msg_if.pub_warn("Connect ID Not Provided") 
            return 
        self.connect_id = connect_id



        # Check Status Msg Type
        if connect_status_msg is None:
            self.msg_if.pub_warn("Connect Status Msg Type Not Provided") 
            return 
        self.connect_status_msg = connect_status_msg


        self.connect_name = nepi_utils.get_clean_name(connect_name)
        if self.connect_name is None or self.connect_name == '':
            self.msg_if.pub_warn("Topic Name Not Valid: " + str(connect_name)) 
            return
        self.msg_if.pub_info("Using Topic Name: " + self.connect_name)
        self.connect_namespace = nepi_sdk.create_namespace(self.node_name,self.connect_name)

        
        ##############################    
        # Initialize Class Variables

        if selected_topic is None:
            selected_topic = "None"
        if selected_topic != "None":
            sselected_topic = nepi_sdk.get_full_namespace(selected_topic)
        if selected_topic == '' or selected_topic == '/':
            selected_topic = "None"
        self.selected_topic = selected_topic

        self.auto_select_enabled = auto_select_enabled
        self.msg_if.pub_info("Auto select enabled: " + str(self.auto_select_enabled))

        self.show_selector = show_selector
        self.show_controls = show_controls
        self.show_data = show_data

        self.filter_topics_list = filter_topics_list          

        self.namespace = nepi_sdk.create_namespace(self.node_namespace,connect_name)
        self.node_if_prefix = self.namespace.replace(self.node_namespace + '/','').replace('/','_') + '_'

        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        CFGS_DICT = {
                'namespace': self.connect_namespace
        }

        # Params Config Dict ####################
        # Persist the selected topic under the connect namespace so the
        # selection survives node restarts (via the config manager). Passing a
        # params_dict is what enables config management on ConnectNodeClassIF.
        CONNECT_PARAMS_DICT = {
            self.node_if_prefix + 'selected_topic': {
                'namespace': self.connect_namespace,
                'factory_val': self.selected_topic
            }
        }


        # Publishers Config Dict ####################
        self.connect_node_pubs_dict = {
            self.node_if_prefix + 'status_pub': {
                'namespace': self.connect_namespace,
                'topic': 'status',
                'msg': ConnectIFStatus,
                'qsize': 1,
                'latch': True
            }
        }




        # Subscribers Config Dict ####################
        self.connect_node_subs_dict = {
            self.node_if_prefix + 'select_topic': {
                'namespace': self.connect_namespace,
                'topic': 'select_topic',
                'msg': String,
                'qsize': None,
                'callback': self._selectTopicCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self._systemStatusCb
            },
        }


        if node_if is None:
            self.node_if = ConnectNodeClassIF(
                            configs_dict = CFGS_DICT,
                            params_dict = CONNECT_PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.connect_node_pubs_dict,
                            subs_dict = self.connect_node_subs_dict,
                            log_name_list = [],
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()
        else:
            self.node_if_shared = True
            try:
                self.node_if = node_if
                self.node_if.register_pubs(self.connect_node_pubs_dict)
                self.node_if.register_subs(self.connect_node_subs_dict)
                # Register the persisted selection param on the shared node_if too.
                self.node_if.add_param(self.node_if_prefix + 'selected_topic', self.connect_namespace, self.selected_topic)
                nepi_sdk.sleep(1)
            except Exception as e:
                self.msg_if.pub_info("Failed to register pubs and subs: " + str(e))
                return


        # Restore any persisted selection. When no explicit topic was requested
        # (selected_topic == "None"), use the value the config manager restored
        # for this connect namespace. Otherwise honor the explicit request.
        self.selected_topic_param = self.node_if_prefix + 'selected_topic'
        if selected_topic == "None":
            persisted = self.node_if.get_param(self.selected_topic_param)
            if persisted is not None and persisted != '' and persisted != "None":
                selected_topic = persisted
        self.selected_topic = selected_topic
        self.msg_if.pub_info("Init Selected Topic: " + str(self.selected_topic))


        ##############################
        # Start updater process
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

        ##############################
        # Complete Initialization
        self.connect_ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_connect_ready_state(self):
        """Return the ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.connect_ready

    def wait_for_connect_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.connect_ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connect_ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connect_ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connect_ready  

    def get_namespace(self):
        """Return the fully-resolved ROS namespace for the connected PTX device.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.connect_namespace
    

    def get_available_topics(self):
        return self.available_topics

    def set_auto_connect_enable(self, enabled):
        self.auto_select_enabled = enabled
            
    def get_selected_topic(self):
        return self.selected_topic
    
    def set_selected_topic(self, selected_topic):
        # An undiscovered topic cannot be selected. Reject it WITHOUT persisting:
        # the old code fell through and wrote the param plus save_config() even
        # though self.selected_topic was left unchanged. Callers that re-assert a
        # derived selection every cycle (applyDerivedTargetsImageSelection in the
        # obstacles app) guard on get_selected_topic(), which never reports the
        # rejected value, so each cycle re-entered here -- hammering the param
        # server and rewriting the config file at 1 Hz, and filling the log with
        # "Failed to set param ... Connection refused" whenever the master was
        # down. Rejection must be cheap and quiet; the caller's re-assert loop is
        # intentional and self-corrects once the topic is discovered.
        if selected_topic not in self.available_topics and selected_topic != "None":
            self.msg_if.pub_debug("Ignoring unavailable selected_topic: " + str(selected_topic),
                                  throttle_s = 5.0)
            self.publish_status()
            return
        # See user_cleared_selection's own comment: this is what makes an
        # explicit "None" stick instead of being auto-reselected on the very
        # next cycle.
        self.user_cleared_selection = (selected_topic == "None")
        self.selected_topic = selected_topic
        self.publish_status()
        # Persist the selection so it survives a node restart. set_param writes
        # the ROS param; save_config asks the config manager to save it to file.
        if self.node_if is not None:
            self.msg_if.pub_debug("selected_topic: " + str(selected_topic))
            self.node_if.set_param(self.node_if_prefix + 'selected_topic', self.selected_topic)
            self.node_if.save_config()
    

    def check_connection(self):
        """Check whether the device is currently connected.

        Returns:
            bool: True if a status message has been received within the connection timeout window,
                False otherwise.
        """
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        """Block until the device is connected or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if connection was established, False if the timeout was reached.
        """
        if self.node_if is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connected


    def subscribe_topic(self):
        pass

    def unsubscribe_topic(self):
        pass


    def unregister(self):
        success = False
        self.unsubscribe_topic()
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.sleep(1)
            else:
                self.unsubscribe_topic()

                if self.node_if is not None:
                    if self.connect_node_subs_dict is not None:
                        for sub_name in self.connect_node_subs_dict.keys():
                            self.node_if.unregister_sub(sub_name)
                self.connect_node_subs_dict = None

                if self.node_if is not None:
                    if self.connect_node_pubs_dict is not None:
                        for pub_name in self.connect_node_pubs_dict.keys():
                            self.node_if.unregister_pub(pub_name)
                self.connect_node_pubs_dict = None
                
        time.sleep(1)
        try:
            self.node_if = None
            self.selected_topic = 'None'
            self.connecting = False 
            self.connected = False 
            self.connected_topic = 'None'
            success = True
        except Exception as e:
            self.msg_if.pub_warn("Failed to unregister:  " + str(e))
        return success


    def _selectTopicCb(self,msg):
        topic = msg.data
        self.set_selected_topic(topic)



    def get_available_names(self, available_topics = []):
        available_names = []
        for topic in available_topics:
            name = topic
            topic = topic[1:]
            topic_split = topic.split('/')
            if len(topic_split) > 2:
                name = topic_split[2]
            available_names.append(name)
        return available_names

    def get_available_path_names(self, available_topics = []):
        """Label each available topic with its node name AND its data product.

        Returns the topic path below the device namespace -- the node name plus
        everything under it -- with the interface tokens in IF_NAMESPACE_TOKENS
        removed. Topics are ``/<prefix>/<device_id>/<node>/<if>/<data_product>``,
        so ``/nepi/device/zed2_14/idx/depth_map`` labels as
        ``zed2_14/depth_map`` and ``/nepi/device/zed2_14/npx/navpose`` as
        ``zed2_14/navpose``.

        This is what get_available_names() cannot do: that method returns only
        the node name, so every data product of one device shares a single label.
        A device offering several products of the same type -- a stereo camera
        publishing color_image, bw_image and depth_map_image, all ImageStatus and
        so all discovered by one ConnectColorImageIF -- yields a selector of
        identically labelled, indistinguishable entries.

        Device connect classes are unaffected. A device topic ends at its
        interface token (``/nepi/device/zed2_14/idx``, and ``.../motors`` for
        ConnectMotorIF, whose _updaterCb strips '/motor_status' rather than
        '/status'), so stripping that token leaves the node name alone and the
        label is byte-identical to get_available_names(). That is why this can
        replace the label source for every connect class at once.

        get_available_names() is left in place and unchanged for callers that
        want the owning node name on its own.

        Args:
            available_topics (list): Fully-qualified topic namespaces to label.

        Returns:
            list: One label per entry in available_topics, in the same order.
                Falls back to the full topic for a namespace too shallow to hold
                a node name.
        """
        available_names = []
        for topic in available_topics:
            name = topic
            topic_split = topic[1:].split('/')
            if len(topic_split) > 2:
                path_tokens = []
                for token in topic_split[2:]:
                    if token not in IF_NAMESPACE_TOKENS:
                        path_tokens.append(token)
                if len(path_tokens) > 0:
                    name = '/'.join(path_tokens)
            available_names.append(name)
        return available_names

    def _publishStatusCb(self,timer):
        self.publish_status()


    def publish_status(self):

        available_topics = copy.deepcopy(self.available_topics)
        selected_topic = copy.deepcopy(self.selected_topic)
        status_msg = ConnectIFStatus()

        status_msg.name = self.connect_name
        status_msg.id = self.connect_id

        status_msg.status_msg_type = self.connect_status_msg

        status_msg.available_topics = available_topics
        # Path names, not node names: a selector must distinguish one device's
        # data products from each other. Device connect classes see no change --
        # their topics reduce to the node name either way. See
        # get_available_path_names().
        available_names = self.get_available_path_names(available_topics)
        status_msg.available_names = available_names

        selected_name = 'None'
        if selected_topic not in available_topics:
            # Auto-select fills an EMPTY selection only. A selection that IS set but
            # not yet discovered must be left alone: available_topics is populated
            # incrementally as sibling device nodes come up, so grabbing
            # available_topics[0] here silently repointed a restored selection at
            # whichever device happened to appear first. On a two-axis node that
            # landed both axes on the same servo -- ptx_servos_node restored pan
            # channel 0 and tilt channel 1 from config, then this method moved tilt
            # to channel 0 before channel 1 was discovered, and the node adopted the
            # wrong topic on its next updateConnectsHandler cycle. Nothing rewrote
            # the param when that happened, so the config file still held the
            # operator's real choice while the live selection had already moved on,
            # which is why the selection looked like it never saved.
            if (selected_topic == 'None' and len(available_topics) > 0 and
                    self.auto_select_enabled == True and self.user_cleared_selection == False):
                selected_topic = available_topics[0]
                self.selected_topic = selected_topic

        if selected_topic in available_topics:
            selected_ind = available_topics.index(selected_topic)
            selected_name = available_names[selected_ind]
        elif selected_topic != 'None':
            # Selected but not discovered yet. Report the selection with a derived
            # label rather than 'None' so the status does not read as unselected
            # while discovery catches up.
            selected_names = self.get_available_path_names([selected_topic])
            if len(selected_names) > 0:
                selected_name = selected_names[0]

        status_msg.selected_topic = selected_topic
        status_msg.selected_name = selected_name

        status_msg.connecting = self.connecting
        status_msg.connected = self.connected
        connected_topic = self.connected_topic
        if connected_topic is None:
            connected_topic = 'None'
        status_msg.connected_topic = connected_topic

        connect_msg = "Not Selected"
        if self.selected_topic != "None":
            connect_msg = "Selected"
            if self.connecting == True:
                connect_msg = "Connecting"
            if self.connected == True:
                connect_msg = "Connected"
        status_msg.connect_msg = connect_msg


        status_msg.show_selector = self.show_selector
        status_msg.show_controls = self.show_controls
        status_msg.show_data = self.show_data


        ###########
        if self.node_if is not None:
            if self.status_has_published == False:
                self.msg_if.pub_warn("Publishing Status: " + str(status_msg))
                self.status_has_published = True
            self.node_if.publish_pub(self.node_if_prefix + 'status_pub', status_msg) 
            #self.node_if.save_config()




    #######################
    # Class Private Methods
    #######################

    # Announce a new connection ONCE per topic, from every subclass's _statusCb.
    #
    # Each subclass's status callback runs its announcement inside
    # 'if self.connected == False:', which reads like a first-connection guard but
    # is not one: the staleness check in _updaterCb clears self.connected whenever
    # no status arrives inside CONNECTED_TIMEOUT, without touching the
    # subscriptions. Any source publishing its status slower than that timeout --
    # or any source whose status stalls briefly -- therefore re-enters that branch
    # on every message and re-announces the same connection at warn level forever.
    #
    # The remembered value is the announced TOPIC rather than a bare once-only
    # flag, because connecting to a DIFFERENT topic is still news worth a warn:
    # these IFs are re-pointed at runtime, by an operator's selector or by a
    # consumer node driving set_selected_topic() (nepi_app_obstacles derives all
    # five of its selections that way). It is deliberately never cleared, so a
    # deselect and reselect of the same topic stays quiet.
    #
    # Reconnects still log, at debug, so the churn is visible when it is being
    # looked for.
    def _announceConnected(self, connect_label):
        if self.selected_topic != self.announced_connected_topic:
            self.announced_connected_topic = self.selected_topic
            self.msg_if.pub_warn("Connected to " + str(connect_label) + " Status:  " + str(self.selected_topic))
        else:
            self.msg_if.pub_debug("Reconnected to " + str(connect_label) + " Status:  " + str(self.selected_topic))


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
        ##############

        selected_topic = copy.deepcopy(self.selected_topic)
        last_available = copy.deepcopy(self.available_topics)

        topics = nepi_sdk.find_topics_by_msg(self.connect_status_msg, topics_list = self.active_topics, types_list = self.active_topic_types)
        available_topics = []
        for topic in topics:
            valid = True
            if len(self.filter_topics_list) > 0:
                valid = False
                for filter in self.filter_topics_list:
                    if filter in topic:
                        valid = True
                        break
            if valid == True:
                available_topics.append(topic.replace('/status',''))
        if available_topics != last_available:
            self.available_topics = available_topics
            needs_publish = True

        ####################
        if self.connected_topic is not None:
            if self.connected_topic not in self.available_topics:
                success = self.unsubscribe_topic()
        # Honor auto_select_enabled here too. This path ignored the flag, so a
        # consumer that disabled auto-select (a pan/tilt pair, where auto-selecting
        # the first servo is guaranteed wrong for at least one axis) still got one
        # picked for it one second after startup.
        if (selected_topic == 'None' and len(self.available_topics) > 0 and
                self.auto_select_enabled == True and self.user_cleared_selection == False):
            self.selected_topic = self.available_topics[0]
        needs_publish = True

        was_connected = copy.deepcopy(self.connected)
        if self.selected_topic in self.available_topics and self.connected_topic != selected_topic:
            success = self.subscribe_topic(self.selected_topic)
        elif self.connect_if is not None:
            self.connected = self.connect_if.check_connection()
        elif self.selected_topic not in self.available_topics:
            self.connected = False
        # else: already subscribed to the selected topic -- leave self.connected
        # to the status callback (sets True on each msg) and the staleness check
        # below, so it does not get clobbered False every cycle.

        ##################
        cur_time = nepi_utils.get_time()
        last_time = copy.deepcopy(self.last_status_time )
        if self.connected == True:
            if (cur_time - last_time) > CONNECTED_TIMEOUT:
                self.connecting = False 
                self.connected = False 
                self.connected_topic = ''
                self.status_msg = None



        ##################
        # Get settings from param server
        # if needs_publish == True:
        #   self.publish_status()
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)










##################################################
### Node Connect Configs Class
'''
EXAMPLE_CFGS_CONFIG_DICT = {
        'namespace': '~'
}
'''

class ConnectNodeConfigsIF:


    msg_if = None
    ready = False
    configs_dict = None
    namespace = '~'

    save_pub = None
    reset_pub = None
    factory_reset_pub = None

    #######################
    ### IF Initialization
    def __init__(self, configs_dict , timeout = float('inf'),
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
        self.msg_if.pub_debug("Starting Connect Node Configs IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################  
        # Initialize Class Variables
        namespace = configs_dict['namespace']
        if namespace is None:
            namespace = self.node_namespace
        else:
            namespace = namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)


        ##############################  
        # Setup Class Publishers for Node Namespace
        self.msg_if.pub_debug("Creating config publishers on namespace: " + self.namespace, log_name_list = self.log_name_list)
        self.save_pub = nepi_sdk.create_publisher(self.namespace + '/save_config', Empty, queue_size=1) 
        self.reset_pub = nepi_sdk.create_publisher(self.namespace + '/reset_config', Empty, queue_size=1) 
        self.factory_reset_pub = nepi_sdk.create_publisher(self.namespace + '/factory_reset_config', Empty, queue_size=1) 
        self.sys_reset_pub = nepi_sdk.create_publisher(self.namespace + '/system_reset', Reset, queue_size=1)

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete", log_name_list = self.log_name_list)
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
            self.msg_if.pub_debug("ready", log_name_list = self.log_name_list)
        return self.ready


    def get_namespace(self):
        return self.namespace

    def save(self):
        success = False
        if self.save_pub is not None:
            try:
                msg = Empty()
                self.save_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send update config: " + str(e), log_name_list = self.log_name_list)
        return success


    def reset(self):
        success = False
        if self.reset_pub is not None:
            try:
                msg = Empty()
                self.reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success

    def factory_reset(self):
        success = False
        if self.factory_reset_pub is not None:
            try:
                msg = Empty()
                self.factory_reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success

    def system_user_reset(self):
        success = False
        if self.sys_reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.USER_RESET
                self.sys_reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success

    def system_factory_reset(self):
        success = False
        if self.sys_reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.FACTORY_RESET
                self.sys_reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success

    def system_software_reset(self):
        success = False
        if self.sys_reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.SOFTWARE_RESET
                self.sys_reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success

    def system_hardware_reset(self):
        success = False
        if self.sys_reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.HARDWARE_RESET
                self.sys_reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success




    def unregister_save_if(self): 
        success = False
        if self.ready is False or self.namespace is None:
            self.msg_if.pub_warn("Save Config IF not running", log_name_list = self.log_name_list)
            success = True
        else:
            self.msg_if.pub_warn("Killing Save Confg IF for trigger: " + self.namespace, log_name_list = self.log_name_list)
            try:
                self.save_pub.unregister()
                self.reset_pub.unregister()
                self.factory_reset_pub.unregister()
                self.syst_reset_pub.unregister()


                success = True
            except:
                pass
            time.sleep(1)
            self.ready = False
            self.save_pub = None
            self.reset_pub = None
            self.factory_reset_pub = None
            self.sys_reset_pub = None
            self.namespace = None
        return success       

    ###############################
    # Class Private Methods
    ###############################





##################################################
### Node Connect Services Class

# Services Dict ####################
EXAMPLE_SRVS_DICT = {
    'service_name': {
        'namespace': '~',
        'topic': 'empty_query',
        'srv': EmptySrv,
        'req': EmptySrvRequest(),
        'resp': EmptySrvResponse()
    }
}

class ConnectNodeServicesIF:


    msg_if = None
    ready = False
    srvs_dict = dict()


    #######################
    ### IF Initialization
    def __init__(self, 
                services_dict = None,
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
        self.msg_if.pub_debug("Starting Connect Node Services IF Initialization Processes", log_name_list = self.log_name_list)
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
        self.msg_if.pub_debug("IF Initialization Complete", log_name_list = self.log_name_list)
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
            self.msg_if.pub_debug("ready", log_name_list = self.log_name_list)
        return self.ready


    def get_services(self):
        return list(self.srvs_dict.keys())

    def create_request_msg(self, service_name):
        req = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'req' in srv_dict.keys():
                req = srv_dict['req']
        return req

    def create_response_msg(self, service_name):
        resp = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'resp' in srv_dict.keys():
                resp = srv_dict['resp']
        return resp


    def call_service(self, service_name, request, verbose = True):
        resp = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'service' in srv_dict.keys():
                service = srv_dict['service']
                if service is not None:
                    resp = nepi_sdk.call_service(service, request, verbose = verbose, log_name_list = self.log_name_list)
        return resp


    def register_service(self,service_name, service_dict):
        self.srvs_dict[service_name] = service_dict
        self._initializeServices()

    def unregister_service(self,service_name):
        self._unregisterService(service_name)

    def unregister_services(self):
        for service_name in self.srvs_dict.keys():
            self._unregisterService(service_name)

    def add_services(self,services_dict):
        self.srvs_dict.update(services_dict)
        self._initializeServices()

    ###############################
    # Class Private Methods
    ###############################

    def _initializeServices(self):
        for service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'service' not in srv_dict.keys():
                srv_namespace = os.path.join(srv_dict['namespace'],srv_dict['topic'])
                srv_msg = srv_dict['srv']
                self.msg_if.pub_debug("Creating service for: " + service_name, log_name_list = self.log_name_list)
                service = None
                try:
                    service = nepi_sdk.connect_service(srv_namespace, srv_msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get service connection: " + service_name + " " + str(e), log_name_list = self.log_name_list) 
                self.srvs_dict[service_name]['service'] = service

    def _unregisterService(self, service_name):
        purge = False
        if service_name in self.srvs_dict.keys():
            purge = True
            if 'service' in srv_dict.keys():
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

class ConnectNodePublishersIF:

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
        self._initializePubs()

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete", log_name_list = self.log_name_list)
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
                has_subs = pub_dict['pub'].get_num_connections() > 0
                #self.msg_if.pub_warn("Pub has subscribers: " + pub_dict['namespace'] + "/" + pub_dict['topic'] + " " + str(has_subs), log_name_list = self.log_name_list)
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
        self._initializePubs()

    def register_pubs(self,pubs_dict = None):
        if pubs_dict is not None:
            self.pubs_dict.update(pubs_dict)
        self._initializePubs()

    def unregister_pub(self,pub_name):
        self._unregisterPub(pub_name)

    def unregister_pubs(self):
        pub_names = list(self.pubs_dict.keys())
        for pub_name in pub_names:
            self._unregisterPub(pub_name)

    def add_pubs(self,pubs_dict):
        self.pubs_dict.update(pubs_dict)
        self.initialize_pubs()

    ###############################
    # Class Private Methods
    ###############################
    def _initializePubs(self):
        for pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' not in pub_dict.keys():
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
                        pub = nepi_sdk.create_publisher(pub_namespace, pub_dict['msg'], queue_size = pub_dict['qsize'],  latch = pub_dict['latch'])
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to create publisher: " + pub_name + " " + str(e), log_name_list = self.log_name_list) 
                    self.pubs_dict[pub_name]['pub'] = pub


    def _unregisterPub(self, pub_name):
        purge = False
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            purge = True
            if 'pub' in pub_dict.keys() and not nepi_sdk.is_shutdown():
                try:
                    self.pubs_dict[pub_name]['pub'].unregister()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister pub: " + pub_name + " " + str(e), log_name_list = self.log_name_list) 
        if purge == True:
            del self.pubs_dict[pub_name]


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
class ConnectNodeSubscribersIF:

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
        self.msg_if.pub_debug("IF Initialization Complete", log_name_list = self.log_name_list)
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

    def register_subs(self,subs_dict = None):
        if subs_dict is not None:
            self.subs_dict.update(subs_dict)
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
            #self.msg_if.pub_warn("Will try to create sub for: " + sub_name, log_name_list = self.log_name_list)
            if 'sub' not in sub_dict.keys() and sub_dict['callback'] is not None and not nepi_sdk.is_shutdown():
                sub_namespace = nepi_sdk.create_namespace(sub_dict['namespace'],sub_dict['topic'])
                self.msg_if.pub_debug("Creating sub for: " + sub_name + " with namespace: " + sub_namespace, log_name_list = self.log_name_list)
                if 'callback_args' not in sub_dict.keys():
                    sub_dict['callback_args'] = ()
                if sub_dict['callback_args'] is None:
                    sub_dict['callback_args'] = ()
                try:
                    if len(sub_dict['callback_args']) == 0:
                        sub = nepi_sdk.create_subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'])
                    else:
                        sub = nepi_sdk.create_subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'], callback_args=sub_dict['callback_args'])
                    self.subs_dict[sub_name]['sub'] = sub
                    success = True
                    #self.msg_if.pub_warn("Created sub for: " + sub_name + " with namespace: " + sub_namespace, log_name_list = self.log_name_list)
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
                    self.msg_if.pub_warn("Failed to get unregister sub: " + sub_name + " " + str(e), log_name_list = self.log_name_list)
        if purge == True:
            del self.subs_dict[sub_name]



##################################################
### Node Class

# Configs Dict ####################
'''
EXAMPLE_CFGS_DICT = {
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
        'factory_val': 100
    },
    'param2_name': {
        'namespace':  self.node_namespace,
        'factory_val': "Something"
    }
}

# Services Dict ####################
EXAMPLE_SRVS_DICT = {
    'service_name': {
        'namespace': '~',
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
        'namespace': '~',
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'latch': False
    }
}


# Subscribers Dict ####################
EXAMPLE_SUBS_DICT = {
    'sub_name': {
        'namespace': '~',
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'callback': EXAMPLE_SUB_CALLBACK,
        'callback_args': ()
    }
}


# Create Node Class ####################
EXAMPLE_NODE_IF = ConnectNodeClassIF(
                configs_dict = EXAMPLE_CFGS_DICT,
                services_dict = EXAMPLE_SRVS_DICT,
                pubs_dict = EXAMPLE_PUBS_DICT,
                subs_dict = EXAMPLE_SUBS_DICT,
                log_class_name = True
)
'''

class ConnectNodeClassIF:

    ready = False

    node_if = None
    msg_if = None

    configs_dict = None
    configs_if = None
    params_if = None
    services_if = None
    pubs_if = None
    subs_if = None

    #######################
    ### IF Initialization
    def __init__(self,
                configs_dict = None,
                params_dict = None,
                services_dict = None,
                pubs_dict = None,
                subs_dict = None,
                node_name = None,
                do_wait = False,
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
        self.msg_if.pub_info("Starting Connect Node IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################
        # Create Config and Param Classes
        #
        # Params always get a (possibly empty) params_if so has_param/get_param/
        # set_param never fail. Config management (which is what actually makes
        # params persist across restarts, via the config manager) is only wired
        # when a params_dict is provided, so connect interfaces that pass no
        # params_dict keep their previous lightweight, non-config-managed
        # behavior unchanged.
        if params_dict is not None and configs_dict is not None:
            # Inject our own config callbacks that reload params first, mirroring
            # NodeClassIF, so a config-manager reload repopulates the params.
            self.configs_dict = configs_dict
            injected_configs_dict = {
                'init_callback': self._initConfigCb,
                'reset_callback': self._resetConfigCb,
                'factory_reset_callback': self._factoryResetConfigCb,
                'init_configs': True,
                'namespace': configs_dict['namespace']
            }
            self.configs_if = NodeConfigsIF(configs_dict = injected_configs_dict,
                                            wait_cfg_mgr = wait_cfg_mgr,
                                            msg_if = self.msg_if,
                                            log_name_list = self.log_name_list)
            nepi_sdk.sleep(1)

        self.params_if = NodeParamsIF(params_dict = params_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)

        ##############################
        # Create Sub Classes
        self.services_if = ConnectNodeServicesIF(services_dict = services_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)
        self.pubs_if = ConnectNodePublishersIF(pubs_dict = pubs_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)
        self.subs_if = ConnectNodeSubscribersIF(subs_dict = subs_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)

        ############################## 
        # Wait for ready
        if do_wait == True:
            if services_dict is not None:
                ready = self.services_if.wait_for_ready()
            if pubs_dict is not None:
                ready = self.pubs_if.wait_for_ready()
            if subs_dict is not None:
                ready = self.subs_if.wait_for_ready()
        else:
            nepi_sdk.sleep(0.1)

        ###############################
        self.ready = True
        self.msg_if.pub_info("Node IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


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
            self.msg_if.pub_debug("ready", log_name_list = self.log_name_list)
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
    def add_param(self, param_name, namespace, value):
        if self.params_if is not None:
            self.params_if.add_param(param_name, namespace, value)

    def get_params(self):
        params = None
        if self.params_if is not None:
            params = self.params_if.get_params()
        return params

    def initialize_params(self):
        if self.params_if is not None:
            self.params_if.initialize_params()

    def reset_params(self):
        if self.params_if is not None:
            self.params_if.reset_params()

    def factory_reset_params(self):
        if self.params_if is not None:
            self.params_if.factory_reset_params()

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
        if self.params_if is not None:
            self.params_if.set_param(param_name, value)


    # Service Methods ####################
    def get_services(self):
        srvs_names = None
        if self.services_if is not None:
            srvs_names = self.services_if.get_services()
        return srvs_names


    def create_request_msg(self,service_name):
        req = None
        if self.services_if is not None:
            req = self.services_if.create_request_msg(service_name)
        return req

    def create_response_msg(self,service_name):
        msg = None
        if self.services_if is not None:
            msg = self.services_if.create_response_msg(service_name)
        return msg

    def call_service(self,service_name,request, verbose = True):
        resp = None
        if self.services_if is not None:
            resp = self.services_if.call_service(service_name,request, verbose = verbose)
        return resp

    def register_service(self,service_name, service_dict):
        if self.services_if is not None:
            self.services_if.register_service(service_name, service_dict)


    def unregister_service(self,service_name):
        if self.services_if is not None:
            self.services_if.unregister_service(service_name)

    def unregister_services(self):
        if self.services_if is not None:
            self.services_if.unregister_services()


    # Publisher Methods ####################
    def get_pubs(self):
        pubs = None
        if self.pubs_if is not None:
            pubs = self.pubs_if.get_pubs()
        return pubs



    def publish_pub(self,pub_name, pub_msg):
        success = False
        if self.pubs_if is not None and not nepi_sdk.is_shutdown():
            succes = self.pubs_if.publish_pub(pub_name, pub_msg)   
        return success


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
                    
    # Subsciber Methods ####################
    def get_subs(self):
        subs = None
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

    # Config-manager callbacks injected into NodeConfigsIF. They reload the
    # params first (so a config reload repopulates them) and then call any
    # caller-provided callback, mirroring NodeClassIF.
    def _initConfigCb(self, do_updates = False):
        self.initialize_params()
        if self.configs_dict is not None:
            if 'init_callback' in self.configs_dict.keys():
                if self.configs_dict['init_callback'] is not None:
                    self.configs_dict['init_callback'](do_updates = do_updates)

    def _resetConfigCb(self):
        self.reset_params()
        if self.configs_dict is not None:
            if 'reset_callback' in self.configs_dict.keys():
                if self.configs_dict['reset_callback'] is not None:
                    self.configs_dict['reset_callback']()

    def _factoryResetConfigCb(self):
        self.factory_reset_params()
        if self.configs_dict is not None:
            if 'factory_reset_callback' in self.configs_dict.keys():
                if self.configs_dict['factory_reset_callback'] is not None:
                    self.configs_dict['factory_reset_callback']()


