#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

import time

from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint

from nepi_interfaces.msg import UpdateBool

from nepi_app_fake_gps.msg import NepiAppFakeGpsStatus

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_fake_gps'


class ConnectAppFakeGps:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

    #######################
    ### IF Initialization

    def __init__(self, namespace=None):
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace, APP_NODE_NAME)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.CFGS_DICT = {'namespace': self.namespace}
        self.SRVS_DICT = None

        ns = self.namespace
        self.PUBS_DICT = {
            'update_device_enable': {'namespace': ns, 'topic': 'update_device_enable', 'msg': UpdateBool, 'qsize': 1},
            'reset':             {'namespace': ns, 'topic': 'reset',             'msg': GeoPoint, 'qsize': 1},
            'go_stop':           {'namespace': ns, 'topic': 'go_stop',           'msg': Empty,    'qsize': 1},
            'goto_position':     {'namespace': ns, 'topic': 'goto_position',     'msg': Point,    'qsize': 1},
            'goto_location':     {'namespace': ns, 'topic': 'goto_location',     'msg': GeoPoint, 'qsize': 1},
            'set_gps_pub_rate':  {'namespace': ns, 'topic': 'set_gps_pub_rate',  'msg': Float32,  'qsize': 1},
            'save_config':          {'namespace': ns, 'topic': 'save_config',          'msg': Empty, 'qsize': None, 'latch': False},
            'reset_config':         {'namespace': ns, 'topic': 'reset_config',         'msg': Empty, 'qsize': None, 'latch': False},
            'factory_reset_config': {'namespace': ns, 'topic': 'factory_reset_config', 'msg': Empty, 'qsize': None, 'latch': False},
        }

        self.SUBS_DICT = {
            'status_sub': {
                'namespace': ns,
                'topic':     'status',
                'msg':       NepiAppFakeGpsStatus,
                'qsize':     1,
                'callback':  self._statusCb,
            }
        }

        self.con_node_if = ConnectNodeClassIF(
            namespace=self.namespace,
            configs_dict=self.CFGS_DICT,
            services_dict=self.SRVS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT,
            log_class_name=True,
            msg_if=self.msg_if,
        )
        self.con_node_if.wait_for_ready()

        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")


    #######################
    # Class Public Methods
    #######################

    def get_ready_state(self):
        return self.ready

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def check_status_connection(self):
        return self.status_connected

    def get_status_dict(self):
        if self.status_msg is not None:
            return nepi_sdk.convert_msg2dict(self.status_msg)
        return None

    def set_device_enabled(self, node_namespace, enabled):
        """Enable or disable HilGPS/GPS_INPUT injection for ONE mavros node
        namespace, independent of every other device's own enabled state."""
        msg = UpdateBool()
        msg.name = str(node_namespace)
        msg.value = bool(enabled)
        self.con_node_if.publish_pub('update_device_enable', msg)

    def reset_location(self, latitude, longitude, altitude):
        """Reset the simulated GPS home position to a new WGS84 geopoint."""
        msg = GeoPoint()
        msg.latitude = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude = float(altitude)
        self.con_node_if.publish_pub('reset', msg)

    def go_stop(self):
        """Stop any active simulated move and hold the current position."""
        self.con_node_if.publish_pub('go_stop', Empty())

    def goto_position(self, x, y, z):
        """Move the simulated position by an ENU offset in meters (east, north, up)."""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.con_node_if.publish_pub('goto_position', msg)

    def goto_location(self, latitude, longitude, altitude):
        """Move the simulated position to an absolute WGS84 geopoint."""
        msg = GeoPoint()
        msg.latitude = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude = float(altitude)
        self.con_node_if.publish_pub('goto_location', msg)

    def set_gps_pub_rate(self, rate_hz):
        """Set the fake GPS publish rate in Hz (clamped to 1-100 by the node)."""
        msg = Float32()
        msg.data = float(rate_hz)
        self.con_node_if.publish_pub('set_gps_pub_rate', msg)

    def save_config(self):
        self.con_node_if.publish_pub('save_config', Empty())

    def reset_config(self):
        self.con_node_if.publish_pub('reset_config', Empty())

    def factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config', Empty())

    def unregister(self):
        self._unregisterNode()


    ###############################
    # Class Private Methods
    ###############################

    def _unregisterNode(self):
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering: " + str(self.namespace))
            try:
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.status_connected = False
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister: " + str(e))

    def _statusCb(self, status_msg):
        self.status_connected = True
        self.connected = True
        self.status_msg = status_msg
