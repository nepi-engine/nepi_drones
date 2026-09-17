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

###################################################
### For Ardupilot Mavlink Support
### These Ardupilot Parameters Must Be Configured First to allow MAVLINK GPS Override:
#GPS_TYPE = 14
#GPS_DELAY_MS = 1
#EK3_POS_I_GATE = 300
#EK3_POSNE_M_NSE = 5
#EK3_SRC_OPTIONS = 0
#EK3_SRC1_POSXY = 3
#EK3_SRC1_POSZ = 3
#EK3_SRC1_VELXY = 3
#EK3_SRC1_VELZ = 3
#EK3_SRC1_YAW = 1
#BARO_OPTION = 1  (required for proper barometer reading on Pixhawk)
#####################################################

import copy
import math
import threading
import time

import numpy as np

from std_msgs.msg import Empty, Float32, Header
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

from mavros_msgs.msg import HilGPS, GPSINPUT

from nepi_interfaces.msg import UpdateBool

from nepi_app_fake_gps.msg import NepiAppFakeGpsStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import NavPoseIF


#########################################
# Factory Defaults

# [Lat, Long, Altitude_WGS84]
FACTORY_START_LATITUDE   = 46.6540828
FACTORY_START_LONGITUDE  = -122.3187578
FACTORY_START_ALTITUDE_M = 0.0

FACTORY_ENABLED_DEVICES  = []
FACTORY_SAT_COUNT        = 20

# Fake GPS publish rate (Hz) and move interpolation tuning
GPS_PUB_RATE_HZ            = 50      # factory default publish rate
MIN_GPS_PUB_RATE_HZ        = 1.0
MAX_GPS_PUB_RATE_HZ        = 100.0
MOVE_UPDATE_TIME_SEC_PER_M = 1.0
MAX_MOVE_TIME_S            = 20.0

# A goto axis value of this sentinel means "hold the current value"
HOLD_SENTINEL = -999.0

STATUS_PUBLISH_RATE_HZ = 1.0
DISCOVER_RATE_HZ       = 1.0

# mavros liveness topic used for target-node discovery
MAVROS_STATE_MSG   = 'State'
MAVROS_STATE_SUFFIX = '/state'
MAVROS_HILGPS_TOPIC = 'hil/gps'
# GPS_INPUT carries a yaw field (HilGPS does not), letting the fake GPS supply the
# heading the EKF would otherwise get from a compass. Injected via the mavros
# gps_input plugin; consumed by the ArduPilot MAV GPS backend (GPS_TYPE=14).
MAVROS_GPS_INPUT_TOPIC = 'gps_input/gps_input'

# GPS_INPUT ignore-flags bits (mavlink GPS_INPUT_IGNORE_FLAGS). We now provide
# lat/lon/alt + yaw AND an earth-frame NED velocity (finite-differenced from the
# simulated ENU position), so no fields are ignored. The EKF velocity source is
# GPS (EK3_SRC1_VELXY/VELZ = GPS); injecting velocity (0 while hovering) rather
# than ignoring it is what lets the EKF declare a healthy horizontal position,
# which ArduPilot GUIDED mode requires ("Mode change to GUIDED failed: requires
# position" otherwise).
GPS_INPUT_IGNORE_VEL_HORIZ = 8
GPS_INPUT_IGNORE_VEL_VERT  = 16
GPS_INPUT_IGNORE_SPEED_ACC = 32
# Reject finite-difference velocity spikes from a position reset/teleport (m/s).
MAX_FAKE_GPS_SPEED_MPS = 100.0
# GPS week/epoch for deriving GPS_INPUT time-of-week from system UTC. A real GPS
# always reports valid, advancing GPS time; without it AP_GPS marks the receiver
# unhealthy ("GPS: Fail") and the EKF will not declare a healthy position.
GPS_EPOCH_UNIX_S = 315964800   # 1980-01-06 00:00:00 UTC, in Unix seconds
GPS_LEAP_SECONDS = 18          # current GPS-UTC offset (leap seconds)
SECONDS_PER_WEEK = 604800


#########################################
# Node Class
#########################################

class NepiFakeGpsApp(object):

    node_if = None

    DEFAULT_NODE_NAME = "app_fake_gps"

    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################
        # Initialize Class Variables
        # List of mavros node namespaces HilGPS/GPS_INPUT injection is
        # currently enabled for -- independently toggleable per device (see
        # NepiAppFakeGpsStatus.msg's own comment for why this replaced a
        # single global enabled+selected_mavros_node).
        self.enabled_devices = list(FACTORY_ENABLED_DEVICES)
        self.satellites_visible = FACTORY_SAT_COUNT
        self.gps_pub_rate_hz = float(GPS_PUB_RATE_HZ)

        self.start_latitude   = FACTORY_START_LATITUDE
        self.start_longitude  = FACTORY_START_LONGITUDE
        self.start_altitude_m = FACTORY_START_ALTITUDE_M

        # Simulated state (protected by _lock)
        self._lock = threading.Lock()
        self.current_location_wgs84_geo = self._makeGeoPoint(
            self.start_latitude, self.start_longitude, self.start_altitude_m)
        self.current_point = self._zeroPoint()
        self.new_point = self._zeroPoint()
        # Previous ENU point + monotonic time for finite-difference velocity
        self._prev_vel_point = None
        self._prev_vel_time = None
        # Simulated orientation: heading (deg true north) and ENU yaw (deg).
        # Derived from the horizontal direction of travel during a move; held
        # at the last value when stopped. Vehicle is treated as level (roll/pitch 0).
        self.current_heading_deg = 0.0
        self.current_yaw_enu_deg = 0.0
        self._move_plan = None
        self._alive = True

        # Discovered mavros targets and this device's own bound publisher per
        # enabled target (mavros_ns -> GPSINPUT publisher).
        self.available_mavros_nodes = []
        self.mavlink_pub_dict = {}
        self._navpose_if = None

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
            'enabled_devices': {
                'namespace': self.node_namespace,
                'factory_val': self.enabled_devices
            },
            'start_latitude': {
                'namespace': self.node_namespace,
                'factory_val': self.start_latitude
            },
            'start_longitude': {
                'namespace': self.node_namespace,
                'factory_val': self.start_longitude
            },
            'start_altitude_m': {
                'namespace': self.node_namespace,
                'factory_val': self.start_altitude_m
            },
            'satellites_visible': {
                'namespace': self.node_namespace,
                'factory_val': self.satellites_visible
            },
            'gps_pub_rate_hz': {
                'namespace': self.node_namespace,
                'factory_val': self.gps_pub_rate_hz
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': NepiAppFakeGpsStatus,
                'qsize': 1,
                'latch': True
            },
            'gps_fix_pub': {
                'namespace': self.node_namespace,
                'topic': 'gps_fix',
                'msg': NavSatFix,
                'qsize': 1,
                'latch': False
            },
            'odom_pub': {
                'namespace': self.node_namespace,
                'topic': 'odom',
                'msg': Odometry,
                'qsize': 1,
                'latch': False
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'update_device_enable': {
                'namespace': self.node_namespace,
                'topic': 'update_device_enable',
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.updateDeviceEnableCb,
                'callback_args': ()
            },
            'reset': {
                'namespace': self.node_namespace,
                'topic': 'reset',
                'msg': GeoPoint,
                'qsize': 10,
                'callback': self.fakeGpsResetLocCb,
                'callback_args': ()
            },
            'go_stop': {
                'namespace': self.node_namespace,
                'topic': 'go_stop',
                'msg': Empty,
                'qsize': 10,
                'callback': self.fakeGpsGoStopCb,
                'callback_args': ()
            },
            'goto_position': {
                'namespace': self.node_namespace,
                'topic': 'goto_position',
                'msg': Point,
                'qsize': 10,
                'callback': self.fakeGpsGoPosCb,
                'callback_args': ()
            },
            'goto_location': {
                'namespace': self.node_namespace,
                'topic': 'goto_location',
                'msg': GeoPoint,
                'qsize': 10,
                'callback': self.fakeGpsGoLocCb,
                'callback_args': ()
            },
            'set_gps_pub_rate': {
                'namespace': self.node_namespace,
                'topic': 'set_gps_pub_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setGpsPubRateCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
            configs_dict=self.CFGS_DICT,
            params_dict=self.PARAMS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT
        )

        self.node_if.wait_for_ready()

        ##############################
        self.initCb(do_updates=True)

        # Optional simulated-position NavPose output (primary output remains HilGPS)
        self._setupNavpose()

        ##############################
        # Start the non-blocking simulate + publish thread
        threading.Thread(target=self._simThreadLoop, daemon=True).start()

        # Start mavros target discovery + status publishing
        time.sleep(1)
        nepi_sdk.start_timer_process(float(1) / DISCOVER_RATE_HZ, self.discoverMavrosCb, oneshot=True)
        nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)

        time.sleep(1)
        self.msg_if.pub_info("Initialization Complete")

        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()


    #######################
    ### Static Helpers

    def _zeroPoint(self):
        p = Point()
        p.x = 0.0
        p.y = 0.0
        p.z = 0.0
        return p

    def _makeGeoPoint(self, lat, lon, alt):
        geo = GeoPoint()
        geo.latitude = float(lat)
        geo.longitude = float(lon)
        geo.altitude = float(alt)
        return geo


    #######################
    ### App Config Functions

    def initCb(self, do_updates=False):
        if self.node_if is not None:
            self.enabled_devices = list(self.node_if.get_param('enabled_devices'))
            self.satellites_visible = self.node_if.get_param('satellites_visible')
            self.gps_pub_rate_hz = self.node_if.get_param('gps_pub_rate_hz')
            self.start_latitude = self.node_if.get_param('start_latitude')
            self.start_longitude = self.node_if.get_param('start_longitude')
            self.start_altitude_m = self.node_if.get_param('start_altitude_m')
        if do_updates:
            # Reset the simulated position to the configured start location
            with self._lock:
                self.current_location_wgs84_geo = self._makeGeoPoint(
                    self.start_latitude, self.start_longitude, self.start_altitude_m)
                self.current_point = self._zeroPoint()
                self.new_point = self._zeroPoint()
                self.current_heading_deg = 0.0
                self.current_yaw_enu_deg = 0.0
                self._move_plan = None
        self.publish_status()

    def resetCb(self, do_updates=True):
        self.msg_if.pub_warn("Resetting")
        if self.node_if is not None:
            pass
        self.initCb(do_updates=do_updates)

    def factoryResetCb(self, do_updates=True):
        self.msg_if.pub_warn("Factory Resetting")
        if self.node_if is not None:
            pass
        self.initCb(do_updates=do_updates)


    #######################
    ### Mavros Target Discovery

    def discoverMavrosCb(self, timer):
        last_available = copy.deepcopy(self.available_mavros_nodes)
        topics = nepi_sdk.find_topics_by_msg(MAVROS_STATE_MSG)
        available = []
        for topic in topics:
            if topic.endswith(MAVROS_STATE_SUFFIX):
                ns = topic[:-len(MAVROS_STATE_SUFFIX)]
                if ns not in available:
                    available.append(ns)
        needs_publish = False
        if available != last_available:
            self.available_mavros_nodes = available
            needs_publish = True

        # Bind any device that's enabled and newly available (including one
        # requested before its mavros node existed yet -- e.g. enabled_devices
        # restored from a saved config before discovery has run). Unbind
        # anything bound whose mavros node has disappeared from the graph, or
        # that's no longer in enabled_devices.
        with self._lock:
            bound = list(self.mavlink_pub_dict.keys())
        for mavros_ns in bound:
            if mavros_ns not in available or mavros_ns not in self.enabled_devices:
                self.unbindMavros(mavros_ns)
                needs_publish = True
        for mavros_ns in self.enabled_devices:
            if mavros_ns in available and mavros_ns not in self.mavlink_pub_dict:
                self.bindMavros(mavros_ns)
                needs_publish = True

        if needs_publish:
            self.publish_status()
        nepi_sdk.start_timer_process(float(1) / DISCOVER_RATE_HZ, self.discoverMavrosCb, oneshot=True)

    def bindMavros(self, mavros_ns):
        self.unbindMavros(mavros_ns)
        topic = nepi_sdk.create_namespace(mavros_ns, MAVROS_GPS_INPUT_TOPIC)
        pub = nepi_sdk.create_publisher(topic, GPSINPUT, queue_size=1)
        with self._lock:
            self.mavlink_pub_dict[mavros_ns] = pub
        self.msg_if.pub_info("Fake GPS will publish GPS_INPUT (with yaw) to: " + topic)

    def unbindMavros(self, mavros_ns):
        with self._lock:
            pub = self.mavlink_pub_dict.pop(mavros_ns, None)
        if pub is not None:
            try:
                pub.unregister()
            except Exception:
                pass


    #######################
    ### NavPose Output (optional)

    def _setupNavpose(self):
        try:
            self._navpose_if = NavPoseIF(
                namespace=nepi_sdk.create_namespace(self.node_namespace, 'navpose'),
                data_source_description='fake_gps',
                data_ref_description='WGS84',
                pub_navpose=True, pub_location=True, pub_altitude=True,
                pub_heading=True, pub_orientation=True,
                msg_if=self.msg_if,
            )
        except Exception as e:
            self.msg_if.pub_warn("Fake GPS: NavPose output unavailable: " + str(e))
            self._navpose_if = None

    def publishNavpose(self, geo, heading_deg, yaw_enu_deg):
        if self._navpose_if is None:
            return
        try:
            t = nepi_sdk.get_time()
            d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
            d['has_location'] = True
            d['latitude'] = geo.latitude
            d['longitude'] = geo.longitude
            d['time_location'] = t
            d['has_altitude'] = True
            d['altitude_m'] = geo.altitude
            d['time_altitude'] = t
            # Simulated heading (deg true north) and orientation. The navpose dict
            # is in the ENU frame (BLANK_NAVPOSE_DICT default), so yaw_deg is ENU yaw;
            # roll/pitch are 0 for a level vehicle.
            d['has_heading'] = True
            d['heading_deg'] = heading_deg
            d['time_heading'] = t
            d['has_orientation'] = True
            d['roll_deg'] = 0.0
            d['pitch_deg'] = 0.0
            d['yaw_deg'] = yaw_enu_deg
            d['time_orientation'] = t
            self._navpose_if.publish_navpose(d)
        except Exception:
            pass


    #######################
    # Node Control Callbacks

    def updateDeviceEnableCb(self, msg):
        # msg.name is a mavros node namespace (one of available_mavros_nodes),
        # msg.value is the enable/disable request for THAT device only --
        # every other device's own enabled state is untouched. This is the
        # fix for two ArduPilot devices (a physical drone + its SITL twin)
        # fighting over a single shared enabled+selected_mavros_node: each
        # now only ever touches its own entry.
        mavros_ns = msg.name
        enabled = bool(msg.value)
        self.msg_if.pub_info("Received Fake GPS enable=" + str(enabled) + " for: " + mavros_ns)
        if mavros_ns not in self.available_mavros_nodes:
            self.msg_if.pub_warn("Fake GPS: '" + mavros_ns + "' is not a currently known mavros node, ignoring")
            return
        if enabled:
            if mavros_ns not in self.enabled_devices:
                self.enabled_devices.append(mavros_ns)
            self.bindMavros(mavros_ns)
        else:
            if mavros_ns in self.enabled_devices:
                self.enabled_devices.remove(mavros_ns)
            self.unbindMavros(mavros_ns)
        if self.node_if is not None:
            self.node_if.set_param('enabled_devices', self.enabled_devices)
            self.node_if.save_config()
        self.publish_status()

    def setGpsPubRateCb(self, msg):
        rate = float(msg.data)
        if rate < MIN_GPS_PUB_RATE_HZ:
            rate = MIN_GPS_PUB_RATE_HZ
        elif rate > MAX_GPS_PUB_RATE_HZ:
            rate = MAX_GPS_PUB_RATE_HZ
        self.msg_if.pub_info("Setting GPS publish rate to: " + "%.2f" % rate + " Hz")
        self.gps_pub_rate_hz = rate
        if self.node_if is not None:
            self.node_if.set_param('gps_pub_rate_hz', self.gps_pub_rate_hz)
            self.node_if.save_config()
        self.publish_status()

    def fakeGpsResetLocCb(self, geo_msg):
        geo_str = str([geo_msg.latitude, geo_msg.longitude, geo_msg.altitude])
        self.msg_if.pub_info("Received Fake GPS Reset to Location Msg: " + geo_str)
        self.resetGpsLoc(geo_msg)
        self.msg_if.pub_info("Reset Complete")

    def resetGpsLoc(self, geo_msg):
        with self._lock:
            self._move_plan = None
            self.current_location_wgs84_geo = self._makeGeoPoint(
                geo_msg.latitude, geo_msg.longitude, geo_msg.altitude)
            self.current_point = self._zeroPoint()
            self.new_point = self._zeroPoint()
        self.publish_status()

    def fakeGpsGoStopCb(self, empty_msg):
        if len(self.enabled_devices) > 0:
            self.msg_if.pub_info("Received go stop message")
            with self._lock:
                self._move_plan = None

    def fakeGpsGoPosCb(self, enu_point_msg):
        if len(self.enabled_devices) > 0:
            self.msg_if.pub_info("Received GoTo Position Message: " + str(enu_point_msg))
            with self._lock:
                cur_geo = copy.deepcopy(self.current_location_wgs84_geo)
            new_enu_position = [enu_point_msg.x, enu_point_msg.y, enu_point_msg.z]
            new_geopoint_wgs84 = nepi_nav.get_geopoint_at_enu_point(cur_geo, new_enu_position)
            self.startMove(new_geopoint_wgs84, enu_point_msg)

    def fakeGpsGoLocCb(self, geo_msg):
        if len(self.enabled_devices) > 0:
            self.msg_if.pub_info("Received GoTo Location Message: " + str(geo_msg))
            self.startMove(geo_msg, self._zeroPoint())


    #######################
    # Move Planning (non-blocking)

    def startMove(self, geopoint_msg, ned_delta_point):
        with self._lock:
            org_geo = np.array([self.current_location_wgs84_geo.latitude,
                                self.current_location_wgs84_geo.longitude,
                                self.current_location_wgs84_geo.altitude])
            new_geo = np.array([geopoint_msg.latitude,
                                geopoint_msg.longitude,
                                geopoint_msg.altitude])
            for ind in range(len(new_geo)):
                if new_geo[ind] == HOLD_SENTINEL:  # use current value for this axis
                    new_geo[ind] = org_geo[ind]
            delta_geo = new_geo - org_geo
            move_dist_m = nepi_nav.distance_geopoints(org_geo, new_geo)

            # Update simulated heading from the horizontal direction of travel
            self._updateHeading(org_geo, new_geo)

            org_point = np.array([self.current_point.x, self.current_point.y, self.current_point.z])
            delta_point = np.array([ned_delta_point.x, ned_delta_point.y, ned_delta_point.z])

            if move_dist_m <= 0:
                # Nothing to interpolate; snap any NED delta and finish
                self._move_plan = None
                self.current_point.x = float(org_point[0] + delta_point[0])
                self.current_point.y = float(org_point[1] + delta_point[1])
                self.current_point.z = float(org_point[2] + delta_point[2])
                return

            move_time = MOVE_UPDATE_TIME_SEC_PER_M * move_dist_m
            if move_time > MAX_MOVE_TIME_S:
                move_time = MAX_MOVE_TIME_S
            move_steps = int(move_time * self.gps_pub_rate_hz)
            if move_steps < 2:
                # Too short to ramp; snap to the target
                self.current_location_wgs84_geo = self._makeGeoPoint(
                    new_geo[0], new_geo[1], new_geo[2])
                self.current_point.x = float(org_point[0] + delta_point[0])
                self.current_point.y = float(org_point[1] + delta_point[1])
                self.current_point.z = float(org_point[2] + delta_point[2])
                self._move_plan = None
                return

            # Hanning-squared cumulative ramp for smooth ease-in/ease-out
            ramp = np.hanning(move_steps)
            ramp = ramp ** 2
            ramp_norm = ramp / np.sum(ramp)
            step_norm = np.zeros(len(ramp_norm))
            for ind in range(len(ramp_norm)):
                step_norm[ind] = np.sum(ramp_norm[0:ind])

            self._move_plan = {
                'org_geo': org_geo,
                'delta_geo': delta_geo,
                'org_point': org_point,
                'delta_point': delta_point,
                'step_norm': step_norm,
                'idx': 0,
            }
            self.msg_if.pub_info("Fake GPS moving %.2f meters in %.2f seconds (%d steps)"
                                 % (move_dist_m, move_time, move_steps))

    def _updateHeading(self, org_geo, new_geo):
        # Caller holds self._lock. Set heading/yaw from the horizontal direction of
        # travel (great-circle approximation over a short move). Holds the previous
        # heading when the move has no horizontal component (e.g. a pure altitude change).
        north = float(new_geo[0] - org_geo[0])
        east = float(new_geo[1] - org_geo[1]) * math.cos(math.radians(float(org_geo[0])))
        if math.hypot(north, east) > 1e-12:
            heading_deg = math.degrees(math.atan2(east, north)) % 360.0
            self.current_heading_deg = heading_deg
            self.current_yaw_enu_deg = nepi_nav.convert_yaw_ned2enu(heading_deg)

    def _advanceMove(self):
        # Caller holds self._lock. Advance the active move plan one step.
        plan = self._move_plan
        if plan is None:
            return
        step_norm = plan['step_norm']
        idx = plan['idx']
        if idx >= len(step_norm):
            # Snap exactly to the target and clear the plan
            final_geo = plan['org_geo'] + plan['delta_geo']
            final_point = plan['org_point'] + plan['delta_point']
            self.current_location_wgs84_geo = self._makeGeoPoint(
                final_geo[0], final_geo[1], final_geo[2])
            self.current_point.x = float(final_point[0])
            self.current_point.y = float(final_point[1])
            self.current_point.z = float(final_point[2])
            self._move_plan = None
            return
        val = step_norm[idx]
        cur_geo = plan['org_geo'] + plan['delta_geo'] * val
        cur_point = plan['org_point'] + plan['delta_point'] * val
        self.current_location_wgs84_geo.latitude = float(cur_geo[0])
        self.current_location_wgs84_geo.longitude = float(cur_geo[1])
        self.current_location_wgs84_geo.altitude = float(cur_geo[2])
        self.current_point.x = float(cur_point[0])
        self.current_point.y = float(cur_point[1])
        self.current_point.z = float(cur_point[2])
        plan['idx'] = idx + 1


    #######################
    # Simulate + Publish Thread

    def _simThreadLoop(self):
        while self._alive and not nepi_sdk.is_shutdown():
            with self._lock:
                self._advanceMove()
                geo = copy.deepcopy(self.current_location_wgs84_geo)
                point = copy.deepcopy(self.current_point)
                heading = self.current_heading_deg
                yaw_enu = self.current_yaw_enu_deg
                enabled = len(self.enabled_devices) > 0
                rate = self.gps_pub_rate_hz
            if enabled:
                self.publishFakeGps(geo, point, heading, yaw_enu)
            # Recompute each loop so a live rate change takes effect immediately
            if rate < MIN_GPS_PUB_RATE_HZ:
                rate = MIN_GPS_PUB_RATE_HZ
            time.sleep(1.0 / rate)

    def publishFakeGps(self, geo, point, heading_deg, yaw_enu_deg):
        if nepi_sdk.is_shutdown():
            return
        stamp = nepi_sdk.get_msg_stamp()

        # NavSatFix output
        navsatfix = NavSatFix()
        navsatfix.header.stamp = stamp
        navsatfix.latitude = geo.latitude
        navsatfix.longitude = geo.longitude
        navsatfix.altitude = geo.altitude

        # Odometry (ENU position relative to last reset) output, with orientation
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.pose.pose.position.x = point.x
        odom_msg.pose.pose.position.y = point.y
        odom_msg.pose.pose.position.z = point.z
        # Orientation quaternion: yaw about the ENU up-axis (roll/pitch 0, level vehicle)
        half_yaw_rad = math.radians(yaw_enu_deg) / 2.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(half_yaw_rad)
        odom_msg.pose.pose.orientation.w = math.cos(half_yaw_rad)

        if self.node_if is not None:
            self.node_if.publish_pub('gps_fix_pub', navsatfix)
            self.node_if.publish_pub('odom_pub', odom_msg)

        # Optional NavPose output (location + altitude + heading + orientation)
        self.publishNavpose(geo, heading_deg, yaw_enu_deg)

        # Earth-frame NED velocity for GPS_INPUT (0 while hovering, move velocity
        # during a goto). Computed every publish so the previous-sample state
        # stays fresh regardless of whether MAVLink injection is active.
        vn, ve, vd = self._computeNedVelocity(point)

        # MAVLink GPS_INPUT injection (the primary required output), fanned out
        # to every device currently enabled -- all bound targets share this one
        # simulated trajectory (see this app's own status msg comment: enabling
        # per-device chooses WHICH vehicles receive it, not separate simulated
        # positions per vehicle). GPS_INPUT carries a yaw field, so the fake GPS
        # also supplies the heading the EKF would otherwise take from a
        # compass. With EK3_SRC1_YAW=2 (GPS) and the compass disabled on the
        # FCU, this lets a compass-less vehicle hold yaw.
        with self._lock:
            mavlink_pubs = list(self.mavlink_pub_dict.values())
        if mavlink_pubs:
            gpsin = GPSINPUT()
            gpsin.header = Header(stamp=stamp, frame_id="mavlink_fake_gps")
            gpsin.fix_type = 3  # 3D fix
            gpsin.gps_id = 0
            # GPS time-of-week. Without valid, advancing GPS time AP_GPS reports
            # the receiver unhealthy and the EKF refuses a position solution, so
            # arming in a GPS mode fails with "Need Position Estimate". Derive it
            # from system UTC (absolute accuracy is not critical; it must be a
            # plausible current week and advance at real rate).
            gps_tow_s = time.time() - GPS_EPOCH_UNIX_S + GPS_LEAP_SECONDS
            gpsin.time_week = int(gps_tow_s // SECONDS_PER_WEEK)
            gpsin.time_week_ms = int((gps_tow_s % SECONDS_PER_WEEK) * 1000.0)
            gpsin.ignore_flags = 0  # provide position, altitude, and velocity
            gpsin.lat = int(round(geo.latitude * 1e7))
            gpsin.lon = int(round(geo.longitude * 1e7))
            gpsin.alt = float(geo.altitude)
            gpsin.hdop = 1.0
            gpsin.vdop = 1.0
            gpsin.vn = float(vn)
            gpsin.ve = float(ve)
            gpsin.vd = float(vd)
            gpsin.speed_accuracy = 0.5
            gpsin.horiz_accuracy = 1.0
            gpsin.vert_accuracy = 1.0
            gpsin.satellites_visible = self.satellites_visible
            # yaw: centidegrees true north, 0 = "not available", 36000 = north.
            # Map heading (0..360) so a 0-deg (north) heading still reports a value.
            yaw_cdeg = int(round(heading_deg * 100.0)) % 36000
            if yaw_cdeg == 0:
                yaw_cdeg = 36000
            gpsin.yaw = yaw_cdeg
            for mavlink_pub in mavlink_pubs:
                try:
                    mavlink_pub.publish(gpsin)
                except Exception:
                    pass

    def _computeNedVelocity(self, point):
        # Finite-difference the simulated ENU position (point) into an
        # earth-frame NED velocity for GPS_INPUT. Returns (vn, ve, vd) m/s: 0
        # while hovering (point constant), the move velocity during a goto.
        # A reset/teleport produces a huge one-sample spike, which is rejected.
        now_s = time.monotonic()
        vn = ve = vd = 0.0
        prev = self._prev_vel_point
        prev_t = self._prev_vel_time
        if prev is not None and prev_t is not None:
            dt = now_s - prev_t
            if dt > 1e-3:
                cand_vn = (point.y - prev.y) / dt   # ENU north -> NED north
                cand_ve = (point.x - prev.x) / dt   # ENU east  -> NED east
                cand_vd = -(point.z - prev.z) / dt  # ENU up    -> NED down
                if max(abs(cand_vn), abs(cand_ve), abs(cand_vd)) < MAX_FAKE_GPS_SPEED_MPS:
                    vn, ve, vd = cand_vn, cand_ve, cand_vd
        self._prev_vel_point = copy.deepcopy(point)
        self._prev_vel_time = now_s
        return vn, ve, vd


    #######################
    ### Status Publishing

    def statusPublishCb(self, timer):
        self.publish_status()

    def publish_status(self):
        if self.node_if is None:
            return
        with self._lock:
            geo = copy.deepcopy(self.current_location_wgs84_geo)
            point = copy.deepcopy(self.current_point)
            heading = self.current_heading_deg
            yaw = self.current_yaw_enu_deg
            moving = self._move_plan is not None

        with self._lock:
            bound_set = set(self.mavlink_pub_dict.keys())

        status_msg = NepiAppFakeGpsStatus()
        status_msg.available_mavros_nodes = self.available_mavros_nodes
        status_msg.mavros_node_enabled = [ns in self.enabled_devices for ns in self.available_mavros_nodes]
        status_msg.mavros_node_bound = [ns in bound_set for ns in self.available_mavros_nodes]

        status_msg.current_latitude = geo.latitude
        status_msg.current_longitude = geo.longitude
        status_msg.current_altitude_m = geo.altitude

        status_msg.start_latitude = self.start_latitude
        status_msg.start_longitude = self.start_longitude
        status_msg.start_altitude_m = self.start_altitude_m

        status_msg.current_point_x = point.x
        status_msg.current_point_y = point.y
        status_msg.current_point_z = point.z

        status_msg.current_heading_deg = float(heading)
        status_msg.current_yaw_deg = float(yaw)

        status_msg.moving = moving
        status_msg.satellites_visible = self.satellites_visible
        status_msg.gps_pub_rate_hz = float(self.gps_pub_rate_hz)

        self.node_if.publish_pub('status_pub', status_msg)


    #######################
    # Utility Functions

    def cleanup_actions(self):
        self.msg_if.pub_info("FAKE_GPS_APP: Shutting down: Executing script cleanup actions")
        self._alive = False
        with self._lock:
            bound = list(self.mavlink_pub_dict.keys())
        for mavros_ns in bound:
            self.unbindMavros(mavros_ns)
        if self._navpose_if is not None:
            try:
                self._navpose_if.unregister_pubs()
            except Exception:
                pass
            self._navpose_if = None


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiFakeGpsApp()
