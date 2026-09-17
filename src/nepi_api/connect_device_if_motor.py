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

import time
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_interfaces.msg import MotorStatus, MotorsStatus, MotorCommand

from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeIF
from nepi_api.connect_node_if import ConnectNodeClassIF


#########################################
# Connect IF Class
#########################################

# Auto-discovering connect interface for the standard NEPI multi-motor status
# interface, modeled on ConnectPTXDeviceIF. It subclasses ConnectNodeIF, which
# owns the <node>/motor_connect connect namespace (ConnectIFStatus selector
# state plus the select_topic subscriber), auto-discovers the motorized devices
# publishing nepi_interfaces/MotorsStatus, and connects to the selected device.
#
# The Nepi_IF_ConnectMotor RUI component talks to that connect namespace
# directly. Alongside monitoring and selection -- subscribe, store the latest
# message, and expose read-only getters over the MotorStatus[] array -- this
# class publishes each of the standard motor command topics through a matching
# public method.

CONNECT_ID = 'MOTOR'
CONNECT_STATUS_MSG = 'MotorsStatus'
CONNECT_NAME = 'motor_connect'

# The dedicated topic every motorized NEPI device publishes MotorsStatus on.
# This is NOT the '/status' topic the ConnectNodeIF base assumes, which is why
# _updaterCb and subscribe_topic below override the base namespace/topic logic.
STATUS_TOPIC = 'motor_status'

# The six standard motor command topics published alongside 'motor_status' on
# each device namespace, each carrying a nepi_interfaces/MotorCommand message.
# goto_rotation and set_rotation_speed drive a rotatable (steering) wheel and
# are ignored by a device whose motors report has_rotation false.
COMMAND_TOPICS = ['set_speed', 'set_direction', 'go_direction', 'stop_motor',
                  'goto_rotation', 'set_rotation_speed']

CONNECTED_TIMEOUT = 2


class ConnectMotorsDeviceIF(ConnectNodeIF):

    # ADD Additional Connect Callback Functions

    msg_if = None
    ready = False
    namespace = '~'

    node_if = None

    status_msg = None
    connected = False
    last_status_time = 0

    statusCb = None  # Backwards Compatibility

    dataCB = None

    connect_topic_subs_dict = None
    connect_topic_pubs_dict = None

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = CONNECT_NAME,
                namespace = None,
                statusCb = None,
                dataCB = None,
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        self.msg_if = msg_if
        self.node_if = node_if
        super().__init__(
                connect_id = CONNECT_ID,
                connect_status_msg = CONNECT_STATUS_MSG,
                connect_name = connect_name,
                selected_topic = namespace,
                auto_select_enabled = True,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = self.msg_if,
                node_if = self.node_if
                )
        ####  IF INIT SETUP ####

        self.wait_for_connect_ready()

        ##############################
        # Initialize Class Variables

        self.statusCb = statusCb
        self.dataCB = dataCB

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################


    #######################
    # Class Public Methods
    #######################

    def get_ready_state(self):
        """Return the ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready

    def get_namespace(self):
        """Return the fully-resolved ROS namespace for the connected motorized device.

        Returns:
            str: The fully-qualified device namespace currently selected, or 'None'
                if no device is selected.
        """
        return self.selected_topic

    def check_connection(self):
        """Check whether the device is currently connected.

        Returns:
            bool: True if a MotorsStatus message has been received within the connection
                timeout window, False otherwise.
        """
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        """Block until the device is connected or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if connection was established, False if the timeout was reached.
        """
        if self.node_if is not None and self.selected_topic != 'None':
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

    def check_status_connection(self):
        """Check whether the status topic from the device is currently connected.

        Returns:
            bool: True if status messages are being received, False otherwise.
        """
        return self.connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        """Block until the device status topic is connected or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the status connection was established, False if the timeout was reached.
        """
        if self.node_if is not None and self.selected_topic != 'None':
            self.msg_if.pub_info("Waiting for status connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to connect to status msg")
            else:
                self.msg_if.pub_info("Status Connected")
        return self.connected

    def get_available_topics(self):
        """Return the motorized device namespaces currently discovered as available.

        Returns:
            list: A list of device namespace strings publishing MotorsStatus.
        """
        return self.available_topics

    def get_selected_topic(self):
        """Return the device namespace currently selected in the motor_connect selector.

        Returns:
            str: The selected device namespace, or 'None' if unselected.
        """
        return self.selected_topic

    def select_topic(self, topic):
        """Select which motorized device the interface connects to.

        Args:
            topic (str): The device namespace to connect to, or 'None' to disconnect.
        """
        self.set_selected_topic(topic)

    def get_status_msg(self):
        """Return the latest motors status as a msg.

        Returns:
            MotorsStatus: The most recent MotorsStatus message, or None if no status
                has been received yet.
        """
        return self.status_msg

    def get_status_dict(self):
        """Return the latest motors status as a dictionary.

        Returns:
            dict: A dictionary representation of the most recent MotorsStatus message,
                or None if no status has been received yet.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def get_device_name(self):
        """Return the reporting device name from the latest status message.

        Returns:
            str: The device_name field of the latest MotorsStatus message, or None if
                no status has been received yet.
        """
        if self.status_msg is not None:
            return self.status_msg.device_name
        return None

    def get_motor_count(self):
        """Return the number of motors reported by the device.

        Returns:
            int: The length of the motors array in the latest MotorsStatus message,
                or 0 if no status has been received yet.
        """
        if self.status_msg is not None:
            return len(self.status_msg.motors)
        return 0

    def get_motor_names(self):
        """Return the list of motor names reported by the device.

        Returns:
            list: A list of motor_name strings in motor-index order, or an empty list
                if no status has been received yet.
        """
        names = []
        if self.status_msg is not None:
            for motor in self.status_msg.motors:
                names.append(motor.motor_name)
        return names

    def get_motor(self, name_or_index):
        """Return the MotorStatus for a single motor by name or index.

        Args:
            name_or_index (str or int): The motor_name string, or the integer index into
                the motors array.

        Returns:
            MotorStatus: The matching per-motor status message, or None if no status has
                been received yet or the motor was not found.
        """
        if self.status_msg is None:
            return None
        motors = self.status_msg.motors
        if isinstance(name_or_index, int):
            if 0 <= name_or_index < len(motors):
                return motors[name_or_index]
            return None
        for motor in motors:
            if motor.motor_name == name_or_index:
                return motor
        return None

    def get_motor_speed_ratio(self, name_or_index):
        """Return the commanded speed ratio for a single motor.

        Args:
            name_or_index (str or int): The motor_name string, or the integer index into
                the motors array.

        Returns:
            float: The motor_speed_ratio (0.0-1.0) for the motor, or None if the motor
                was not found or no status has been received yet.
        """
        motor = self.get_motor(name_or_index)
        if motor is not None:
            return motor.motor_speed_ratio
        return None

    def get_motor_position(self, name_or_index):
        """Return the current position for a single motor.

        Args:
            name_or_index (str or int): The motor_name string, or the integer index into
                the motors array.

        Returns:
            float: The motor_position in degrees for the motor, or None if the motor was
                not found or no status has been received yet.
        """
        motor = self.get_motor(name_or_index)
        if motor is not None:
            return motor.motor_position
        return None

    def get_motor_has_rotation(self, name_or_index):
        """Return whether a single motor can rotate its wheel about the steering axis.

        Args:
            name_or_index (str or int): The motor_name string, or the integer index into
                the motors array.

        Returns:
            bool: The has_rotation flag for the motor, or None if the motor was not
                found or no status has been received yet.
        """
        motor = self.get_motor(name_or_index)
        if motor is not None:
            return motor.has_rotation
        return None

    def get_motor_has_continuous_rotation(self, name_or_index):
        """Return whether a single motor's wheel rotation is unlimited.

        When True, rotation_min_limit and rotation_max_limit are not meaningful.

        Args:
            name_or_index (str or int): The motor_name string, or the integer index into
                the motors array.

        Returns:
            bool: The has_continuous_rotation flag for the motor, or None if the motor
                was not found or no status has been received yet.
        """
        motor = self.get_motor(name_or_index)
        if motor is not None:
            return motor.has_continuous_rotation
        return None

    def get_motor_rotation_limits(self, name_or_index):
        """Return the wheel rotation limits for a single motor.

        Meaningful only where the motor reports has_rotation True and
        has_continuous_rotation False.

        Args:
            name_or_index (str or int): The motor_name string, or the integer index into
                the motors array.

        Returns:
            list: A two-element [rotation_min_limit, rotation_max_limit] list in degrees,
                or None if the motor was not found or no status has been received yet.
        """
        motor = self.get_motor(name_or_index)
        if motor is not None:
            return [motor.rotation_min_limit, motor.rotation_max_limit]
        return None

    def get_motor_rotation_max_speed(self, name_or_index):
        """Return the max wheel rotation rate for a single motor.

        This is the rate that rotation_speed_ratio = 1.0 maps to. It is a separate
        axis from motor_max_speed, which governs how fast the wheel drives.

        Args:
            name_or_index (str or int): The motor_name string, or the integer index into
                the motors array.

        Returns:
            float: The rotation_max_speed in deg/sec for the motor, or None if the motor
                was not found or no status has been received yet.
        """
        motor = self.get_motor(name_or_index)
        if motor is not None:
            return motor.rotation_max_speed
        return None

    def get_motor_rotation_speed_ratio(self, name_or_index):
        """Return the commanded wheel rotation speed ratio for a single motor.

        Args:
            name_or_index (str or int): The motor_name string, or the integer index into
                the motors array.

        Returns:
            float: The rotation_speed_ratio (0.0-1.0) for the motor, or None if the motor
                was not found or no status has been received yet.
        """
        motor = self.get_motor(name_or_index)
        if motor is not None:
            return motor.rotation_speed_ratio
        return None

    def get_motor_rotation(self, name_or_index):
        """Return the current wheel rotation for a single motor.

        Args:
            name_or_index (str or int): The motor_name string, or the integer index into
                the motors array.

        Returns:
            float: The rotation in degrees for the motor, or None if the motor was not
                found or no status has been received yet.
        """
        motor = self.get_motor(name_or_index)
        if motor is not None:
            return motor.rotation
        return None

    def set_speed(self, motor, speed_ratio):
        """Publish a set_speed command to a motor.

        Applies only where the motor has adjustable speed; ignored otherwise by
        the device.

        Args:
            motor (str): Target motor_name ("motor_0", "motor_1", ...) or "all".
            speed_ratio (float): Desired speed as a ratio from 0.0 (off) to 1.0 (max).
        """
        msg = MotorCommand()
        msg.motor_name = str(motor)
        msg.speed_ratio = float(speed_ratio)
        msg.direction = 0
        msg.rotation = 0.0
        msg.rotation_speed_ratio = 0.0
        self.publishCommand('set_speed', msg)

    def set_direction(self, motor, direction):
        """Publish a set_direction command to a motor.

        Args:
            motor (str): Target motor_name ("motor_0", "motor_1", ...) or "all".
            direction (int): Intended direction, 1 = clockwise, -1 = counter-clockwise.
        """
        msg = MotorCommand()
        msg.motor_name = str(motor)
        msg.speed_ratio = 0.0
        msg.direction = 1 if direction >= 0 else -1
        msg.rotation = 0.0
        msg.rotation_speed_ratio = 0.0
        self.publishCommand('set_direction', msg)

    def go_direction(self, motor):
        """Publish a go_direction command to start a motor moving continuously.

        The motor moves in its currently set direction at its set speed until a
        stop_motor command is issued.

        Args:
            motor (str): Target motor_name ("motor_0", "motor_1", ...) or "all".
        """
        msg = MotorCommand()
        msg.motor_name = str(motor)
        msg.speed_ratio = 0.0
        msg.direction = 0
        msg.rotation = 0.0
        msg.rotation_speed_ratio = 0.0
        self.publishCommand('go_direction', msg)

    def stop_motor(self, motor):
        """Publish a stop_motor command to stop a motor.

        Companion to go_direction; motion is continuous-until-stop.

        Args:
            motor (str): Target motor_name ("motor_0", "motor_1", ...) or "all".
        """
        msg = MotorCommand()
        msg.motor_name = str(motor)
        msg.speed_ratio = 0.0
        msg.direction = 0
        msg.rotation = 0.0
        msg.rotation_speed_ratio = 0.0
        self.publishCommand('stop_motor', msg)

    def goto_rotation(self, motor, rotation):
        """Publish a goto_rotation command to drive a motor's wheel to a rotation angle.

        Applies only where the motor reports has_rotation True; ignored otherwise
        by the device. Where the motor reports has_continuous_rotation False, the
        device clamps the target to the motor's rotation_min_limit and
        rotation_max_limit.

        Args:
            motor (str): Target motor_name ("motor_0", "motor_1", ...) or "all".
            rotation (float): Target rotation about the steering axis, in degrees.
        """
        msg = MotorCommand()
        msg.motor_name = str(motor)
        msg.speed_ratio = 0.0
        msg.direction = 0
        msg.rotation = float(rotation)
        msg.rotation_speed_ratio = 0.0
        self.publishCommand('goto_rotation', msg)

    def set_rotation_speed(self, motor, rotation_speed_ratio):
        """Publish a set_rotation_speed command to a motor.

        Sets how fast the wheel steers, a separate axis from set_speed, which sets
        how fast the wheel drives. Applies only where the motor reports
        has_rotation True; ignored otherwise by the device.

        Args:
            motor (str): Target motor_name ("motor_0", "motor_1", ...) or "all".
            rotation_speed_ratio (float): Desired rotation speed as a ratio from 0.0
                (off) to 1.0 (max), scaled by the motor's rotation_max_speed.
        """
        msg = MotorCommand()
        msg.motor_name = str(motor)
        msg.speed_ratio = 0.0
        msg.direction = 0
        msg.rotation = 0.0
        msg.rotation_speed_ratio = float(rotation_speed_ratio)
        self.publishCommand('set_rotation_speed', msg)

    ###############################
    # Class Private Methods
    ###############################

    def publishCommand(self, pub_name, msg):
        if self.node_if is not None and self.connect_topic_pubs_dict is not None:
            self.node_if.publish_pub(pub_name, msg)

    # Discovery/connection timer. Overrides ConnectNodeIF._updaterCb only to
    # strip the dedicated '/motor_status' topic (instead of the base's
    # '/status') when recovering the device namespace from a discovered topic.
    # Everything else mirrors the base: find matching topics, auto-select,
    # subscribe, and drop the connection on staleness.
    def _updaterCb(self, timer):
        selected_topic = copy.deepcopy(self.selected_topic)
        last_available = copy.deepcopy(self.available_topics)

        topics = nepi_sdk.find_topics_by_msg(self.connect_status_msg, topics_list = self.active_topics, types_list = self.active_topic_types)
        available_topics = []
        for topic in topics:
            available_topics.append(topic.replace('/' + STATUS_TOPIC, ''))
        if available_topics != last_available:
            self.available_topics = available_topics

        if self.connected_topic is not None:
            if self.connected_topic not in self.available_topics:
                success = self.unsubscribe_topic()
        if selected_topic == 'None' and len(self.available_topics) > 0:
            self.selected_topic = self.available_topics[0]

        if self.selected_topic in self.available_topics and self.connected_topic != selected_topic:
            success = self.subscribe_topic(self.selected_topic)
        elif self.selected_topic not in self.available_topics:
            self.connected = False

        cur_time = nepi_utils.get_time()
        last_time = copy.deepcopy(self.last_status_time)
        if self.connected == True:
            if (cur_time - last_time) > CONNECTED_TIMEOUT:
                self.connecting = False
                self.connected = False
                self.connected_topic = ''
                self.status_msg = None

        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)

    def subscribe_topic(self, topic):
        success = False
        success = self.unsubscribe_topic()

        # Subscribers Config Dict ####################
        # Single subscriber to the selected device's dedicated 'motor_status'
        # topic carrying a nepi_interfaces/MotorsStatus message.
        self.connect_topic_subs_dict = {
            'status_sub': {
                'namespace': self.selected_topic,
                'topic': STATUS_TOPIC,
                'msg': MotorsStatus,
                'qsize': 10,
                'callback': self._statusCb
            }
        }

        # Publishers Config Dict ####################
        # One publisher per standard motor command topic, each carrying a
        # nepi_interfaces/MotorCommand message, on the selected device namespace.
        self.connect_topic_pubs_dict = {}
        for command_topic in COMMAND_TOPICS:
            self.connect_topic_pubs_dict[command_topic] = {
                'namespace': self.selected_topic,
                'topic': command_topic,
                'msg': MotorCommand,
                'qsize': 1,
            }

        if self.node_if is not None:
            self.node_if.register_subs(self.connect_topic_subs_dict)
            self.node_if.register_pubs(self.connect_topic_pubs_dict)
            self.connecting = True
            self.connected = False
            self.connected_topic = 'None'
            self.status_msg = None

        return success

    def unsubscribe_topic(self):
        success = False
        if self.connecting == True or self.connected == True:
            self.msg_if.pub_debug("unsubscribe_topic Called")

            if self.node_if is not None:
                if self.connect_topic_subs_dict is not None:
                    for sub_name in self.connect_topic_subs_dict.keys():
                        self.node_if.unregister_sub(sub_name)
            self.connect_topic_subs_dict = None

            if self.node_if is not None:
                if self.connect_topic_pubs_dict is not None:
                    for pub_name in self.connect_topic_pubs_dict.keys():
                        self.node_if.unregister_pub(pub_name)
            self.connect_topic_pubs_dict = None

            nepi_sdk.sleep(1)
            self.connecting = False
            self.connected = False
            self.connected_topic = 'None'
            self.status_msg = None
            success = True
        return success

    def _statusCb(self, status_msg):
        self.last_status_time = nepi_utils.get_time()
        if self.connected == False:
            self._announceConnected('Motors')
            self.connecting = False
            self.connected_topic = self.selected_topic
        self.connected = True
        self.status_msg = status_msg
        status_dict = self.get_status_dict()
        if self.statusCb is not None:
            self.statusCb(status_dict)
        if self.dataCB is not None:
            self.dataCB(status_dict)
