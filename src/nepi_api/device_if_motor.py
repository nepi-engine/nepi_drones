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


import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Header

from nepi_interfaces.msg import MotorStatus, MotorsStatus, MotorCommand

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


#########################################
# Motors Device IF Class
#########################################

# Producer-side interface for the standard NEPI multi-motor device, modeled
# structurally on device_if_rbx.py (RBXRobotIF). It owns a NodeClassIF that
# publishes the standard nepi_interfaces/MotorsStatus message on the dedicated
# 'motor_status' topic and subscribes to the six standard motor command topics
# (set_speed, set_direction, go_direction, stop_motor, goto_rotation,
# set_rotation_speed), each carrying a nepi_interfaces/MotorCommand message.
#
# goto_rotation and set_rotation_speed drive a rotatable (steering) wheel about
# its steering axis. They are additive: a device that supplies no rotation
# callbacks still registers both subscribers and simply ignores the commands,
# and reports has_rotation false in every MotorStatus.
#
# This is the wire contract the connect-side ConnectMotorsDeviceIF
# (connect_device_if_motor.py) discovers and drives: it finds devices by the
# MotorsStatus topic (stripping '/motor_status' to recover the device
# namespace) and publishes MotorCommand on the command topics. RBXRobotIF
# already publishes MotorsStatus via its 'motor_status_pub'; this class is the
# dedicated, driver-agnostic producer of that same contract.

STATUS_UPDATE_RATE_HZ = 2

# The dedicated topic every motorized NEPI device publishes MotorsStatus on.
STATUS_TOPIC = 'motor_status'

# The six standard motor command topics, each carrying a MotorCommand message.
COMMAND_TOPICS = ['set_speed', 'set_direction', 'go_direction', 'stop_motor',
                  'goto_rotation', 'set_rotation_speed']


class MotorsDeviceIF:
    # Define class variables
    ready = False
    status_msg = MotorsStatus()

    device_name = ''

    node_if = None

    status_msg_pub_interval = float(1)/float(STATUS_UPDATE_RATE_HZ)

    ### IF Initialization
    def __init__(self, device_info,
                 data_source_description = 'motor_controller',
                 data_ref_description = 'device',
                 getMotorsStatusFunction = None,
                 setSpeedFunction = None,
                 setDirectionFunction = None,
                 goDirectionFunction = None,
                 stopMotorFunction = None,
                 gotoRotationFunction = None,
                 setRotationSpeedFunction = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.namespace = nepi_sdk.create_namespace(self.node_namespace, 'motors')

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

        ##############################
        # Initialize Class Variables
        self.device_name = device_info["device_name"]
        self.path = device_info["path"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description

        self.getMotorsStatusFunction = getMotorsStatusFunction
        self.setSpeedFunction = setSpeedFunction
        self.setDirectionFunction = setDirectionFunction
        self.goDirectionFunction = goDirectionFunction
        self.stopMotorFunction = stopMotorFunction
        self.gotoRotationFunction = gotoRotationFunction
        self.setRotationSpeedFunction = setRotationSpeedFunction

        # Initialize status message
        self.status_msg.device_name = self.device_name

        ##################################################
        ### Node Class Setup
        self.msg_if.pub_info("Starting Node IF Initialization", log_name_list = self.log_name_list)

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Publishers Config Dict ####################
        # The dedicated 'motor_status' topic carrying MotorsStatus, discovered by
        # the connect-side ConnectMotorsDeviceIF.
        self.PUBS_DICT = {
            'motor_status_pub': {
                'namespace': self.namespace,
                'topic': STATUS_TOPIC,
                'msg': MotorsStatus,
                'qsize': 10,
                'latch': False
            }
        }

        # Subscribers Config Dict ####################
        # One subscriber per standard motor command topic, each carrying a
        # MotorCommand message, on this device namespace.
        self.SUBS_DICT = {
            'set_speed': {
                'namespace': self.namespace,
                'topic': 'set_speed',
                'msg': MotorCommand,
                'qsize': 20,
                'callback': self.setSpeedCb,
                'callback_args': ()
            },
            'set_direction': {
                'namespace': self.namespace,
                'topic': 'set_direction',
                'msg': MotorCommand,
                'qsize': 20,
                'callback': self.setDirectionCb,
                'callback_args': ()
            },
            'go_direction': {
                'namespace': self.namespace,
                'topic': 'go_direction',
                'msg': MotorCommand,
                'qsize': 20,
                'callback': self.goDirectionCb,
                'callback_args': ()
            },
            'stop_motor': {
                'namespace': self.namespace,
                'topic': 'stop_motor',
                'msg': MotorCommand,
                'qsize': 20,
                'callback': self.stopMotorCb,
                'callback_args': ()
            },
            'goto_rotation': {
                'namespace': self.namespace,
                'topic': 'goto_rotation',
                'msg': MotorCommand,
                'qsize': 20,
                'callback': self.gotoRotationCb,
                'callback_args': ()
            },
            'set_rotation_speed': {
                'namespace': self.namespace,
                'topic': 'set_rotation_speed',
                'msg': MotorCommand,
                'qsize': 20,
                'callback': self.setRotationSpeedCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = None,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )
        self.node_if.wait_for_ready()

        ##############################
        # Update vals from param server and publish first status
        self.initCb(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(self.status_msg_pub_interval, self.statusPublishCb)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete", log_name_list = self.log_name_list)
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

    def wait_for_ready(self, timeout = float('inf')):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for ready", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
        return self.ready

    def get_namespace(self):
        """Return the fully-resolved ROS namespace for this Motors device interface.

        Returns:
            str: The fully-qualified device namespace under which the motor_status
                topic and command topics are advertised.
        """
        return self.namespace

    def get_motors_status_msg(self):
        """Builds the standard MotorsStatus message for this device.

        Populates one MotorStatus per motor from the driver-supplied
        getMotorsStatusFunction, which returns an ordered list of per-motor
        dictionaries. Missing dictionary keys fall back to safe defaults
        (motor_enable True, motor_dir 1, speeds/position 0.0). The rotatable-wheel
        (steering) keys default to has_rotation False, has_continuous_rotation
        False, and 0.0 for every rotation limit, speed and angle, so a
        non-steering device that supplies none of them reports has_rotation
        False and is otherwise unaffected.

        Returns:
            MotorsStatus: Message published on the dedicated motor_status topic.
        """
        msg = MotorsStatus()
        msg.header = Header()
        msg.timestamp = nepi_utils.get_time()
        msg.device_name = self.device_name

        if self.getMotorsStatusFunction is not None:
            motors_list = self.getMotorsStatusFunction()
        else:
            motors_list = []
        if motors_list is None:
            motors_list = []

        motors = []
        for i in range(len(motors_list)):
            motor_dict = motors_list[i]
            motor = MotorStatus()
            motor.motor_name = motor_dict.get('motor_name', "motor_" + str(i))
            motor.motor_enable = motor_dict.get('motor_enable', True)
            motor.motor_dir = motor_dict.get('motor_dir', 1)
            motor.motor_max_speed = motor_dict.get('motor_max_speed', 0.0)
            motor.motor_speed_ratio = motor_dict.get('motor_speed_ratio', 0.0)
            motor.motor_speed = motor_dict.get('motor_speed', 0.0)
            motor.motor_position = motor_dict.get('motor_position', 0.0)
            motor.has_rotation = motor_dict.get('has_rotation', False)
            motor.has_continuous_rotation = motor_dict.get('has_continuous_rotation', False)
            motor.rotation_min_limit = motor_dict.get('rotation_min_limit', 0.0)
            motor.rotation_max_limit = motor_dict.get('rotation_max_limit', 0.0)
            motor.rotation_max_speed = motor_dict.get('rotation_max_speed', 0.0)
            motor.rotation_speed_ratio = motor_dict.get('rotation_speed_ratio', 0.0)
            motor.rotation = motor_dict.get('rotation', 0.0)
            motors.append(motor)
        msg.motors = motors

        return msg

    def publish_status(self):
        """Assembles and publishes the MotorsStatus message on the motor_status topic."""
        if self.node_if is None:
            return
        self.status_msg = self.get_motors_status_msg()
        if not nepi_sdk.is_shutdown():
            self.node_if.publish_pub('motor_status_pub', self.status_msg)


    #######################
    # Class Private Methods
    #######################

    ### Config callbacks
    def initCb(self, do_updates = False):
        if do_updates == True:
            pass
        self.publish_status()

    def resetCb(self, do_updates = True):
        self.msg_if.pub_warn("Resetting", log_name_list = self.log_name_list)
        self.initCb(do_updates = do_updates)

    def factoryResetCb(self, do_updates = True):
        self.msg_if.pub_warn("Factory Resetting", log_name_list = self.log_name_list)
        self.initCb(do_updates = do_updates)

    ### Status publish timer callback
    def statusPublishCb(self, timer):
        self.publish_status()

    ### Motor command callbacks
    def setSpeedCb(self, motor_msg):
        if self.setSpeedFunction is not None:
            self.setSpeedFunction(motor_msg.motor_name, motor_msg.speed_ratio)

    def setDirectionCb(self, motor_msg):
        if self.setDirectionFunction is not None:
            self.setDirectionFunction(motor_msg.motor_name, motor_msg.direction)

    def goDirectionCb(self, motor_msg):
        if self.goDirectionFunction is not None:
            self.goDirectionFunction(motor_msg.motor_name)

    def stopMotorCb(self, motor_msg):
        if self.stopMotorFunction is not None:
            self.stopMotorFunction(motor_msg.motor_name)

    def gotoRotationCb(self, motor_msg):
        if self.gotoRotationFunction is not None:
            self.gotoRotationFunction(motor_msg.motor_name, motor_msg.rotation)

    def setRotationSpeedCb(self, motor_msg):
        if self.setRotationSpeedFunction is not None:
            self.setRotationSpeedFunction(motor_msg.motor_name, motor_msg.rotation_speed_ratio)
