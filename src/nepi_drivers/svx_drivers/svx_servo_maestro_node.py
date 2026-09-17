#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

# SVX (servo) node for the Pololu Micro Maestro USB servo controller.
#
# Structure mirrors svx_servo_generic_node.py (the SVX category template, itself
# derived from the IQR PTX node); the serial send/receive machinery follows
# ptx_sidus_ss109_serial_node.py. The stub callbacks in the generic node are
# filled in here with the real Maestro protocol.
#
# One SVX device = one servo = one Maestro channel. A pan/tilt turret is two of
# these nodes (e.g. pan on channel 0, tilt on channel 1) coordinated by
# nepi_app_servo_auto. Multiple channel nodes can share one Maestro's single USB
# serial port safely: every serial transaction is wrapped in an advisory file
# lock (fcntl.flock) on the port, so two channel nodes never interleave bytes.
#
# The board is a plug-and-play USB device running Pololu factory firmware, so the
# whole hardware layer here is USB serial. The Maestro command set is documented
# at https://www.pololu.com/docs/0J40/5.e . This driver speaks the "Compact"
# protocol by default (works for a single controller regardless of its device
# number) and can speak the "Pololu" protocol (0xAA + device number) for
# daisy-chained boards, selected via the driver options.
#
# The SVX API speaks degrees. Converting a degree command to a servo pulse width
# (Maestro targets are in quarter-microseconds) is done here in the driver. The
# SVX IF handles the reverse-axis conversion, soft-limit clamping, and ratio math
# before it calls these callbacks, so the driver keeps a single pure linear
# degree <-> pulse-width map with no reverse logic of its own.

import copy
import threading

import serial

try:
    import fcntl  # Linux advisory locking so channel nodes can share one port
    HAVE_FCNTL = True
except Exception:
    HAVE_FCNTL = False

try:
    import usb.core  # Native USB control transfers, only for the firmware Min/Max
    HAVE_PYUSB = True  # push below -- separate channel from the pyserial Command Port.
except Exception:
    HAVE_PYUSB = False

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls

from nepi_api.device_if_svx import SVXActuatorIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'SVX_SERVO_MAESTRO' # Use in display menus
FILE_TYPE = 'NODE'


class SvxServoMaestroNode:

    #############################
    # Pololu Maestro serial command set (Compact-protocol command bytes). The
    # Pololu-protocol form of each command is the same byte with its MSB cleared
    # (byte & 0x7F), sent after 0xAA and the device number.
    CMD_SET_TARGET = 0x84
    CMD_SET_SPEED = 0x87
    CMD_SET_ACCEL = 0x89
    CMD_GET_POSITION = 0x90
    CMD_GET_MOVING = 0x93
    CMD_GET_ERRORS = 0xA1
    CMD_GO_HOME = 0xA2

    # Maestro speed units are (0.25 us) / (10 ms) == 25 us/sec per unit.
    US_PER_SPEED_UNIT_PER_SEC = 25.0
    # Maestro targets/positions are in quarter-microseconds.
    QUS_PER_US = 4

    SERIAL_TIMEOUT_SEC = 0.25
    MAX_POSITION_UPDATE_RATE = 5

    # Native USB control-transfer protocol, used only to push this channel's
    # pulse_min_us/pulse_max_us as the Maestro's own onboard Set-Target clamp
    # (PARAMETER_SERVO0_MIN/MAX) so it always matches this node's config instead
    # of depending on leftover state from manual bench configuration. This is a
    # separate wire protocol from the Compact-protocol serial bytes above: native
    # USB control transfers via pyusb, needing raw USB permission (root or a udev
    # rule for VID 0x1FFB) rather than the dialout access send_cmd() uses. Entirely
    # best-effort -- see driver_pushFirmwareLimits() below.
    USB_VENDOR_ID = 0x1FFB
    REQUEST_SET_PARAMETER = 0x82
    REQUEST_REINITIALIZE = 0x90
    # Channel 0's parameter numbers are confirmed against Pololu's Usc_protocol.cs.
    # The +9-per-channel stride for channel > 0 is INFERRED from that enum's layout
    # and not independently verified -- driver_pushFirmwareLimits() logs what it
    # reads back so a wrong offset is obvious rather than silent.
    PARAMETER_SERVO0_MIN = 32
    PARAMETER_SERVO0_MAX = 33
    CHANNEL_PARAM_STRIDE = 9

    #############################
    # Per-servo calibration.
    #
    # These describe THE SERVO, not the Maestro -- a pan servo and a tilt servo
    # on the same board routinely have different horns, travel, and pulse
    # endpoints. So they are node settings (see CAP_SETTINGS below), adjustable
    # live per device in the RUI and persisted per device, and the values here
    # are only the bootstrap defaults a servo starts from before anyone
    # calibrates it. The driver's discovery OPTIONS deliberately say nothing
    # about them.
    DEFAULT_PULSE_MIN_US = 500.0
    DEFAULT_PULSE_MAX_US = 2300.0
    DEFAULT_MIN_DEG = -90.0
    DEFAULT_MAX_DEG = 90.0
    DEFAULT_ACCEL_UNITS = 0

    # Outer bounds the settings will accept. The pulse range covers any hobby
    # servo worth driving; the Maestro's own acceleration parameter is a single
    # byte, hence 0-255.
    PULSE_US_MIN = 100.0
    PULSE_US_MAX = 3000.0
    DEG_MIN = -180.0
    DEG_MAX = 180.0
    ACCEL_UNITS_MIN = 0
    ACCEL_UNITS_MAX = 255

    # Smallest degree span a servo may be calibrated to. Below this the
    # degree <-> pulse-width map stops meaning anything, and deg2us() would be
    # dividing by ~zero.
    MIN_DEG_SPAN = 1.0

    # Board-agnostic factory soft/hard limits (degrees) and the pulse-width
    # endpoints they map to. Deep-copied per instance in __init__ so the settings
    # write to this node's own limits rather than the class-level dict.
    FACTORY_LIMITS_DICT = dict()
    FACTORY_LIMITS_DICT['min_hardstop_deg'] = DEFAULT_MIN_DEG
    FACTORY_LIMITS_DICT['max_hardstop_deg'] = DEFAULT_MAX_DEG
    FACTORY_LIMITS_DICT['min_softstop_deg'] = DEFAULT_MIN_DEG
    FACTORY_LIMITS_DICT['max_softstop_deg'] = DEFAULT_MAX_DEG

    limits_dict = copy.deepcopy(FACTORY_LIMITS_DICT)

    # Pulse-width mapping (microseconds). pulse_min_us <-> min_hardstop_deg and
    # pulse_max_us <-> max_hardstop_deg.
    pulse_min_us = DEFAULT_PULSE_MIN_US
    pulse_max_us = DEFAULT_PULSE_MAX_US
    accel_units = DEFAULT_ACCEL_UNITS  # 0 == no acceleration limit

    CAP_SETTINGS = {
        'pulse_min_us' : {"type":"Float","name":"pulse_min_us",
                          "options":[str(PULSE_US_MIN), str(PULSE_US_MAX)]},
        'pulse_max_us' : {"type":"Float","name":"pulse_max_us",
                          "options":[str(PULSE_US_MIN), str(PULSE_US_MAX)]},
        'min_deg' :      {"type":"Float","name":"min_deg",
                          "options":[str(DEG_MIN), str(DEG_MAX)]},
        'max_deg' :      {"type":"Float","name":"max_deg",
                          "options":[str(DEG_MIN), str(DEG_MAX)]},
        'accel_units' :  {"type":"Int","name":"accel_units",
                          "options":[str(ACCEL_UNITS_MIN), str(ACCEL_UNITS_MAX)]}
    }

    FACTORY_SETTINGS_OVERRIDES = dict()

    init_settings_dict = dict()
    settings_dict = dict()

    settingFunctions = {
        'pulse_min_us' : {'get':'getPulseMinUs', 'set': 'setPulseMinUs'},
        'pulse_max_us' : {'get':'getPulseMaxUs', 'set': 'setPulseMaxUs'},
        'min_deg' :      {'get':'getMinDeg',     'set': 'setMinDeg'},
        'max_deg' :      {'get':'getMaxDeg',     'set': 'setMaxDeg'},
        'accel_units' :  {'get':'getAccelUnits', 'set': 'setAccelUnits'}
    }

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")

    # Initialize some parameters
    serial_num = "Unknown"
    hw_version = "Unknown"
    sw_version = "Unknown"
    svx_if = None

    # Board link
    serial_port = None
    baud_int = 9600
    channel = 0
    device_number = 12          # Maestro factory-default device number
    use_pololu_protocol = False # Compact protocol by default

    # Open loop from the servo's point of view, but the Maestro reports the pulse
    # width it is currently transmitting, so getPosition can reflect a slew.
    position_deg = 0.0
    goal_deg = 0.0
    home_pos_deg = 0.0
    speed_ratio = 0.5
    speed_max_dps = 20.0

    connected = False

    drv_dict = dict()


    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

    def __init__(self):
        ####  NODE Initialization ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        # In-process guard around each serial transaction (paired with the
        # cross-process fcntl lock inside send_cmd).
        self.serial_lock = threading.Lock()

        ##############################
        # Initialize Class Variables

        # Get required drv driver dict info
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
        try:
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.port_str = self.drv_dict['DEVICE_DICT']['device_path']

            self.channel = int(self.drv_dict['DEVICE_DICT'].get('channel', 0))
            self.baud_int = int(self.drv_dict['DEVICE_DICT'].get('baud_str', '9600'))
            self.device_number = int(self.drv_dict['DEVICE_DICT'].get('device_number', 12))
            self.serial_num = str(self.drv_dict['DEVICE_DICT'].get('serial_number', 'Unknown'))
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        # Per-instance copy of the calibration limits. The settings below write
        # into these, and the class-level dict must not be the thing they write
        # to.
        self.FACTORY_LIMITS_DICT = copy.deepcopy(SvxServoMaestroNode.FACTORY_LIMITS_DICT)
        self.limits_dict = copy.deepcopy(self.FACTORY_LIMITS_DICT)

        # The only discovery OPTION this node reads. Everything describing the
        # servo itself (pulse endpoints, degree range, acceleration) is a node
        # setting instead -- see CAP_SETTINGS. The board-level options stay with
        # the board.
        try:
            options = self.drv_dict.get('DISCOVERY_DICT', {}).get('OPTIONS', {})
            protocol = options.get('protocol', {}).get('value', 'Compact')
            self.use_pololu_protocol = (str(protocol).lower() == 'pololu')
        except Exception as e:
            self.msg_if.pub_warn("Failed to parse driver OPTIONS, using defaults: " + str(e))

        ################################################
        self.msg_if.pub_info("Connecting to Maestro on port " + str(self.port_str) +
                             " channel " + str(self.channel))
        self.connected = self.connect()
        if self.connected == False:
            self.msg_if.pub_info("Shutting down node, Unable to connect to Maestro servo controller")
            nepi_sdk.signal_shutdown("Unable to connect to servo device")
            return

        self.msg_if.pub_info("... Connected!")

        # Initialize settings
        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()

        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version

        #Factory Control Values
        self.FACTORY_CONTROLS = {
            'reverse_enabled' : False,
            'speed_ratio' : 0.5
        }

        # Push the initial acceleration + speed to the board before we come up.
        self.driver_setAcceleration(self.accel_units)
        self.driver_setSpeedRatio(self.speed_ratio)
        self.driver_pushFirmwareLimits()

        # Launch the SVX interface -- it subscribes/advertises all the servo
        # control topics, publishes status, and serves capabilities. We pass the
        # Figure 1 callbacks; the interface derives each has_* capability flag
        # from which callbacks are non-None. Spin/continuous mode is owned entirely
        # by the IF (continuous_enabled param + set_continuous_mode topic); the
        # driver declares nothing about it.
        self.msg_if.pub_info("Launching NEPI SVX interface...")
        self.svx_if = SVXActuatorIF(device_info = self.device_info_dict,
                                    getSettingsFunction = self.getSettingsFunction,
                                    setSettingFunction = self.setSettingFunction,
                                    factoryControls = self.FACTORY_CONTROLS,
                                    factoryLimits = self.FACTORY_LIMITS_DICT,
                                    stopMovingCb = self.stopMoving,
                                    gotoPositionCb = self.gotoPosition,
                                    getPositionCb = self.getPosition,
                                    setSoftLimitsCb = self.setSoftLimits,
                                    getSoftLimitsCb = self.getSoftLimits,
                                    setSpeedRatioCb = self.setSpeedRatio,
                                    getSpeedRatioCb = self.getSpeedRatio,
                                    setSpeedMaxCb = self.setSpeedMax,
                                    getSpeedMaxCb = self.getSpeedMax,
                                    goHomeCb = self.goHome,
                                    setHomePositionCb = self.setHomePosition,
                                    setHomePositionHereCb = self.setHomePositionHere,
                                    deviceResetCb = self.resetDevice
                                    )
        self.msg_if.pub_info(" ... SVX interface running")

        # Persisted min_deg/max_deg may have been applied by SettingsIF while svx_if was
        # still None (mid-construction), so setDegRange could not push them to the IF and
        # the IF kept the factory range. Re-push the current calibrated range now that the
        # IF exists, so its clamp range (and the reported travel) matches the settings.
        self.svx_if.setHardstopLimits(self.limits_dict['min_hardstop_deg'],
                                      self.limits_dict['max_hardstop_deg'])

        # Poll the Maestro for the pulse width it is currently transmitting so
        # the reported position tracks a speed/accel-limited slew.
        update_interval = float(1.0) / self.MAX_POSITION_UPDATE_RATE
        nepi_sdk.start_timer_process(update_interval, self.updatePositionHandler, oneshot = True)

        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever
        nepi_sdk.spin()


    def updatePositionHandler(self, timer):
        stime = nepi_utils.get_time()
        pos_deg = self.driver_getPosition()
        if pos_deg is not None:
            self.position_deg = pos_deg
        # Surface any accumulated board errors to the log/UI.
        self.driver_checkErrors()
        gtime = nepi_utils.get_time() - stime
        next_delay = max(0.05, float(1.0) / self.MAX_POSITION_UPDATE_RATE - gtime)
        nepi_sdk.start_timer_process(next_delay, self.updatePositionHandler, oneshot = True)


    #**********************
    # Device setting functions

    def initSettingsDict(self):
        init_settings_dict = dict()
        for setting_name in self.CAP_SETTINGS.keys():
            cap_setting = self.CAP_SETTINGS[setting_name]
            setting_type = cap_setting['type']
            setting_dict = dict()
            setting_dict['type'] = setting_type
            # The retired cap-settings form carried an Int/Float control's min
            # and max in an 'options' pair. The controls contract calls that
            # 'bounds'.
            if 'options' in cap_setting.keys():
                try:
                    if setting_type == 'Int':
                        setting_dict['bounds'] = [int(cap_setting['options'][0]),int(cap_setting['options'][1])]
                    elif setting_type == 'Float':
                        setting_dict['bounds'] = [float(cap_setting['options'][0]),float(cap_setting['options'][1])]
                    else:
                        setting_dict['options'] = [str(option) for option in cap_setting['options']]
                except Exception as e:
                    self.msg_if.pub_warn("Invalid bounds for setting: " + setting_name + " : " + str(e))

            default = self.readSettingValue(setting_name)
            if setting_name in self.FACTORY_SETTINGS_OVERRIDES.keys():
                default = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
            if default is None:
                continue
            setting_dict['default'] = default
            init_settings_dict[setting_name] = setting_dict

        self.init_settings_dict = init_settings_dict
        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        settings_dict_values = nepi_controls.get_controls_values_dict(settings_dict)
        self.msg_if.pub_info("Initialized Settings: " + str(settings_dict_values))
        return settings_dict

    def refreshSettingsDict(self):
        # These are per-servo calibration values held by this node, so this only
        # reads the current values back. Bounds do not move.
        settings_dict = copy.deepcopy(self.settings_dict)
        for setting_name in settings_dict.keys():
            value = self.readSettingValue(setting_name)
            if value is None:
                continue
            settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, value)
        return settings_dict

    def readSettingValue(self, setting_name):
        # Returns the node's current value for one setting, typed for the
        # control, or None if it could not be read.
        if setting_name not in self.settingFunctions.keys():
            return None
        setting_type = self.CAP_SETTINGS[setting_name]['type'] if setting_name in self.CAP_SETTINGS.keys() else 'String'
        try:
            get_function = globals()[self.settingFunctions[setting_name]['get']]
            val = get_function(self)
        except Exception as e:
            self.msg_if.pub_warn("Failed to read setting " + setting_name + " : " + str(e))
            return None
        if val is None:
            return None
        try:
            if setting_type == 'Int':
                val = int(float(val))
            elif setting_type == 'Float':
                val = float(val)
            elif setting_type == 'Toggle':
                val = (val == True)
            else:
                val = str(val)
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert setting " + setting_name + " : " + str(e))
            return None
        return val

    def getSettingsFunction(self):
        return self.settings_dict

    def setSettingFunction(self, setting_name, setting_value):
        setting_str = setting_name + ":" + str(setting_value)
        success = False
        msg = ""
        if setting_name not in self.settings_dict.keys():
            msg = (self.node_name + " Setting name " + setting_str + " is not supported")
            return False, msg, self.settings_dict
        if setting_name not in self.settingFunctions.keys():
            msg = "No set function registered for setting: " + str(setting_name)
            return False, msg, self.settings_dict

        function_str_name = self.settingFunctions[setting_name]['set']
        set_function = getattr(self, function_str_name, None)
        if set_function is None:
            msg = "Missing set function: " + function_str_name
            self.msg_if.pub_warn(msg)
        else:
            # Every setter returns [success, msg].
            [success, msg] = set_function(setting_value)
            if success:
                msg = ( self.node_name + " UPDATED SETTINGS " + setting_str)

        self.settings_dict = self.refreshSettingsDict()
        return success, msg, self.settings_dict


    ##############
    ### Settings Functions
    #
    # Per-servo calibration -- what makes one servo on a Maestro different from
    # the next one on the same board. Kept here rather than in the driver's
    # discovery OPTIONS because these describe the servo, not the controller.
    #
    # Convention: readSettingValue() above resolves getters through globals(), so each
    # getter needs the 'global' line in front of it. Setters resolve through
    # getattr(self, ...) and do not. Every setter returns [success, msg].
    #
    # The pulse endpoints and the degree range are set independently by the UI
    # but only make sense as a pair, so each setter routes through a shared
    # range function that validates the resulting pair as a whole.

    global getPulseMinUs
    def getPulseMinUs(self):
        return self.pulse_min_us

    def setPulseMinUs(self, val):
        return self.setPulseRange(new_min = val)

    global getPulseMaxUs
    def getPulseMaxUs(self):
        return self.pulse_max_us

    def setPulseMaxUs(self, val):
        return self.setPulseRange(new_max = val)

    def setPulseRange(self, new_min = None, new_max = None):
        try:
            pulse_min = self.pulse_min_us if new_min is None else float(new_min)
            pulse_max = self.pulse_max_us if new_max is None else float(new_max)
        except Exception as e:
            return False, "Pulse width is not a number: " + str(e)

        for val in [pulse_min, pulse_max]:
            if val < self.PULSE_US_MIN or val > self.PULSE_US_MAX:
                return False, ("Pulse width " + str(val) + "us is outside the supported range " +
                               str(self.PULSE_US_MIN) + "-" + str(self.PULSE_US_MAX) + "us")
        if pulse_max <= pulse_min:
            return False, ("Rejecting pulse range " + str(pulse_min) + "-" + str(pulse_max) +
                           "us: max must be greater than min")

        self.pulse_min_us = pulse_min
        self.pulse_max_us = pulse_max

        # Two things downstream depend on the pulse span. The board's onboard
        # clamp is set from the endpoints, and ratio2speedUnits() converts
        # through usPerDeg() -- so the same 0.0-1.0 speed dial means a different
        # number of Maestro speed units once the span changes.
        self.driver_pushFirmwareLimits()
        self.driver_setSpeedRatio(self.speed_ratio)
        return True, ("Pulse range set to " + str(pulse_min) + "-" + str(pulse_max) + "us")

    global getMinDeg
    def getMinDeg(self):
        return self.limits_dict['min_hardstop_deg']

    def setMinDeg(self, val):
        return self.setDegRange(new_min = val)

    global getMaxDeg
    def getMaxDeg(self):
        return self.limits_dict['max_hardstop_deg']

    def setMaxDeg(self, val):
        return self.setDegRange(new_max = val)

    def setDegRange(self, new_min = None, new_max = None):
        # The servo's mechanical travel. Open loop, so nothing can measure this
        # -- the operator finds the hardstops and declares them here, and can
        # come back to the factory defaults with a settings reset.
        try:
            min_deg = self.limits_dict['min_hardstop_deg'] if new_min is None else float(new_min)
            max_deg = self.limits_dict['max_hardstop_deg'] if new_max is None else float(new_max)
        except Exception as e:
            return False, "Degree limit is not a number: " + str(e)

        for val in [min_deg, max_deg]:
            if val < self.DEG_MIN or val > self.DEG_MAX:
                return False, ("Degree limit " + str(val) + " is outside the supported range " +
                               str(self.DEG_MIN) + " to " + str(self.DEG_MAX))
        if (max_deg - min_deg) < self.MIN_DEG_SPAN:
            return False, ("Rejecting degree range " + str(min_deg) + " to " + str(max_deg) +
                           ": span must be at least " + str(self.MIN_DEG_SPAN) + " deg")

        # Hand the new travel to the SVX IF first, so its soft-limit validation
        # matches, and let it pull the softstops in if they now sit outside.
        # Going through the IF BEFORE updating our own map matters: the IF
        # enforces its own minimum span (MIN_LIMIT_ANGLE), which is stricter than
        # MIN_DEG_SPAN above, and a range it rejects must not leave this node
        # converting degrees over a range the IF has never heard of.
        #
        # Guarded because SettingsIF can apply a persisted value before the IF
        # finishes coming up -- in that case the constructor reads
        # FACTORY_LIMITS_DICT and picks the new range up anyway.
        if self.svx_if is not None:
            if self.svx_if.setHardstopLimits(min_deg, max_deg) == False:
                return False, ("SVX interface rejected degree range " + str(min_deg) +
                               " to " + str(max_deg))

        for limits in [self.FACTORY_LIMITS_DICT, self.limits_dict]:
            limits['min_hardstop_deg'] = min_deg
            limits['max_hardstop_deg'] = max_deg
        # Keep the factory softstops in step, so a device reset restores a
        # softstop range that still fits inside the declared travel.
        self.FACTORY_LIMITS_DICT['min_softstop_deg'] = min_deg
        self.FACTORY_LIMITS_DICT['max_softstop_deg'] = max_deg

        # Speed units are per degree, so the dial has to be re-pushed here too.
        self.driver_setSpeedRatio(self.speed_ratio)
        return True, ("Degree range set to " + str(min_deg) + " to " + str(max_deg))

    global getAccelUnits
    def getAccelUnits(self):
        return self.accel_units

    def setAccelUnits(self, val):
        try:
            units = int(float(val))
        except Exception as e:
            return False, "Acceleration is not a number: " + str(e)
        if units < self.ACCEL_UNITS_MIN or units > self.ACCEL_UNITS_MAX:
            return False, ("Acceleration " + str(units) + " is outside the supported range " +
                           str(self.ACCEL_UNITS_MIN) + "-" + str(self.ACCEL_UNITS_MAX))
        self.accel_units = units
        self.driver_setAcceleration(units)
        return True, ("Acceleration set to " + str(units) + " units")


    #######################
    ### SVX IF Callbacks
    #
    # The SVX interface clamps any commanded position against the soft limits and
    # applies the reverse-axis conversion before calling gotoPosition, so the
    # driver receives a safe degree value in its own (non-reversed) frame.

    def stopMoving(self):
        # Positional servo: "stop" == hold where it is now. Read the pulse width
        # the Maestro is currently transmitting and re-command it as the target,
        # which halts any in-progress speed/accel-limited slew.
        pos_deg = self.driver_getPosition()
        if pos_deg is not None:
            self.driver_moveToPosition(pos_deg)

    def gotoPosition(self, position_deg):
        # position_deg is already soft-limit clamped and reverse-adjusted by the IF.
        self.goal_deg = position_deg
        self.driver_moveToPosition(position_deg)

    def getPosition(self):
        # Reports the pulse width the Maestro is transmitting (refreshed by the
        # poll timer); falls back to the last commanded value if a read fails.
        return self.position_deg

    def setSoftLimits(self, min_deg, max_deg):
        self.limits_dict['min_softstop_deg'] = min_deg
        self.limits_dict['max_softstop_deg'] = max_deg

    def getSoftLimits(self):
        return self.limits_dict['min_softstop_deg'], self.limits_dict['max_softstop_deg']

    def setSpeedRatio(self, ratio):
        self.speed_ratio = ratio
        self.driver_setSpeedRatio(ratio)

    def getSpeedRatio(self):
        return self.speed_ratio

    def setSpeedMax(self, speed_max_dps):
        self.speed_max_dps = speed_max_dps
        # Re-apply so the ratio still means the same fraction of the new max.
        self.driver_setSpeedRatio(self.speed_ratio)

    def getSpeedMax(self):
        return self.speed_max_dps

    def goHome(self):
        self.gotoPosition(self.home_pos_deg)

    def setHomePosition(self, position_deg):
        self.home_pos_deg = position_deg

    def setHomePositionHere(self):
        self.home_pos_deg = self.position_deg

    def resetDevice(self):
        # Send the board to its firmware home positions, clear errors, and
        # restore this node's factory config.
        self.driver_goHome()
        self.driver_checkErrors(clear_only = True)
        self.speed_ratio = 0.5
        self.speed_max_dps = 20.0
        self.limits_dict = copy.deepcopy(self.FACTORY_LIMITS_DICT)
        self.driver_setAcceleration(self.accel_units)
        self.driver_setSpeedRatio(self.speed_ratio)


    #######################
    ### Unit Conversions (degrees <-> microseconds <-> quarter-microseconds)
    #
    # Pure linear map; reverse-axis handling lives in the SVX IF, not here.

    def deg2us(self, deg):
        # The degree span is a live setting now, so guard the division rather
        # than trust the value. setDegRange() enforces MIN_DEG_SPAN, which makes
        # this unreachable -- but an unreachable guard is cheaper than a
        # ZeroDivisionError on every move command if that ever stops being true.
        min_deg = self.FACTORY_LIMITS_DICT['min_hardstop_deg']
        max_deg = self.FACTORY_LIMITS_DICT['max_hardstop_deg']
        span_deg = (max_deg - min_deg)
        if span_deg == 0:
            return self.pulse_min_us
        frac = (deg - min_deg) / span_deg
        us = self.pulse_min_us + frac * (self.pulse_max_us - self.pulse_min_us)
        # Never let a conversion escape the servo's pulse-width endpoints.
        return max(self.pulse_min_us, min(self.pulse_max_us, us))

    def us2deg(self, us):
        min_deg = self.FACTORY_LIMITS_DICT['min_hardstop_deg']
        max_deg = self.FACTORY_LIMITS_DICT['max_hardstop_deg']
        span_us = (self.pulse_max_us - self.pulse_min_us)
        if span_us == 0:
            return min_deg
        frac = (us - self.pulse_min_us) / span_us
        return min_deg + frac * (max_deg - min_deg)

    def deg2target(self, deg):
        # Maestro target is in quarter-microseconds (rounded to an int).
        return int(round(self.deg2us(deg) * self.QUS_PER_US))

    def target2deg(self, target_qus):
        return self.us2deg(float(target_qus) / self.QUS_PER_US)

    def usPerDeg(self):
        min_deg = self.FACTORY_LIMITS_DICT['min_hardstop_deg']
        max_deg = self.FACTORY_LIMITS_DICT['max_hardstop_deg']
        span_deg = (max_deg - min_deg)
        if span_deg == 0:
            return 0.0
        return (self.pulse_max_us - self.pulse_min_us) / span_deg

    def ratio2speedUnits(self, ratio):
        # Maestro speed unit == 25 us/sec. Convert deg/sec to us/sec to units.
        dps = ratio * self.speed_max_dps
        us_per_sec = dps * self.usPerDeg()
        units = int(round(us_per_sec / self.US_PER_SPEED_UNIT_PER_SEC))
        # A Maestro speed of 0 means "unlimited", which is not what ratio 0.0
        # should do, so clamp to the slowest non-zero speed.
        if units < 1:
            units = 1
        return units


    #######################
    ### Driver Interface Functions (Maestro serial transactions)

    def driver_moveToPosition(self, deg):
        target = self.deg2target(deg)
        low = target & 0x7F
        high = (target >> 7) & 0x7F
        payload = bytes([self.channel & 0x7F, low, high])
        return self.send_cmd(self.CMD_SET_TARGET, payload)

    def driver_setSpeedRatio(self, ratio):
        units = self.ratio2speedUnits(ratio)
        low = units & 0x7F
        high = (units >> 7) & 0x7F
        payload = bytes([self.channel & 0x7F, low, high])
        return self.send_cmd(self.CMD_SET_SPEED, payload)

    def driver_setAcceleration(self, units):
        units = max(0, int(units))
        low = units & 0x7F
        high = (units >> 7) & 0x7F
        payload = bytes([self.channel & 0x7F, low, high])
        return self.send_cmd(self.CMD_SET_ACCEL, payload)

    def driver_getPosition(self):
        payload = bytes([self.channel & 0x7F])
        resp = self.send_cmd(self.CMD_GET_POSITION, payload, read_len = 2)
        if resp is None or len(resp) < 2:
            return None
        target_qus = resp[0] | (resp[1] << 8)
        if target_qus == 0:
            # 0 means the Maestro is not driving the channel (no pulse) -- no
            # meaningful position to report.
            return None
        return self.target2deg(target_qus)

    def driver_getMovingState(self):
        resp = self.send_cmd(self.CMD_GET_MOVING, b'', read_len = 1)
        if resp is None or len(resp) < 1:
            return None
        return resp[0] == 0x01

    def driver_goHome(self):
        return self.send_cmd(self.CMD_GO_HOME, b'')

    def driver_checkErrors(self, clear_only = False):
        # Get Errors returns two little-endian bytes of error flags and clears
        # them on the board. Surface any set flags for the operator.
        resp = self.send_cmd(self.CMD_GET_ERRORS, b'', read_len = 2)
        if resp is None or len(resp) < 2:
            return
        if clear_only:
            return
        error_bits = resp[0] | (resp[1] << 8)
        if error_bits != 0:
            self.msg_if.pub_warn("Maestro error flags: 0x{:04X}".format(error_bits))


    def driver_pushFirmwareLimits(self):
        # Best-effort: push pulse_min_us/pulse_max_us as the board's own onboard
        # Set-Target clamp. This is a *different* wire path (native USB control
        # transfers) than the Compact-protocol serial commands above, and needs
        # raw USB permission (root or a udev rule for VID 0x1FFB) separate from the
        # dialout-group access send_cmd() uses -- if that's not available, log a
        # warning and continue on the serial path rather than fail node startup.
        if not HAVE_PYUSB:
            self.msg_if.pub_warn("pyusb not available; skipping firmware Min/Max push "
                                 "(board keeps whatever onboard limits it already has)")
            return
        if self.channel != 0:
            # Channel 0's parameter numbers are confirmed; the +9-per-channel
            # stride for higher channels is inferred and unverified (see
            # CHANNEL_PARAM_STRIDE above). These are writes to the board's
            # PERSISTENT settings followed by a reinitialize, so a wrong offset
            # would corrupt board configuration rather than just fail. Skip until
            # the stride is confirmed against real hardware. Nothing is lost:
            # SVXActuatorIF clamps every commanded position to the soft limits
            # before it ever reaches this driver, so the onboard clamp is
            # belt-and-braces, and the board simply keeps its existing limits.
            self.msg_if.pub_info("Skipping firmware Min/Max push on channel %d "
                                 "(per-channel parameter stride not yet hardware-verified); "
                                 "board keeps its existing onboard limits" % self.channel)
            return
        try:
            dev = None
            for d in usb.core.find(idVendor=self.USB_VENDOR_ID, find_all=True):
                if str(getattr(d, 'serial_number', '')) == str(self.serial_num):
                    dev = d
                    break
            if dev is None:
                self.msg_if.pub_warn("Could not find Maestro on native USB (serial=" +
                                     str(self.serial_num) + ") to push firmware Min/Max")
                return

            param_min = self.PARAMETER_SERVO0_MIN + self.channel * self.CHANNEL_PARAM_STRIDE
            param_max = self.PARAMETER_SERVO0_MAX + self.channel * self.CHANNEL_PARAM_STRIDE
            min_raw = max(0, min(255, round(self.pulse_min_us / 16)))
            max_raw = max(0, min(255, round(self.pulse_max_us / 16)))

            dev.ctrl_transfer(0x40, self.REQUEST_SET_PARAMETER, min_raw, (1 << 8) | param_min)
            dev.ctrl_transfer(0x40, self.REQUEST_SET_PARAMETER, max_raw, (1 << 8) | param_max)
            dev.ctrl_transfer(0x40, self.REQUEST_REINITIALIZE, 0, 0)
            nepi_sdk.sleep(0.05)
            self.msg_if.pub_info("Pushed firmware Min/Max for channel %d: %dus/%dus" %
                                 (self.channel, min_raw * 16, max_raw * 16))
        except Exception as e:
            self.msg_if.pub_warn("Failed to push firmware Min/Max (continuing without it): " + str(e))


    #######################
    ### Connection

    def connect(self):
        success = False
        try:
            self.msg_if.pub_info("Opening serial port " + str(self.port_str) +
                                 " with baudrate: " + str(self.baud_int))
            self.serial_port = serial.Serial(self.port_str, self.baud_int,
                                             timeout = self.SERIAL_TIMEOUT_SEC)
            self.msg_if.pub_info("Serial port opened")
        except Exception as e:
            self.msg_if.pub_warn("Failed to open serial port at: " + str(self.port_str) +
                                 " (" + str(e) + ")")
            return False

        # Handshake: read (and clear) the error register. A Maestro answers Get
        # Errors with exactly two bytes; anything else means this is not the
        # Command Port (or the Pololu device number is wrong).
        resp = self.send_cmd(self.CMD_GET_ERRORS, b'', read_len = 2)
        if resp is not None and len(resp) == 2:
            success = True
            self.hw_version = "Pololu Maestro"
            self.sw_version = "Factory Firmware"
        else:
            self.msg_if.pub_warn("No valid Maestro response on " + str(self.port_str) +
                                 "; is this the Maestro Command Port?")
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        return success


    def send_cmd(self, cmd_byte, payload = b'', read_len = 0):
        # Frame a command in the configured protocol, send it, and (optionally)
        # read a fixed-length response. Wrapped in an in-process lock AND an
        # advisory file lock on the serial device so that multiple channel nodes
        # sharing one Maestro port never interleave their bytes.
        if self.serial_port is None:
            return None

        if self.use_pololu_protocol:
            frame = bytes([0xAA, self.device_number & 0x7F, cmd_byte & 0x7F]) + payload
        else:
            frame = bytes([cmd_byte]) + payload

        response = None
        with self.serial_lock:
            locked = False
            try:
                if HAVE_FCNTL:
                    fcntl.flock(self.serial_port.fileno(), fcntl.LOCK_EX)
                    locked = True
                self.serial_port.reset_input_buffer()
                self.serial_port.write(frame)
                self.serial_port.flush()
                if read_len > 0:
                    data = self.serial_port.read(read_len)
                    if data is not None and len(data) == read_len:
                        response = bytearray(data)
            except Exception as e:
                self.msg_if.pub_warn("Serial transaction failed: " + str(e))
                response = None
            finally:
                if locked:
                    try:
                        fcntl.flock(self.serial_port.fileno(), fcntl.LOCK_UN)
                    except Exception:
                        pass
        return response


    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        if self.serial_port is not None:
            try:
                # Leave the channel un-driven (target 0 == no pulse) so the servo
                # is not held under power after the node exits.
                self.send_cmd(self.CMD_SET_TARGET, bytes([self.channel & 0x7F, 0, 0]))
            except Exception:
                pass
            try:
                self.serial_port.close()
            except Exception:
                pass


if __name__ == '__main__':
    node = SvxServoMaestroNode()
