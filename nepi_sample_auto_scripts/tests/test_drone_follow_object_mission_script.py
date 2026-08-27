#!/usr/bin/env python3
#
# Mock-stub unit test for drone_follow_object_mission_script.py.
#
# WHY A MOCK-STUB TEST (not a real rospy/nepi_sdk import test):
# This machine has no built catkin workspace (no devel/setup.bash). rospy IS
# genuinely importable here (real /opt/ros/noetic package), but
# `nepi_sdk.nepi_ros` fails to import (ModuleNotFoundError:
# rospy_message_converter, a dependency normally provided by the NEPI
# device's catkin install), `nepi_interfaces.msg` / `nepi_interfaces.srv`
# have no generated message/service classes at all (produced by catkin at
# build time, nothing built here), and `geographic_msgs` (used for the
# GeoPoint home-location message) is not installed either. So this test
# stubs: nepi_sdk.nepi_ros, nepi_sdk.nepi_settings, nepi_api.messages_if,
# nepi_interfaces.msg / .srv, and geographic_msgs.msg -- and uses the REAL
# rospy module only for the bits it still needs to stand in for (Publisher/
# Subscriber/spin/logwarn/is_shutdown), monkeypatched per-test so no actual
# ROS master contact is attempted.
#
# The point of exercising the real __init__ path (rather than hand-building
# a partial instance) is to catch exactly the class of bug this session's
# API-drift fixes were about: a renamed field, topic, class, or call
# signature slipping through unnoticed. The stub message classes below carry
# EXACTLY the current field lists read directly from the real
# src/nepi_interfaces/msg/*.msg and src/nepi_interfaces/srv/*.srv files this
# session -- if the script referenced a field name that no longer exists,
# these stubs (real classes with only the real fields, not a permissive
# Mock/dict) would raise AttributeError here exactly as the real message
# would.
#
# IMPORTANT DESIGN NOTE: the stub modules are installed into sys.modules
# exactly ONCE (in setUpClass), before the script module is imported via
# importlib. Python binds `from nepi_sdk import nepi_settings` / `from
# nepi_interfaces.msg import DeviceRBXInfo, ...` etc. into the *script's own
# module namespace* at import time -- so the script keeps referencing the
# very same stub module/class objects for its whole lifetime regardless of
# what sys.modules holds afterward. Per-test configuration (queueing a
# capabilities response, auto-firing subscriber callbacks with synthetic
# status/info/settings messages) therefore reads those same objects back off
# the imported script module (self.module.nepi_ros, self.module.rospy, ...)
# rather than creating fresh stub instances the script would never see.
#
# time.sleep is patched to a no-op for the duration of each test: the
# script's inlined RBX helpers (wait_for_rbx_status_ready/busy,
# rbx_initialize's polling loops) call the REAL stdlib time.sleep() directly
# (not nepi_ros.sleep()), and several of those loops have multi-second
# timeouts (CMD_ACTION_TIMEOUT_SEC=20 polled every 0.1s) that would otherwise
# make this test take tens of real seconds for no benefit -- the loop
# *logic* (counters, timeout arithmetic) still runs exactly as written, only
# the wall-clock delay is removed.
#
# Run directly with:
#   python3 -m unittest tests.test_drone_follow_object_mission_script -v
# (from the nepi_sample_auto_scripts/ directory)

import importlib.util
import math
import os
import sys
import types
import unittest
from unittest import mock

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_PATH = os.path.join(SCRIPT_DIR, "drone_follow_object_mission_script.py")


# ---------------------------------------------------------------------
# rospy stand-ins (real rospy IS importable here, but we don't want the
# script touching a live ROS master, so we monkeypatch just the entry
# points it calls directly: Publisher/Subscriber/spin/logwarn/is_shutdown).
# ---------------------------------------------------------------------
class _FakePublisher:
    def __init__(self, topic, msg_class, queue_size=1, latch=False):
        self.topic = topic
        self.msg_class = msg_class
        self.queue_size = queue_size
        self.latch = latch
        self.published = []

    def publish(self, *args, **kwargs):
        if args:
            self.published.append(args[0])
        else:
            self.published.append(kwargs)


class _FakeSubscriber:
    def __init__(self, topic, msg_class, callback, queue_size=1, callback_args=()):
        self.topic = topic
        self.msg_class = msg_class
        self.callback = callback
        self.queue_size = queue_size


class _FakeServiceProxy:
    """Records the service topic/type it was constructed against and, when
    called, returns whatever response `_next_response` (set by the test)
    holds -- or raises whatever `_next_exception` holds. Class-level state
    is intentional: the script only ever sees ONE object built from
    nepi_ros.connect_service (bound at rbx_initialize() call time).
    """

    _next_response = None
    _next_exception = None

    def __init__(self, topic, srv_class):
        self.topic = topic
        self.srv_class = srv_class

    def __call__(self, *args, **kwargs):
        if _FakeServiceProxy._next_exception is not None:
            raise _FakeServiceProxy._next_exception
        return _FakeServiceProxy._next_response


class _FakeMsgIF:
    def __init__(self, log_name=None):
        self.log_name = log_name
        self.calls = []

    def pub_info(self, msg, throttle_s=None, log_name_list=[]):
        self.calls.append(("info", msg))

    def pub_warn(self, msg, throttle_s=None, log_name_list=[]):
        self.calls.append(("warn", msg))

    def pub_debug(self, msg, throttle_s=None, log_name_list=[]):
        self.calls.append(("debug", msg))

    def pub_error(self, msg, throttle_s=None, log_name_list=[]):
        self.calls.append(("error", msg))


# ---------------------------------------------------------------
# nepi_interfaces.msg / .srv stubs -- field lists copied verbatim from the
# real src/nepi_interfaces/msg/*.msg and src/nepi_interfaces/srv/*.srv files
# read this session.
# ---------------------------------------------------------------
def _slot_class(name, fields):
    """Build a simple attribute-holding message stub class with exactly the
    given field names (no permissive extra attributes), mirroring a real
    generated ROS message class closely enough to catch renamed-field bugs.
    """

    def __init__(self, **kwargs):
        for field in fields:
            setattr(self, field, kwargs.get(field))

    return type(name, (), {"__slots__": tuple(fields), "__init__": __init__})


_DeviceRBXInfo = _slot_class(
    "DeviceRBXInfo",
    [
        "connected", "device_name", "serial_num", "hw_version", "sw_version",
        "standby", "state", "mode", "error_bounds", "cmd_timeout",
        "image_source", "image_status_overlay",
        "home_lat", "home_long", "home_alt", "home_depth",
    ],
)

_DeviceRBXStatus = _slot_class(
    "DeviceRBXStatus",
    [
        "device_name", "device_path", "device_node_name",
        "serial_num", "hw_version", "sw_version",
        "data_source_description", "data_ref_description",
        "settings_topic", "navpose_topic", "save_data_topic",
        "data_products", "data_product_topics", "data_product_image_topics",
        "process_current", "process_last", "ready", "battery",
        "errors_current", "errors_prev", "cmd_success",
        "manual_control_mode_ready", "autonomous_control_mode_ready",
        "current_motor_control_settings", "last_cmd_string",
        "last_error_message", "navpose_frame_transform",
    ],
)

_GotoErrors = _slot_class(
    "GotoErrors", ["x_m", "y_m", "z_m", "heading_deg", "roll_deg", "pitch_deg", "yaw_deg"]
)
_ErrorBounds = _slot_class(
    "ErrorBounds", ["max_distance_error_m", "max_rotation_error_deg", "min_stabilize_time_s"]
)
_GotoPose = _slot_class("GotoPose", ["roll_deg", "pitch_deg", "yaw_deg"])
_GotoPosition = _slot_class("GotoPosition", ["x_meters", "y_meters", "z_meters", "yaw_deg"])
_GotoLocation = _slot_class("GotoLocation", ["lat", "long", "altitude_meters", "yaw_deg"])
_AxisControls = _slot_class("AxisControls", ["x", "y", "z", "roll", "pitch", "yaw"])
_MotorControl = _slot_class("MotorControl", ["motor_ind", "speed_ratio"])
_Setting = _slot_class("Setting", ["type_str", "name_str", "value_str"])
_Settings = _slot_class("Settings", ["settings"])
_SettingsStatus = _slot_class(
    "SettingsStatus",
    ["node_name", "settings_topic", "settings_count", "setting_caps_list",
     "settings_list", "has_cap_updates"],
)
_Target = _slot_class(
    "Target",
    [
        "timestamp", "name", "id", "uid", "confidence",
        "xmin_pixel", "xmax_pixel", "ymin_pixel", "ymax_pixel",
        "width_pixels", "height_pixels", "area_ratio", "area_pixels",
        "vel_pixels", "width_meters", "height_meters", "depth_meters",
        "area_meters", "volume_meters", "vel_xyz_mps", "center_xyz_meters",
        "range_m", "azimuth_deg", "elevation_deg",
        "color_black", "color_white", "color_red", "color_blue",
        "color_yellow", "color_cyan", "color_magenta", "color_green",
        "contour_moments", "shape_triangle", "shape_rectangle",
        "shape_quadrilateral", "shape_pentagon", "shape_hexagon",
        "shape_circle",
    ],
)
_Targets = _slot_class(
    "Targets",
    [
        "timestamp", "process_name", "process_namespace", "process_type",
        "process_description", "source_topic", "source_type",
        "source_timestamp", "source_nav_pose", "has_2d_data", "has_3d_data",
        "has_range_data", "has_bearing_data", "has_navpose_data",
        "has_color_data", "has_countour_data", "has_shape_data", "targets",
    ],
)

_RBXCapabilitiesQueryResponse = _slot_class(
    "RBXCapabilitiesQueryResponse",
    [
        "device_name", "device_path", "device_node_name", "control_support",
        "has_battery_feedback", "has_manual_controls", "has_autonomous_controls",
        "has_set_home", "has_go_home", "has_go_stop", "has_goto_pose",
        "has_goto_position", "has_goto_location", "state_options",
        "mode_options", "setup_action_options", "go_action_options",
        "data_products",
    ],
)


class _RBXCapabilitiesQuery:
    """Service *type* placeholder -- the script only ever passes this class
    to nepi_ros.connect_service(topic, RBXCapabilitiesQuery); it never
    constructs a request directly.
    """

    pass


_GeoPoint = _slot_class("GeoPoint", ["latitude", "longitude", "altitude"])


def _install_stub_modules():
    """Inject stub nepi_sdk.nepi_ros / nepi_sdk.nepi_settings /
    nepi_api.messages_if / nepi_interfaces.msg / nepi_interfaces.srv /
    geographic_msgs.msg modules into sys.modules and return the previous
    sys.modules entries so the caller can restore them afterward. Called
    exactly once, in setUpClass. rospy itself is NOT stubbed here (it is
    genuinely importable) -- its live-graph-touching entry points
    (Publisher/Subscriber/spin/is_shutdown/logwarn) are monkeypatched
    per-instance-build in _build_instance() instead.
    """
    nepi_sdk_pkg = types.ModuleType("nepi_sdk")

    # nepi_sdk.nepi_ros stub -- signatures match the current nepi_ros.py:
    # init_node(name, disable_signals=False), get_node_name(),
    # get_base_namespace(), wait_for_node(node_name, timeout=60,
    # log_name_list=[]), wait_for_topic(topic_name, timeout=60,
    # log_name_list=[], topics_list=None, types_list=None),
    # wait_for_service(service_name, ...), connect_service(namespace, msg,
    # log_name_list=[]), create_publisher(namespace, msg, queue_size=10,
    # latch=False, log_name_list=[]), create_subscriber(namespace, msg,
    # callback, queue_size=10, callback_args=(), log_name_list=[]),
    # sleep(sleep_sec, sleep_steps=None), is_shutdown().
    nepi_ros_stub = types.ModuleType("nepi_sdk.nepi_ros")
    nepi_ros_stub.init_node = lambda name, disable_signals=False: None
    nepi_ros_stub.get_node_name = lambda: "drone_follow_object_mission"
    nepi_ros_stub.get_base_namespace = lambda: "/nepi/device1/"
    nepi_ros_stub.wait_for_node = lambda node_name, timeout=60, log_name_list=[]: (
        "/nepi/device1/" + node_name
    )

    def _wait_for_topic(topic_name, timeout=60, log_name_list=[], topics_list=None, types_list=None):
        # Real nepi_ros.wait_for_topic() polls the live ROS graph for a
        # topic whose name *contains* topic_name and returns the fully
        # resolved topic string. Simulate that resolution deterministically.
        if topic_name.startswith("/"):
            return topic_name
        return "/nepi/device1/" + topic_name.lstrip("/")

    nepi_ros_stub.wait_for_topic = _wait_for_topic
    nepi_ros_stub.wait_for_service = lambda service_name, timeout=60, log_name_list=[]: service_name
    nepi_ros_stub.connect_service = lambda namespace, msg, log_name_list=[]: _FakeServiceProxy(namespace, msg)
    nepi_ros_stub.create_publisher = lambda namespace, msg, queue_size=10, latch=False, log_name_list=[]: (
        _FakePublisher(namespace, msg, queue_size, latch)
    )

    def _create_subscriber(namespace, msg, callback, queue_size=10, callback_args=(), log_name_list=[]):
        return _FakeSubscriber(namespace, msg, callback, queue_size, callback_args)

    nepi_ros_stub.create_subscriber = _create_subscriber
    nepi_ros_stub.sleep = lambda sleep_sec, sleep_steps=None: None
    nepi_ros_stub.is_shutdown = lambda: False
    nepi_sdk_pkg.nepi_ros = nepi_ros_stub

    # nepi_sdk.nepi_settings stub -- only the two functions the script uses,
    # matching the real current implementations exactly (see
    # src/nepi_engine/nepi_sdk/src/nepi_sdk/nepi_settings.py):
    # parse_setting_msgs_list(settings_msg) reads settings_msg.settings_list
    # (entries with name_str/type_str/value_str); create_msg_from_setting
    # builds a Setting() from a dict with 'type'/'name'/'value' keys.
    nepi_settings_stub = types.ModuleType("nepi_sdk.nepi_settings")

    def _parse_setting_msgs_list(settings_msg):
        settings = dict()
        for entry in settings_msg.settings_list:
            settings[entry.name_str] = {
                "name": entry.name_str,
                "type": entry.type_str,
                "value": entry.value_str,
            }
        return settings

    def _create_msg_from_setting(setting):
        setting_msg = _Setting()
        setting_msg.type_str = setting["type"]
        setting_msg.name_str = setting["name"]
        setting_msg.value_str = setting["value"]
        return setting_msg

    nepi_settings_stub.parse_setting_msgs_list = _parse_setting_msgs_list
    nepi_settings_stub.create_msg_from_setting = _create_msg_from_setting
    nepi_sdk_pkg.nepi_settings = nepi_settings_stub

    # nepi_api / nepi_api.messages_if.MsgIF stub.
    nepi_api_pkg = types.ModuleType("nepi_api")
    messages_if_stub = types.ModuleType("nepi_api.messages_if")
    messages_if_stub.MsgIF = _FakeMsgIF
    nepi_api_pkg.messages_if = messages_if_stub

    nepi_interfaces_pkg = types.ModuleType("nepi_interfaces")
    msg_stub = types.ModuleType("nepi_interfaces.msg")
    srv_stub = types.ModuleType("nepi_interfaces.srv")
    msg_stub.DeviceRBXInfo = _DeviceRBXInfo
    msg_stub.DeviceRBXStatus = _DeviceRBXStatus
    msg_stub.AxisControls = _AxisControls
    msg_stub.ErrorBounds = _ErrorBounds
    msg_stub.GotoErrors = _GotoErrors
    msg_stub.MotorControl = _MotorControl
    msg_stub.GotoPose = _GotoPose
    msg_stub.GotoPosition = _GotoPosition
    msg_stub.GotoLocation = _GotoLocation
    msg_stub.Setting = _Setting
    msg_stub.Settings = _Settings
    msg_stub.SettingsStatus = _SettingsStatus
    msg_stub.Target = _Target
    msg_stub.Targets = _Targets
    srv_stub.RBXCapabilitiesQuery = _RBXCapabilitiesQuery
    srv_stub.RBXCapabilitiesQueryResponse = _RBXCapabilitiesQueryResponse
    nepi_interfaces_pkg.msg = msg_stub
    nepi_interfaces_pkg.srv = srv_stub

    geographic_msgs_pkg = types.ModuleType("geographic_msgs")
    geographic_msgs_msg_stub = types.ModuleType("geographic_msgs.msg")
    geographic_msgs_msg_stub.GeoPoint = _GeoPoint
    geographic_msgs_pkg.msg = geographic_msgs_msg_stub

    stub_modules = {
        "nepi_sdk": nepi_sdk_pkg,
        "nepi_sdk.nepi_ros": nepi_ros_stub,
        "nepi_sdk.nepi_settings": nepi_settings_stub,
        "nepi_api": nepi_api_pkg,
        "nepi_api.messages_if": messages_if_stub,
        "nepi_interfaces": nepi_interfaces_pkg,
        "nepi_interfaces.msg": msg_stub,
        "nepi_interfaces.srv": srv_stub,
        "geographic_msgs": geographic_msgs_pkg,
        "geographic_msgs.msg": geographic_msgs_msg_stub,
    }

    previous = {}
    for name, mod in stub_modules.items():
        previous[name] = sys.modules.get(name)
        sys.modules[name] = mod
    return previous


def _restore_modules(previous):
    for name, mod in previous.items():
        if mod is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = mod


class TestDroneFollowObjectMissionScript(unittest.TestCase):
    """Imports drone_follow_object_mission_script.py against stub modules for
    the modules unavailable in this environment (nepi_sdk.nepi_ros,
    nepi_sdk.nepi_settings, nepi_api.messages_if, nepi_interfaces.msg/.srv,
    geographic_msgs.msg), drives the real __init__ path (with rospy's
    Publisher/Subscriber/spin/is_shutdown monkeypatched so nothing touches a
    live ROS master), and exercises the RBX helper methods and
    move_to_object_callback to confirm no broken attribute/field references
    remain post-API-drift-fix.

    NOTE on the KNOWN GAP: the real script blocks forever at
    nepi_ros.wait_for_topic(AI_TARGETING_TOPIC) because no app_ai_targeting
    app exists in this workspace (documented in the script's module
    docstring). This test's wait_for_topic stub never blocks (it
    deterministically resolves any name), so this test exercises the
    script's actual Python logic (RBX init/capabilities parsing, settings
    update, fake-gps/home-location setup, goto-position math, callback field
    access) rather than reproducing that missing-app hang, which is a
    dependency gap in the *environment*, not a bug in this script's code.
    """

    @classmethod
    def setUpClass(cls):
        previous = _install_stub_modules()
        try:
            spec = importlib.util.spec_from_file_location(
                "drone_follow_object_mission_script_under_test", SCRIPT_PATH
            )
            cls.module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = cls.module
            spec.loader.exec_module(cls.module)
        finally:
            _restore_modules(previous)

        # SAME objects the script bound at its own import time (see module
        # docstring's "IMPORTANT DESIGN NOTE").
        cls.nepi_ros_stub = cls.module.nepi_ros
        # Wrapped in staticmethod() so accessing it via `self.real_create_subscriber`
        # returns the plain function itself rather than Python's descriptor
        # protocol binding `self` (the test instance) as an implicit first
        # positional argument (a real bug this test hit: a bare function
        # stored as a class attribute becomes a bound method when read off
        # an instance, shifting every subsequent positional arg by one).
        cls.real_create_subscriber = staticmethod(cls.module.nepi_ros.create_subscriber)

    def setUp(self):
        # Reset shared class-level stub state before every test.
        _FakeServiceProxy._next_response = None
        _FakeServiceProxy._next_exception = None
        self.nepi_ros_stub.create_subscriber = self.real_create_subscriber

        # Patch the REAL rospy entry points the script calls directly, and
        # the REAL stdlib time.sleep (see module docstring). Both restored
        # in tearDown via addCleanup.
        self._rospy_patchers = [
            mock.patch.object(self.module.rospy, "Publisher", _FakePublisher),
            mock.patch.object(self.module.rospy, "Subscriber", _FakeSubscriber),
            mock.patch.object(self.module.rospy, "spin", lambda: None),
            mock.patch.object(self.module.rospy, "is_shutdown", lambda: False),
            mock.patch.object(self.module.rospy, "logwarn", lambda *a, **k: None),
            mock.patch("time.sleep", lambda *a, **k: None),
        ]
        for patcher in self._rospy_patchers:
            patcher.start()
            self.addCleanup(patcher.stop)

    # ------------------------------------------------------------
    # 1) Clean import + no accidental regression to removed/renamed API.
    # ------------------------------------------------------------
    def test_module_imports_cleanly(self):
        self.assertTrue(hasattr(self.module, "drone_follow_object_mission"))
        self.assertEqual(self.module.RBX_ROBOT_NAME, "ardupilot")
        self.assertEqual(self.module.TARGET_TO_FOLLOW, "chair")

    def test_source_has_no_removed_or_stale_api(self):
        with open(SCRIPT_PATH) as f:
            src = f.read()
        self.assertNotIn("from nepi_sdk import nepi_msg", src)
        self.assertNotIn("nepi_msg.createMsgPublishers", src)
        self.assertNotIn("nepi_msg.publishMsgInfo", src)
        self.assertNotIn("from nepi_ros_interfaces", src)
        self.assertNotIn("import nepi_rbx", src)
        # NOTE: "RBXInfo"/"RBXStatus" as bare substrings legitimately appear
        # in the module docstring's changelog note (documenting the OLD ->
        # NEW rename, matching the documentation style of the other
        # rewritten scripts) -- so check the actual import statement uses
        # the current class names, not the substrings' absence.
        self.assertIn("from nepi_interfaces.msg import DeviceRBXInfo, DeviceRBXStatus", src)
        self.assertNotIn("parse_settings_msg_data", src)
        # NOTE: "rbx/enable_fake_gps" as a bare substring legitimately
        # appears in the module docstring's changelog note (documenting the
        # OLD topic layout) -- check the actual current-API publisher
        # construction instead.
        self.assertIn('FAKE_GPS_NAMESPACE = os.path.join(self.base_namespace, "app_fake_gps") + "/"', src)
        self.assertNotIn("set_cmd_timeout\"", src)
        self.assertIn("MsgIF", src)
        self.assertIn("DeviceRBXInfo", src)
        self.assertIn("DeviceRBXStatus", src)
        self.assertIn("from nepi_interfaces.msg import", src)
        self.assertIn("from nepi_interfaces.srv import", src)
        self.assertIn("set_goto_timeout", src)
        self.assertIn("app_fake_gps/", src)

    # ------------------------------------------------------------
    # Helper: build a fully-initialized instance. Wires create_subscriber to
    # auto-fire callbacks with synthetic current-API messages so every
    # `while self.x is None` wait in rbx_initialize()/__init__ resolves
    # immediately (no real ROS graph, no thread involved).
    # ------------------------------------------------------------
    def _build_instance(self, setup_actions=("LAUNCH",), go_actions=(), states=("STANDBY", "ARM"), modes=("GUIDED", "RTL", "LOITER")):
        caps_response = _RBXCapabilitiesQueryResponse(
            device_name="ardupilot",
            device_path="",
            device_node_name="",
            control_support=_AxisControls(x=True, y=True, z=True, roll=False, pitch=False, yaw=True),
            has_battery_feedback=True,
            has_manual_controls=False,
            has_autonomous_controls=True,
            has_set_home=True,
            has_go_home=True,
            has_go_stop=True,
            has_goto_pose=True,
            has_goto_position=True,
            has_goto_location=True,
            state_options=list(states),
            mode_options=list(modes),
            setup_action_options=list(setup_actions),
            go_action_options=list(go_actions),
            data_products=[],
        )
        _FakeServiceProxy._next_response = caps_response

        settings_msg = _SettingsStatus(
            node_name="ardupilot_rbx",
            settings_topic="rbx/settings",
            settings_count=1,
            setting_caps_list=[],
            settings_list=[_Setting(type_str="Float", name_str="takeoff_height_m", value_str="10.0")],
            has_cap_updates=False,
        )
        info_msg = _DeviceRBXInfo(
            connected=True, device_name="ardupilot", serial_num="", hw_version="", sw_version="",
            standby=False, state=0, mode=0,
            error_bounds=_ErrorBounds(max_distance_error_m=2.0, max_rotation_error_deg=2.0, min_stabilize_time_s=1.0),
            cmd_timeout=20.0, image_source="", image_status_overlay=False,
            home_lat=47.65, home_long=-122.31, home_alt=0.0, home_depth=0.0,
        )
        status_msg = _DeviceRBXStatus(
            device_name="ardupilot", device_path="", device_node_name="",
            serial_num="", hw_version="", sw_version="",
            data_source_description="", data_ref_description="",
            settings_topic="", navpose_topic="", save_data_topic="",
            data_products=[], data_product_topics=[], data_product_image_topics=[],
            process_current="None", process_last="None", ready=True, battery=1.0,
            errors_current=_GotoErrors(x_m=0.0, y_m=0.0, z_m=0.0, heading_deg=0.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0),
            errors_prev=_GotoErrors(x_m=0.0, y_m=0.0, z_m=0.0, heading_deg=0.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0),
            cmd_success=True, manual_control_mode_ready=False, autonomous_control_mode_ready=True,
            current_motor_control_settings=[], last_cmd_string="", last_error_message="",
            navpose_frame_transform=None,
        )

        def _auto_firing_create_subscriber(namespace, msg, callback, queue_size=10, callback_args=(), log_name_list=[]):
            sub = self.real_create_subscriber(namespace, msg, callback, queue_size=queue_size, callback_args=callback_args)
            if msg is self.module.SettingsStatus:
                callback(settings_msg)
            elif msg is self.module.DeviceRBXInfo:
                callback(info_msg)
            elif msg is self.module.DeviceRBXStatus:
                callback(status_msg)
            return sub

        self.nepi_ros_stub.create_subscriber = _auto_firing_create_subscriber
        try:
            instance = self.module.drone_follow_object_mission()
        finally:
            self.nepi_ros_stub.create_subscriber = self.real_create_subscriber
        return instance

    # ------------------------------------------------------------
    # 2) Full __init__ path runs against the stubbed current API with no
    #    AttributeError/TypeError, exercising rbx_initialize, the settings
    #    override publish, fake-gps enable, set-home, and pre_mission_actions
    #    (the LAUNCH setup action) in one pass.
    # ------------------------------------------------------------
    def test_init_runs_against_stubbed_current_api(self):
        instance = self._build_instance()

        # msg_if constructed via current MsgIF(log_name=...) API.
        self.assertEqual(instance.msg_if.log_name, "drone_follow_object_mission")
        self.assertTrue(any(level == "info" for level, _ in instance.msg_if.calls))

        # Capabilities parsed from the (stubbed) RBXCapabilitiesQuery response.
        self.assertIn("LAUNCH", instance.rbx_cap_setup_actions)
        self.assertIn("RTL", instance.rbx_cap_modes)

        # Settings override: create_msg_from_setting -> Setting(type_str/
        # name_str/value_str) published to the settings/update_setting topic.
        published_settings = instance.rbx_setting_update_pub.published
        self.assertEqual(len(published_settings), 1)
        self.assertEqual(published_settings[0].name_str, "takeoff_height_m")
        self.assertEqual(published_settings[0].value_str, "10.0")

        # Fake GPS / set-home are both deliberately OFF against a SITL target
        # (ENABLE_FAKE_GPS = False, SET_HOME = False -- see the module's own
        # 2026-08-12 root-cause comments: a SITL has its own GPS, and either
        # of these actively breaks EKF convergence/home-altitude against it).
        # The publisher topic must still be built correctly even though it's
        # never fired this run.
        self.assertEqual(instance.fake_gps_enable_pub.topic, "/nepi/device1/app_fake_gps/enable")
        self.assertEqual(instance.fake_gps_enable_pub.published, [])
        self.assertEqual(instance.rbx_set_home_pub.published, [])

        # goto_timeout topic renamed from set_cmd_timeout, publisher exists.
        self.assertTrue(instance.rbx_set_cmd_timeout_pub.topic.endswith("set_goto_timeout"))

        # Snapshot trigger publisher constructed via base_namespace.
        self.assertEqual(instance.snapshot_trigger_pub.topic, "/nepi/device1/snapshot_trigger")

        # pre_mission_actions() ran the LAUNCH setup action against the
        # stubbed status (ready=True, cmd_success=True) without raising.
        self.assertIn(0, [i for i, a in enumerate(instance.rbx_cap_setup_actions) if a == "LAUNCH"])
        self.assertTrue(any(isinstance(p, int) for p in instance.rbx_setup_action_pub.published))

        # Image topic set from the (stubbed, missing-app-substitute) AI
        # targeting image topic resolution.
        self.assertEqual(
            instance.rbx_set_image_topic_pub.published[-1],
            "/nepi/device1/app_ai_targeting/targeting_image",
        )

    # ------------------------------------------------------------
    # 3) setup_rbx_action / set_rbx_state / set_rbx_mode helpers use the
    #    real current Int32-index-by-name protocol and real DeviceRBXInfo/
    #    DeviceRBXStatus field names (state, mode, ready, cmd_success).
    # ------------------------------------------------------------
    def test_set_rbx_mode_publishes_index_and_checks_info_mode_field(self):
        instance = self._build_instance()
        instance.rbx_info.mode = 1  # simulate device having already switched to RTL (index 1)
        success = instance.set_rbx_mode("RTL", timeout_sec=1)
        self.assertTrue(success)
        self.assertEqual(instance.rbx_set_mode_pub.published[-1], 1)

    def test_set_rbx_mode_returns_false_for_unknown_mode(self):
        instance = self._build_instance()
        success = instance.set_rbx_mode("NOT_A_REAL_MODE", timeout_sec=1)
        self.assertFalse(success)
        self.assertTrue(any(level == "warn" for level, _ in instance.msg_if.calls))

    def test_goto_rbx_position_publishes_real_gotoposition_fields(self):
        instance = self._build_instance()
        success = instance.goto_rbx_position([1.0, 2.0, -3.0, 45.0], timeout_sec=1)
        self.assertTrue(success)  # rbx_status.cmd_success stubbed True
        goto_msg = instance.rbx_goto_position_pub.published[-1]
        self.assertEqual(goto_msg.x_meters, 1.0)
        self.assertEqual(goto_msg.y_meters, 2.0)
        self.assertEqual(goto_msg.z_meters, -3.0)
        self.assertEqual(goto_msg.yaw_deg, 45.0)

    # ------------------------------------------------------------
    # 4) move_to_object_callback: real Target/Targets field names
    #    (target_name, range_m, azimuth_deg, elevation_deg) drive the
    #    setpoint-position-body math and goto_rbx_position call.
    # ------------------------------------------------------------
    def test_move_to_object_callback_drives_goto_on_matching_target(self):
        instance = self._build_instance()
        target = _Target(name="chair", range_m=2.0, azimuth_deg=0.0, elevation_deg=0.0)
        targets_msg = _Targets(targets=[target])

        instance.move_to_object_callback(targets_msg)

        goto_msg = instance.rbx_goto_position_pub.published[-1]
        # azimuth/elevation 0 -> straight ahead, offset 0.1m subtracted from range.
        self.assertAlmostEqual(goto_msg.x_meters, 1.9, places=5)
        self.assertAlmostEqual(goto_msg.y_meters, 0.0, places=5)
        # IGNORE_YAW_CONTROL is True in this script's USER SETTINGS -> -999 sentinel.
        self.assertEqual(goto_msg.yaw_deg, -999)

    def test_move_to_object_callback_converts_sensor_frame_to_driver_frame(self):
        # Regression test for the 2026-08-26 sign-inversion bug: the
        # az=0/el=0 case above can't distinguish right-vs-left or
        # down-vs-up since sin(0) == 0 on both axes. Use nonzero angles so
        # a reintroduced sign flip actually fails this test.
        #
        # ai_targeting_controller_ardupilot.py's sensor convention: X
        # forward, Y RIGHT, Z DOWN; azimuth+ = target to the right,
        # elevation+ = target above the drone.
        #
        # device_if_rbx.py's setpoint_position_local_body() driver
        # convention (its own docstring): X forward, Y LEFT, Z UP.
        #
        # A target 10m out, 30 deg to the right (azimuth=+30) and 20 deg
        # above (elevation=+20) must therefore produce a NEGATIVE
        # y_meters (right, in a Y-is-left frame) and a POSITIVE z_meters
        # (above, in a Z-is-up frame).
        instance = self._build_instance()
        target = _Target(name="chair", range_m=10.0, azimuth_deg=30.0, elevation_deg=20.0)
        targets_msg = _Targets(targets=[target])

        instance.move_to_object_callback(targets_msg)

        goto_msg = instance.rbx_goto_position_pub.published[-1]
        setpoint_range_m = 10.0 - self.module.TARGET_OFFSET_GOAL_M
        expected_x = setpoint_range_m * math.cos(math.radians(30.0))
        expected_y = -setpoint_range_m * math.sin(math.radians(30.0))
        expected_z = setpoint_range_m * math.sin(math.radians(20.0))
        self.assertAlmostEqual(goto_msg.x_meters, expected_x, places=5)
        self.assertAlmostEqual(goto_msg.y_meters, expected_y, places=5)
        self.assertAlmostEqual(goto_msg.z_meters, expected_z, places=5)
        self.assertLess(goto_msg.y_meters, 0.0, "target to the right must be negative y (driver's y+ is left)")
        self.assertGreater(goto_msg.z_meters, 0.0, "target above must be positive z (driver's z+ is up)")

    def test_move_to_object_callback_ignores_non_matching_target_and_invalid_range(self):
        instance = self._build_instance()
        other_target = _Target(name="person", range_m=2.0, azimuth_deg=0.0, elevation_deg=0.0)
        invalid_range_target = _Target(name="chair", range_m=-999, azimuth_deg=0.0, elevation_deg=0.0)
        targets_msg = _Targets(targets=[other_target, invalid_range_target])

        before = list(instance.rbx_goto_position_pub.published)
        instance.move_to_object_callback(targets_msg)
        # Neither target should have triggered a goto publish.
        self.assertEqual(instance.rbx_goto_position_pub.published, before)

    # ------------------------------------------------------------
    # 5) post_mission_actions() uses the real set_rbx_mode helper (RTL).
    # ------------------------------------------------------------
    def test_post_mission_actions_sets_rtl_mode(self):
        instance = self._build_instance()
        instance.rbx_info.mode = instance.rbx_cap_modes.index("RTL")
        success = instance.post_mission_actions()
        self.assertTrue(success)
        self.assertEqual(
            instance.rbx_set_mode_pub.published[-1],
            instance.rbx_cap_modes.index("RTL"),
        )

    # ------------------------------------------------------------
    # 6) cleanup_actions() runs without raising (only logs via MsgIF).
    # ------------------------------------------------------------
    def test_cleanup_actions_does_not_raise(self):
        instance = self._build_instance()
        instance.cleanup_actions()
        self.assertTrue(any(level == "info" for level, _ in instance.msg_if.calls))


if __name__ == "__main__":
    unittest.main(verbosity=2)
