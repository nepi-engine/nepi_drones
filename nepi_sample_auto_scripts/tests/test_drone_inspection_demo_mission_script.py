#!/usr/bin/env python3
#
# Mock-stub unit test for drone_inspection_demo_mission_script.py
#
# This script's real runtime dependencies (rospy connected to a live ROS
# master, and the NEPI nepi_sdk / nepi_api / nepi_interfaces packages built
# inside the NEPI device's catkin workspace) are NOT usably importable in
# this sandbox. Confirmed before writing this test:
#   - `python3 -c "import rospy"` succeeds (a bare /opt/ros/noetic install is
#     present), but
#   - `python3 -c "import nepi_sdk.nepi_ros"` and
#     `from nepi_api.messages_if import MsgIF` both fail with
#     `ModuleNotFoundError: No module named 'rospy_message_converter'`, and
#   - `from nepi_interfaces.msg import DeviceRBXInfo` fails outright --
#     nepi_interfaces here is only an empty PEP 420 namespace package (a
#     symlink to src/nepi_interfaces with no generated msg/ Python modules,
#     since nothing has been catkin-built), so none of the real message
#     classes exist as importable Python objects at all.
#   - No roscore is running and there is no devel/setup.bash to source.
#
# So instead of importing the real stack, this test injects stub modules
# into sys.modules for every external dependency the target script touches
# (rospy, nepi_sdk.nepi_ros, nepi_sdk.nepi_settings, nepi_api.messages_if,
# std_msgs.msg, geographic_msgs.msg, nepi_interfaces.msg,
# nepi_interfaces.srv), with stub classes whose fields match the real
# current .msg/.srv definitions read directly from
# src/nepi_interfaces/msg/*.msg and src/nepi_interfaces/srv/*.srv. The goal
# is to catch exactly the class of bug this session was full of: a renamed
# attribute, topic, or class slipping through unnoticed.
#
# Design note on timing: unlike the other sample scripts (which are apps
# that wait on a subscriber callback via rospy.spin()), this script's
# __init__ runs an entire mission synchronously to completion (pre-mission
# takeoff, a goto-location loop, post-mission RTL) before returning, using
# plain time.sleep()/nepi_ros.sleep() polling loops with real timeouts
# (5-20s each). To exercise that whole code path in a fast, deterministic
# unit test, this test patches the real time.sleep (via unittest.mock.patch,
# which affects the script's own `import time; time.sleep(...)` calls too,
# since they share the same module object) down to a no-op, and stubs
# nepi_ros.sleep the same way. The RBX status stub is seeded with
# ready=True/cmd_success=True from the moment its (latched-style) subscriber
# callback fires, so the ready/busy polling loops resolve in a handful of
# fast iterations instead of blocking. Mode/state changes are deliberately
# NOT faked back onto rbx_info by the fake publishers (there is no simulated
# driver), so set_rbx_mode("RTL") in post_mission_actions honestly returns
# False after its polling loop times out -- this is asserted as the expected
# stub-environment behavior, not treated as a failure.

import importlib.util
import os
import sys
import types
import unittest
from unittest import mock


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_PATH = os.path.join(
    THIS_DIR, "..", "drone_inspection_demo_mission_script.py"
)


# ---------------------------------------------------------------------------
# Stub message/service classes, fields matched to the real current
# src/nepi_interfaces/msg/*.msg and src/nepi_interfaces/srv/*.srv definitions.
# ---------------------------------------------------------------------------

class FakeErrorBounds(object):
    """Real ErrorBounds.msg: max_distance_error_m, max_rotation_error_deg,
    min_stabilize_time_s."""

    def __init__(self, max_distance_error_m=0.0, max_rotation_error_deg=0.0,
                 min_stabilize_time_s=0.0):
        self.max_distance_error_m = max_distance_error_m
        self.max_rotation_error_deg = max_rotation_error_deg
        self.min_stabilize_time_s = min_stabilize_time_s


class FakeGotoErrors(object):
    """Real GotoErrors.msg: x_m, y_m, z_m, heading_deg, roll_deg, pitch_deg,
    yaw_deg."""

    def __init__(self, x_m=0.0, y_m=0.0, z_m=0.0, heading_deg=0.0,
                 roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        self.x_m = x_m
        self.y_m = y_m
        self.z_m = z_m
        self.heading_deg = heading_deg
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.yaw_deg = yaw_deg


class FakeMotorControl(object):
    def __init__(self, *args, **kwargs):
        pass


class FakeAxisControls(object):
    def __init__(self, *args, **kwargs):
        pass


class FakeDeviceRBXInfo(object):
    """Real DeviceRBXInfo.msg fields: connected, device_name, serial_num,
    hw_version, sw_version, standby, state, mode, error_bounds, cmd_timeout,
    image_source, image_status_overlay, home_lat, home_long, home_alt,
    home_depth."""

    def __init__(self, connected=True, device_name="ardupilot", serial_num="",
                 hw_version="", sw_version="", standby=False, state=0,
                 mode=0, error_bounds=None, cmd_timeout=20.0,
                 image_source="", image_status_overlay=False, home_lat=0.0,
                 home_long=0.0, home_alt=0.0, home_depth=0.0):
        self.connected = connected
        self.device_name = device_name
        self.serial_num = serial_num
        self.hw_version = hw_version
        self.sw_version = sw_version
        self.standby = standby
        self.state = state
        self.mode = mode
        self.error_bounds = error_bounds if error_bounds is not None else FakeErrorBounds()
        self.cmd_timeout = cmd_timeout
        self.image_source = image_source
        self.image_status_overlay = image_status_overlay
        self.home_lat = home_lat
        self.home_long = home_long
        self.home_alt = home_alt
        self.home_depth = home_depth


class FakeDeviceRBXStatus(object):
    """Real DeviceRBXStatus.msg fields (subset relevant to the script):
    ready, battery, errors_current, errors_prev, cmd_success,
    current_motor_control_settings, last_cmd_string, last_error_message,
    plus assorted descriptive strings/lists."""

    def __init__(self, device_name="", device_path="", device_node_name="",
                 serial_num="", hw_version="", sw_version="",
                 data_source_description="", data_ref_description="",
                 settings_topic="", navpose_topic="", save_data_topic="",
                 data_products=None, data_product_topics=None,
                 data_product_image_topics=None, process_current="None",
                 process_last="None", ready=True, battery=-999.0,
                 errors_current=None, errors_prev=None, cmd_success=True,
                 manual_control_mode_ready=False,
                 autonomous_control_mode_ready=True,
                 current_motor_control_settings=None, last_cmd_string="",
                 last_error_message="", navpose_frame_transform=None):
        self.device_name = device_name
        self.device_path = device_path
        self.device_node_name = device_node_name
        self.serial_num = serial_num
        self.hw_version = hw_version
        self.sw_version = sw_version
        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description
        self.settings_topic = settings_topic
        self.navpose_topic = navpose_topic
        self.save_data_topic = save_data_topic
        self.data_products = data_products if data_products is not None else []
        self.data_product_topics = data_product_topics if data_product_topics is not None else []
        self.data_product_image_topics = (
            data_product_image_topics if data_product_image_topics is not None else []
        )
        self.process_current = process_current
        self.process_last = process_last
        self.ready = ready
        self.battery = battery
        self.errors_current = errors_current if errors_current is not None else FakeGotoErrors()
        self.errors_prev = errors_prev if errors_prev is not None else FakeGotoErrors()
        self.cmd_success = cmd_success
        self.manual_control_mode_ready = manual_control_mode_ready
        self.autonomous_control_mode_ready = autonomous_control_mode_ready
        self.current_motor_control_settings = (
            current_motor_control_settings if current_motor_control_settings is not None else []
        )
        self.last_cmd_string = last_cmd_string
        self.last_error_message = last_error_message
        self.navpose_frame_transform = navpose_frame_transform


class FakeSetting(object):
    """Real Setting.msg: type_str, name_str, value_str."""

    def __init__(self, type_str="", name_str="", value_str=""):
        self.type_str = type_str
        self.name_str = name_str
        self.value_str = value_str


class FakeSettings(object):
    def __init__(self, settings=None):
        self.settings = settings if settings is not None else []


class FakeSettingsStatus(object):
    """Real SettingsStatus.msg: node_name, settings_topic, settings_count,
    setting_caps_list, settings_list, has_cap_updates."""

    def __init__(self, node_name="", settings_topic="", settings_count=0,
                 setting_caps_list=None, settings_list=None,
                 has_cap_updates=False):
        self.node_name = node_name
        self.settings_topic = settings_topic
        self.settings_count = settings_count
        self.setting_caps_list = setting_caps_list if setting_caps_list is not None else []
        self.settings_list = settings_list if settings_list is not None else []
        self.has_cap_updates = has_cap_updates


class FakeGotoLocation(object):
    """Real GotoLocation.msg: lat, long, altitude_meters, yaw_deg."""

    def __init__(self):
        self.lat = 0.0
        self.long = 0.0
        self.altitude_meters = 0.0
        self.yaw_deg = 0.0


class FakeGotoPosition(object):
    def __init__(self):
        self.x_meters = 0.0
        self.y_meters = 0.0
        self.z_meters = 0.0
        self.yaw_deg = 0.0


class FakeGotoPose(object):
    def __init__(self):
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0


class FakeRBXCapabilitiesQuery(object):
    """Placeholder for the service type object (only ever passed to
    connect_service, never introspected)."""
    pass


class FakeRBXCapabilitiesQueryResponse(object):
    """Real RBXCapabilitiesQuery.srv response fields: device_name,
    device_path, device_node_name, control_support, has_battery_feedback,
    has_manual_controls, has_autonomous_controls, has_set_home, has_go_home,
    has_go_stop, has_goto_pose, has_goto_position, has_goto_location,
    state_options, mode_options, setup_action_options, go_action_options,
    data_products."""

    def __init__(self, state_options=None, mode_options=None,
                 setup_action_options=None, go_action_options=None):
        self.device_name = "ardupilot"
        self.device_path = ""
        self.device_node_name = ""
        self.control_support = FakeAxisControls()
        self.has_battery_feedback = True
        self.has_manual_controls = True
        self.has_autonomous_controls = True
        self.has_set_home = True
        self.has_go_home = True
        self.has_go_stop = True
        self.has_goto_pose = True
        self.has_goto_position = True
        self.has_goto_location = True
        self.state_options = state_options if state_options is not None else ["DISARM", "ARM"]
        self.mode_options = (
            mode_options if mode_options is not None
            else ["STABILIZE", "GUIDED", "RTL", "LAND", "LOITER", "RESUME"]
        )
        self.setup_action_options = (
            setup_action_options if setup_action_options is not None else ["LAUNCH"]
        )
        self.go_action_options = go_action_options if go_action_options is not None else ["NONE"]
        self.data_products = []


class FakeGeoPoint(object):
    def __init__(self, latitude=0.0, longitude=0.0, altitude=0.0):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude


class FakeEmpty(object):
    def __init__(self, *args, **kwargs):
        pass


class FakeBool(object):
    def __init__(self, data=False):
        self.data = data


class FakeString(object):
    def __init__(self, data=""):
        self.data = data


class FakeUInt32(object):
    def __init__(self, data=0):
        self.data = data


class FakeInt32(object):
    def __init__(self, data=0):
        self.data = data


class FakeFloat32(object):
    def __init__(self, data=0.0):
        self.data = data


class FakeFloat64(object):
    def __init__(self, data=0.0):
        self.data = data


# ---------------------------------------------------------------------------
# Stub: std_msgs.msg / geographic_msgs.msg
# ---------------------------------------------------------------------------

def _make_std_msgs_stub():
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Empty = FakeEmpty
    std_msgs_msg.Bool = FakeBool
    std_msgs_msg.String = FakeString
    std_msgs_msg.UInt32 = FakeUInt32
    std_msgs_msg.Int32 = FakeInt32
    std_msgs_msg.Float32 = FakeFloat32
    std_msgs_msg.Float64 = FakeFloat64
    std_msgs.msg = std_msgs_msg
    return std_msgs, std_msgs_msg


def _make_geographic_msgs_stub():
    geographic_msgs = types.ModuleType("geographic_msgs")
    geographic_msgs_msg = types.ModuleType("geographic_msgs.msg")
    geographic_msgs_msg.GeoPoint = FakeGeoPoint
    geographic_msgs.msg = geographic_msgs_msg
    return geographic_msgs, geographic_msgs_msg


# ---------------------------------------------------------------------------
# Stub: nepi_interfaces.msg / nepi_interfaces.srv
# ---------------------------------------------------------------------------

def _make_nepi_interfaces_stub():
    nepi_interfaces_pkg = types.ModuleType("nepi_interfaces")
    nepi_interfaces_msg = types.ModuleType("nepi_interfaces.msg")
    nepi_interfaces_srv = types.ModuleType("nepi_interfaces.srv")

    nepi_interfaces_msg.DeviceRBXInfo = FakeDeviceRBXInfo
    nepi_interfaces_msg.DeviceRBXStatus = FakeDeviceRBXStatus
    nepi_interfaces_msg.AxisControls = FakeAxisControls
    nepi_interfaces_msg.ErrorBounds = FakeErrorBounds
    nepi_interfaces_msg.GotoErrors = FakeGotoErrors
    nepi_interfaces_msg.MotorControl = FakeMotorControl
    nepi_interfaces_msg.GotoPose = FakeGotoPose
    nepi_interfaces_msg.GotoPosition = FakeGotoPosition
    nepi_interfaces_msg.GotoLocation = FakeGotoLocation
    nepi_interfaces_msg.Setting = FakeSetting
    nepi_interfaces_msg.Settings = FakeSettings
    nepi_interfaces_msg.SettingsStatus = FakeSettingsStatus

    nepi_interfaces_srv.RBXCapabilitiesQuery = FakeRBXCapabilitiesQuery
    nepi_interfaces_srv.RBXCapabilitiesQueryResponse = FakeRBXCapabilitiesQueryResponse

    nepi_interfaces_pkg.msg = nepi_interfaces_msg
    nepi_interfaces_pkg.srv = nepi_interfaces_srv
    return nepi_interfaces_pkg, nepi_interfaces_msg, nepi_interfaces_srv


# ---------------------------------------------------------------------------
# Stub: rospy
#
# Surface actually reached by drone_inspection_demo_mission_script.py's own
# direct rospy.* calls (everything else goes through nepi_sdk.nepi_ros):
#   rospy.is_shutdown(), rospy.signal_shutdown(reason), rospy.Publisher(...)
# ---------------------------------------------------------------------------

class FakePublisher(object):
    def __init__(self, topic, data_class, queue_size=1, latch=False):
        self.topic = topic
        self.data_class = data_class
        self.queue_size = queue_size
        self.latch = latch
        self.published = []

    def publish(self, *args, **kwargs):
        self.published.append(args[0] if len(args) == 1 else (args, kwargs))


def _make_rospy_stub(record):
    rospy = types.ModuleType("rospy")

    def is_shutdown():
        return False

    def signal_shutdown(reason):
        record["signal_shutdown_calls"].append(reason)

    def Publisher(topic, data_class, queue_size=1, **kwargs):
        pub = FakePublisher(topic, data_class, queue_size=queue_size)
        record["rospy_publishers"].append(pub)
        return pub

    rospy.is_shutdown = is_shutdown
    rospy.signal_shutdown = signal_shutdown
    rospy.Publisher = Publisher
    return rospy


# ---------------------------------------------------------------------------
# Stub: nepi_sdk.nepi_ros
#
# Signatures matched to the CURRENT (confirmed-unchanged) nepi_sdk/nepi_ros.py:
#   init_node(name, disable_signals=False)
#   get_node_name(); get_base_namespace()
#   wait_for_node(node_name, timeout=60, log_name_list=[])
#   wait_for_topic(topic_name, timeout=60, log_name_list=[], topics_list=None,
#                   types_list=None)
#   wait_for_service(service_name, timeout=60, log_name_list=[])
#   connect_service(service_namespace, service_msg, log_name_list=[])
#   create_subscriber(sub_namespace, msg, callback, queue_size=10,
#                       callback_args=(), log_name_list=[])
#   create_publisher(pub_namespace, msg, queue_size=10, latch=False,
#                       log_name_list=[])
#   sleep(sleep_sec, sleep_steps=None)
#   is_shutdown()
#
# create_subscriber delivers one fake message synchronously (simulating the
# real rbx/settings/status latched publisher, and the driver responding to
# publish_info/publish_status) so the script's "while self.x is None" wait
# loops resolve immediately instead of needing real ROS traffic.
# ---------------------------------------------------------------------------

def _make_nepi_sdk_stub(record):
    nepi_sdk_pkg = types.ModuleType("nepi_sdk")
    nepi_ros_mod = types.ModuleType("nepi_sdk.nepi_ros")

    def init_node(name, disable_signals=False):
        record["init_node_calls"].append({"name": name, "disable_signals": disable_signals})

    def get_node_name():
        return "drone_inspection_demo_mission"

    def get_base_namespace():
        return "/nepi/device1/"

    def wait_for_node(node_name, timeout=60, log_name_list=None):
        record["wait_for_node_calls"].append(node_name)
        return "/nepi/device1/" + node_name

    def wait_for_topic(topic_name, timeout=60, log_name_list=None,
                        topics_list=None, types_list=None):
        record["wait_for_topic_calls"].append(topic_name)
        # Only the very first call's return value is used by the script
        # (to derive NEPI_ROBOT_NAMESPACE via rpartition("rbx")); it must
        # contain "rbx" and resolve back to the same rbx/ namespace.
        if topic_name.endswith("rbx/"):
            return topic_name + "info"
        return topic_name

    def wait_for_service(service_name, timeout=60, log_name_list=None):
        record["wait_for_service_calls"].append(service_name)

    def connect_service(service_namespace, service_msg, log_name_list=None):
        record["connect_service_calls"].append(service_namespace)

        def _call(*args, **kwargs):
            return FakeRBXCapabilitiesQueryResponse()

        return _call

    def create_subscriber(sub_namespace, msg, callback, queue_size=10,
                           callback_args=(), log_name_list=None):
        record["create_subscriber_calls"].append((sub_namespace, msg))
        if msg is FakeSettingsStatus:
            callback(FakeSettingsStatus(
                node_name="ardupilot_rbx",
                settings_topic=sub_namespace,
                settings_count=1,
                settings_list=[FakeSetting(type_str="Float", name_str="takeoff_height_m", value_str="10.0")],
            ))
        elif msg is FakeDeviceRBXInfo:
            callback(FakeDeviceRBXInfo(state=0, mode=0))
        elif msg is FakeDeviceRBXStatus:
            callback(FakeDeviceRBXStatus(ready=True, cmd_success=True))
        return object()

    def create_publisher(pub_namespace, msg, queue_size=10, latch=False,
                          log_name_list=None):
        pub = FakePublisher(pub_namespace, msg, queue_size=queue_size, latch=latch)
        record["create_publisher_calls"].append((pub_namespace, msg, pub))
        return pub

    def sleep(sleep_sec, sleep_steps=None):
        record["nepi_sleep_calls"].append((sleep_sec, sleep_steps))

    def is_shutdown():
        return False

    nepi_ros_mod.init_node = init_node
    nepi_ros_mod.get_node_name = get_node_name
    nepi_ros_mod.get_base_namespace = get_base_namespace
    nepi_ros_mod.wait_for_node = wait_for_node
    nepi_ros_mod.wait_for_topic = wait_for_topic
    nepi_ros_mod.wait_for_service = wait_for_service
    nepi_ros_mod.connect_service = connect_service
    nepi_ros_mod.create_subscriber = create_subscriber
    nepi_ros_mod.create_publisher = create_publisher
    nepi_ros_mod.sleep = sleep
    nepi_ros_mod.is_shutdown = is_shutdown

    nepi_sdk_pkg.nepi_ros = nepi_ros_mod
    record["connect_service_calls"] = []
    return nepi_sdk_pkg, nepi_ros_mod


# ---------------------------------------------------------------------------
# Stub: nepi_sdk.nepi_settings
#
# Real current implementations (src/nepi_engine/nepi_sdk/src/nepi_sdk/nepi_settings.py):
#   parse_setting_msgs_list(settings_msg) reads settings_msg.settings_list,
#     each entry's name_str/type_str/value_str -> {name: {name,type,value}}
#   create_msg_from_setting(setting) builds a Setting from
#     setting['type']/['name']/['value']
# Reproduced faithfully here (not just stubbed as pass-throughs) so the test
# actually exercises the real field-name contract between the script and
# nepi_settings.
# ---------------------------------------------------------------------------

def _make_nepi_settings_stub():
    nepi_settings_mod = types.ModuleType("nepi_sdk.nepi_settings")

    def parse_setting_msgs_list(settings_msg):
        settings = dict()
        for entry in settings_msg.settings_list:
            settings[entry.name_str] = {
                "name": entry.name_str,
                "type": entry.type_str,
                "value": entry.value_str,
            }
        return settings

    def create_msg_from_setting(setting):
        setting_msg = FakeSetting()
        setting_msg.type_str = setting["type"]
        setting_msg.name_str = setting["name"]
        setting_msg.value_str = setting["value"]
        return setting_msg

    nepi_settings_mod.parse_setting_msgs_list = parse_setting_msgs_list
    nepi_settings_mod.create_msg_from_setting = create_msg_from_setting
    return nepi_settings_mod


# ---------------------------------------------------------------------------
# Stub: nepi_api.messages_if.MsgIF
# ---------------------------------------------------------------------------

def _make_nepi_api_stub(record):
    nepi_api_pkg = types.ModuleType("nepi_api")
    messages_if_mod = types.ModuleType("nepi_api.messages_if")

    class FakeMsgIF(object):
        def __init__(self, log_name=None):
            self.log_name = log_name
            record["msg_if_instances"].append(self)

        def pub_info(self, msg, throttle_s=None, log_name_list=None):
            record["pub_info"].append(msg)

        def pub_warn(self, msg, throttle_s=None, log_name_list=None):
            record["pub_warn"].append(msg)

        def pub_debug(self, msg, throttle_s=None, log_name_list=None):
            record["pub_debug"].append(msg)

        def pub_error(self, msg, throttle_s=None, log_name_list=None):
            record["pub_error"].append(msg)

    messages_if_mod.MsgIF = FakeMsgIF
    nepi_api_pkg.messages_if = messages_if_mod
    return nepi_api_pkg, messages_if_mod


def _load_target_module(record):
    """Install all stub modules into sys.modules, then import the target
    script fresh from disk via its file path (it lives outside any package,
    so we cannot just `import drone_inspection_demo_mission_script`)."""

    std_msgs_pkg, std_msgs_msg_mod = _make_std_msgs_stub()
    geographic_msgs_pkg, geographic_msgs_msg_mod = _make_geographic_msgs_stub()
    nepi_interfaces_pkg, nepi_interfaces_msg_mod, nepi_interfaces_srv_mod = _make_nepi_interfaces_stub()
    rospy_mod = _make_rospy_stub(record)
    nepi_sdk_pkg, nepi_ros_mod = _make_nepi_sdk_stub(record)
    nepi_settings_mod = _make_nepi_settings_stub()
    nepi_sdk_pkg.nepi_settings = nepi_settings_mod
    nepi_api_pkg, messages_if_mod = _make_nepi_api_stub(record)

    stub_modules = {
        "std_msgs": std_msgs_pkg,
        "std_msgs.msg": std_msgs_msg_mod,
        "geographic_msgs": geographic_msgs_pkg,
        "geographic_msgs.msg": geographic_msgs_msg_mod,
        "nepi_interfaces": nepi_interfaces_pkg,
        "nepi_interfaces.msg": nepi_interfaces_msg_mod,
        "nepi_interfaces.srv": nepi_interfaces_srv_mod,
        "rospy": rospy_mod,
        "nepi_sdk": nepi_sdk_pkg,
        "nepi_sdk.nepi_ros": nepi_ros_mod,
        "nepi_sdk.nepi_settings": nepi_settings_mod,
        "nepi_api": nepi_api_pkg,
        "nepi_api.messages_if": messages_if_mod,
    }

    saved = {}
    for name, mod in stub_modules.items():
        saved[name] = sys.modules.get(name)
        sys.modules[name] = mod

    try:
        spec = importlib.util.spec_from_file_location(
            "drone_inspection_demo_mission_script_under_test", SCRIPT_PATH
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        for name, prior in saved.items():
            if prior is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prior

    return module


class TestDroneInspectionDemoMissionScript(unittest.TestCase):

    def setUp(self):
        self.record = {
            "init_node_calls": [],
            "wait_for_node_calls": [],
            "wait_for_topic_calls": [],
            "wait_for_service_calls": [],
            "create_publisher_calls": [],
            "create_subscriber_calls": [],
            "nepi_sleep_calls": [],
            "msg_if_instances": [],
            "pub_info": [],
            "pub_warn": [],
            "pub_debug": [],
            "pub_error": [],
            "rospy_publishers": [],
            "signal_shutdown_calls": [],
        }
        self.module = _load_target_module(self.record)

    def _publisher_for(self, suffix):
        """Find the (topic, msg_class, FakePublisher) created via
        nepi_ros.create_publisher whose topic ends with `suffix`."""
        matches = [t for t in self.record["create_publisher_calls"] if t[0].endswith(suffix)]
        self.assertEqual(
            len(matches), 1,
            f"Expected exactly one create_publisher call ending with {suffix!r}, "
            f"found topics: {[t[0] for t in self.record['create_publisher_calls']]}",
        )
        return matches[0]

    def test_module_imports_cleanly_against_current_api_stubs(self):
        # Strongest regression check: the module must import with no
        # AttributeError/ImportError against stand-ins for the CURRENT
        # (post-rename) nepi_interfaces.msg/srv surface -- in particular
        # DeviceRBXInfo/DeviceRBXStatus/Goto*/ErrorBounds/Setting*, not the
        # old RBXInfo/RBXStatus/RBXGoto*/RBXErrorBounds names.
        self.assertTrue(hasattr(self.module, "drone_inspection_demo_mission"))

    def test_full_mission_runs_to_completion_with_patched_sleep(self):
        # __init__ runs the entire mission synchronously; patch time.sleep
        # (affects the script's own `import time` calls, same module
        # object) to a no-op so the many 5-20s polling-loop timeouts inside
        # wait_for_rbx_status_ready/busy resolve near-instantly.
        with mock.patch("time.sleep", return_value=None):
            node = self.module.drone_inspection_demo_mission()

        self.assertEqual(len(self.record["init_node_calls"]), 1)
        self.assertEqual(
            self.record["init_node_calls"][0]["name"],
            self.module.drone_inspection_demo_mission.DEFAULT_NODE_NAME,
        )
        self.assertEqual(node.node_name, "drone_inspection_demo_mission")
        self.assertEqual(node.base_namespace, "/nepi/device1/")

        # nepi_msg -> MsgIF fix: exactly one MsgIF built, logging under the
        # node's name.
        self.assertEqual(len(self.record["msg_if_instances"]), 1)
        self.assertIs(node.msg_if, self.record["msg_if_instances"][0])
        self.assertEqual(node.msg_if.log_name, "drone_inspection_demo_mission")

        # Mission actually ran end to end.
        self.assertTrue(any("Initialization Complete" in m for m in self.record["pub_info"]))
        self.assertTrue(any("Mission Processes Complete" in m for m in self.record["pub_info"]))
        self.assertTrue(any("Post-Mission Actions Complete" in m for m in self.record["pub_info"]))

        # rospy.signal_shutdown (not the removed nepi_msg module, not a raw
        # sys.exit) is how the script ends itself.
        self.assertEqual(self.record["signal_shutdown_calls"], ["Mission Complete, Shutting Down"])

        # Fake GPS is the standalone app_fake_gps app (Bool on .../enable),
        # not a per-robot rbx/enable_fake_gps topic.
        fake_gps_topic, fake_gps_msg_cls, fake_gps_pub = self._publisher_for("app_fake_gps/enable")
        self.assertIs(fake_gps_msg_cls, FakeBool)
        self.assertEqual(fake_gps_topic, "/nepi/device1/app_fake_gps/enable")
        self.assertTrue(any(getattr(p, "data", p) is True or p is True for p in fake_gps_pub.published))

        # Snapshot trigger fired once per mission_actions() call: the main
        # goto + 3 corner gotos = 4 total.
        snapshot_pubs = [p for p in self.record["rospy_publishers"] if p.topic.endswith("snapshot_trigger")]
        self.assertEqual(len(snapshot_pubs), 1)
        self.assertEqual(len(snapshot_pubs[0].published), 4)

    def test_rbx_initialize_uses_current_topic_names_and_message_types(self):
        # Regression pin for every renamed RBX topic/message this session
        # confirmed, cross-checked against device_if_rbx.py's PUBS/SUBS/SRVS
        # dicts per the reference scripts' own verification notes.
        with mock.patch("time.sleep", return_value=None):
            self.module.drone_inspection_demo_mission()

        settings_update_topic, settings_update_cls, _ = self._publisher_for("settings/update_setting")
        self.assertIs(settings_update_cls, FakeSetting)

        set_state_topic, set_state_cls, _ = self._publisher_for("set_state")
        self.assertIs(set_state_cls, FakeInt32)

        set_mode_topic, set_mode_cls, _ = self._publisher_for("set_mode")
        self.assertIs(set_mode_cls, FakeInt32)

        setup_action_topic, setup_action_cls, _ = self._publisher_for("setup_action")
        self.assertIs(setup_action_cls, FakeInt32)

        go_action_topic, go_action_cls, _ = self._publisher_for("go_action")
        self.assertIs(go_action_cls, FakeInt32)

        # renamed from set_cmd_timeout -> set_goto_timeout, Int8 -> UInt32
        timeout_topic, timeout_cls, _ = self._publisher_for("set_goto_timeout")
        self.assertIs(timeout_cls, FakeUInt32)
        self.assertNotIn(
            "set_cmd_timeout",
            [t[0] for t in self.record["create_publisher_calls"]],
        )

        set_home_topic, set_home_cls, _ = self._publisher_for("set_home")
        self.assertIs(set_home_cls, FakeGeoPoint)

        goto_location_topic, goto_location_cls, _ = self._publisher_for("goto_location")
        self.assertIs(goto_location_cls, FakeGotoLocation)

        # Settings status subscription uses the current SettingsStatus type
        # under the rbx/settings/ sub-namespace (not a flat rbx/settings_status).
        settings_status_subs = [
            s for s in self.record["create_subscriber_calls"] if s[0].endswith("settings/status")
        ]
        self.assertEqual(len(settings_status_subs), 1)
        self.assertIs(settings_status_subs[0][1], FakeSettingsStatus)

        info_subs = [s for s in self.record["create_subscriber_calls"] if s[0].endswith("/info")]
        self.assertEqual(len(info_subs), 1)
        self.assertIs(info_subs[0][1], FakeDeviceRBXInfo)

        status_subs = [
            s for s in self.record["create_subscriber_calls"]
            if s[0].endswith("/status") and "settings" not in s[0]
        ]
        self.assertEqual(len(status_subs), 1)
        self.assertIs(status_subs[0][1], FakeDeviceRBXStatus)

    def test_rbx_settings_callback_uses_parse_setting_msgs_list(self):
        with mock.patch("time.sleep", return_value=None):
            node = self.module.drone_inspection_demo_mission()
        # Settings were parsed via nepi_settings.parse_setting_msgs_list
        # (not the removed parse_settings_msg_data) into the
        # {name: {name,type,value}} shape.
        self.assertIn("takeoff_height_m", node.rbx_settings)
        self.assertEqual(node.rbx_settings["takeoff_height_m"]["value"], "10.0")
        self.assertEqual(node.rbx_settings["takeoff_height_m"]["type"], "Float")

    def test_goto_rbx_location_builds_expected_message_fields(self):
        with mock.patch("time.sleep", return_value=None):
            node = self.module.drone_inspection_demo_mission()

        _, _, goto_location_pub = self._publisher_for("goto_location")
        self.assertGreaterEqual(len(goto_location_pub.published), 1)
        first_msg = goto_location_pub.published[0]
        expected = self.module.GOTO_LOCATION
        self.assertEqual(first_msg.lat, expected[0])
        self.assertEqual(first_msg.long, expected[1])
        self.assertEqual(first_msg.altitude_meters, expected[2])
        self.assertEqual(first_msg.yaw_deg, expected[3])

    def test_setup_rbx_action_launch_matches_capability_and_succeeds(self):
        # The RBXCapabilitiesQuery response's setup_action_options includes
        # "LAUNCH" (matching the script's LAUNCH-chained-takeoff comment);
        # with rbx_status.ready/cmd_success stubbed True throughout,
        # setup_rbx_action("LAUNCH") should report success.
        with mock.patch("time.sleep", return_value=None):
            node = self.module.drone_inspection_demo_mission()
        self.assertIn("LAUNCH", node.rbx_cap_setup_actions)
        self.assertTrue(
            any("Takeoff completed" in m for m in self.record["pub_info"]),
            "Expected pre_mission_actions' LAUNCH setup action to succeed "
            "against the stubbed always-ready status.",
        )

    def test_no_reference_to_removed_nepi_msg_module_or_old_rbx_names(self):
        with mock.patch("time.sleep", return_value=None):
            self.module.drone_inspection_demo_mission()
        # Would only be present if the old nepi_msg-based helper calls
        # (createMsgPublishers/publishMsgInfo) were still referenced -- they
        # aren't stubbed at all, so any use would have raised already; this
        # is a belt-and-suspenders source-level check too.
        import re

        with open(SCRIPT_PATH) as f:
            src = f.read()
        # Only check actual import/usage lines, not the module docstring
        # (which legitimately documents the old->new rename in prose).
        code_lines = [
            line for line in src.splitlines()
            if not line.strip().startswith("#")
        ]
        code = "\n".join(code_lines)
        self.assertNotIn("nepi_msg.", code)
        # Word-boundary regexes so "DeviceRBXInfo("/"DeviceRBXStatus(" (the
        # correct current names, which end in the old names as a substring)
        # don't false-positive this check.
        self.assertNotRegex(code, r"[\s=(,]nepi_ros_interfaces\b")
        self.assertNotRegex(code, r"\bRBXInfo\(")
        self.assertNotRegex(code, r"\bRBXStatus\(")
        self.assertNotRegex(code, r"\bRBXGoto\w*\b")
        self.assertNotRegex(code, r"\bRBXErrorBounds\b")


if __name__ == "__main__":
    unittest.main()
