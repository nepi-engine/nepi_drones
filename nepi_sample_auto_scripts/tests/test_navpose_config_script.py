#!/usr/bin/env python3
#
# Mock-stub unit test for navpose_config_script.py.
#
# WHY A MOCK-STUB TEST (not a real rospy/nepi_sdk import test):
# This machine has no built catkin workspace (no devel/setup.bash). rospy
# itself IS genuinely importable here (real /opt/ros/noetic install), but
# `nepi_sdk.nepi_ros` fails to import (ModuleNotFoundError:
# rospy_message_converter, a dependency normally provided by the NEPI
# device's catkin install) and `nepi_interfaces.msg` has no generated
# message classes at all (they're produced by catkin at build time, and
# nothing has been built here). Confirmed directly this session:
#   python3 -c "from nepi_sdk import nepi_ros"      -> ModuleNotFoundError
#   python3 -c "from nepi_interfaces.msg import UpdateString" -> ImportError
# So, matching the sibling test files in this directory (e.g.
# test_led_auto_level_process_script.py), this test stubs rospy entirely
# (a real rospy.Publisher()/rospy.spin() would try to contact a ROS
# master), plus nepi_sdk.nepi_ros, nepi_api.messages_if.MsgIF, and
# nepi_interfaces.msg.UpdateString.
#
# The stub signatures are typed to match the CURRENT nepi_sdk.nepi_ros /
# nepi_api.messages_if.MsgIF APIs (see nepi_engine_ws/src/nepi_engine/
# nepi_sdk/src/nepi_sdk/nepi_ros.py and .../nepi_api/src/nepi_api/
# messages_if.py) and the CURRENT nepi_interfaces/msg/UpdateString.msg
# field list (name, name2, name3, value -- confirmed by reading the .msg
# file directly this session), so that if the script used a renamed
# field/topic/class the way this session's other scripts did before being
# fixed, these stubs would raise an AttributeError/TypeError here too.
#
# Run directly with:
#   python3 -m unittest tests.test_navpose_config_script -v
# (from the nepi_sample_auto_scripts/ directory)

import importlib.util
import os
import sys
import types
import unittest

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_PATH = os.path.join(SCRIPT_DIR, "navpose_config_script.py")


def _install_stub_modules():
    """Inject stub rospy / nepi_sdk.nepi_ros / nepi_api.messages_if /
    nepi_interfaces.msg modules into sys.modules and return the previous
    sys.modules entries (or None) so the caller can restore them afterward.
    """

    # ---------------------------------------------------------------
    # rospy stub -- only the surface navpose_config_script.py uses:
    # Publisher, Timer, Duration, spin.
    # ---------------------------------------------------------------
    rospy_stub = types.ModuleType("rospy")

    class _FakePublisher:
        def __init__(self, topic, msg_class, queue_size=1):
            self.topic = topic
            self.msg_class = msg_class
            self.queue_size = queue_size
            self.published = []

        def publish(self, *args, **kwargs):
            if args:
                self.published.append(args[0])
            else:
                self.published.append(self.msg_class(**kwargs))

    class _FakeTimer:
        def __init__(self, duration, callback, oneshot=False):
            self.duration = duration
            self.callback = callback
            self.oneshot = oneshot

    class _FakeDuration:
        def __init__(self, secs=0, nsecs=0):
            self.secs = secs
            self.nsecs = nsecs

    rospy_stub.Publisher = _FakePublisher
    rospy_stub.Timer = _FakeTimer
    rospy_stub.Duration = _FakeDuration
    rospy_stub.spin = lambda: None  # must NOT block the test
    rospy_stub.is_shutdown = lambda: False
    rospy_stub.signal_shutdown = lambda reason=None: None
    rospy_stub.init_node = lambda *a, **k: None
    rospy_stub.get_name = lambda: "/navpose_config"
    rospy_stub.loginfo = lambda *a, **k: None
    rospy_stub.logwarn = lambda *a, **k: None
    rospy_stub.logerr = lambda *a, **k: None
    rospy_stub.logdebug = lambda *a, **k: None

    # ---------------------------------------------------------------
    # nepi_sdk / nepi_sdk.nepi_ros stub -- signatures match the current
    # nepi_ros.py: init_node(name, disable_signals=False), get_node_name(),
    # get_base_namespace(), wait_for_topic(topic_name, timeout=60,
    # log_name_list=[], topics_list=None, types_list=None).
    # ---------------------------------------------------------------
    nepi_sdk_pkg = types.ModuleType("nepi_sdk")
    nepi_ros_stub = types.ModuleType("nepi_sdk.nepi_ros")

    nepi_ros_stub.init_node = lambda name, disable_signals=False: None
    nepi_ros_stub.get_node_name = lambda: "navpose_config"
    nepi_ros_stub.get_base_namespace = lambda: "/nepi/device1/"
    nepi_ros_stub.is_shutdown = lambda: False

    def _wait_for_topic(topic_name, timeout=60, log_name_list=None,
                         topics_list=None, types_list=None):
        # Real nepi_ros.wait_for_topic() polls the live ROS graph for a
        # topic whose name *contains* topic_name and returns the fully
        # resolved topic string. Simulate that resolution deterministically.
        if topic_name.startswith("/"):
            return topic_name
        return "/nepi/device1/" + topic_name.lstrip("/")

    nepi_ros_stub.wait_for_topic = _wait_for_topic
    nepi_sdk_pkg.nepi_ros = nepi_ros_stub

    # ---------------------------------------------------------------
    # nepi_api / nepi_api.messages_if.MsgIF stub -- records every
    # pub_info/pub_warn/pub_debug/pub_error call so the test can assert
    # the script uses ONLY this (current) logging convention and never
    # falls back to the removed nepi_msg.publishMsgInfo(self, msg) style.
    # ---------------------------------------------------------------
    nepi_api_pkg = types.ModuleType("nepi_api")
    messages_if_stub = types.ModuleType("nepi_api.messages_if")

    class _FakeMsgIF:
        def __init__(self, log_name=None):
            self.log_name = log_name
            self.calls = []

        def pub_info(self, msg, **kwargs):
            self.calls.append(("info", msg))

        def pub_warn(self, msg, **kwargs):
            self.calls.append(("warn", msg))

        def pub_debug(self, msg, **kwargs):
            self.calls.append(("debug", msg))

        def pub_error(self, msg, **kwargs):
            self.calls.append(("error", msg))

    messages_if_stub.MsgIF = _FakeMsgIF
    nepi_api_pkg.messages_if = messages_if_stub

    # ---------------------------------------------------------------
    # nepi_interfaces.msg.UpdateString stub -- field list (name, name2,
    # name3, value) matches nepi_interfaces/msg/UpdateString.msg exactly
    # as read from the .msg file this session.
    # ---------------------------------------------------------------
    nepi_interfaces_pkg = types.ModuleType("nepi_interfaces")
    nepi_interfaces_msg_stub = types.ModuleType("nepi_interfaces.msg")

    class _FakeUpdateString:
        __slots__ = ("name", "name2", "name3", "value")

        def __init__(self, name="", name2="", name3="", value=""):
            self.name = name
            self.name2 = name2
            self.name3 = name3
            self.value = value

        def __repr__(self):
            return ("UpdateString(name=%r, name2=%r, name3=%r, value=%r)"
                     % (self.name, self.name2, self.name3, self.value))

    nepi_interfaces_msg_stub.UpdateString = _FakeUpdateString
    nepi_interfaces_pkg.msg = nepi_interfaces_msg_stub

    stub_modules = {
        "rospy": rospy_stub,
        "nepi_sdk": nepi_sdk_pkg,
        "nepi_sdk.nepi_ros": nepi_ros_stub,
        "nepi_api": nepi_api_pkg,
        "nepi_api.messages_if": messages_if_stub,
        "nepi_interfaces": nepi_interfaces_pkg,
        "nepi_interfaces.msg": nepi_interfaces_msg_stub,
    }

    previous = {}
    for name, mod in stub_modules.items():
        previous[name] = sys.modules.get(name)
        sys.modules[name] = mod
    return previous, _FakeUpdateString


def _restore_modules(previous):
    for name, mod in previous.items():
        if mod is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = mod


class TestNavposeConfigScript(unittest.TestCase):
    """Imports navpose_config_script.py against stub modules for the
    modules unavailable/broken in this environment (nepi_sdk.nepi_ros,
    nepi_interfaces.msg) plus a fully-stubbed rospy (to avoid contacting a
    ROS master), drives the real __init__ path, and exercises the timer
    callback to confirm no broken attribute/field references remain
    post-API-drift-fix.
    """

    @classmethod
    def setUpClass(cls):
        previous, update_string_cls = _install_stub_modules()
        try:
            spec = importlib.util.spec_from_file_location(
                "navpose_config_script_under_test", SCRIPT_PATH
            )
            cls.module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = cls.module
            spec.loader.exec_module(cls.module)
        finally:
            _restore_modules(previous)
        cls.UpdateString = update_string_cls

    def _build_instance(self):
        previous, _ = _install_stub_modules()
        try:
            instance = self.module.navpose_config()
        finally:
            _restore_modules(previous)
        return instance

    # ------------------------------------------------------------
    # 1) Clean import + no accidental regression to the removed API
    # ------------------------------------------------------------
    def test_module_imports_cleanly(self):
        self.assertTrue(hasattr(self.module, "navpose_config"))
        self.assertEqual(self.module.NEPI_NAVPOSE_FRAME_NAME, "base_frame")

    def test_source_has_no_removed_nepi_msg_api(self):
        # The module docstring intentionally documents the OLD nepi_msg
        # API in prose (as the drone_* reference scripts do) -- what must
        # actually be absent is any *usage* of it (an import, a call, or
        # the old createMsgPublishers/publishMsgInfo call forms).
        with open(SCRIPT_PATH) as f:
            src = f.read()
        self.assertNotIn("import nepi_msg", src)
        self.assertNotIn("nepi_msg.", src)
        self.assertNotIn("publishMsgInfo(", src)
        self.assertNotIn("createMsgPublishers(", src)
        self.assertIn("MsgIF", src)

    def test_source_has_no_removed_navpose_mgr_api(self):
        # The old nav_pose_mgr per-robot "point at a source topic" surface
        # (set_gps_fix_topic/set_heading_topic/set_orientation_topic/
        # enable_gps_clock_sync) no longer exists in the current
        # navpose_mgr. The module docstring documents these OLD names in
        # prose (expected, matches the drone_* reference scripts' style),
        # so what must actually be absent is a *call* to one of them.
        with open(SCRIPT_PATH) as f:
            src = f.read()
        for removed in ("set_gps_fix_topic(", "set_heading_topic(",
                         "set_orientation_topic(", "enable_gps_clock_sync(",
                         "import nepi_ros_interfaces",
                         "from nepi_ros_interfaces"):
            self.assertNotIn(removed, src)
        self.assertIn("set_frame_comp_topic", src)

    # ------------------------------------------------------------
    # 2) Full __init__ path runs against the stubbed current API with
    #    no AttributeError/TypeError (topic-wait, publisher setup,
    #    MsgIF construction/logging, timer registration).
    # ------------------------------------------------------------
    def test_init_runs_against_stubbed_current_api(self):
        instance = self._build_instance()

        # msg_if was constructed via the current MsgIF(log_name=...) API
        # and used for logging (not the removed nepi_msg module).
        self.assertEqual(instance.msg_if.log_name, "navpose_config")
        self.assertTrue(len(instance.msg_if.calls) > 0)
        self.assertTrue(any(level == "info" for level, _ in instance.msg_if.calls))

        # The frame-comp-topic publisher was created against the resolved
        # navpose_mgr/set_frame_comp_topic topic, using the current
        # nepi_interfaces/UpdateString message class (not a removed
        # RBX-style or nav_pose_mgr-style message).
        self.assertEqual(
            instance.set_frame_comp_topic_pub.topic,
            "/nepi/device1/navpose_mgr/set_frame_comp_topic",
        )
        self.assertIs(instance.set_frame_comp_topic_pub.msg_class, self.UpdateString)

        # Source topics resolved via nepi_ros.wait_for_topic (not a
        # removed nav_pose_mgr publisher call).
        self.assertEqual(instance.gps_topic, "/nepi/device1/rbx/gps_fix")
        self.assertEqual(instance.odom_topic, "/nepi/device1/rbx/odom")
        self.assertEqual(instance.heading_topic, "/nepi/device1/rbx/heading")

    # ------------------------------------------------------------
    # 3) Timer callback: exercises the actual set_frame_comp_topic
    #    publish calls and confirms field names/values match the current
    #    UpdateString layout and the documented component/type mapping.
    # ------------------------------------------------------------
    def test_timer_callback_publishes_expected_update_messages(self):
        instance = self._build_instance()
        pub = instance.set_frame_comp_topic_pub
        self.assertEqual(len(pub.published), 0)

        instance.set_nepi_navpose_topics_callback(None)  # rospy.Timer passes a TimerEvent

        self.assertEqual(len(pub.published), 3)
        by_comp = {msg.name2: msg for msg in pub.published}
        self.assertEqual(set(by_comp.keys()), {"location", "orientation", "heading"})

        location_msg = by_comp["location"]
        self.assertEqual(location_msg.name, "base_frame")
        self.assertEqual(location_msg.name3, "update")
        self.assertEqual(location_msg.value, "/nepi/device1/rbx/gps_fix")

        orientation_msg = by_comp["orientation"]
        self.assertEqual(orientation_msg.name, "base_frame")
        self.assertEqual(orientation_msg.name3, "update")
        self.assertEqual(orientation_msg.value, "/nepi/device1/rbx/odom")

        heading_msg = by_comp["heading"]
        self.assertEqual(heading_msg.name, "base_frame")
        self.assertEqual(heading_msg.name3, "update")
        self.assertEqual(heading_msg.value, "/nepi/device1/rbx/heading")

        for msg in pub.published:
            self.assertIsInstance(msg, self.UpdateString)

    # ------------------------------------------------------------
    # 4) "" (ignore) source topics are skipped entirely -- neither
    #    resolved via wait_for_topic nor published by the timer callback.
    # ------------------------------------------------------------
    def test_empty_string_source_topic_is_skipped(self):
        previous, _ = _install_stub_modules()
        try:
            orig_gps = self.module.NEPI_NAVPOSE_SOURCE_GPS_TOPIC
            orig_heading = self.module.NEPI_NAVPOSE_SOURCE_HEADING_TOPIC
            self.module.NEPI_NAVPOSE_SOURCE_GPS_TOPIC = ""
            self.module.NEPI_NAVPOSE_SOURCE_HEADING_TOPIC = ""
            try:
                instance = self.module.navpose_config()
            finally:
                self.module.NEPI_NAVPOSE_SOURCE_GPS_TOPIC = orig_gps
                self.module.NEPI_NAVPOSE_SOURCE_HEADING_TOPIC = orig_heading
        finally:
            _restore_modules(previous)

        self.assertIsNone(instance.gps_topic)
        self.assertIsNone(instance.heading_topic)
        self.assertEqual(instance.odom_topic, "/nepi/device1/rbx/odom")

        instance.set_nepi_navpose_topics_callback(None)
        pub = instance.set_frame_comp_topic_pub
        self.assertEqual(len(pub.published), 1)
        self.assertEqual(pub.published[0].name2, "orientation")

    # ------------------------------------------------------------
    # 5) cleanup_actions doesn't raise and still uses msg_if.pub_info
    #    (not a removed nepi_msg call).
    # ------------------------------------------------------------
    def test_cleanup_actions_does_not_raise(self):
        instance = self._build_instance()
        instance.cleanup_actions()
        self.assertTrue(any(level == "info" for level, _ in instance.msg_if.calls))


if __name__ == "__main__":
    unittest.main(verbosity=2)
