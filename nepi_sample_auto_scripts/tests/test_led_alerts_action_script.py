#!/usr/bin/env python3
#
# Mock-stub unit test for led_alerts_action_script.py.
#
# WHY A MOCK-STUB TEST (not a real rospy/nepi_sdk import test):
# This machine has no built catkin workspace (no devel/setup.bash) and the
# `nepi_sdk` / `nepi_interfaces` "packages" importable from the repo root are
# just namespace-package symlinks into source trees -- `nepi_sdk.nepi_ros`
# itself fails to import here (ModuleNotFoundError: rospy_message_converter,
# a dependency normally provided by the NEPI device's catkin install), and
# `nepi_interfaces.msg`/`nepi_interfaces.srv` have no generated message/service
# classes at all (they're produced by catkin at build time, and nothing has
# been built here). rospy and std_msgs ARE genuinely importable here (real
# `/opt/ros/noetic` packages), so this test uses real std_msgs where it
# matters and stubs out only the four modules that are unavailable/broken in
# this environment: rospy (would otherwise try to contact a ROS master),
# nepi_sdk.nepi_ros, nepi_api.messages_if.MsgIF, and nepi_interfaces.msg/.srv
# (DeviceLSXStatus, LSXCapabilitiesQuery / LSXCapabilitiesQueryResponse).
#
# The point of exercising the real __init__ path (rather than hand-building
# a partial instance) is to catch exactly the class of bug this session's
# API-drift fixes were about: a renamed field, topic, class, or call
# signature slipping through unnoticed. The stub message classes below carry
# EXACTLY the current field lists read directly from
# src/nepi_interfaces/msg/DeviceLSXStatus.msg and
# src/nepi_interfaces/srv/LSXCapabilitiesQuery.srv this session -- if the
# script referenced a field name that no longer exists on either message,
# these stubs (real classes with only the real fields, not a permissive
# Mock/dict) would raise AttributeError here exactly as the real message
# would.
#
# IMPORTANT DESIGN NOTE: the stub modules are installed into sys.modules
# exactly ONCE (in setUpClass), before the script module is imported via
# importlib. Python binds `import rospy` / `from nepi_interfaces.msg import
# DeviceLSXStatus` etc. into the *script's own module namespace* at import
# time -- so the script keeps referencing the very same stub module/class
# objects for its whole lifetime regardless of what sys.modules holds
# afterward. All per-test configuration (queueing a capabilities response,
# temporarily wrapping Subscriber so it auto-fires a synthetic status
# message) therefore reads those same objects back off the imported script
# module (self.module.rospy, self.module.DeviceLSXStatus, ...) rather than
# creating fresh stub instances that the script would never see.
#
# RE-WRITTEN (2026-08-06) to match the script's real 2026-08-06 fix: the
# previous version of this test exercised alert_stateCb, fed by a
# std_msgs/Bool topic (app_ai_alerts/alert_state) that never existed
# anywhere in this workspace and blocked __init__ forever waiting for it.
# The script now derives the same True/False "alert" signal from the real
# current AI detection output (<base_namespace>/bounding_boxes,
# nepi_interfaces/AiBoundingBoxes) instead -- see the script's own module
# docstring, and led_adjust_on_object_detect_action_script.py's matching
# fix/test for the same underlying mechanism. This test now exercises
# boundingBoxesCb instead of alert_stateCb, with a BoundingBox/AiBoundingBoxes
# stub matching the real current field names (confirmed against
# src/nepi_interfaces/msg/BoundingBox.msg and AiBoundingBoxes.msg).
#
# Run directly with:
#   python3 -m unittest tests.test_led_alerts_action_script -v
# (from the nepi_sample_auto_scripts/ directory)

import importlib.util
import os
import sys
import types
import unittest

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_PATH = os.path.join(SCRIPT_DIR, "led_alerts_action_script.py")


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
            self.published.append(kwargs)


class _FakeSubscriber:
    def __init__(self, topic, msg_class, callback, queue_size=1):
        self.topic = topic
        self.msg_class = msg_class
        self.callback = callback
        self.queue_size = queue_size


class _FakeServiceProxy:
    """Records the service topic/type it was constructed against and, when
    called, returns whatever response `_next_response` (set by the test)
    holds -- or raises whatever `_next_exception` holds, to exercise the
    script's try/except capabilities-query fallback path. Class-level state
    is intentional: the script only ever sees ONE ServiceProxy class object
    (bound at its own import time), so tests configure that same class.
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

    def pub_info(self, msg, **kwargs):
        self.calls.append(("info", msg))

    def pub_warn(self, msg, **kwargs):
        self.calls.append(("warn", msg))

    def pub_debug(self, msg, **kwargs):
        self.calls.append(("debug", msg))

    def pub_error(self, msg, **kwargs):
        self.calls.append(("error", msg))


# ---------------------------------------------------------------
# nepi_interfaces.msg.DeviceLSXStatus stub -- field list copied verbatim
# from src/nepi_interfaces/msg/DeviceLSXStatus.msg.
# ---------------------------------------------------------------
class _DeviceLSXStatus:
    __slots__ = (
        "device_name", "device_path", "device_node_name",
        "serial_num", "hw_version", "sw_version",
        "data_source_description", "data_ref_description",
        "settings_topic", "navpose_topic", "save_data_topic",
        "data_products", "data_product_topics", "data_product_image_topics",
        "standby_state", "on_off_state", "blink_state", "strobe_state",
        "blink_interval", "intensity_ratio", "color_setting",
        "kelvin_setting", "temp_c", "power_w",
    )

    def __init__(self, **kwargs):
        for field in self.__slots__:
            setattr(self, field, kwargs.get(field))


# LSXCapabilitiesQuery.srv has no request fields, only a response section --
# model that as an empty service-type placeholder plus a response class
# carrying exactly the response field list from the .srv file.
class _LSXCapabilitiesQueryResponse:
    __slots__ = (
        "device_name", "device_path", "device_node_name",
        "has_standby_mode", "has_on_off_control", "has_intensity_control",
        "has_color_control", "color_options_list", "has_kelvin_control",
        "kelvin_min", "kelvin_max", "has_blink_control",
        "has_hw_strobe", "reports_temperature", "reports_power",
    )

    def __init__(self, **kwargs):
        for field in self.__slots__:
            setattr(self, field, kwargs.get(field))


class _LSXCapabilitiesQuery:
    """Service *type* placeholder -- the script only ever passes this class
    to rospy.ServiceProxy(topic, LSXCapabilitiesQuery); it never constructs
    a request directly (the service has no request fields), matching the
    real generated service class shape.
    """
    pass


# ---------------------------------------------------------------
# nepi_interfaces.msg stub -- BoundingBox/AiBoundingBoxes, real current
# field names confirmed against src/nepi_interfaces/msg/BoundingBox.msg and
# AiBoundingBoxes.msg (same shape used by
# led_adjust_on_object_detect_action_script.py's own re-port/test).
# ---------------------------------------------------------------
class _BoundingBox:
    __slots__ = ("Class", "id", "uid", "probability", "xmin", "ymin", "xmax",
                 "ymax", "area_ratio", "area_pixels")

    def __init__(self, **kwargs):
        for field in self.__slots__:
            setattr(self, field, kwargs.get(field))


class _AiBoundingBoxes:
    __slots__ = ("model_name", "detect_timestamp", "image_topic",
                 "image_timestamp", "image_height", "image_width",
                 "prc_height", "prc_width", "bounding_boxes", "localizations")

    def __init__(self, **kwargs):
        for field in self.__slots__:
            setattr(self, field, kwargs.get(field))
        if self.bounding_boxes is None:
            self.bounding_boxes = []


def _install_stub_modules():
    """Inject stub rospy / nepi_sdk.nepi_ros / nepi_api.messages_if /
    nepi_interfaces.msg / nepi_interfaces.srv modules into sys.modules and
    return the previous sys.modules entries so the caller can restore them
    afterward. Called exactly once, in setUpClass.
    """
    rospy_stub = types.ModuleType("rospy")
    rospy_stub.Publisher = _FakePublisher
    rospy_stub.Subscriber = _FakeSubscriber
    rospy_stub.ServiceProxy = _FakeServiceProxy
    rospy_stub.spin = lambda: None  # must NOT block the test
    rospy_stub.is_shutdown = lambda: False
    rospy_stub.signal_shutdown = lambda reason=None: None
    rospy_stub.init_node = lambda *a, **k: None
    rospy_stub.get_name = lambda: "/led_alert_actions"
    rospy_stub.loginfo = lambda *a, **k: None
    rospy_stub.logwarn = lambda *a, **k: None
    rospy_stub.logerr = lambda *a, **k: None
    rospy_stub.logdebug = lambda *a, **k: None

    # nepi_sdk / nepi_sdk.nepi_ros stub -- signatures match the current
    # nepi_ros.py: init_node(name, disable_signals=False), get_node_name(),
    # get_base_namespace(), wait_for_topic(topic_name, ...).
    nepi_sdk_pkg = types.ModuleType("nepi_sdk")
    nepi_ros_stub = types.ModuleType("nepi_sdk.nepi_ros")
    nepi_ros_stub.init_node = lambda name, disable_signals=False: None
    nepi_ros_stub.get_node_name = lambda: "led_alert_actions"
    nepi_ros_stub.get_base_namespace = lambda: "/nepi/device1/"

    def _wait_for_topic(topic_name, timeout=60, log_name_list=None,
                         topics_list=None, types_list=None):
        # Real nepi_ros.wait_for_topic() polls the live ROS graph for a
        # topic whose name *contains* topic_name and returns the fully
        # resolved topic string. Simulate that resolution deterministically
        # (this test only ever runs against nonexistent live infrastructure).
        if topic_name.startswith("/"):
            return topic_name
        return "/nepi/device1/" + topic_name.lstrip("/")

    nepi_ros_stub.wait_for_topic = _wait_for_topic
    nepi_sdk_pkg.nepi_ros = nepi_ros_stub

    # nepi_api / nepi_api.messages_if.MsgIF stub -- records every
    # pub_info/pub_warn/pub_debug/pub_error call so tests can assert the
    # script uses ONLY this (current) logging convention and never falls
    # back to the removed nepi_msg.publishMsgInfo(self, msg) style.
    nepi_api_pkg = types.ModuleType("nepi_api")
    messages_if_stub = types.ModuleType("nepi_api.messages_if")
    messages_if_stub.MsgIF = _FakeMsgIF
    nepi_api_pkg.messages_if = messages_if_stub

    nepi_interfaces_pkg = types.ModuleType("nepi_interfaces")
    msg_stub = types.ModuleType("nepi_interfaces.msg")
    srv_stub = types.ModuleType("nepi_interfaces.srv")
    msg_stub.DeviceLSXStatus = _DeviceLSXStatus
    msg_stub.AiBoundingBoxes = _AiBoundingBoxes
    srv_stub.LSXCapabilitiesQuery = _LSXCapabilitiesQuery
    srv_stub.LSXCapabilitiesQueryResponse = _LSXCapabilitiesQueryResponse
    nepi_interfaces_pkg.msg = msg_stub
    nepi_interfaces_pkg.srv = srv_stub

    stub_modules = {
        "rospy": rospy_stub,
        "nepi_sdk": nepi_sdk_pkg,
        "nepi_sdk.nepi_ros": nepi_ros_stub,
        "nepi_api": nepi_api_pkg,
        "nepi_api.messages_if": messages_if_stub,
        "nepi_interfaces": nepi_interfaces_pkg,
        "nepi_interfaces.msg": msg_stub,
        "nepi_interfaces.srv": srv_stub,
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


class TestLedAlertsActionScript(unittest.TestCase):
    """Imports led_alerts_action_script.py against stub modules for the
    modules unavailable in this environment (rospy/nepi_sdk/nepi_api/
    nepi_interfaces), drives the real __init__ path with a full-capabilities
    LED status/capabilities response, and exercises the status/alert
    callbacks and cleanup to confirm no broken attribute/field references
    remain post-API-drift-fix.

    The "alert" signal itself now comes from boundingBoxesCb (fed by
    AiBoundingBoxes messages) rather than the old, never-existent
    app_ai_alerts app -- see module docstring above.
    """

    @classmethod
    def setUpClass(cls):
        previous = _install_stub_modules()
        try:
            spec = importlib.util.spec_from_file_location(
                "led_alerts_action_script_under_test", SCRIPT_PATH
            )
            cls.module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = cls.module
            spec.loader.exec_module(cls.module)
        finally:
            _restore_modules(previous)

        # These are the SAME objects the script bound at its own import
        # time (see module docstring's "IMPORTANT DESIGN NOTE") -- grab
        # them here so every test configures/monkeypatches the objects the
        # script actually uses, not a throwaway freshly-stubbed copy.
        cls.rospy_stub = cls.module.rospy
        cls.real_subscriber_ctor = cls.module.rospy.Subscriber
        cls.DeviceLSXStatus = cls.module.DeviceLSXStatus
        cls.LSXCapabilitiesQueryResponse = cls.module.LSXCapabilitiesQueryResponse
        cls.AiBoundingBoxes = cls.module.AiBoundingBoxes
        cls.BoundingBox = _BoundingBox

    def setUp(self):
        # Reset shared class-level stub state before every test so tests
        # don't leak configuration into each other.
        _FakeServiceProxy._next_response = None
        _FakeServiceProxy._next_exception = None
        self.rospy_stub.Subscriber = self.real_subscriber_ctor
        self.rospy_stub.signal_shutdown = lambda reason=None: None

    # ------------------------------------------------------------
    # 1) Clean import + no accidental regression to removed/renamed API.
    # ------------------------------------------------------------
    def test_module_imports_cleanly(self):
        self.assertTrue(hasattr(self.module, "led_alert_actions"))
        self.assertEqual(self.module.LED_LEVEL_MAX, 0.3)
        self.assertEqual(self.module.LED_STATUS_TOPIC_NAME, "lsx/status")

    def test_source_has_no_removed_or_stale_api(self):
        with open(SCRIPT_PATH) as f:
            src = f.read()
        # NOTE: "nepi_msg" / "nepi_ros_interfaces" as bare substrings
        # legitimately appear in the module docstring's changelog note
        # (documenting the OLD -> NEW rename, matching the documentation
        # style of the other rewritten scripts) -- so check for the actual
        # removed/renamed API *usages* (import statements, call sites)
        # rather than the substrings alone.
        self.assertNotIn("from nepi_sdk import nepi_msg", src)
        self.assertNotIn("publishMsgInfo", src)
        self.assertNotIn("publishMsgWarn", src)
        self.assertNotIn("createMsgPublishers", src)
        self.assertNotIn("from nepi_ros_interfaces", src)
        self.assertNotIn("from nepi_app_ai_alerts", src)
        # "app_ai_alerts" legitimately appears in the module docstring's
        # historical explanation of the fix -- check for the actual old
        # topic construction, not the bare substring.
        self.assertNotIn('"app_ai_alerts/alert_state"', src)
        self.assertIn("MsgIF", src)
        self.assertIn("DeviceLSXStatus", src)
        self.assertIn("AiBoundingBoxes", src)
        self.assertIn("OBJECT_LABEL_OF_INTEREST", src)
        self.assertIn("from nepi_interfaces.msg import", src)
        self.assertIn("from nepi_interfaces.srv import", src)

    # ------------------------------------------------------------
    # Helper: build a fully-initialized instance against a queued
    # capabilities response, auto-firing the LED status subscriber
    # callback with a synthetic DeviceLSXStatus so __init__'s
    # `while self.led_state == None` wait resolves immediately.
    # ------------------------------------------------------------
    def _build_instance(self, caps_response=None, caps_exception=None):
        _FakeServiceProxy._next_response = caps_response
        _FakeServiceProxy._next_exception = caps_exception

        # Deliberately DIFFERENT from START_STATE ([True,0.2,False,0.0,"GREEN"]):
        # updateLedState() only publishes when new_led_state != last_led_state,
        # and ledStatusCb() seeds last_led_state from this first status
        # message -- if it happened to equal START_STATE, __init__'s
        # updateLedState(START_STATE) call would be a no-op and the
        # publish-during-init assertions below would spuriously see nothing
        # published (a real bug this test hit and had to correct for).
        status_msg = self.DeviceLSXStatus(
            device_name="test_lsx",
            on_off_state=False,
            intensity_ratio=0.9,
            blink_state=True,
            blink_interval=2.0,
            color_setting="BLUE",
        )

        def _auto_firing_subscriber(topic, msg_class, callback, queue_size=1):
            sub = self.real_subscriber_ctor(topic, msg_class, callback, queue_size=queue_size)
            if msg_class is self.DeviceLSXStatus:
                callback(status_msg)
            return sub

        self.rospy_stub.Subscriber = _auto_firing_subscriber
        try:
            instance = self.module.led_alert_actions()
        finally:
            self.rospy_stub.Subscriber = self.real_subscriber_ctor
        return instance

    def _full_capabilities_response(self):
        return self.LSXCapabilitiesQueryResponse(
            device_name="test_lsx",
            device_path="",
            device_node_name="",
            has_standby_mode=True,
            has_on_off_control=True,
            has_intensity_control=True,
            has_color_control=True,
            color_options_list=["RED", "GREEN", "BLUE"],
            has_kelvin_control=False,
            kelvin_min=0,
            kelvin_max=1,
            has_blink_control=True,
            has_hw_strobe=False,
            reports_temperature=False,
            reports_power=False,
        )

    # ------------------------------------------------------------
    # 2) Full __init__ path runs against the stubbed current API with no
    #    AttributeError/TypeError, for a device that reports every LSX
    #    capability (on/off, intensity, blink, color) -- exercising every
    #    publisher-construction branch in one pass.
    # ------------------------------------------------------------
    def test_init_runs_against_stubbed_current_api_full_capabilities(self):
        instance = self._build_instance(caps_response=self._full_capabilities_response())

        # msg_if constructed via current MsgIF(log_name=...) API.
        self.assertEqual(instance.msg_if.log_name, "led_alert_actions")
        self.assertTrue(any(level == "info" for level, _ in instance.msg_if.calls))

        # Capabilities parsed correctly from the (stubbed) service response.
        self.assertTrue(instance.has_on_off_control)
        self.assertTrue(instance.has_intensity_control)
        self.assertTrue(instance.has_blink_control)
        self.assertTrue(instance.has_color_control)
        self.assertEqual(instance.color_options_list, ["RED", "GREEN", "BLUE"])

        # All publishers were constructed (every capability True).
        self.assertIsNotNone(instance.led_on_off_pub)
        self.assertIsNotNone(instance.led_intensity_pub)
        self.assertIsNotNone(instance.led_blink_on_off_pub)
        self.assertIsNotNone(instance.led_blink_interval_pub)
        self.assertIsNotNone(instance.led_color_pub)

        # START_STATE was published to every publisher during init.
        self.assertEqual(instance.led_on_off_pub.published[-1], True)
        self.assertAlmostEqual(instance.led_intensity_pub.published[-1], 0.2)
        self.assertEqual(instance.led_color_pub.published[-1], "GREEN")

        # ledStatusCb populated led_state/last_led_state from the real
        # DeviceLSXStatus field names (on_off_state, intensity_ratio,
        # blink_state, blink_interval, color_setting) -- reflects the
        # synthetic status message, not the just-published START_STATE
        # (led_state tracks reported device status, only updated by
        # ledStatusCb, not by updateLedState's publishes).
        self.assertEqual(instance.led_state, [False, 0.9, True, 2.0, "BLUE"])

    # ------------------------------------------------------------
    # 3) Capabilities-service exception path: confirms the except branch
    #    (all capabilities False, warn logged via current MsgIF.pub_warn)
    #    and the "no capabilities" shutdown branch are reached without
    #    raising.
    # ------------------------------------------------------------
    def test_init_capabilities_service_exception_falls_back_and_shuts_down(self):
        shutdown_reasons = []
        self.rospy_stub.signal_shutdown = lambda reason=None: shutdown_reasons.append(reason)

        instance = self._build_instance(caps_exception=RuntimeError("service unavailable"))

        # Except branch populated all-False capabilities via the current
        # MsgIF.pub_warn call (not the removed nepi_msg.publishMsgWarn).
        self.assertFalse(instance.has_on_off_control)
        self.assertFalse(instance.has_intensity_control)
        self.assertFalse(instance.has_color_control)
        self.assertFalse(instance.has_blink_control)
        self.assertTrue(any(level == "warn" for level, _ in instance.msg_if.calls))

        # No capabilities -> script calls rospy.signal_shutdown(...), and
        # (since that branch skips publisher construction) all LED
        # publishers remain unset.
        self.assertEqual(len(shutdown_reasons), 1)
        self.assertIsNone(instance.led_on_off_pub)

    # ------------------------------------------------------------
    # 4) boundingBoxesCb drives LED actions on True/False alert transitions,
    #    derived from whether OBJECT_LABEL_OF_INTEREST appears in the
    #    message -- using the real BoundingBox/AiBoundingBoxes field names
    #    (same source led_adjust_on_object_detect_action_script.py's own
    #    fix/test uses).
    # ------------------------------------------------------------
    def _bbox_msg(self, boxes):
        return self.AiBoundingBoxes(bounding_boxes=boxes)

    def test_bounding_boxes_callback_true_then_false_drives_led_updates(self):
        instance = self._build_instance(caps_response=self._full_capabilities_response())
        label = self.module.OBJECT_LABEL_OF_INTEREST

        matching_box = self.BoundingBox(Class=label, xmin=0, xmax=10, ymin=0, ymax=10)
        instance.boundingBoxesCb(self._bbox_msg([matching_box]))
        self.assertTrue(instance.alert_state)
        self.assertEqual(instance.led_on_off_pub.published[-1], True)
        self.assertAlmostEqual(instance.led_intensity_pub.published[-1], 0.2)
        self.assertEqual(instance.led_color_pub.published[-1], "RED")
        self.assertEqual(instance.led_blink_on_off_pub.published[-1], True)
        self.assertAlmostEqual(instance.led_blink_interval_pub.published[-1], 0.5)

        # Debounced over ALERT_LOST_COUNT_THRESHOLD consecutive misses --
        # exceed it so alert_state actually flips False.
        empty_msg = self._bbox_msg([])
        for _ in range(self.module.ALERT_LOST_COUNT_THRESHOLD + 1):
            instance.boundingBoxesCb(empty_msg)
        self.assertFalse(instance.alert_state)
        self.assertEqual(instance.led_on_off_pub.published[-1], True)
        self.assertAlmostEqual(instance.led_intensity_pub.published[-1], 0.4)
        self.assertEqual(instance.led_color_pub.published[-1], "GREEN")
        # ALERT_FALSE_ACTIONS blink_state is False, blink_time_sec is -999
        # (ignored) -- blink_on_off_pub still gets the False, but interval
        # pub is not re-published for this transition.
        self.assertEqual(instance.led_blink_on_off_pub.published[-1], False)

    def test_bounding_boxes_callback_sustains_alert_across_a_few_misses(self):
        instance = self._build_instance(caps_response=self._full_capabilities_response())
        label = self.module.OBJECT_LABEL_OF_INTEREST

        matching_box = self.BoundingBox(Class=label, xmin=0, xmax=10, ymin=0, ymax=10)
        instance.boundingBoxesCb(self._bbox_msg([matching_box]))
        self.assertTrue(instance.alert_state)

        # A single missed frame (below threshold) must not clear the alert.
        instance.boundingBoxesCb(self._bbox_msg([]))
        self.assertTrue(instance.alert_state)
        self.assertEqual(instance.lost_count, 1)

    def test_bounding_boxes_callback_ignores_non_matching_label(self):
        instance = self._build_instance(caps_response=self._full_capabilities_response())

        other_box = self.BoundingBox(Class="chair", xmin=0, xmax=10, ymin=0, ymax=10)
        instance.boundingBoxesCb(self._bbox_msg([other_box]))
        self.assertFalse(instance.alert_state)

    def test_led_status_callback_updates_state_from_real_field_names(self):
        instance = self._build_instance(caps_response=self._full_capabilities_response())

        status_msg = self.DeviceLSXStatus(
            on_off_state=False,
            intensity_ratio=0.6,
            blink_state=True,
            blink_interval=1.5,
            color_setting="BLUE",
        )
        instance.ledStatusCb(status_msg)
        self.assertEqual(instance.led_state, [False, 0.6, True, 1.5, "BLUE"])

    # ------------------------------------------------------------
    # 5) cleanup_actions() restores START_STATE without raising.
    # ------------------------------------------------------------
    def test_cleanup_actions_restores_start_state(self):
        instance = self._build_instance(caps_response=self._full_capabilities_response())

        label = self.module.OBJECT_LABEL_OF_INTEREST
        matching_box = self.BoundingBox(Class=label, xmin=0, xmax=10, ymin=0, ymax=10)
        instance.boundingBoxesCb(self._bbox_msg([matching_box]))  # drive away from START_STATE first

        instance.cleanup_actions()

        self.assertEqual(instance.led_on_off_pub.published[-1], True)
        self.assertAlmostEqual(instance.led_intensity_pub.published[-1], 0.2)
        self.assertEqual(instance.led_color_pub.published[-1], "GREEN")


if __name__ == "__main__":
    unittest.main(verbosity=2)
