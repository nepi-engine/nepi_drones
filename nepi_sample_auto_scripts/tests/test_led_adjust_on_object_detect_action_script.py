#!/usr/bin/env python3
#
# Mock-stub unit test for led_adjust_on_object_detect_action_script.py.
#
# WHY A MOCK-STUB TEST (not a real rospy/nepi_sdk import test):
# This machine has no built catkin workspace (no devel/setup.bash). Real ROS
# Noetic IS installed here (rospy, std_msgs, sensor_msgs import fine), but
# `nepi_sdk.nepi_ros` fails to import (ModuleNotFoundError: no module named
# 'rospy_message_converter', a dependency normally provided by the NEPI
# device's catkin install) and `nepi_interfaces.msg` is an empty namespace
# package with no generated message classes at all (produced by catkin at
# build time; nothing has been built here). So this test stubs rospy,
# nepi_sdk.nepi_ros, nepi_api.messages_if.MsgIF, and nepi_interfaces.msg in
# sys.modules before importing the script, and uses the REAL std_msgs /
# sensor_msgs packages (genuinely importable here) unstubbed.
#
# RE-WRITTEN (2026-08-06) to match the script's real 2026-08-06 fix: the
# previous version of this test stubbed the third-party darknet_ros_msgs
# package (which never existed in this workspace) purely to make the
# script's then-broken import succeed. The script has since been re-ported
# to subscribe to the real current engine's aggregated
# <base_namespace>/bounding_boxes (nepi_interfaces/AiBoundingBoxes) instead
# -- see the script's own module docstring. This test now stubs
# AiBoundingBoxes/BoundingBox with their real current field names (confirmed
# against src/nepi_interfaces/msg/AiBoundingBoxes.msg and BoundingBox.msg)
# instead of darknet_ros_msgs, and there is no more separate
# found_object_callback/ObjectCount to test -- that logic folded into
# object_detected_callback's own once-per-message lost_count debounce.
#
# The point of exercising the real __init__ path (rather than hand-building
# a partial instance) is to catch exactly the class of bug this project's
# API-drift fixes are about: a renamed field, topic, class, or call
# signature slipping through unnoticed.
#
# IMPORTANT DESIGN NOTE: the stub modules are installed into sys.modules
# exactly ONCE (in setUpClass), before the script module is imported via
# importlib. Python binds `import rospy` / `from nepi_sdk import nepi_ros`
# etc. into the *script's own module namespace* at import time -- so the
# script keeps referencing the very same stub module/class objects for its
# whole lifetime regardless of what sys.modules holds afterward. Per-test
# configuration (queueing wait_for_topic/find_topic resolution, resetting the
# fake rospy's shutdown flag) therefore reads those same objects back off the
# imported script module (self.module.rospy, self.module.nepi_ros, ...)
# rather than creating fresh stub instances the script would never see.
#
# Run directly with:
#   python3 -m unittest tests.test_led_adjust_on_object_detect_action_script -v
# (from the nepi_sample_auto_scripts/ directory)

import importlib.util
import os
import sys
import types
import unittest


SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_PATH = os.path.join(SCRIPT_DIR, "led_adjust_on_object_detect_action_script.py")


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
            self.published.append(kwargs.get("data", kwargs))


class _FakeSubscriber:
    def __init__(self, topic, msg_class, callback, queue_size=1):
        self.topic = topic
        self.msg_class = msg_class
        self.callback = callback
        self.queue_size = queue_size


class _FakeTimer:
    def __init__(self, duration, callback):
        self.duration = duration
        self.callback = callback


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
# nepi_interfaces.msg stub -- real current field names, confirmed against
# src/nepi_interfaces/msg/BoundingBox.msg and AiBoundingBoxes.msg:
#   BoundingBox: Class, id, uid, probability, xmin, ymin, xmax, ymax,
#                area_ratio, area_pixels
#   AiBoundingBoxes: model_name, detect_timestamp, image_topic,
#                     image_timestamp, image_height, image_width,
#                     prc_height, prc_width, bounding_boxes, localizations
# Only the fields this script actually reads are populated by the test
# helper below; the rest default to None/empty.
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
    nepi_interfaces.msg modules into sys.modules and return the previous
    sys.modules entries so the caller can restore them afterward. Called
    exactly once, in setUpClass. std_msgs / sensor_msgs are NOT stubbed --
    they are real, genuinely-importable ROS Noetic packages on this machine.
    """
    rospy_stub = types.ModuleType("rospy")
    rospy_stub.Publisher = _FakePublisher
    rospy_stub.Subscriber = _FakeSubscriber
    rospy_stub.Timer = _FakeTimer
    rospy_stub.Duration = lambda secs: secs
    rospy_stub.spin = lambda: None  # must NOT block the test
    rospy_stub.is_shutdown = lambda: False
    rospy_stub.signal_shutdown = lambda reason=None: None
    rospy_stub.init_node = lambda *a, **k: None
    rospy_stub.get_name = lambda: "/led_adjust_on_object_detect"
    rospy_stub.loginfo = lambda *a, **k: None
    rospy_stub.logwarn = lambda *a, **k: None
    rospy_stub.logerr = lambda *a, **k: None
    rospy_stub.logdebug = lambda *a, **k: None

    # nepi_sdk / nepi_sdk.nepi_ros stub -- signatures match the current
    # nepi_ros.py: init_node(name, disable_signals=False), get_node_name(),
    # get_base_namespace(), wait_for_topic(topic_name, ...),
    # find_topic(topic_name, ...). This stub resolves any non-empty topic
    # name deterministically (as if a matching LSX driver/detector were
    # attached) so __init__'s has_intensity/has_blink branches (and
    # therefore the AI bounding-boxes subscriber/timer wiring nested under
    # them) can be exercised in this test.
    nepi_sdk_pkg = types.ModuleType("nepi_sdk")
    nepi_ros_stub = types.ModuleType("nepi_sdk.nepi_ros")
    nepi_ros_stub.init_node = lambda name, disable_signals=False: None
    nepi_ros_stub.get_node_name = lambda: "led_adjust_on_object_detect"
    nepi_ros_stub.get_base_namespace = lambda: "/nepi/device1/"

    def _resolve_topic(topic_name, timeout=60, **kwargs):
        if topic_name.startswith("/"):
            return topic_name
        return "/nepi/device1/" + topic_name.lstrip("/")

    nepi_ros_stub.wait_for_topic = _resolve_topic
    nepi_ros_stub.find_topic = _resolve_topic
    nepi_sdk_pkg.nepi_ros = nepi_ros_stub

    # nepi_api / nepi_api.messages_if.MsgIF stub -- records every
    # pub_info/pub_warn/pub_debug/pub_error call so tests can assert the
    # script uses ONLY this (current) logging convention and never falls
    # back to the removed nepi_msg.publishMsgInfo(self, msg) style.
    nepi_api_pkg = types.ModuleType("nepi_api")
    messages_if_stub = types.ModuleType("nepi_api.messages_if")
    messages_if_stub.MsgIF = _FakeMsgIF
    nepi_api_pkg.messages_if = messages_if_stub

    # nepi_interfaces.msg stub -- real current AiBoundingBoxes/BoundingBox
    # field names, see class docstrings above.
    nepi_interfaces_pkg = types.ModuleType("nepi_interfaces")
    nepi_interfaces_msg_stub = types.ModuleType("nepi_interfaces.msg")
    nepi_interfaces_msg_stub.AiBoundingBoxes = _AiBoundingBoxes
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
    return previous


def _restore_modules(previous):
    for name, mod in previous.items():
        if mod is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = mod


class TestLedAdjustOnObjectDetectActionScript(unittest.TestCase):
    """Imports led_adjust_on_object_detect_action_script.py against stub
    modules for the modules unavailable/broken in this environment
    (rospy/nepi_sdk.nepi_ros/nepi_api.messages_if/nepi_interfaces.msg),
    drives the real __init__ path with both LED intensity and blink control
    "attached", and exercises the object-detection/watchdog timer callbacks
    plus cleanup to confirm no broken attribute/field references remain.
    """

    @classmethod
    def setUpClass(cls):
        previous = _install_stub_modules()
        try:
            spec = importlib.util.spec_from_file_location(
                "led_adjust_on_object_detect_action_script_under_test", SCRIPT_PATH
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
        cls.AiBoundingBoxes = cls.module.AiBoundingBoxes
        cls.BoundingBox = _BoundingBox

    def setUp(self):
        self.rospy_stub.is_shutdown = lambda: False
        self.rospy_stub.signal_shutdown = lambda reason=None: None

    # ------------------------------------------------------------
    # 1) Clean import + no accidental regression to removed/renamed API.
    # ------------------------------------------------------------
    def test_module_imports_cleanly(self):
        self.assertTrue(hasattr(self.module, "led_adjust_on_object_detect"))
        self.assertEqual(self.module.LED_LEVEL_MAX, 0.3)
        self.assertEqual(self.module.OBJECT_LABEL_OF_INTEREST, "bottle")

    def test_source_has_no_removed_or_stale_api(self):
        with open(SCRIPT_PATH) as f:
            src = f.read()
        self.assertNotIn("from nepi_sdk import nepi_msg", src)
        self.assertNotIn("nepi_msg.publishMsgInfo(self", src)
        self.assertNotIn("nepi_msg.publishMsgWarn(self", src)
        self.assertNotIn("createMsgPublishers", src)
        self.assertNotIn("nepi_ros_interfaces", src)
        # "darknet_ros_msgs" legitimately appears in the module docstring's
        # historical explanation of the fix -- check for the actual import,
        # not the bare substring.
        self.assertNotIn("import darknet_ros_msgs", src)
        self.assertNotIn("from darknet_ros_msgs", src)
        self.assertNotIn("def found_object_callback", src)
        self.assertIn("MsgIF", src)
        self.assertIn("self.msg_if = MsgIF(log_name = self.node_name)", src)
        self.assertIn("AiBoundingBoxes", src)

    # ------------------------------------------------------------
    # Helper: build a fully-initialized instance with both LED intensity
    # and blink control "attached" (find_topic resolves both), so __init__
    # walks all the way through publisher construction, the AI bounding-
    # boxes subscriber wiring, and the watchdog timer registration without
    # blocking (rospy.spin is a stubbed no-op).
    # ------------------------------------------------------------
    def _build_instance(self):
        return self.module.led_adjust_on_object_detect()

    def _bbox_msg(self, img_width, img_height, boxes):
        return self.AiBoundingBoxes(
            image_width=img_width,
            image_height=img_height,
            bounding_boxes=boxes,
        )

    # ------------------------------------------------------------
    # 2) Full __init__ path runs against the stubbed current API with no
    #    AttributeError/TypeError, wiring up every LED publisher and
    #    driving the initial LED-off state via the current MsgIF-based
    #    logging convention.
    # ------------------------------------------------------------
    def test_init_runs_against_stubbed_current_api_and_wires_led_publishers(self):
        instance = self._build_instance()

        # msg_if constructed via current MsgIF(log_name=...) API.
        self.assertEqual(instance.msg_if.log_name, "led_adjust_on_object_detect")
        self.assertTrue(any(level == "info" for level, _ in instance.msg_if.calls))

        self.assertTrue(instance.has_intensity)
        self.assertTrue(instance.has_blink)
        self.assertIsNotNone(instance.led_on_off_pub)
        self.assertIsNotNone(instance.led_intensity_pub)
        self.assertIsNotNone(instance.led_blink_on_off_pub)
        self.assertIsNotNone(instance.led_blink_interval_pub)

        # Initial LED-off state published during __init__.
        self.assertEqual(instance.led_intensity_pub.published[-1], 0)
        self.assertEqual(instance.led_blink_on_off_pub.published[-1], False)
        self.assertEqual(instance.led_on_off_pub.published[-1], True)

        # AI bounding-boxes subscriber wiring reached.
        self.assertFalse(instance.object_detected)

    # ------------------------------------------------------------
    # 3) object_detected_callback: box of interest exactly centered
    #    horizontally computes the expected max-intensity value and arms
    #    blinking, using the real BoundingBox field names
    #    (Class/xmin/xmax/ymin/ymax) and AiBoundingBoxes
    #    (image_width/image_height/bounding_boxes) this script reads.
    # ------------------------------------------------------------
    def test_object_detected_callback_centered_box_sets_max_intensity_and_blink(self):
        instance = self._build_instance()

        # img_width=200: xmin=90/xmax=110 -> x-center=100 -> x_ratio=0.5
        # (perfectly centered) -> box_abs_error_x_ratio=0 ->
        # mean_center_ratio=1.0 -> intensity = LED_LEVEL_MAX * 1**4 = 0.3
        # -> intensity_history (AVG_LENGTH=2) rolled to [0.3, 0], mean=0.15.
        # mean_center_ratio(1.0) > LED_BLINK_THRESHOLD(0.5) -> blink armed.
        box = self.BoundingBox(Class="bottle", xmin=90, xmax=110, ymin=40, ymax=60)
        msg = self._bbox_msg(img_width=200, img_height=100, boxes=[box])

        instance.object_detected_callback(msg)

        self.assertTrue(instance.object_detected)
        self.assertEqual(instance.lost_count, 0)
        self.assertAlmostEqual(instance.set_intensity, 0.15)
        self.assertEqual(instance.set_blink_interval, self.module.LED_BLINK_RATE)
        self.assertEqual(instance.img_width, 200)
        self.assertEqual(instance.img_height, 100)

    def test_object_detected_callback_ignores_non_matching_label(self):
        instance = self._build_instance()

        box = self.BoundingBox(Class="chair", xmin=0, xmax=10, ymin=0, ymax=10)
        msg = self._bbox_msg(img_width=200, img_height=100, boxes=[box])

        instance.object_detected_callback(msg)

        # A single miss doesn't clear an object that wasn't detected yet --
        # only increments lost_count (see next test for the debounce path).
        self.assertFalse(instance.object_detected)
        self.assertEqual(instance.lost_count, 1)

    # ------------------------------------------------------------
    # 4) object_detected_callback debounce: once self.object_detected is
    #    True, a handful of consecutive misses (up to LOST_COUNT_THRESHOLD)
    #    must NOT immediately clear it -- only exceeding the threshold does.
    #    This also guards the real bug fixed in this re-port: the old code
    #    incremented lost_count once per non-matching box in a single
    #    frame, not once per frame.
    # ------------------------------------------------------------
    def test_object_detected_callback_sustains_detection_across_a_few_misses(self):
        instance = self._build_instance()
        box = self.BoundingBox(Class="bottle", xmin=90, xmax=110, ymin=40, ymax=60)
        instance.object_detected_callback(self._bbox_msg(200, 100, [box]))
        self.assertTrue(instance.object_detected)

        # A frame with several non-matching boxes must only count as ONE
        # miss, not one per box.
        other_boxes = [
            self.BoundingBox(Class="chair", xmin=0, xmax=10, ymin=0, ymax=10),
            self.BoundingBox(Class="table", xmin=20, xmax=30, ymin=0, ymax=10),
            self.BoundingBox(Class="lamp", xmin=40, xmax=50, ymin=0, ymax=10),
        ]
        instance.object_detected_callback(self._bbox_msg(200, 100, other_boxes))
        self.assertEqual(instance.lost_count, 1)
        self.assertTrue(instance.object_detected, "one missed frame must not clear detection yet")

    def test_object_detected_callback_clears_after_threshold_exceeded(self):
        instance = self._build_instance()
        box = self.BoundingBox(Class="bottle", xmin=90, xmax=110, ymin=40, ymax=60)
        instance.object_detected_callback(self._bbox_msg(200, 100, [box]))
        self.assertTrue(instance.object_detected)

        empty_msg = self._bbox_msg(200, 100, [])
        for _ in range(self.module.LOST_COUNT_THRESHOLD + 1):
            instance.object_detected_callback(empty_msg)

        self.assertFalse(instance.object_detected)

    # ------------------------------------------------------------
    # 5) led_timer_callback: publishes intensity + arms blink when an
    #    object is currently detected and the watchdog hasn't timed out.
    # ------------------------------------------------------------
    def test_led_timer_callback_publishes_intensity_and_arms_blink_when_detected(self):
        instance = self._build_instance()
        instance.object_detected = True
        instance.set_intensity = 0.15
        instance.set_blink_interval = self.module.LED_BLINK_RATE
        instance.wd_timer = 0
        instance.is_blinking = False

        instance.led_timer_callback(None)

        self.assertEqual(instance.wd_timer, instance.wd_check_interval_sec)
        self.assertAlmostEqual(instance.led_intensity_pub.published[-1], 0.15)
        self.assertEqual(instance.led_blink_on_off_pub.published[-1], True)
        self.assertAlmostEqual(
            instance.led_blink_interval_pub.published[-1], self.module.LED_BLINK_RATE
        )
        self.assertTrue(instance.is_blinking)

    # ------------------------------------------------------------
    # 6) led_timer_callback: past the watchdog timeout, LEDs are forced off
    #    regardless of detection state.
    # ------------------------------------------------------------
    def test_led_timer_callback_turns_leds_off_past_watchdog_timeout(self):
        instance = self._build_instance()
        instance.object_detected = True
        instance.is_blinking = True
        instance.wd_timer = self.module.WATCHDOG_TIME + 1

        instance.led_timer_callback(None)

        self.assertEqual(instance.led_intensity_pub.published[-1], 0)
        self.assertEqual(instance.led_blink_on_off_pub.published[-1], False)
        self.assertEqual(instance.led_on_off_pub.published[-1], False)
        self.assertFalse(instance.is_blinking)

    # ------------------------------------------------------------
    # 7) cleanup_actions() publishes a zero intensity level without raising.
    # ------------------------------------------------------------
    def test_cleanup_actions_publishes_zero_intensity(self):
        instance = self._build_instance()
        instance.led_intensity_pub.published.clear()

        instance.cleanup_actions()

        self.assertEqual(instance.led_intensity_pub.published[-1], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
