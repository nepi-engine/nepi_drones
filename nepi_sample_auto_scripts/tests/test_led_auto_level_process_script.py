#!/usr/bin/env python3
#
# Mock-stub unit test for led_auto_level_process_script.py.
#
# WHY A MOCK-STUB TEST (not a real rospy/nepi_sdk import test):
# This machine has no built catkin workspace (no devel/setup.bash) and the
# `nepi_sdk` / `nepi_interfaces` "packages" importable from the repo root are
# just namespace-package symlinks into source trees -- `nepi_sdk.nepi_ros`
# itself fails to import here (ModuleNotFoundError: rospy_message_converter,
# a dependency normally provided by the NEPI device's catkin install), and
# `nepi_interfaces.msg` has no generated message classes at all (they're
# produced by catkin at build time, and nothing has been built). rospy,
# cv_bridge, std_msgs, sensor_msgs, numpy and cv2 ARE genuinely importable
# here (real `/opt/ros/noetic` packages / pip packages), so this test uses
# the REAL message classes and the REAL cv_bridge for the image-processing
# path, and only stubs out the three modules that are unavailable/broken in
# this environment: rospy (would otherwise try to contact a ROS master),
# nepi_sdk.nepi_ros, and nepi_api.messages_if.MsgIF.
#
# The point of exercising the real __init__ path (rather than hand-building
# a partial instance) is to catch exactly the class of bug this session's
# API-drift fixes were about: a renamed field, topic, class, or call
# signature slipping through unnoticed. The stub signatures below are typed
# to match the CURRENT nepi_sdk.nepi_ros / nepi_api.messages_if.MsgIF APIs
# (see nepi_engine_ws/src/nepi_engine/nepi_sdk/src/nepi_sdk/nepi_ros.py and
# .../nepi_api/src/nepi_api/messages_if.py) as read directly this session --
# if the script called an argument, attribute, or topic in a way the real
# current API no longer supports, these stubs are strict enough that the
# call would raise a TypeError/AttributeError here too.
#
# Run directly with:
#   python3 -m unittest tests.test_led_auto_level_process_script -v
# (from the nepi_sample_auto_scripts/ directory)

import importlib.util
import os
import sys
import types
import unittest

import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_PATH = os.path.join(SCRIPT_DIR, "led_auto_level_process_script.py")


def _install_stub_modules():
    """Inject stub rospy / nepi_sdk.nepi_ros / nepi_api.messages_if modules
    into sys.modules and return the previous sys.modules entries (or None)
    so the caller can restore them afterward.
    """

    # ---------------------------------------------------------------
    # rospy stub -- only the surface led_auto_level_process_script.py uses:
    # Publisher, Subscriber, Timer, Duration, spin, is_shutdown, init_node.
    # ---------------------------------------------------------------
    rospy_stub = types.ModuleType("rospy")

    class _FakePublisher:
        def __init__(self, topic, msg_class, queue_size=1):
            self.topic = topic
            self.msg_class = msg_class
            self.queue_size = queue_size
            self.published = []

        def publish(self, *args, **kwargs):
            # Script calls both styles historically; mirror real
            # rospy.Publisher.publish which accepts either a message
            # instance OR field kwargs used to construct one.
            if args:
                self.published.append(args[0])
            else:
                self.published.append(self.msg_class(**kwargs))

    class _FakeSubscriber:
        def __init__(self, topic, msg_class, callback, queue_size=1):
            self.topic = topic
            self.msg_class = msg_class
            self.callback = callback
            self.queue_size = queue_size

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
    rospy_stub.Subscriber = _FakeSubscriber
    rospy_stub.Timer = _FakeTimer
    rospy_stub.Duration = _FakeDuration
    rospy_stub.spin = lambda: None  # must NOT block the test
    rospy_stub.is_shutdown = lambda: False
    rospy_stub.signal_shutdown = lambda reason=None: None
    rospy_stub.init_node = lambda *a, **k: None
    rospy_stub.get_name = lambda: "/led_auto_level"
    rospy_stub.loginfo = lambda *a, **k: None
    rospy_stub.logwarn = lambda *a, **k: None
    rospy_stub.logerr = lambda *a, **k: None
    rospy_stub.logdebug = lambda *a, **k: None

    # ---------------------------------------------------------------
    # nepi_sdk / nepi_sdk.nepi_ros stub -- signatures match the current
    # nepi_ros.py: init_node(name, disable_signals=False),
    # get_node_name(), get_base_namespace(),
    # wait_for_topic(topic_name, timeout=60, log_name_list=[],
    #                topics_list=None, types_list=None).
    # ---------------------------------------------------------------
    nepi_sdk_pkg = types.ModuleType("nepi_sdk")
    nepi_ros_stub = types.ModuleType("nepi_sdk.nepi_ros")

    nepi_ros_stub.init_node = lambda name, disable_signals=False: None
    nepi_ros_stub.get_node_name = lambda: "led_auto_level"
    nepi_ros_stub.get_base_namespace = lambda: "/nepi/device1/"

    # A fixed set of topics standing in for what would actually be live on a
    # real device with an LSX light and a camera attached. This intentionally
    # mirrors the REAL current nepi_sdk.nepi_ros.find_topic() matching rule
    # (topic.find(topic_name) != -1 and topic.find(topic_name + "_") == -1 --
    # i.e. a candidate is excluded if it is immediately followed by another
    # "_"-joined word segment) rather than trivially prefixing whatever
    # topic_name the script passes in. A naive "always succeeds" stub would
    # silently pass even if the script asked for a topic name that could
    # never actually resolve on a real device (e.g. "lsx/set_intensity"
    # instead of the real "lsx/set_intensity_ratio") -- exactly the class of
    # bug this test needs to catch.
    _SIMULATED_LIVE_TOPICS = [
        "/nepi/device1/lsx/status",
        "/nepi/device1/lsx/set_intensity_ratio",
        "/nepi/device1/color_2d_image",
        "/nepi/device1/bw_2d_image",
    ]

    def _find_topic(topic_name, topics_list):
        for topic in topics_list:
            if topic.find(topic_name) != -1 and topic.find(topic_name + "_") == -1:
                return topic
        return ""

    def _wait_for_topic(topic_name, timeout=60, log_name_list=None,
                         topics_list=None, types_list=None):
        candidates = topics_list if topics_list is not None else _SIMULATED_LIVE_TOPICS
        return _find_topic(topic_name, candidates)

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

    stub_modules = {
        "rospy": rospy_stub,
        "nepi_sdk": nepi_sdk_pkg,
        "nepi_sdk.nepi_ros": nepi_ros_stub,
        "nepi_api": nepi_api_pkg,
        "nepi_api.messages_if": messages_if_stub,
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


def _make_test_image(bridge, color_bgr, height=40, width=40):
    """Build a real sensor_msgs/Image (bgr8) of a solid color via the
    genuine cv_bridge, so image_brightness_callback is exercised against
    exactly what a live camera driver would publish.
    """
    cv_img = np.zeros((height, width, 3), dtype=np.uint8)
    cv_img[:, :] = color_bgr
    return bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")


class TestLedAutoLevelProcessScript(unittest.TestCase):
    """Imports led_auto_level_process_script.py against stub modules for the
    modules unavailable in this environment (rospy/nepi_sdk/nepi_api), drives
    the real __init__ path, and exercises the image/timer/cleanup callbacks
    to confirm no broken attribute/field references remain post-API-drift-fix.
    """

    @classmethod
    def setUpClass(cls):
        previous = _install_stub_modules()
        try:
            spec = importlib.util.spec_from_file_location(
                "led_auto_level_process_script_under_test", SCRIPT_PATH
            )
            cls.module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = cls.module
            spec.loader.exec_module(cls.module)
        finally:
            _restore_modules(previous)
        cls.bridge = CvBridge()

    # ------------------------------------------------------------
    # 1) Clean import + no accidental regression to the removed API
    # ------------------------------------------------------------
    def test_module_imports_cleanly(self):
        self.assertTrue(hasattr(self.module, "led_auto_level"))
        self.assertEqual(self.module.LED_LEVEL_MAX, 0.3)
        self.assertEqual(self.module.AVG_LENGTH, 40)

    def test_source_has_no_removed_nepi_msg_api(self):
        with open(SCRIPT_PATH) as f:
            src = f.read()
        self.assertNotIn("nepi_msg", src)
        self.assertNotIn("publishMsgInfo", src)
        self.assertNotIn("createMsgPublishers", src)
        self.assertIn("MsgIF", src)

    # ------------------------------------------------------------
    # 2) Full __init__ path runs against the stubbed current API with
    #    no AttributeError/TypeError (topic-wait, publisher/subscriber/
    #    timer setup, MsgIF construction/logging).
    # ------------------------------------------------------------
    def test_init_runs_against_stubbed_current_api(self):
        previous = _install_stub_modules()
        try:
            instance = self.module.led_auto_level()
        finally:
            _restore_modules(previous)

        # msg_if was constructed via the current MsgIF(log_name=...) API
        # and used for logging (not the removed nepi_msg module).
        self.assertEqual(instance.msg_if.log_name, "led_auto_level")
        self.assertTrue(len(instance.msg_if.calls) > 0)
        self.assertTrue(any(level == "info" for level, _ in instance.msg_if.calls))

        # The LED control publisher was created against the resolved
        # topic name, still publishing plain std_msgs/Float32 (confirmed
        # this topic layout did NOT change in the API-drift audit). The
        # real current topic is "lsx/set_intensity_ratio" (confirmed against
        # nepi_api/device_if_lsx.py's SUBS_DICT) -- NOT "lsx/set_intensity",
        # which never resolves against a live device (see find_topic's
        # trailing-"_" exclusion rule replicated in _wait_for_topic above).
        self.assertEqual(instance.led_intensity_pub.topic,
                          "/nepi/device1/lsx/set_intensity_ratio")
        self.assertIs(instance.led_intensity_pub.msg_class, Float32)

        self.led_auto_level_instance = instance

    # ------------------------------------------------------------
    # 3) Image callback: real cv_bridge + real sensor_msgs/Image, driven
    #    through the actual brightness -> intensity -> publish pipeline.
    # ------------------------------------------------------------
    def _build_instance(self):
        previous = _install_stub_modules()
        try:
            instance = self.module.led_auto_level()
        finally:
            _restore_modules(previous)
        return instance

    def test_image_brightness_callback_dark_image_drives_led_up(self):
        instance = self._build_instance()
        dark_image_msg = _make_test_image(self.bridge, color_bgr=(0, 0, 0))
        self.assertIsInstance(dark_image_msg, Image)

        instance.image_brightness_callback(dark_image_msg)

        self.assertAlmostEqual(instance.img_brightness_ratio, 0.0, places=3)
        # intensity = led_level_max * (1 - ratio) -> led_level_max for this one
        # sample, but avg_intensity is the mean over an AVG_LENGTH-sample
        # rolling buffer (initialized to zeros), so after a single sample the
        # published average is led_level_max / AVG_LENGTH.
        expected_avg = instance.led_level_max / self.module.AVG_LENGTH
        self.assertAlmostEqual(instance.avg_intensity, expected_avg, places=4)

        # Published via the real Float32(data=...) field -- confirms
        # `data` is still the correct field name on this message class.
        self.assertGreaterEqual(len(instance.led_intensity_pub.published), 1)
        last_msg = instance.led_intensity_pub.published[-1]
        self.assertIsInstance(last_msg, Float32)
        self.assertAlmostEqual(last_msg.data, instance.avg_intensity, places=3)

    def test_image_brightness_callback_bright_image_drives_led_down(self):
        instance = self._build_instance()
        bright_image_msg = _make_test_image(self.bridge, color_bgr=(255, 255, 255))

        instance.image_brightness_callback(bright_image_msg)

        self.assertAlmostEqual(instance.img_brightness_ratio, 1.0, places=3)
        self.assertAlmostEqual(instance.avg_intensity, 0.0, places=3)

    def test_image_brightness_callback_averages_over_history(self):
        instance = self._build_instance()
        dark_image_msg = _make_test_image(self.bridge, color_bgr=(0, 0, 0))
        bright_image_msg = _make_test_image(self.bridge, color_bgr=(255, 255, 255))

        instance.image_brightness_callback(dark_image_msg)
        instance.image_brightness_callback(bright_image_msg)

        # Rolling history of AVG_LENGTH samples: after one dark (-> led_level_max)
        # then one bright (-> 0) sample, the mean should sit between the two.
        self.assertGreater(instance.avg_intensity, 0.0)
        self.assertLess(instance.avg_intensity, instance.led_level_max)

    # ------------------------------------------------------------
    # 4) brightness_ratio(): both the color (ndim==3) and grayscale
    #    (ndim==2) branches, exercised directly with numpy arrays.
    # ------------------------------------------------------------
    def test_brightness_ratio_color_branch(self):
        instance = self._build_instance()
        black = np.zeros((10, 10, 3), dtype=np.uint8)
        white = np.full((10, 10, 3), 255, dtype=np.uint8)
        self.assertAlmostEqual(instance.brightness_ratio(black), 0.0, places=3)
        self.assertAlmostEqual(instance.brightness_ratio(white), 1.0, places=3)

    def test_brightness_ratio_grayscale_branch(self):
        instance = self._build_instance()
        black = np.zeros((10, 10), dtype=np.uint8)
        white = np.full((10, 10), 255, dtype=np.uint8)
        self.assertAlmostEqual(instance.brightness_ratio(black), 0.0, places=3)
        self.assertAlmostEqual(instance.brightness_ratio(white), 2.0, places=3)

    # ------------------------------------------------------------
    # 5) Timer callback and cleanup callback don't raise, and cleanup
    #    still publishes via the correct Float32(data=...) field.
    # ------------------------------------------------------------
    def test_lxs_print_callback_does_not_raise(self):
        instance = self._build_instance()
        instance.lxs_print_callback(None)  # rospy.Timer passes a TimerEvent

    def test_cleanup_actions_publishes_zero(self):
        instance = self._build_instance()
        instance.cleanup_actions()
        last_msg = instance.led_intensity_pub.published[-1]
        self.assertIsInstance(last_msg, Float32)
        self.assertEqual(last_msg.data, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
