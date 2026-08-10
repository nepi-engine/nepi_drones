#!/usr/bin/env python3
#
# Mock-stub unit test for opencv_image_contours_process_script.py.
#
# WHY A MOCK-STUB TEST (not a real rospy/nepi_sdk import test):
# This machine has no built catkin workspace (no devel/setup.bash). The
# `nepi_sdk` / `nepi_interfaces` "packages" importable from the repo root are
# just namespace-package symlinks into source trees:
#   - `nepi_sdk.nepi_ros` fails to import here (ModuleNotFoundError:
#     rospy_message_converter, a dependency normally provided by the NEPI
#     device's catkin install).
#   - `nepi_sdk.nepi_img` fails to import here too (ModuleNotFoundError:
#     colormath, used elsewhere in that module for color-space helpers this
#     script doesn't call).
#   - `nepi_interfaces.msg` has no generated message classes at all (they're
#     produced by catkin at build time, and nothing has been built) -- not
#     that this script uses any; it only touches sensor_msgs/std_msgs.
# rospy, cv_bridge, std_msgs, sensor_msgs, numpy and cv2 ARE genuinely
# importable here (real `/opt/ros/noetic` packages / pip packages), so this
# test uses the REAL message classes and REAL cv_bridge/cv2 for the
# image-processing path, and only stubs out the three modules that are
# unavailable/broken in this environment: rospy (would otherwise try to
# contact a ROS master), nepi_sdk.nepi_ros, and nepi_api.messages_if.MsgIF.
#
# The nepi_img stub below is not a loose fake -- its four functions
# (rosimg_to_cv2img, cv2img_to_rosimg, get_contours, overlay_contours) are
# copied verbatim from nepi_sdk/nepi_img.py (confirmed unchanged against the
# current API this session), using the same real CvBridge/cv2 calls, so the
# actual image pipeline behavior is exercised, not just call-signature shape.
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
#   python3 -m unittest tests.test_opencv_image_contours_process_script -v
# (from the nepi_sample_auto_scripts/ directory)

import copy
import importlib.util
import os
import sys
import types
import unittest

import cv2
import numpy as np

from std_msgs.msg import UInt8, Empty, String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_PATH = os.path.join(SCRIPT_DIR, "opencv_image_contours_process_script.py")


def _install_stub_modules():
    """Inject stub rospy / nepi_sdk.nepi_ros / nepi_sdk.nepi_img /
    nepi_api.messages_if modules into sys.modules and return the previous
    sys.modules entries (or None) so the caller can restore them afterward.
    """

    # ---------------------------------------------------------------
    # rospy stub -- only the surface opencv_image_contours_process_script.py
    # uses: Publisher, Subscriber, spin, is_shutdown, init_node.
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

    class _FakeSubscriber:
        def __init__(self, topic, msg_class, callback, queue_size=1):
            self.topic = topic
            self.msg_class = msg_class
            self.callback = callback
            self.queue_size = queue_size

    rospy_stub.Publisher = _FakePublisher
    rospy_stub.Subscriber = _FakeSubscriber
    rospy_stub.spin = lambda: None  # must NOT block the test
    rospy_stub.is_shutdown = lambda: False
    rospy_stub.signal_shutdown = lambda reason=None: None
    rospy_stub.init_node = lambda *a, **k: None
    rospy_stub.get_name = lambda: "/opencv_image_contours"
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
    nepi_ros_stub.get_node_name = lambda: "opencv_image_contours"
    nepi_ros_stub.get_base_namespace = lambda: "/nepi/device1/"

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
    # nepi_sdk.nepi_img stub -- the four functions this script calls,
    # copied verbatim (same real CvBridge/cv2 calls) from nepi_sdk/nepi_img.py
    # (confirmed unchanged against the current API), so the actual image
    # pipeline logic is genuinely exercised rather than faked away.
    # ---------------------------------------------------------------
    nepi_img_stub = types.ModuleType("nepi_sdk.nepi_img")
    _bridge = CvBridge()

    def _rosimg_to_cv2img(ros_img_msg, encoding='passthrough'):
        return _bridge.imgmsg_to_cv2(ros_img_msg, desired_encoding=encoding)

    def _cv2img_to_rosimg(cv2_img, encoding="bgr8"):
        return _bridge.cv2_to_imgmsg(cv2_img, encoding=encoding)

    def _get_contours(cv2_img):
        cv2_mat_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        ret, thresh2 = cv2.threshold(cv2_mat_gray, 150, 255, cv2.THRESH_BINARY)
        contours3, hierarchy3 = cv2.findContours(
            thresh2, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        return contours3, hierarchy3

    def _overlay_contours(cv2_img, contours3, color_rgb=(0, 255, 0)):
        cv2_img_out = copy.deepcopy(cv2_img)
        cv2.drawContours(cv2_img_out, contours3, -1, color_rgb, 2, cv2.LINE_AA)
        return cv2_img_out

    nepi_img_stub.rosimg_to_cv2img = _rosimg_to_cv2img
    nepi_img_stub.cv2img_to_rosimg = _cv2img_to_rosimg
    nepi_img_stub.get_contours = _get_contours
    nepi_img_stub.overlay_contours = _overlay_contours
    nepi_sdk_pkg.nepi_img = nepi_img_stub

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
        "nepi_sdk.nepi_img": nepi_img_stub,
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


def _make_test_image(bridge, color_bgr=(255, 255, 255), height=40, width=40,
                      draw_black_square=False):
    """Build a real sensor_msgs/Image (bgr8) via the genuine cv_bridge, so
    image_custom_callback is exercised against exactly what a live camera
    driver would publish. Optionally draws a black square so get_contours()
    (threshold @150) has a real edge to find.
    """
    cv_img = np.zeros((height, width, 3), dtype=np.uint8)
    cv_img[:, :] = color_bgr
    if draw_black_square:
        cv_img[10:30, 10:30] = (0, 0, 0)
    return bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")


class TestOpencvImageContoursProcessScript(unittest.TestCase):
    """Imports opencv_image_contours_process_script.py against stub modules
    for the modules unavailable in this environment (rospy/nepi_sdk/
    nepi_api), drives the real __init__ path, and exercises the image
    callback and cleanup callback to confirm no broken attribute/field
    references remain post-API-drift-fix.
    """

    @classmethod
    def setUpClass(cls):
        previous = _install_stub_modules()
        try:
            spec = importlib.util.spec_from_file_location(
                "opencv_image_contours_process_script_under_test", SCRIPT_PATH
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
        self.assertTrue(hasattr(self.module, "opencv_image_contours"))
        self.assertEqual(self.module.IMAGE_INPUT_TOPIC_NAME, "color_2d_image")

    def test_source_has_no_removed_nepi_msg_api(self):
        with open(SCRIPT_PATH) as f:
            src = f.read()
        # The module docstring legitimately *mentions* "nepi_msg" as prose
        # describing the fix that was applied ("nepi_msg module ->
        # nepi_api.messages_if.MsgIF"). What must be gone is any actual
        # usage of the removed module/API.
        self.assertNotIn("from nepi_sdk import nepi_msg", src)
        self.assertNotIn("nepi_msg.", src)
        self.assertNotIn("publishMsgInfo", src)
        self.assertNotIn("createMsgPublishers", src)
        self.assertIn("MsgIF", src)

    # ------------------------------------------------------------
    # 2) Full __init__ path runs against the stubbed current API with
    #    no AttributeError/TypeError (topic-wait, publisher/subscriber
    #    setup, MsgIF construction/logging).
    # ------------------------------------------------------------
    def _build_instance(self):
        previous = _install_stub_modules()
        try:
            instance = self.module.opencv_image_contours()
        finally:
            _restore_modules(previous)
        return instance

    def test_init_runs_against_stubbed_current_api(self):
        instance = self._build_instance()

        # msg_if was constructed via the current MsgIF(log_name=...) API
        # and used for logging (not the removed nepi_msg module).
        self.assertEqual(instance.msg_if.log_name, "opencv_image_contours")
        self.assertTrue(len(instance.msg_if.calls) > 0)
        self.assertTrue(any(level == "info" for level, _ in instance.msg_if.calls))

        # The contour-image publisher was created against the resolved
        # base-namespace topic name, still publishing sensor_msgs/Image
        # (confirmed this topic layout did NOT change in the API-drift audit).
        self.assertEqual(instance.contour_image_pub.topic,
                          "/nepi/device1/image_contours")
        self.assertIs(instance.contour_image_pub.msg_class, Image)

        self.instance = instance

    # ------------------------------------------------------------
    # 3) Image callback: real cv_bridge + real sensor_msgs/Image, driven
    #    through the actual contour-detect / overlay / republish pipeline.
    # ------------------------------------------------------------
    def test_image_custom_callback_publishes_contour_image(self):
        instance = self._build_instance()
        img_msg = _make_test_image(self.bridge, draw_black_square=True)
        self.assertIsInstance(img_msg, Image)

        instance.image_custom_callback(img_msg)

        self.assertEqual(len(instance.contour_image_pub.published), 1)
        out_msg = instance.contour_image_pub.published[-1]
        self.assertIsInstance(out_msg, Image)
        self.assertEqual(out_msg.encoding, "bgr8")

        # Round-trip back to cv2 and confirm the drawn contour (green
        # rectangle outline, BGR (0,255,0)) actually made it into the
        # republished image -- i.e. get_contours()/overlay_contours()
        # genuinely ran end-to-end and were not silently no-ops.
        out_cv2 = self.bridge.imgmsg_to_cv2(out_msg, desired_encoding="bgr8")
        green_pixel_present = np.any(
            (out_cv2[:, :, 0] == 0) & (out_cv2[:, :, 1] == 255) & (out_cv2[:, :, 2] == 0)
        )
        self.assertTrue(green_pixel_present)

    def test_image_custom_callback_solid_image_no_contours(self):
        instance = self._build_instance()
        # A solid all-white image has no threshold edge, so get_contours()
        # should find nothing -- exercise that this doesn't raise even when
        # the contours list is empty.
        img_msg = _make_test_image(self.bridge, color_bgr=(255, 255, 255))

        instance.image_custom_callback(img_msg)

        self.assertEqual(len(instance.contour_image_pub.published), 1)
        out_msg = instance.contour_image_pub.published[-1]
        self.assertIsInstance(out_msg, Image)

    # ------------------------------------------------------------
    # 4) Cleanup callback doesn't raise.
    # ------------------------------------------------------------
    def test_cleanup_actions_does_not_raise(self):
        instance = self._build_instance()
        instance.cleanup_actions()


if __name__ == "__main__":
    unittest.main(verbosity=2)
