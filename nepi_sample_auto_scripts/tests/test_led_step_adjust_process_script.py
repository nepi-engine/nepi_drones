#!/usr/bin/env python3
#
# Mock-stub unit test for led_step_adjust_process_script.py
#
# This script's real runtime dependencies (rospy connected to a live ROS
# master, and the NEPI nepi_sdk / nepi_api packages built inside the NEPI
# device's catkin workspace) are not importable in this sandbox -- there is
# no roscore, and nepi_sdk/nepi_api/nepi_interfaces exist here only as
# unbuilt source trees (see nepi_engine_ws/nepi_sdk, /nepi_api symlinks),
# not as installed Python packages with generated message classes.
#
# So instead of importing the real stack, this test injects minimal stub
# modules into sys.modules for every external dependency the target script
# touches (rospy, nepi_sdk.nepi_ros, nepi_api.messages_if, std_msgs.msg),
# with stub classes/functions whose call signatures and field names match
# the current API exactly (per this session's confirmed API-change notes).
# The goal is to catch exactly the class of bug this session was full of:
# a renamed attribute, topic, or class slipping through unnoticed.
#
# What this test verifies:
#   1. The script module imports cleanly with no AttributeError/ImportError
#      against stand-ins for the CURRENT (post-rename) API surface.
#   2. Instantiating led_step_adjust() drives real __init__ logic: it calls
#      nepi_ros.init_node/get_node_name/get_base_namespace, constructs
#      MsgIF(log_name=...) (the nepi_msg module removal fix), waits for the
#      "lsx/set_intensity" topic, creates a Float32 Publisher, and arms a
#      rospy.Timer -- all without ever needing a real ROS master.
#   3. The registered timer callback (led_step_callback) is invoked directly
#      to exercise the step/wrap-around math and confirm it still publishes
#      through self.msg_if.pub_info (not the removed nepi_msg.publishMsgInfo)
#      and through self.led_intensity_pub.publish(data=...).
#   4. cleanup_actions() publishes a zero level on shutdown.

import importlib.util
import os
import sys
import types
import unittest


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_PATH = os.path.join(THIS_DIR, "..", "led_step_adjust_process_script.py")


# ---------------------------------------------------------------------------
# Stub: std_msgs.msg (Empty, Float32)
#
# Real std_msgs/Empty.msg has no fields.
# Real std_msgs/Float32.msg has a single field: float32 data
# ---------------------------------------------------------------------------
class FakeEmpty(object):
    __slots__ = ()

    def __init__(self, *args, **kwargs):
        pass


class FakeFloat32(object):
    def __init__(self, data=0.0):
        self.data = data


def _make_std_msgs_stub():
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Empty = FakeEmpty
    std_msgs_msg.Float32 = FakeFloat32
    std_msgs.msg = std_msgs_msg
    return std_msgs, std_msgs_msg


# ---------------------------------------------------------------------------
# Stub: rospy
#
# Only the surface actually used by led_step_adjust_process_script.py:
#   rospy.Publisher(topic, msg_class, queue_size=...)
#   rospy.Timer(rospy.Duration(secs), callback)
#   rospy.Duration(secs)
#   rospy.spin()
#   rospy.is_shutdown()
# ---------------------------------------------------------------------------
class FakePublisher(object):
    """Records every publish() call; does not require a ROS master."""

    def __init__(self, topic, data_class, queue_size=1):
        self.topic = topic
        self.data_class = data_class
        self.queue_size = queue_size
        self.published = []  # list of dict of kwargs passed to publish()

    def publish(self, *args, **kwargs):
        # Script always calls publish(data=<float>), but accept args too.
        if args and not kwargs:
            kwargs = {"data": args[0]}
        self.published.append(dict(kwargs))


class FakeDuration(object):
    def __init__(self, secs=0.0):
        self.secs = secs


class FakeTimerHandle(object):
    def __init__(self, duration, callback):
        self.duration = duration
        self.callback = callback
        self.shutdown_called = False

    def shutdown(self):
        self.shutdown_called = True


class FakeTimerEvent(object):
    """Stand-in for the rospy.TimerEvent passed into timer callbacks."""

    def __init__(self):
        self.last_expected = None
        self.last_real = None
        self.current_expected = None
        self.current_real = None


def _make_rospy_stub():
    rospy = types.ModuleType("rospy")
    rospy._timers = []  # test hook: every FakeTimerHandle created
    rospy._shutdown = False

    def Publisher(topic, data_class, queue_size=1, **kwargs):
        return FakePublisher(topic, data_class, queue_size=queue_size)

    def Timer(duration, callback, **kwargs):
        handle = FakeTimerHandle(duration, callback)
        rospy._timers.append(handle)
        return handle

    def Duration(secs=0.0, *args, **kwargs):
        return FakeDuration(secs)

    def spin():
        # In real rospy this blocks forever; for the test it must return
        # immediately so __init__ completes and we can drive the timer
        # callback ourselves.
        return None

    def is_shutdown():
        return rospy._shutdown

    rospy.Publisher = Publisher
    rospy.Timer = Timer
    rospy.Duration = Duration
    rospy.spin = spin
    rospy.is_shutdown = is_shutdown
    rospy.TimerEvent = FakeTimerEvent
    return rospy


# ---------------------------------------------------------------------------
# Stub: nepi_sdk.nepi_ros
#
# Signatures matched to the CURRENT (confirmed-unchanged) nepi_ros.py:
#   init_node(name, disable_signals=False)
#   get_node_name()
#   get_base_namespace()
#   wait_for_topic(topic_name, timeout=60, log_name_list=[], topics_list=None,
#                   types_list=None)
# ---------------------------------------------------------------------------
def _make_nepi_sdk_stub(record):
    nepi_sdk_pkg = types.ModuleType("nepi_sdk")
    nepi_ros_mod = types.ModuleType("nepi_sdk.nepi_ros")

    def init_node(name, disable_signals=False):
        record["init_node_calls"].append(
            {"name": name, "disable_signals": disable_signals}
        )

    def get_node_name():
        return "led_step_adjust"

    def get_base_namespace():
        return "/nepi/device1/"

    def wait_for_topic(topic_name, timeout=60, log_name_list=None,
                        topics_list=None, types_list=None):
        record["wait_for_topic_calls"].append(topic_name)
        # Simulate resolution to a fully-qualified topic name, as the real
        # implementation does.
        base = get_base_namespace().rstrip("/")
        return base + "/" + topic_name

    nepi_ros_mod.init_node = init_node
    nepi_ros_mod.get_node_name = get_node_name
    nepi_ros_mod.get_base_namespace = get_base_namespace
    nepi_ros_mod.wait_for_topic = wait_for_topic

    nepi_sdk_pkg.nepi_ros = nepi_ros_mod
    return nepi_sdk_pkg, nepi_ros_mod


# ---------------------------------------------------------------------------
# Stub: nepi_api.messages_if.MsgIF
#
# Real current MsgIF signature: MsgIF(log_name=None), with pub_info/
# pub_warn/pub_debug/pub_error(msg, throttle_s=None, log_name_list=[]).
# This replaces the removed nepi_msg.createMsgPublishers/publishMsgInfo.
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
    so we cannot just `import led_step_adjust_process_script`)."""

    std_msgs_pkg, std_msgs_msg_mod = _make_std_msgs_stub()
    rospy_mod = _make_rospy_stub()
    nepi_sdk_pkg, nepi_ros_mod = _make_nepi_sdk_stub(record)
    nepi_api_pkg, messages_if_mod = _make_nepi_api_stub(record)

    stub_modules = {
        "std_msgs": std_msgs_pkg,
        "std_msgs.msg": std_msgs_msg_mod,
        "rospy": rospy_mod,
        "nepi_sdk": nepi_sdk_pkg,
        "nepi_sdk.nepi_ros": nepi_ros_mod,
        "nepi_api": nepi_api_pkg,
        "nepi_api.messages_if": messages_if_mod,
    }

    saved = {}
    for name, mod in stub_modules.items():
        saved[name] = sys.modules.get(name)
        sys.modules[name] = mod

    try:
        spec = importlib.util.spec_from_file_location(
            "led_step_adjust_process_script_under_test", SCRIPT_PATH
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        # Leave the stubs installed for the duration of the test process --
        # tests instantiate the class after loading and need the same stub
        # instances (e.g. rospy._timers) to still be in sys.modules-backed
        # objects. We only need to guarantee we don't leak into unrelated
        # test files if this suite grows; restoring here would break the
        # freshly-loaded module's closures over the stub module objects
        # (which is fine either way since it holds direct references), so
        # for simplicity/isolation we restore prior state now.
        for name, prior in saved.items():
            if prior is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prior

    return module, rospy_mod


class TestLedStepAdjustProcessScript(unittest.TestCase):

    def setUp(self):
        self.record = {
            "init_node_calls": [],
            "wait_for_topic_calls": [],
            "msg_if_instances": [],
            "pub_info": [],
            "pub_warn": [],
            "pub_debug": [],
            "pub_error": [],
        }
        self.module, self.rospy_mod = _load_target_module(self.record)

    def test_module_imports_cleanly(self):
        self.assertTrue(hasattr(self.module, "led_step_adjust"))

    def test_init_waits_for_current_lsx_intensity_topic(self):
        node = self.module.led_step_adjust()
        self.assertEqual(
            self.record["wait_for_topic_calls"], ["lsx/set_intensity"]
        )
        # Publisher was created against the resolved (namespaced) topic.
        self.assertEqual(
            node.led_intensity_pub.topic,
            "/nepi/device1/lsx/set_intensity",
        )
        self.assertIs(node.led_intensity_pub.data_class, self.module.Float32)

    def test_init_uses_msgif_not_removed_nepi_msg_module(self):
        node = self.module.led_step_adjust()
        # Confirms the nepi_msg -> MsgIF fix: exactly one MsgIF was built,
        # logging under the node's name, and info messages flowed through
        # pub_info (never through a nonexistent nepi_msg.publishMsgInfo).
        self.assertEqual(len(self.record["msg_if_instances"]), 1)
        self.assertIs(node.msg_if, self.record["msg_if_instances"][0])
        self.assertEqual(node.msg_if.log_name, "led_step_adjust")
        self.assertGreater(len(self.record["pub_info"]), 0)
        self.assertTrue(
            any("Initialization Complete" in m for m in self.record["pub_info"])
        )

    def test_timer_registered_with_configured_period(self):
        node = self.module.led_step_adjust()
        self.assertEqual(len(self.rospy_mod._timers), 1)
        timer = self.rospy_mod._timers[0]
        self.assertEqual(timer.duration.secs, self.module.LED_STEP_SEC)
        self.assertEqual(timer.callback, node.led_step_callback)

    def test_step_callback_increments_and_publishes_float32_data_field(self):
        node = self.module.led_step_adjust()
        node.led_last_level = 0.0

        node.led_step_callback(self.rospy_mod._timers[0])

        self.assertAlmostEqual(node.led_last_level, self.module.LED_LEVEL_STEP)
        last_pub = node.led_intensity_pub.published[-1]
        self.assertIn("data", last_pub)
        self.assertAlmostEqual(last_pub["data"], self.module.LED_LEVEL_STEP)

    def test_step_callback_wraps_around_at_max_level(self):
        node = self.module.led_step_adjust()
        # Push last_level right up against the configured max so the next
        # tick should wrap back to 0.0 rather than exceed LED_LEVEL_MAX.
        node.led_last_level = node.led_level_max
        node.led_step_callback(self.rospy_mod._timers[0])

        self.assertEqual(node.led_last_level, 0.0)
        last_pub = node.led_intensity_pub.published[-1]
        self.assertEqual(last_pub["data"], 0.0)

    def test_step_callback_never_exceeds_configured_max(self):
        node = self.module.led_step_adjust()
        node.led_last_level = 0.0
        levels = []
        # Run enough ticks to wrap at least once and confirm the ceiling
        # is respected on every single tick.
        steps = int(node.led_level_max / node.led_level_step) + 3
        for _ in range(steps):
            node.led_step_callback(self.rospy_mod._timers[0])
            levels.append(node.led_last_level)

        for lvl in levels:
            self.assertLessEqual(lvl, node.led_level_max)
        self.assertIn(0.0, levels)  # confirms at least one wrap happened

    def test_cleanup_actions_publishes_zero_level(self):
        node = self.module.led_step_adjust()
        node.cleanup_actions()
        last_pub = node.led_intensity_pub.published[-1]
        self.assertEqual(last_pub["data"], 0)


if __name__ == "__main__":
    unittest.main()
