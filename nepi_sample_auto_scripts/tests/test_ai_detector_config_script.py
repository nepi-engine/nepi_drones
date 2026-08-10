#!/usr/bin/env python3
#
# Mock-stub unit test for ai_detector_config_script.py
#
# This script's real runtime dependencies (rospy connected to a live ROS
# master, and the NEPI nepi_sdk / nepi_api / nepi_interfaces packages built
# inside the NEPI device's catkin workspace) are not importable in this
# sandbox -- there is no roscore, and nepi_sdk/nepi_api/nepi_interfaces exist
# here only as unbuilt source trees, not as installed Python packages with
# generated message classes (confirmed: `python3 -c "import rospy"` fails
# with ModuleNotFoundError in this environment; no /opt/ros/noetic present).
#
# So instead of importing the real stack, this test injects minimal stub
# modules into sys.modules for every external dependency the target script
# touches (rospy, nepi_sdk.nepi_ros, nepi_api.messages_if, sensor_msgs.msg,
# std_msgs.msg, nepi_interfaces.msg), with stub classes/functions whose call
# signatures and field names match the current API exactly. The goal is to
# catch exactly the class of bug this project has been full of: a renamed
# attribute, topic, or class slipping through unnoticed.
#
# RE-WRITTEN (2026-08-06) to match the script's real 2026-08-06 fix: the
# previous version of this test asserted the script's ai_detector_mgr
# mechanism stayed fully commented-out/inert (no publishers, no
# wait_for_topic calls) -- that was correct for the code as it stood then,
# but the script has since been re-ported against the real current
# ai_models_mgr + nepi_api.ai_if_detector.AiDetectorIF mechanism (see the
# script's own module docstring). This test now verifies THAT real behavior
# instead: framework/model enable publishers, wait_for_node for the launched
# detection node, and the per-detector set_img_topic/set_threshold/enable
# publishers -- exercised via two scenarios (the model's node comes up vs.
# doesn't), matching how the real nepi_ros.wait_for_node behaves either way.

import importlib.util
import os
import sys
import types
import unittest


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_PATH = os.path.join(THIS_DIR, "..", "ai_detector_config_script.py")


# ---------------------------------------------------------------------------
# Stub: sensor_msgs.msg (Image) and std_msgs.msg (UInt8, Empty, String, Bool,
# Float32)
# ---------------------------------------------------------------------------
class FakeImage(object):
    def __init__(self, *args, **kwargs):
        pass


class FakeUInt8(object):
    def __init__(self, data=0):
        self.data = data


class FakeEmpty(object):
    __slots__ = ()

    def __init__(self, *args, **kwargs):
        pass


class FakeString(object):
    def __init__(self, data=""):
        self.data = data


class FakeBool(object):
    def __init__(self, data=False):
        self.data = data


class FakeFloat32(object):
    def __init__(self, data=0.0):
        self.data = data


def _make_sensor_msgs_stub():
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = FakeImage
    sensor_msgs.msg = sensor_msgs_msg
    return sensor_msgs, sensor_msgs_msg


def _make_std_msgs_stub():
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.UInt8 = FakeUInt8
    std_msgs_msg.Empty = FakeEmpty
    std_msgs_msg.String = FakeString
    std_msgs_msg.Bool = FakeBool
    std_msgs_msg.Float32 = FakeFloat32
    std_msgs.msg = std_msgs_msg
    return std_msgs, std_msgs_msg


# ---------------------------------------------------------------------------
# Stub: nepi_interfaces.msg (StringArray, UpdateBool)
#
# Real current UpdateBool.msg (src/nepi_interfaces/msg/UpdateBool.msg):
#   string name; string name2; string name3; bool value
# ---------------------------------------------------------------------------
class FakeStringArray(object):
    def __init__(self, array=None):
        self.array = array if array is not None else []


class FakeUpdateBool(object):
    def __init__(self, name="", name2="", name3="", value=False):
        self.name = name
        self.name2 = name2
        self.name3 = name3
        self.value = value


def _make_nepi_interfaces_stub():
    nepi_interfaces_pkg = types.ModuleType("nepi_interfaces")
    nepi_interfaces_msg = types.ModuleType("nepi_interfaces.msg")
    nepi_interfaces_msg.StringArray = FakeStringArray
    nepi_interfaces_msg.UpdateBool = FakeUpdateBool
    nepi_interfaces_pkg.msg = nepi_interfaces_msg
    return nepi_interfaces_pkg, nepi_interfaces_msg


# ---------------------------------------------------------------------------
# Stub: rospy
# ---------------------------------------------------------------------------
class FakePublisher(object):
    def __init__(self, topic, data_class, queue_size=1):
        self.topic = topic
        self.data_class = data_class
        self.queue_size = queue_size
        self.published = []

    def publish(self, *args, **kwargs):
        self.published.append((args, kwargs))


def _make_rospy_stub(record):
    rospy = types.ModuleType("rospy")
    rospy._publishers = []
    rospy._spin_calls = 0

    def Publisher(topic, data_class, queue_size=1, **kwargs):
        pub = FakePublisher(topic, data_class, queue_size=queue_size)
        rospy._publishers.append(pub)
        record["publisher_calls"].append(topic)
        return pub

    def spin():
        rospy._spin_calls += 1
        # Real rospy.spin() blocks forever; the stub must return immediately
        # so __init__ completes and the test can drive the instance further.
        return None

    def is_shutdown():
        return False

    rospy.Publisher = Publisher
    rospy.spin = spin
    rospy.is_shutdown = is_shutdown
    return rospy


# ---------------------------------------------------------------------------
# Stub: nepi_sdk.nepi_ros
#
# wait_for_node is new here -- the fixed script calls it to find the
# detection node ai_models_mgr launches. record["wait_for_node_result"]
# controls what it returns, so tests can exercise both the "node came up"
# and "node never came up" paths, matching real nepi_ros.wait_for_node's
# either-a-name-or-empty-string return contract.
# ---------------------------------------------------------------------------
def _make_nepi_sdk_stub(record):
    nepi_sdk_pkg = types.ModuleType("nepi_sdk")
    nepi_ros_mod = types.ModuleType("nepi_sdk.nepi_ros")

    def init_node(name, disable_signals=False):
        record["init_node_calls"].append(
            {"name": name, "disable_signals": disable_signals}
        )

    def get_node_name():
        return "ai_detector_config"

    def get_base_namespace():
        return "/nepi/device1/"

    def wait_for_topic(topic_name, timeout=60, log_name_list=None,
                        topics_list=None, types_list=None):
        record["wait_for_topic_calls"].append(topic_name)
        if record.get("wait_for_topic_result", "found") == "not_found":
            return ""
        base = get_base_namespace().rstrip("/")
        return base + "/" + topic_name

    def wait_for_node(node_name, timeout=60, log_name_list=None):
        record["wait_for_node_calls"].append(node_name)
        if record.get("wait_for_node_result") == "not_found":
            return ""
        base = get_base_namespace().rstrip("/")
        return base + "/" + node_name

    nepi_ros_mod.init_node = init_node
    nepi_ros_mod.get_node_name = get_node_name
    nepi_ros_mod.get_base_namespace = get_base_namespace
    nepi_ros_mod.wait_for_topic = wait_for_topic
    nepi_ros_mod.wait_for_node = wait_for_node

    nepi_sdk_pkg.nepi_ros = nepi_ros_mod
    return nepi_sdk_pkg, nepi_ros_mod


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
    so we cannot just `import ai_detector_config_script`)."""

    sensor_msgs_pkg, sensor_msgs_msg_mod = _make_sensor_msgs_stub()
    std_msgs_pkg, std_msgs_msg_mod = _make_std_msgs_stub()
    nepi_interfaces_pkg, nepi_interfaces_msg_mod = _make_nepi_interfaces_stub()
    rospy_mod = _make_rospy_stub(record)
    nepi_sdk_pkg, nepi_ros_mod = _make_nepi_sdk_stub(record)
    nepi_api_pkg, messages_if_mod = _make_nepi_api_stub(record)

    stub_modules = {
        "sensor_msgs": sensor_msgs_pkg,
        "sensor_msgs.msg": sensor_msgs_msg_mod,
        "std_msgs": std_msgs_pkg,
        "std_msgs.msg": std_msgs_msg_mod,
        "nepi_interfaces": nepi_interfaces_pkg,
        "nepi_interfaces.msg": nepi_interfaces_msg_mod,
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
            "ai_detector_config_script_under_test", SCRIPT_PATH
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        for name, prior in saved.items():
            if prior is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prior

    return module, rospy_mod


class TestAiDetectorConfigScript(unittest.TestCase):

    def setUp(self):
        self.record = {
            "init_node_calls": [],
            "wait_for_topic_calls": [],
            "wait_for_node_calls": [],
            "publisher_calls": [],
            "msg_if_instances": [],
            "pub_info": [],
            "pub_warn": [],
            "pub_debug": [],
            "pub_error": [],
        }
        self.module, self.rospy_mod = _load_target_module(self.record)

    def test_module_imports_cleanly(self):
        self.assertTrue(hasattr(self.module, "ai_detector_config"))
        self.assertFalse(hasattr(self.module, "ClassifierSelection"))

    def test_init_sets_up_node_name_and_namespace(self):
        node = self.module.ai_detector_config()
        self.assertEqual(len(self.record["init_node_calls"]), 1)
        self.assertEqual(
            self.record["init_node_calls"][0]["name"],
            self.module.ai_detector_config.DEFAULT_NODE_NAME,
        )
        self.assertEqual(node.node_name, "ai_detector_config")
        self.assertEqual(node.base_namespace, "/nepi/device1/")

    def test_init_uses_msgif_not_removed_nepi_msg_module(self):
        node = self.module.ai_detector_config()
        self.assertEqual(len(self.record["msg_if_instances"]), 1)
        self.assertIs(node.msg_if, self.record["msg_if_instances"][0])
        self.assertEqual(node.msg_if.log_name, "ai_detector_config")
        self.assertTrue(
            any("Initialization Complete" in m for m in self.record["pub_info"])
        )

    def test_init_enables_framework_and_model_via_ai_models_mgr(self):
        # The real current mechanism: update_framework_state then
        # update_model_state, both published to ai_models_mgr's namespace.
        node = self.module.ai_detector_config()
        topics = self.record["publisher_calls"]
        self.assertIn(
            "/nepi/device1/ai_models_mgr/update_framework_state", topics
        )
        self.assertIn(
            "/nepi/device1/ai_models_mgr/update_model_state", topics
        )
        fw_pub = next(
            p for p in self.rospy_mod._publishers
            if p.topic == "/nepi/device1/ai_models_mgr/update_framework_state"
        )
        model_pub = next(
            p for p in self.rospy_mod._publishers
            if p.topic == "/nepi/device1/ai_models_mgr/update_model_state"
        )
        fw_msg = fw_pub.published[0][0][0]
        self.assertEqual(fw_msg.name, self.module.AI_FRAMEWORK_NAME)
        self.assertEqual(fw_msg.value, True)
        model_msg = model_pub.published[0][0][0]
        self.assertEqual(model_msg.name, self.module.DETECTION_MODEL)
        self.assertEqual(model_msg.value, True)

    def test_init_configures_detector_when_node_comes_up(self):
        self.record["wait_for_node_result"] = "found"
        node = self.module.ai_detector_config()
        self.assertIn(self.module.DETECTION_MODEL, self.record["wait_for_node_calls"])
        self.assertIsNotNone(node.detector_namespace)
        topics = self.record["publisher_calls"]
        self.assertIn(node.detector_namespace + "set_img_topic", topics)
        self.assertIn(node.detector_namespace + "set_threshold", topics)
        self.assertIn(node.detector_namespace + "enable", topics)
        enable_pub = next(
            p for p in self.rospy_mod._publishers
            if p.topic == node.detector_namespace + "enable"
        )
        self.assertEqual(enable_pub.published[-1][0][0].data, True)

    def test_init_idles_safely_when_node_never_comes_up(self):
        self.record["wait_for_node_result"] = "not_found"
        node = self.module.ai_detector_config()
        self.assertIsNone(node.detector_namespace)
        self.assertTrue(
            any("did not come up" in m for m in self.record["pub_warn"])
        )

    def test_init_calls_rospy_spin_exactly_once(self):
        self.module.ai_detector_config()
        self.assertEqual(self.rospy_mod._spin_calls, 1)

    def test_cleanup_actions_does_not_raise_when_node_never_came_up(self):
        self.record["wait_for_node_result"] = "not_found"
        node = self.module.ai_detector_config()
        try:
            node.cleanup_actions()
        except Exception as exc:  # pragma: no cover - failure path
            self.fail(f"cleanup_actions() raised unexpectedly: {exc!r}")
        self.assertTrue(
            any("cleanup" in m.lower() for m in self.record["pub_info"])
        )

    def test_cleanup_actions_disables_detector_when_node_came_up(self):
        self.record["wait_for_node_result"] = "found"
        node = self.module.ai_detector_config()
        node.cleanup_actions()
        self.assertEqual(node.enable_pub.published[-1][0][0].data, False)


if __name__ == "__main__":
    unittest.main()
