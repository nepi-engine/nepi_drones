/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_rui) repo
# (see https://github.com/nepi-engine/nepi_rui)
#
# License: NEPI RUI repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
 */

// Per-script usage guides shown in the Scripts manager, keyed by exact
// filename as returned by get_scripts (nepi_interfaces/GetScriptsQuery).
// Hand-authored rather than parsed from each script's own header comment --
// those comments are developer-facing changelog/architecture notes, not
// end-user instructions, and several are wrong about the script actually
// working (see the knownIssue field on the two AiBoundingBoxes-based scripts
// below). A script with no entry here falls back to a generic "no guide yet"
// message in NepiMgrScripts.js rather than a broken lookup.
//
// testCommands topics are intentionally left as hints, not live topic paths
// -- the actual node namespace depends on what's actually running on this
// device, and the datalist in the Advanced Debugging publisher already lists
// every live topic to pick from. "Try It" only pre-fills messageType/json.
const ScriptDocs = {
  "ai_detector_config_script.py": {
    summary: "Turns on an AI detection model and points it at a camera image topic, so any script or panel that watches for detections has something to watch.",
    requires: [
      "An AI model actually installed on this device under the framework named by AI_FRAMEWORK_NAME (check the AI Models panel before running -- the sample DETECTION_MODEL value is a placeholder, not a real installed model)",
      "A live camera topic matching IMAGE_INPUT_TOPIC_NAME (default: any topic containing \"color_2d_image\")",
    ],
    requiredTopics: [
      // Both a sim/RBX-relayed camera ("color_2d_image") and a real IDX
      // camera driver's own topic ("idx/color_image") satisfy this --
      // confirmed against a real USB webcam (device_if_idx.py's own
      // convention), which doesn't match the sim/RBX name at all.
      {label: "Camera image topic", patterns: ["color_2d_image", "idx/color_image"]},
    ],
    testCommands: [
      {
        label: "Re-enable detection on the model's own node",
        topicHint: "<detector node>/enable",
        messageType: "std_msgs/Bool",
        json: "{\"data\": true}",
        notes: "Once running, the detection node's own name is the model's display_name. Useful if you toggled detection off and want it back without restarting the script.",
      },
      {
        label: "Adjust detection confidence threshold",
        topicHint: "<detector node>/set_threshold",
        messageType: "std_msgs/Float32",
        json: "{\"data\": 0.5}",
        notes: "Lower catches more (noisier); higher is stricter.",
      },
    ],
    tips: [
      "Check the AI Models panel to confirm the framework and model actually show as enabled/running -- if the model never launched, nothing downstream (LED scripts, mission scripts) will see detections.",
      "This script only starts detection; it doesn't stop when you stop it manually mid-run except via its own shutdown handler -- stop it from this Scripts panel rather than killing it externally.",
    ],
  },

  "drone_inspection_demo_mission_script.py": {
    summary: "Runs a full autonomous inspection mission on an ArduPilot-driven drone: takes off, flies to a set of waypoints, takes a snapshot action at each, then returns home.",
    requires: [
      "The ArduPilot RBX driver running and connected (real hardware, or SITL/Gazebo -- see nepi_drones for sim setup)",
      "GOTO_LOCATION / GOTO_LOCATION_CORNERS / HOME_LOCATION at the top of the script edited to match your actual test site before running for real",
      "(Optional) a snapshot action script running to actually do something at each waypoint",
    ],
    requiredTopics: [
      {label: "ArduPilot RBX driver", patterns: ["rbx/status"]},
    ],
    // Whichever live vehicle is actually connected -- SITL/Gazebo on the dev
    // VM or a real physical drone -- publishes its own camera under
    // "<device_name>/color_2d_image" (see rbx_ardupilot_node.py). A plain
    // substring match finds whichever one is actually live with no
    // SITL-vs-hardware special-casing, so the embedded feed below just
    // follows whatever vehicle is really connected.
    liveFeedTopicPattern: "color_2d_image",
    testCommands: [
      {
        label: "Test the RBX driver directly (bypassing the mission script)",
        topicHint: "<rbx node>/rbx/goto_location",
        messageType: "nepi_interfaces/GotoLocation",
        json: "{\"lat\": 47.6541208, \"long\": -122.3186620, \"altitude_m\": 10.0, \"yaw_deg\": -999}",
        notes: "Confirms the drone itself responds to a goto before trusting the full mission sequence. -999 for yaw keeps the current heading.",
      },
    ],
    tips: [
      "Watch this script's own Messages panel above, and the RBX device panel's status, for cmd_success on each leg -- a mission that silently sits at one waypoint usually means a goto's error tolerance was never satisfied.",
      "TAKEOFF_HEIGHT_M and the fake-GPS/home settings are all edited in the script file itself, not from this RUI -- there's no live settings UI for this script.",
    ],
  },

  "drone_follow_object_mission_script.py": {
    summary: "Watches for a specific AI-detected object class and commands the drone to loiter, take a snapshot, then continue watching -- a simple detect-and-react mission.",
    requires: [
      "The ArduPilot RBX driver running and connected (real hardware, or SITL/Gazebo -- see nepi_drones for sim setup)",
      "ai_detector_config_script.py already running and detecting",
      "A target-localization source publishing range/azimuth/elevation for the detected object -- " +
        "on real hardware this means an actual app_ai_targeting-shaped app (none exists in this " +
        "workspace yet). In SITL/Gazebo, it's a THREE-piece chain: (1) sitl_gazebo/sitl_gazebo_full " +
        "running on the dev VM, (2) ai_targeting_controller_ardupilot.py ALSO running on the dev VM, " +
        "which produces the raw feed on VM port 9027, and (3) sim_ai_targeting_bridge_script.py " +
        "running as a NEPI script on THIS device, which relays that raw feed into the real " +
        "target_localizations topic below. Use the \"Auto-Start Sim Requirements\" button below " +
        "instead of starting these by hand -- it launches (3), which in turn reaches over the " +
        "existing tunnel and triggers sitl_gazebo_full for (1)+(2) if they aren't already running.",
    ],
    requiredTopics: [
      {label: "ArduPilot RBX driver", patterns: ["rbx/status"]},
      {label: "Target-localization feed", patterns: ["target_localizations"]},
    ],
    // See drone_inspection_demo_mission_script.py's comment above -- same
    // per-vehicle camera convention, SITL or physical, whichever is live.
    liveFeedTopicPattern: "color_2d_image",
    // Launches sim_ai_targeting_bridge_script.py directly from this guide via
    // the same startLaunchScriptService the main Start button uses. That
    // script itself (see its own entry below) checks for the RBX driver and,
    // if missing, triggers sitl_gazebo_full on the dev VM over the existing
    // tunnel -- so this one button brings up all three pieces above, unless
    // the VM/tunnel has never been started at all this session (nothing
    // there yet to receive the trigger -- that one first step still has to
    // happen by hand, on the VM).
    autoStartScript: "sim_ai_targeting_bridge_script.py",
    knownIssue: "No real target-localization app exists in this workspace yet -- this script's expected input topics have no producer on real hardware today. It has only been verified against a Gazebo/SITL stand-in bridge (see the script's own header comment for the exact sim tooling). Treat this one as sim-only until a real targeting app exists.",
    testCommands: [],
    tips: [
      "If you're testing this in Gazebo/SITL, use the sim stand-in tools referenced in the script's header rather than trying to hand-publish a target message here -- the message shape those tools produce is the only one currently confirmed to work.",
      "The Target-localization feed checklist item above only turns green once sim_ai_targeting_bridge_script.py is actually running as a script -- the \"Auto-Start Sim Requirements\" button below (or starting that script yourself from the Scripts list) is what actually satisfies it, not opening Gazebo/SITL alone.",
      "If \"Auto-Start Sim Requirements\" doesn't bring the RBX driver up within 30-60s, the dev VM/tunnel probably isn't reachable at all -- run sitl_gazebo or sitl_gazebo_full manually on the VM once, after which this button will work normally.",
    ],
  },

  "led_alerts_action_script.py": {
    summary: "Flips a light on/off as a simple True/False alert whenever a chosen object is detected.",
    requires: [
      "A running LSX light driver",
      "A model running and detecting via ai_detector_config_script.py, publishing to <base_namespace>/all/detections",
    ],
    requiredTopics: [
      {label: "LSX light driver", patterns: ["lsx/status"]},
      {label: "AI detections topic", patterns: ["all/detections"]},
    ],
    testCommands: [
      {
        label: "Simulate a matching detection (triggers the alert)",
        topicHint: "<base_namespace>/all/detections",
        messageType: "nepi_interfaces/Detections",
        json: "{\"source_topic\": \"/nepi/device1/idx/color_2d_image\", \"detections\": [{\"name\": \"bottle\", \"confidence\": 0.9, \"xmin\": 270, \"ymin\": 190, \"xmax\": 370, \"ymax\": 290}]}",
        notes: "Fires the alert without needing a real camera/model -- OBJECT_LABEL_OF_INTEREST defaults to \"bottle\", change the name field to test a different label.",
      },
      {
        label: "Simulate no detection (clears the alert after ALERT_LOST_COUNT_THRESHOLD misses)",
        topicHint: "<base_namespace>/all/detections",
        messageType: "nepi_interfaces/Detections",
        json: "{\"source_topic\": \"/nepi/device1/idx/color_2d_image\", \"detections\": []}",
        notes: "Publish this a few times in a row (more than ALERT_LOST_COUNT_THRESHOLD) to see the alert clear.",
      },
    ],
    tips: [
      "The alert is debounced -- a single missed detection frame won't immediately turn it off, by design, so don't expect an instant flip back after one \"no detection\" publish.",
    ],
  },

  "led_adjust_on_object_detect_action_script.py": {
    summary: "Ramps a light's brightness up or down based on where a detected object sits in the camera frame -- centered lights it up more, off to the side dims it.",
    requires: [
      "A running LSX light driver with intensity control",
      "A model running and detecting via ai_detector_config_script.py, publishing to <base_namespace>/all/detections",
      "The real image topic named in source_topic actually publishing -- this script fetches frame width/height from it once, and needs that to succeed to compute anything",
    ],
    requiredTopics: [
      {label: "LSX light driver", patterns: ["lsx/status"]},
      {label: "AI detections topic", patterns: ["all/detections"]},
    ],
    testCommands: [
      {
        label: "Simulate a centered detection (max intensity)",
        topicHint: "<base_namespace>/all/detections",
        messageType: "nepi_interfaces/Detections",
        json: "{\"source_topic\": \"/nepi/device1/idx/color_2d_image\", \"detections\": [{\"name\": \"bottle\", \"confidence\": 0.9, \"xmin\": 270, \"ymin\": 190, \"xmax\": 370, \"ymax\": 290}]}",
        notes: "Box centered in a 640x480 frame -- replace source_topic with your real running camera topic, or this will just warn and skip (no image to measure).",
      },
      {
        label: "Simulate an off-center detection (lower intensity, may trigger blink)",
        topicHint: "<base_namespace>/all/detections",
        messageType: "nepi_interfaces/Detections",
        json: "{\"source_topic\": \"/nepi/device1/idx/color_2d_image\", \"detections\": [{\"name\": \"bottle\", \"confidence\": 0.9, \"xmin\": 570, \"ymin\": 190, \"xmax\": 670, \"ymax\": 290}]}",
        notes: "Box pushed toward the frame edge -- watch the intensity drop and, past LED_BLINK_THRESHOLD, the light start blinking.",
      },
    ],
    tips: [
      "If nothing happens at all, check this script's own log for \"Failed to get image dimensions\" -- that means source_topic in the detections message doesn't match a real, currently-publishing image topic.",
    ],
  },

  "led_auto_level_process_script.py": {
    summary: "Reads brightness from a live camera image and dims or brightens a connected light to compensate, so a scene stays evenly lit.",
    requires: [
      "An LSX light driver running",
      "An IDX camera driver (or simulator) streaming a topic matching IMAGE_INPUT_TOPIC_NAME (default: any topic containing \"color_2d_image\")",
    ],
    requiredTopics: [
      {label: "LSX light driver", patterns: ["lsx/status"]},
      // See ai_detector_config_script.py's requiredTopics comment above --
      // same sim/RBX-vs-real-IDX-camera dual convention.
      {label: "Camera image topic", patterns: ["color_2d_image", "idx/color_image"]},
    ],
    testCommands: [
      {
        label: "Test the light directly (bypassing this script's brightness logic)",
        topicHint: "<light node>/set_intensity_ratio",
        messageType: "std_msgs/Float32",
        json: "{\"data\": 0.3}",
        notes: "There's no practical way to hand-type a camera image into the JSON publisher below -- if the light doesn't respond to this, the problem is the light driver, not this script.",
      },
    ],
    tips: [
      "If the light never settles or hunts up/down continuously, try raising SENSITIVITY_RATIO's denominator effect by lowering the value in the script file -- it controls how aggressively brightness changes react to measured light level.",
    ],
  },

  "led_step_adjust_process_script.py": {
    summary: "Continuously steps a connected light's brightness up in small increments, wrapping back to off once it hits the configured max -- a simple bench test for LED intensity control, not tied to any sensor input.",
    requires: [
      "A running LSX light driver with intensity control",
    ],
    requiredTopics: [
      {label: "LSX light driver", patterns: ["lsx/status"]},
    ],
    testCommands: [
      {
        label: "Test the light directly (bypassing this script's step logic)",
        topicHint: "<light node>/set_intensity_ratio",
        messageType: "std_msgs/Float32",
        json: "{\"data\": 0.3}",
        notes: "If the light doesn't respond to this, the problem is the light driver, not this script.",
      },
    ],
    tips: [
      "LED_LEVEL_MAX / LED_LEVEL_STEP / LED_STEP_SEC are all edited in the script file itself -- there's no live settings UI for this script.",
      "This script only ever steps up and wraps -- it never dims back down gradually. If you want a smooth up/down sweep, this isn't that; use led_auto_level_process_script.py for brightness that reacts to something instead.",
    ],
  },

  "navpose_config_script.py": {
    summary: "Wires live GPS/odometry/heading source topics into navpose_mgr's base_frame, so the rest of the platform (NavPose displays, mission scripts, data overlays) sees a real, continuously-updating position/orientation solution instead of nothing.",
    requires: [
      "A running driver that actually publishes GPS/odom/heading topics matching NEPI_NAVPOSE_SOURCE_GPS_TOPIC / _ODOM_TOPIC / _HEADING_TOPIC (defaults: \"rbx/gps_fix\", \"rbx/odom\", \"rbx/heading\" -- edit these to match your actual device if it uses different topic names)",
      "Your heading source must publish nepi_interfaces/NavPoseHeading specifically -- navpose_mgr silently ignores any other message type for the heading component with no error (see the script's own header comment for the full type list per component)",
    ],
    requiredTopics: [
      {label: "GPS/odom/heading source (RBX driver)", patterns: ["rbx/gps_fix", "rbx/odom", "rbx/heading"]},
    ],
    testCommands: [],
    tips: [
      "To check what base_frame is currently wired to, echo navpose_mgr/status via Advanced Debugging's topic list below -- it's a status topic, not something you publish to, so there's no \"Try It\" button for it.",
      "If a component silently never updates, double check the actual message type of your source topic against the type list in this script's own header comment -- a mismatch is not reported as an error anywhere.",
      "This script only wires topics once at startup -- if the source driver restarts and republishes under a different topic name, you'll need to restart this script too.",
    ],
  },

  "navpose_set_fixed_config_script.py": {
    summary: "Sets a fixed, manually-entered NavPose (lat/long/altitude/heading/roll/pitch/yaw) on navpose_mgr's base_frame -- for a system with no GPS/IMU/compass attached at all, so the platform still has a NavPose solution to report.",
    requires: [
      "Nothing external -- navpose_mgr is a core manager that's always running. Just edit START_GEOPOINT / START_HEADING_DEG / START_ORIENTATION_DEGS in the script file to your actual fixed location before running.",
    ],
    testCommands: [],
    tips: [
      "This and navpose_config_script.py both target base_frame and will fight each other if run together -- use this one only when you have no real NavPose source at all, not alongside a script that wires up live topics.",
      "The fixed values apply continuously for as long as the affected components stay set to 'Fixed' -- there's no separate one-shot \"reinit\" step needed, unlike the old pre-navpose_mgr API this replaced.",
    ],
  },

  "opencv_image_contours_process_script.py": {
    summary: "Subscribes to a live camera image, overlays OpenCV-detected contours and text on it, and republishes the result on a new \"image_contours\" topic -- a simple example of building a custom image-processing pipeline.",
    requires: [
      "A live camera topic matching IMAGE_INPUT_TOPIC_NAME (default: any topic containing \"color_2d_image\", with a real-IDX-camera fallback to \"idx/color_image\")",
    ],
    requiredTopics: [
      {label: "Camera image topic", patterns: ["color_2d_image", "idx/color_image"]},
    ],
    testCommands: [],
    tips: [
      "The output topic is <base_namespace>/image_contours -- view it with the Image Viewer app or Advanced Debugging's topic list below, it isn't shown anywhere in this script's own panel.",
      "This is meant as a starting template -- the actual OpenCV customization happens in image_custom_callback in the script file, edit it directly to change what gets drawn.",
    ],
  },

  "sim_ai_targeting_bridge_script.py": {
    summary: "SITL/Gazebo-only test scaffolding that stands in for a real app_ai_targeting app -- relays the dev VM's synthetic target-tracking feed into the real target_localizations topic drone_follow_object_mission_script.py expects, so that mission script's follow logic can actually be exercised without a real targeting app existing yet.",
    requires: [
      "The ArduPilot RBX driver running and connected (SITL/Gazebo, since this is sim-only test scaffolding -- see knownIssue). If it isn't detected on startup, this script automatically tries to trigger sitl_gazebo_full on the dev VM over the existing tunnel (port 9028, sim_launch_listener.py) -- this only works if the VM/tunnel has been started at least once already (e.g. a prior sitl_gazebo or sitl_gazebo_full run this session); a completely cold VM has nothing there yet to receive the trigger.",
      "ai_targeting_controller_ardupilot.py running on the dev VM at 127.0.0.1:9027, reached over the existing reverse SSH tunnel -- normally handled automatically by the auto-trigger above via sitl_gazebo_full, which starts it if it isn't already running.",
    ],
    requiredTopics: [
      {label: "ArduPilot RBX driver", patterns: ["rbx/status"]},
    ],
    knownIssue: "This is explicitly NOT a real app_ai_targeting replacement -- it only works against the SITL/Gazebo sim stand-in (ai_targeting_controller_ardupilot.py), which has no real-hardware equivalent yet. Running this without that VM-side process just leaves it retrying the bridge connection forever.",
    testCommands: [],
    tips: [
      "Run this alongside drone_follow_object_mission_script.py, not instead of it -- this script only produces the target_localizations feed; it doesn't fly the drone itself. drone_follow_object_mission_script.py's own \"Auto-Start Sim Requirements\" button starts this one directly, so you don't normally need to launch it separately.",
      "If drone_follow_object_mission_script.py's Peripheral Status checklist still shows the target-localization feed as missing after starting this script, check this script's own Messages panel for \"Could not reach sim_launch_listener\" -- that means the dev VM/tunnel isn't reachable at all, and sitl_gazebo/sitl_gazebo_full needs to be run there manually at least once.",
    ],
  },
}

export default ScriptDocs
