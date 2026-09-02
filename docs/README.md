# nepi_drones docs — index

Start here: **`SIMULATION_OVERVIEW.md`** — the orientation doc. Read it first if you're
new to this repo; everything else here goes deeper on one piece of what it describes.

## Current reference docs

- **`SIMULATION_OVERVIEW.md`** — what's built, how the pieces fit together, file-by-file
  walkthrough of the sim environment and the RBX driver integrations.
- **`SIM_DEVICE_IF_CONTRACT.md`** — the generic NEPI↔simulator interface contract itself:
  data flow, capability/status fields, the two-camera convention, the rule for adding a
  new simulator without touching the core contract.
- **`SIM_CONNECTOR_REMAINING_WORK.md`** — current, living answer to "what's left to build
  and where to start." Read this before starting new sim-connector work.
- **`MULTI_SIMULATOR_INTEGRATION_PLAN.md`** — the phased build/status log for wiring
  Gazebo, Webots, PyBullet, Unity, and WPILib HAL Sim into the generic contract (ROS
  Stage support was tried and then dropped — not a priority). The authoritative
  per-simulator status table lives here.
- **`SIMULATOR_AUTO_LAUNCH_PLAN.md`** — the SSH-based remote auto-launch feature (deploy/
  start/stop a simulator on a dev VM from the app itself) — design and build log.
- **`SIM_OS_INSTANCES_PLAN.md`** — additive multi-OS-instance deploy-target registry: the
  "OS selected" picker + guided setup wizard for registering more than one dev machine,
  built on top of the single-VM auto-launch feature above without changing it.
- **`SCAN_TO_SIM_ENVIRONMENT_PLAN.md`** — design plan (not yet built) for converting a
  phone LiDAR/IMU scan (Stray Scanner) of a real obstacle course into a spawnable Gazebo
  environment, so a course can be tested in sim without hand-authoring an SDF world.
- **`RBX_CAPABILITIES.md`** — the canonical RBX (robot) interface reference: namespaces,
  capabilities/status contract, command topics, the goto handshake, building a new RBX
  driver. Includes the motor-control-ratios deep dive as its own section.
- **`DISCOVERY_EXPLAINED.md`** — how NEPI's driver-discovery framework works, using the
  ArduPilot RBX driver as the worked example. Includes an appendix on standing up
  ArduPilot SITL (build/run steps, TCP-vs-UDP reasoning, gotchas).
- **`SIM_CONNECTOR_TESTING_GUIDE.md`** — practical how-to for testing
  `nepi_app_sim_connector` against a real simulator, plus a legacy hardware/SITL
  `rostopic`/`rosservice` command-reference appendix.
- **`NEPI_APP_BUILD_AND_TEST_CHECKLIST.md`** — general runbook for building/deploying/
  debugging any NEPI app end-to-end (not sim-specific).
- **`Tutorial-NEPI Engine_Autopilot Interfacing and Automation (Ardupilot).docx`** — the
  only doc covering physical (non-simulated) hardware wiring/config. Its own automation
  scripts are deprecated — use the RUI, or `SIM_CONNECTOR_TESTING_GUIDE.md`'s command
  appendix for raw CLI commands.

## `completed/` — fully closed work, not general reference

Things that are entirely done (verified end-to-end, including on the real device where
applicable) get moved here once closed, so the docs above stay focused on what's current/
ongoing rather than accumulating finished-work narrative. Currently:

- **`completed/GAZEBO_SIM_CONNECTOR_INTEGRATION.md`** — the Gazebo simulator's full
  integration into the generic `sim_connector` contract. The only simulator integration
  fully closed so far (Webots/PyBullet/WPILib are VM-verified but still open per
  `SIM_CONNECTOR_REMAINING_WORK.md` — not moved here yet).
- **`completed/SIM_CONNECTOR_NAVPOSE_HANG_BUG.md`** — closed bug postmortem (an apparent
  startup hang that turned out to be stdout buffering, plus a real bare-roscore
  `wait_for_param` gotcha worth knowing before testing any `nepi_sdk` node standalone).

## Note on history

Several earlier planning/task-list docs (the original rover bridge plan, an early
ArduPilot-only task guide, a SITL implementation plan, a motor-control-ratios doc, and a
mid-project forward-plan) have been merged into the current docs above or removed once
superseded, rather than kept around as stale snapshots. If you need that history, it's in
git log, not a live file.
