# Status for the additive simulator auto-launch capability (see
# docs/SIMULATOR_AUTO_LAUNCH_PLAN.md). Published separately from SimStatus on
# purpose -- launching infrastructure on a dev VM is a different concern from
# the bridge/robot-capability contract SimStatus already reports, and keeping
# it a separate message means nothing about SimDeviceIF's existing contract
# has to change to add it.

# Launch targets this deployment knows about, read from
# simulator_launch_targets.yaml (nepi_drones, not shipped with the app -- see
# the plan doc for why). Same reported-list-plus-active-selection shape
# SimStatus already uses for its two selectors.
string[] available_launch_targets
string[] available_launch_target_names
string selected_launch_target

# What the operator picked in the selector (e.g. "Gazebo") is NOT always
# what actually launched -- a robot config redirects some combinations to a
# different, hidden_from_selector target entirely (see
# simulator_launcher.py's resolve_launch_target, e.g. "Gazebo" + a
# quadcopter robot config actually runs gazebo_quadcopter, a different
# world/bridge/install requirement). Added live (2026-08-19) after exactly
# that confusion -- reported as "I selected Gazebo and Quadcopter but it
# launched the rover instead" when what actually happened was invisible
# from the RUI: selected_launch_target only ever showed "gazebo_rover"
# (the raw selector key) regardless of which target was truly running, so
# there was no way to tell a successful quadcopter redirect apart from a
# genuine wrong-target bug, or apart from an already-running target from
# an earlier session simply never having been replaced (this target's own
# "refuse to launch if a gzserver already exists" guard fails the NEW
# launch silently from the operator's point of view, leaving whatever was
# already running -- often a stale leftover from unrelated testing --
# looking exactly like "it deployed the wrong thing"). Empty when nothing
# has ever launched this session.
string active_launch_target
string active_launch_target_name

# Parallel to available_launch_targets: whether each target's own
# check_installed_command last reported success. A target that has never
# been checked yet (e.g. right after this app started, before the
# background check thread finishes) reports false here -- "not known to be
# installed" -- rather than blocking on a real check just to answer the
# first status request. installed_check_state below is what actually
# distinguishes "checked, and it's missing" from "haven't checked yet".
bool[] available_launch_target_installed

# Parallel to available_launch_targets: unknown | checking | installed |
# not_installed. Separate from launcher_state (which describes the
# selected target's launch/stop lifecycle, one target at a time) because
# install-checking happens for every target independently and isn't tied to
# which one is selected.
string[] available_launch_target_installed_check_state

# idle | launching | running | stopping | failed | installing
string launcher_state

# Empty when launcher_state is not "failed"
string last_error

# Copy-paste terminal commands for the operator to run manually as a
# worst-case fallback -- populated whenever a launch/install fails because a
# dependency is missing (see simulator_launcher.py's
# get_manual_fallback_commands), and for a target like gazebo_quadcopter that
# has no auto-install at all. One command per line, meant to be rendered
# verbatim (a <pre>/copyable text block), not parsed. Empty whenever there is
# nothing to offer -- e.g. launcher_state isn't "failed", or the target that
# failed has neither install_command nor manual_fallback_commands configured.
string manual_fallback_commands
