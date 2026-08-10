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
