# Status for the additive multi-OS-instance deploy-target registry (see
# docs/SIM_OS_INSTANCES_PLAN.md). Published separately from SimLauncherStatus
# on purpose -- WHICH machine simulator_launch_targets.yaml's targets should
# currently point at is a different concern from the launch/install/stop
# lifecycle SimLauncherStatus already reports, and keeping it a separate
# message means nothing about that existing contract has to change to add
# this. Mirrors SimLauncherStatus's own reported-list-plus-selection shape.

# Every currently-registered instance (any status), read from
# os_instance_registry.py's persisted store
# (/mnt/nepi_storage/databases/nepi_app_sim_connector/os_instances/*.yaml).
# Parallel arrays, same convention every other reported-list field in this
# app's messages already uses.
string[] instance_ids
string[] instance_display_names
string[] instance_hosts
string[] instance_ssh_users
int32[] instance_ssh_ports

# ssh | shared_storage (see os_instance_registry.py's CONNECTION_MODES) --
# host/ssh_user/ssh_port above are meaningless for a shared_storage entry
# (it talks to the device over the nepi_storage share instead, see
# vm_command_watcher.py), which is why this is its own parallel array
# rather than folded into instance_hosts as a special value.
string[] instance_connection_modes

# pending | verified | unreachable -- pending until Test Connection succeeds
# at least once; unreachable after a Test Connection attempt fails (distinct
# from pending, which just means never tried yet).
string[] instance_statuses

# Which instance simulator_launch_targets.yaml's targets are currently
# pointed at (see os_instance_registry.apply_to_launcher) -- empty when
# nothing has ever been selected, in which case every target keeps whatever
# host/ssh_user/ssh_port simulator_launch_targets.yaml itself hardcodes (the
# single-VM default this whole registry is additive to, unchanged).
string selected_instance_id

# idle | registering | verifying | failed -- the setup-flow's own state,
# independent of any individual instance's own instance_statuses entry above
# (registering/verifying always refers to whichever instance
# pending_instance_id names below).
string setup_state
string pending_instance_id
string last_error

# Copy-paste terminal commands for the operator to run on the machine they
# just registered -- SSH key + reverse-tunnel setup (systemd and plain-autossh
# variants, see os_instance_registry.py's build_setup_commands), populated
# whenever setup_state is "registering" or "failed" for pending_instance_id.
# One command block, meant to be rendered verbatim (a <pre>/copyable text
# block), not parsed. Empty once the instance is verified -- nothing left to
# copy at that point.
string pending_setup_commands
