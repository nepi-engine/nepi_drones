# Connecting a NEPI device to a sim VM (Deploy-button path)

This covers ONE specific thing: making the Sim Connector app's "Deploy"
button (the one-click Gazebo/Webots/MuJoCo auto-launch in the RUI) work
between YOUR NEPI device and YOUR sim VM. If you don't need that
convenience, skip all of this and see `SIM_CONNECTOR_TESTING_GUIDE.md`
instead -- running a simulator yourself and pointing it at the device's
already-open TCP listener needs no SSH setup at all.

## Why this exists

Two machines, no shared ROS master, connected only by SSH:

- **The NEPI device** runs the actual ROS master, `drivers_mgr`, and the
  RUI. It's the thing you browse to.
- **A sim VM** runs Gazebo/Webots/MuJoCo. It has no NEPI software installed
  beyond this repo checked out for its `sim_container/` scripts.

Clicking Deploy makes the DEVICE ssh OUT to the VM to run that target's
`launch_command` (see `simulator_launch_targets.yaml`). That SSH connection
is the one thing this doc is about. A second, separate connection --
the VM's bridge script dialing back INTO the device's already-listening
port 9030 -- usually needs nothing beyond both machines sharing a LAN, and
isn't covered here.

Out of the box, this repo's own `simulator_launch_targets.yaml` and
`nepi_tunnel()` are wired to one specific developer's own personal setup
(a VM username, a device hostname alias, specific ports). Reported live:
"for people that dont have the vm and the nepi device set up properly
together... it would be hard for the nepi device to talk to the vm." This
doc is the fix for that -- three env vars on the device side, three more
on the VM side, no editing of any tracked file required.

## The two directions

```
                    ssh -R (reverse tunnel, run FROM the VM)
   NEPI device  <───────────────────────────────────────  Sim VM
        │        127.0.0.1:12222 on the device = VM's real sshd
        │
        └── ssh (the Deploy click itself, run FROM the device,
             THROUGH that same tunnel) ──────────────────> Sim VM
```

1. **VM → device**: `nepi_tunnel()` (or the `nepi-tunnel.service` systemd
   unit) opens a reverse tunnel FROM the VM, forwarding a long list of
   ports back to the device -- including port 12222, which forwards to the
   VM's OWN sshd. This has to be running before Deploy can work at all.
2. **Device → VM**: when you click Deploy, `simulator_launcher.py` (running
   as part of `app_sim_connector` on the device) SSHes to
   `127.0.0.1:12222` -- which, because of the tunnel above, actually lands
   on the VM's sshd. This is the connection `simulator_launch_targets.yaml`
   configures per target.

Both directions need their own SSH key and their own username, and both
are currently hardcoded to one developer's own setup. Fixing both:

## Step 1 — SSH keys

You need two separate keypairs (despite sharing a filename convention,
`nepi_default_ssh_key`, they are NOT the same key):

- **On the device**: a keypair whose PRIVATE half lives at
  `/home/nepi/.ssh/nepi_default_ssh_key` (or wherever `NEPI_SSH_KEY_PATH`/
  `NEPI_SSH_KEY` point -- see `simulator_launcher.py`'s
  `_ssh_key_candidates`) and whose PUBLIC half is in the VM user's
  `~/.ssh/authorized_keys`.
- **On the VM**: a keypair whose private half lives at
  `~/.ssh/nepi_default_ssh_key` (the filename `nepi_tunnel()` and
  `nepi-tunnel.service` both assume) and whose public half is in the
  device's `nepi` user's `~/.ssh/authorized_keys`.

If you don't have these yet:

```bash
# On the device (as the nepi user, or whatever runs app_sim_connector):
ssh-keygen -t ed25519 -f ~/.ssh/nepi_default_ssh_key -N ""
# Copy the PUBLIC half (nepi_default_ssh_key.pub) to your VM user's
# ~/.ssh/authorized_keys.

# On the VM:
ssh-keygen -t ed25519 -f ~/.ssh/nepi_default_ssh_key -N ""
# Copy the PUBLIC half to the device's nepi user's ~/.ssh/authorized_keys.
```

## Step 2 — point the VM's tunnel at your device

`nepi_tunnel()` and `nepi-tunnel.service` both default to `nepi@nepi:2222`
(the platform's own default device username and dev-VM-facing sshd port --
not a value unique to any one setup, but still worth overriding if your
device's sshd differs). Override without editing any tracked file:

**Manual/ad-hoc** (source `nepi_sitl_dev_env.sh` from your `~/.bashrc`
first, per its own header):
```bash
export NEPI_DEVICE_SSH_HOST=192.168.1.50   # your device's real IP or hostname
export NEPI_DEVICE_SSH_USER=nepi           # only if your device's user differs
export NEPI_DEVICE_SSH_PORT=22             # only if your device's sshd differs
nepi_tunnel
```

**systemd (recommended -- survives reboots on both ends)**:
```bash
mkdir -p ~/.config/systemd/user ~/.config
cp sim_container/systemd/nepi-tunnel.service ~/.config/systemd/user/
cat > ~/.config/nepi-tunnel.env <<'EOF'
DEVICE_SSH_HOST=192.168.1.50
DEVICE_SSH_USER=nepi
DEVICE_SSH_PORT=22
EOF
systemctl --user daemon-reload
systemctl --user enable --now nepi-tunnel.service
loginctl enable-linger $(id -un)   # starts the tunnel at boot, before login
```

Verify: `systemctl --user status nepi-tunnel.service` should show it
active, and `ssh -p 12222 <your-vm-user>@localhost echo ok` run ON THE
DEVICE should print `ok` once the tunnel is up.

## Step 3 — point the device's Deploy path at your VM's username

`simulator_launch_targets.yaml`'s five targets all hardcode
`ssh_user: "suraj"`. Override wherever `app_sim_connector` actually runs
(typically set in the device's own environment/config, not per-session):

```bash
export NEPI_SIM_VM_SSH_USER=yourname
# Only if you're skipping the tunnel and using a routable VM instead:
# export NEPI_SIM_VM_HOST=192.168.1.100
# export NEPI_SIM_VM_SSH_PORT=22
```

Restart `app_sim_connector` (disable/re-enable it in the RUI's Apps page,
or restart the container) so it picks up the new environment. See
`simulator_launcher.py`'s `_apply_connection_env_overrides` for exactly
what these three variables do -- they're applied once at config-load time,
so every target picks them up with no yaml edits.

## Step 4 — verify end to end

1. In the RUI, open the Sim Connector app. The Simulator dropdown's
   install-check state (`available_launch_target_installed_check_state`)
   should settle to `installed` for whichever targets you have set up,
   rather than staying stuck on `unknown`/`checking` -- that's the sign
   `_ssh_cmd`'s connection is actually reaching your VM.
2. Pick a target and click Deploy. If the tunnel or the device-side
   username is still wrong, the app surfaces the raw SSH failure in
   `last_error` (something like `ssh: connect to host 127.0.0.1 port
   12222: Connection refused` means the tunnel isn't up; `Permission
   denied (publickey)` means the keys aren't authorized correctly).

## The SSH-free alternative

None of the above is required just to try the sim connector *protocol*.
`SIM_CONNECTOR_TESTING_GUIDE.md` covers running `sim_rover_gazebo` (or
your own simulator, speaking the same JSON-lines bridge protocol) directly
on your own machine and pointing it at the device's IP on port 9030 --
no SSH, no tunnel, no Deploy button. You lose the one-click convenience,
not the underlying feature.
