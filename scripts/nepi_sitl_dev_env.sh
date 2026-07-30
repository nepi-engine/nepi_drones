# NEPI ArduPilot SITL dev VM environment -- source this from ~/.bashrc.
#
# Provides the one-command Gazebo + ArduPilot SITL launcher used for testing
# the rbx_ardupilot driver against the real NEPI device's RUI (see
# docs/SIMULATOR_DEV_GUIDE.md). Add to ~/.bashrc:
#
#   source /path/to/nepi_drones/scripts/nepi_sitl_dev_env.sh
#
# Requires: gazebo, ArduPilot's sim_vehicle.py on PATH, ~/ardupilot_gazebo
# world files, and autossh (`sudo apt-get install autossh`) for nepi_tunnel.

alias nepi_gazebo='gazebo ~/ardupilot_gazebo/worlds/iris_arducopter_cmac.world'
alias nepi_sitl='sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map'
alias sitl='sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map'

# Resolved at source time so sitl_gazebo works from any cwd (camera-rig
# feature: needs nepi_drones/sim_container/models on GAZEBO_MODEL_PATH so
# `model://camera_rig` resolves -- see camera_rig_controller_ardupilot.py).
NEPI_DRONES_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/sim_container"

# Launches Gazebo, waits for it to fully load (so the ArduPilotPlugin's FDM
# socket is up before SITL starts sending), then runs SITL in the foreground
# so its MAVProxy prompt is right there in the terminal. Ctrl-C / `quit` in
# MAVProxy tears Gazebo down too.
# Tiny local trigger for "reset the sim": listens on 127.0.0.1:<port> and runs
# `gz world -o` (reset model poses only -- NOT -r/time, which crashes the
# connected ArduPilot SITL binary on a time discontinuity) on any connection.
# Exists so the NEPI RBX driver -- which runs on the remote NEPI device, not
# this VM -- can reach across the existing reverse SSH tunnel and trigger a
# Gazebo reset without its own SSH creds here. See gz_reset_listener.py
# (installed alongside this script; copy or symlink it to ~/.local/bin/).
gz_reset_listener() {
    local port="${1:-9021}"
    if pgrep -f "gz_reset_listener.py $port" > /dev/null; then
        echo "gz reset listener already running on 127.0.0.1:$port"
        return 0
    fi
    nohup python3 -u ~/.local/bin/gz_reset_listener.py "$port" > /tmp/gz_reset_listener.log 2>&1 &
    disown
    echo "gz reset listener started on 127.0.0.1:$port"
}

# Keeps this VM linked to the real NEPI device so its RBX ArduPilot driver
# (which runs on the NEPI device, not here) can reach this VM's SITL/reset
# listener over their shared loopback. Persistent/idempotent on purpose --
# not tied to sitl_gazebo's lifecycle, so it's fine to leave running between
# SITL sessions. Forwards: 5771 (MAVProxy's dedicated --out port -- this is
# the one the RBX driver's discovery actually connects to; see sitl_gazebo
# below), 5760 (SITL's raw/primary port, forwarded for any other direct use --
# NOT used by driver discovery, which only tries 5771), 9021
# (gz_reset_listener, for the RESET_SIM RUI action), 9022
# (sim_heartbeat_listener, the rover-sim liveness port the rbx_sim driver's
# discovery probes -- see sim_rover_dev_env.sh; chosen as the next port in the
# 902x block of tiny sim-utility listeners started by gz_reset_listener's
# 9021, clear of the 576x MAVLink ports), 9023 (the rover-sim
# command/telemetry bridge served by sim_bridge_node.py -- the persistent
# JSON-lines connection rbx_sim_node.py holds open, next port in the same
# 902x block after the 9022 heartbeat), and 9024/9025 (the second rover's
# heartbeat/bridge pair for the multi-robot workflow's sim_rover_gazebo_multi
# -- see sim_rover_dev_env.sh; rover1 reuses 9022/9023, rover2 gets the next
# pair in the same 902x block, served by sim_heartbeat_listener.py 9024 and
# sim_bridge_multi_node.py), and 9026 (the ArduPilot camera-rig bridge --
# camera_rig_controller_ardupilot.py's own TCP JSON-lines server, next free
# port in the 902x block; carries ONLY camera settings in and compressed
# frames out, since MAVLink over 5771 already carries telemetry/commands for
# this driver). This one tunnel serves both
# simulation workflows (ArduPilot SITL and the generic rover sim).
# Uses autossh (not plain ssh) so the tunnel reconnects on its own whenever
# either side restarts -- a power-cycle of the NEPI device kills its sshd and
# drops a plain ssh tunnel for good, requiring a manual nepi_tunnel re-run.
# With autossh, it doesn't matter which of sitl_gazebo / the NEPI device
# comes up first or restarts later -- autossh just keeps retrying the
# connection until the other side is reachable again.
nepi_tunnel() {
    if pgrep -f "autossh.*R 5771:127.0.0.1:5771.*nepi@nepi" > /dev/null; then
        echo "NEPI reverse tunnel already running"
        return 0
    fi
    AUTOSSH_GATETIME=0 nohup autossh -M 0 -p 2222 -i ~/.ssh/nepi_default_ssh_key \
        -o ConnectTimeout=5 -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o ExitOnForwardFailure=yes \
        -R 5760:127.0.0.1:5760 -R 5771:127.0.0.1:5771 -R 9021:127.0.0.1:9021 -R 9022:127.0.0.1:9022 -R 9023:127.0.0.1:9023 -R 9024:127.0.0.1:9024 -R 9025:127.0.0.1:9025 -R 9026:127.0.0.1:9026 \
        -N nepi@nepi > /tmp/nepi_tunnel.log 2>&1 &
    disown
    sleep 2
    if pgrep -f "autossh.*R 5771:127.0.0.1:5771.*nepi@nepi" > /dev/null; then
        echo "NEPI reverse tunnel started (auto-reconnecting via autossh)"
    else
        echo "NEPI reverse tunnel FAILED to start -- check /tmp/nepi_tunnel.log"
    fi
}

sitl_gazebo() {
    # Ctrl-C only signals SITL's own process group, not this shell, so we
    # can't rely on a trap firing when this function returns -- clean up
    # Gazebo explicitly after sim_vehicle.py exits (below), and also on
    # INT/TERM in case you cancel during the "waiting for Gazebo" phase.
    # Camera-rig feature: this workflow previously ran Gazebo standalone
    # (plain `gazebo`, no ROS at all -- MAVLink was the only channel). The
    # camera rig's libgazebo_ros_camera.so plugin and /gazebo/model_states
    # pose feed (read by camera_rig_controller_ardupilot.py) both require the
    # gazebo_ros API plugin, which requires a roscore. started_roscore tracks
    # whether THIS invocation started one, so it's only torn down if we
    # started it (matches sim_rover_gazebo's own convention in
    # sim_rover_dev_env.sh).
    local started_roscore=0
    trap 'pkill -x gzclient 2>/dev/null; pkill -x gzserver 2>/dev/null; pkill -f gz_reset_listener.py 2>/dev/null;
          if [ "$started_roscore" = "1" ]; then pkill -x roscore 2>/dev/null; pkill -x rosmaster 2>/dev/null; pkill -f "rosout/rosout" 2>/dev/null; fi;
          trap - INT TERM; return' INT TERM

    source /opt/ros/noetic/setup.bash
    export GAZEBO_MODEL_PATH="$NEPI_DRONES_SIM_DIR/models:$GAZEBO_MODEL_PATH"

    if ! rostopic list > /dev/null 2>&1; then
        echo "Starting local roscore..."
        nohup roscore > /tmp/sitl_gazebo_roscore.log 2>&1 &
        disown
        started_roscore=1
        until rostopic list > /dev/null 2>&1; do
            sleep 1
        done
    else
        echo "roscore already running -- reusing it"
    fi

    echo "Starting Gazebo..."
    rosrun gazebo_ros gazebo ~/ardupilot_gazebo/worlds/iris_arducopter_cmac.world &

    echo "Waiting for Gazebo to finish loading..."
    until pgrep -x gzserver > /dev/null; do
        sleep 1
    done
    sleep 8

    gz_reset_listener
    nepi_tunnel

    # --out=tcpin:0.0.0.0:5771 gives MAVProxy a second, dedicated TCP port for
    # the NEPI RBX driver's mavros -- without it, MAVProxy alone occupies the
    # primary port 5760 as SITL's sole MAVLink client, and mavros can never
    # connect (the drone never shows up under Devices in the RUI). MAVProxy
    # still keeps 5760/--console/--map for you exactly as before.
    echo "Starting ArduPilot SITL..."
    sim_vehicle.py -v ArduCopter -f gazebo-iris --out=tcpin:0.0.0.0:5771 --console --map

    pkill -x gzclient 2>/dev/null
    pkill -x gzserver 2>/dev/null
    pkill -f gz_reset_listener.py 2>/dev/null
    if [ "$started_roscore" = "1" ]; then
        pkill -x roscore 2>/dev/null
        pkill -x rosmaster 2>/dev/null
        pkill -f "rosout/rosout" 2>/dev/null
    fi
    trap - INT TERM
}

# Camera-rig controller for the ArduPilot SITL sim (Universal Simulator
# Bridge camera feature, ArduPilot port). Not auto-started by sitl_gazebo --
# same manual-launch convention as the rover workflow's camera controllers
# (see sim_rover_dev_env.sh / camera_rig_controller.py). Run this in a
# separate terminal/screen session after sitl_gazebo is up.
camera_rig_controller_ardupilot() {
    source /opt/ros/noetic/setup.bash
    python3 "$NEPI_DRONES_SIM_DIR/scripts/camera_rig_controller_ardupilot.py"
}

# Alias for typos / muscle memory -- identical to sitl_gazebo.
gazebo_sitl() {
    sitl_gazebo "$@"
}
