#!/usr/bin/env python3
"""Tiny liveness listener for the generic-rover Gazebo simulation.

Listens on 127.0.0.1:<port>. On each connection, checks whether gzserver is
ACTUALLY still running (see GZSERVER_PGREP_PATTERN below) and only then
replies ALIVE, closing the connection either way. Deliberately NOT a ROS
node: the remote NEPI device and this dev VM run separate ROS masters
bridged only by a reverse SSH tunnel that forwards raw TCP ports, so the
NEPI rbx_sim driver's discovery cannot see this VM's ROS graph (no
/sim/heartbeat topic) -- it probes this plain TCP port instead. The ALIVE
reply matters: with an ssh -R forward, a connect() on the device side
succeeds against the device's sshd even when nothing is listening here, so
discovery must read the reply, not just connect.

Checks gzserver's real liveness (2026-08-26) rather than unconditionally
replying ALIVE just because this process itself is running -- this
listener and gzserver are separate processes, started together by
launch_command but with no guaranteed teardown coupling after that: a
gzserver crash, an operator manually killing just gzserver/gzclient (e.g.
to debug something), or any stop path that doesn't happen to hit this
listener's own PID all leave it running and happily lying "ALIVE" forever.
Reported live: "even though the rover is killed in gazebo, it still shows
it in robots... this is a recurring issue" -- rbx_sim_discovery.py's own
liveness check (checkForSimDevice) is only as honest as this reply, so a
stale reply means a stale robot entry that never clears from Devices ->
Robots no matter how long gzserver has been gone.

Started and stopped by the sim_heartbeat_listener function in
sim_rover_dev_env.sh as part of sim_rover_gazebo, alongside roscore, Gazebo,
and sim_bridge_node.py -- so reachability of this port USED TO mean the
rover sim stack is up; now it only means the stack was launched at some
point AND gzserver is still actually alive right now, closing that gap.
Modeled on gz_reset_listener.py (same pattern, ArduPilot workflow).
"""

import socket
import subprocess
import sys

DEFAULT_PORT = 9022

# Matches gazebo_rover's own ready_check_command in
# simulator_launch_targets.yaml -- scoped to this specific world, not a bare
# "gzserver", so an unrelated Gazebo instance (a different launch target)
# can't produce a false ALIVE here.
GZSERVER_PGREP_PATTERN = 'gzserver.*generic_rover.world'


def gzserver_is_alive():
    try:
        return subprocess.run(
            ['pgrep', '-f', GZSERVER_PGREP_PATTERN],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        ).returncode == 0
    except Exception:
        return False


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', port))
    srv.listen(1)
    print(f"sim_heartbeat_listener listening on 127.0.0.1:{port}", flush=True)
    while True:
        conn, _ = srv.accept()
        try:
            if gzserver_is_alive():
                conn.sendall(b'ALIVE\n')
            # Else: send nothing and just close -- checkForSimDevice's
            # reply.startswith(SIM_ALIVE_REPLY) check on an empty read
            # correctly reads as "not alive", the same as no listener at
            # all, rather than a confusing partial/wrong reply.
        except Exception:
            pass
        finally:
            conn.close()


if __name__ == '__main__':
    main()
