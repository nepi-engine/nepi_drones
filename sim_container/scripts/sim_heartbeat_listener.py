#!/usr/bin/env python3
"""Tiny liveness listener for the generic-rover Gazebo simulation.

Listens on 127.0.0.1:<port> and replies ALIVE to every connection received,
then closes it. Deliberately NOT a ROS node: the remote NEPI device and this
dev VM run separate ROS masters bridged only by a reverse SSH tunnel that
forwards raw TCP ports, so the NEPI rbx_sim driver's discovery cannot see
this VM's ROS graph (no /sim/heartbeat topic) -- it probes this plain TCP
port instead. The ALIVE reply matters: with an ssh -R forward, a connect()
on the device side succeeds against the device's sshd even when nothing is
listening here, so discovery must read the reply, not just connect.

Started and stopped by the sim_heartbeat_listener function in
sim_rover_dev_env.sh as part of sim_rover_gazebo, alongside roscore, Gazebo,
and sim_bridge_node.py -- so reachability of this port means the rover sim
stack is up. Modeled on gz_reset_listener.py (same pattern, ArduPilot
workflow).
"""

import socket
import sys

DEFAULT_PORT = 9022


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
            conn.sendall(b'ALIVE\n')
        except Exception:
            pass
        finally:
            conn.close()


if __name__ == '__main__':
    main()
