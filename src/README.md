Files and folders in this folder are synced to NEPI's source-code build folder (/mnt/nepi_storage/nepi_src/nepi_engine_ws/src) and copied over it at build time: each folder here lands on the build workspace folder of the same name (nepi_apps, nepi_drivers, nepi_rui, nepi_api).

This folder holds drones-only files and nothing else: apps, drivers, and RUI files that have no counterpart in the engine repos. As of 2026-10-15 it contains no copy of any engine file. Every build copies these files with no staleness check, so a copy of an engine file silently reverts the engine to whatever version was copied, and breaks the build or the running system once the engine moves on. That is how a stale copy of device_if_rbx.py crashed rbx_sim_node with "'RBXCapabilitiesQueryResponse' object has no attribute 'device_name'" after the engine's capabilities refactor.

Rules for this folder:

- A change to an engine file goes into the engine repo that owns it (nepi_engine, nepi_drivers, nepi_rui, nepi_apps, nepi_interfaces), committed there. Do not copy the engine file here to change it.
- A new drones-only file goes here only if its path does not exist in the engine. If the engine later adds a file at the same path, move the drone version into the engine and delete it here.
- Before adding a file, check its mapped engine path: nepi_api/X -> src/nepi_engine/nepi_api/src/nepi_api/X, nepi_drivers/X -> src/nepi_drivers/X, nepi_apps/X -> src/nepi_apps/X, nepi_rui/X -> src/nepi_rui/src/rui_webserver/rui-app/src/X. If that path exists, the change belongs in the engine.
- There is no nepi_engine folder here; the build never copied one (no matching build symlink), so changes to managers or the SDK go straight into the nepi_engine repo.

What is here now:

- nepi_apps/nepi_app_sim_connector -- the simulator connector app (SimDeviceIF, Robot Link, simulator launcher, RUI).
- nepi_apps/nepi_app_fake_gps -- the per-device Fake GPS app the ArduPilot driver reconciles against.
- nepi_drivers/rbx_drivers -- the rbx_sim, rbx_webots and rbx_webots_quadcopter drivers. The ArduPilot driver (rbx_ardupilot, including its SITL entry rbx_ardupilot_sitl_params.yaml) and rbx_gazebo live in the engine's nepi_drivers repo.
- nepi_rui/Nepi_IF_ConnectProcess.js -- not imported by any engine page.

On 2026-10-15 every former overlay copy was merged into the engine repos or deleted. A build workspace that was already built with the old copies still has them on disk: redeploy this repo and resync the engine source into the build workspace before the next build.
