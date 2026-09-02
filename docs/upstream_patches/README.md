# Upstream patches

Patch files for commits made directly in the **production** `nepi_drivers`
repo (`nepi_engine_ws/src/nepi_drivers`) that couldn't be pushed to
`origin/main` there from this checkout (no write access), so they'd
otherwise only exist as an unpushed local commit at risk of being lost.

These are records of real, already-applied fixes -- not something to
apply from here. If the corresponding fix isn't already present in this
sandbox's own copy of the same file (check first), port it over by hand;
if it is already present, the patch is just a durable copy of the
reasoning/commit message for reference.

| Patch | Production commit | Sandbox status as of the patch date |
|---|---|---|
| `nepi_drivers-3061ee2-fix-ardupilot-goto_pose-seq-and-forced-descent.patch` | `3061ee2` in `nepi_drivers` (unpushed as of 2026-09-01) | Fix already present in this repo's `src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py` -- no action needed, kept for the record. |
