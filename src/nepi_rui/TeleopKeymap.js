/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_rui) repo
# (see https://github.com/nepi-engine/nepi_rui)
#
# License: NEPI RUI repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
 */

// Shared source of truth for Teleop keyboard bindings -- read by
// NepiDeviceRBX-Controls.js's own Teleop control type (buildTeleopKeyMap/
// loadTeleopBindings) and meant to be written by a future Sim Connector
// keybind editor (not built yet -- this module's read-with-fallback shape
// works correctly either way, so that editor can be added later with no
// change needed here beyond an actual write call).
//
// Bindings are named ACTIONS (forward/backward/turn_left/turn_right/
// strafe_left/strafe_right/altitude_up/altitude_down), each mapped to a
// single lowercase keyboard key (matching event.key.toLowerCase()), stored
// as JSON under STORAGE_KEY in localStorage. strafe/altitude are drone-only
// axes -- a rover's own driver (rbx_sim_node.py's setTeleopVelocity) ignores
// linear_y/linear_z entirely, same as NepiDeviceRBX-Controls.js's own
// module comment documents.

const STORAGE_KEY = "nepi_teleop_key_bindings"

const DEFAULT_BINDINGS = {
  forward: "w",
  backward: "s",
  turn_left: "a",
  turn_right: "d",
  strafe_left: "q",
  strafe_right: "e",
  altitude_up: "r",
  altitude_down: "f",
}

// Read the current bindings, falling back to DEFAULT_BINDINGS for any
// action missing from a stored (possibly older/partial) binding set, and
// falling back entirely on any storage/parse error -- never let a bad
// localStorage value break Teleop.
export function loadTeleopBindings() {
  try {
    const stored = window.localStorage.getItem(STORAGE_KEY)
    if (!stored) {
      return Object.assign({}, DEFAULT_BINDINGS)
    }
    const parsed = JSON.parse(stored)
    return Object.assign({}, DEFAULT_BINDINGS, parsed)
  } catch (e) {
    return Object.assign({}, DEFAULT_BINDINGS)
  }
}

export function saveTeleopBindings(bindings) {
  try {
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify(bindings))
  } catch (e) {
    // Best-effort -- a private-browsing/storage-disabled tab just keeps
    // using whatever was already in memory, same as every other
    // localStorage use in this codebase.
  }
}

// Builds the {key: [linear_x, linear_y, linear_z, angular_z]} contribution
// map NepiDeviceRBX-Controls.js's own sendTeleopVector sums over every
// currently-held key. Rebuilt fresh (not cached) so a rebind takes effect
// the next time Teleop key capture starts, per that file's own comment.
export function buildTeleopKeyMap() {
  const bindings = loadTeleopBindings()
  const map = {}
  map[bindings.forward] = [1, 0, 0, 0]
  map[bindings.backward] = [-1, 0, 0, 0]
  map[bindings.turn_left] = [0, 0, 0, -1]
  map[bindings.turn_right] = [0, 0, 0, 1]
  map[bindings.strafe_left] = [0, -1, 0, 0]
  map[bindings.strafe_right] = [0, 1, 0, 0]
  map[bindings.altitude_up] = [0, 0, 1, 0]
  map[bindings.altitude_down] = [0, 0, -1, 0]
  return map
}
