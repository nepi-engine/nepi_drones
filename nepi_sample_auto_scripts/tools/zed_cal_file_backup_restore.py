#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Updated for current NEPI Engine API (2026-07): copy_files_from_folder moved from
# nepi_sdk.nepi_ros to nepi_sdk.nepi_utils (confirmed by reading nepi_utils.py -- same
# signature/return shape [success, files_copied, files_not_copied]; nepi_ros.py no longer
# defines this function at all).

import os
from nepi_sdk import nepi_utils


if __name__ == '__main__':
  CAL_SRC_PATH = "/usr/local/zed/settings"
  USER_CFG_PATH = "/mnt/nepi_storage/user_cfg"
  CAL_BACKUP_PATH = USER_CFG_PATH + "/zed_cals"
  # Try to backup camera calibration files
  [success,files_copied,files_not_copied] = nepi_utils.copy_files_from_folder(CAL_SRC_PATH,CAL_BACKUP_PATH)
  if success:
    #print("Backed up zed cal files")
    if len(files_copied) > 0:
      strList = str(files_copied)
      print("Backed up zed cal files: " + strList)
  else:
    print("Failed to back up up zed cal files")


    # Try to restore camera calibration files from
  [success,files_copied,files_not_copied] = nepi_utils.copy_files_from_folder(CAL_BACKUP_PATH,CAL_SRC_PATH)
  if success:
    if len(files_copied) > 0:
      strList = str(files_copied)
      print("Restored zed cal files: " + strList)
  else:
    print("Failed to restore zed cal files")


