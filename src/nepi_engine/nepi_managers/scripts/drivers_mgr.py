#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
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


import os
import glob
import sys
import subprocess
import time
import warnings
import copy
import shutil

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_serial

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header

from nepi_interfaces.msg import MgrDriversStatus, DriverStatus, UpdateBool, UpdateOrder, UpdateString, MgrSystemStatus
from nepi_interfaces.srv import DriverStatusQuery, DriverStatusQueryRequest, DriverStatusQueryResponse

from nepi_interfaces.msg import SettingsStatus, UpdateControl

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF

from nepi_api.system_if import SettingsIF


#########################################

#########################################
# Node Class
#########################################

class NepiDriversMgr(object):

  NODE_LAUNCH_TIME_SEC = 10  # Will not check node status before

  UPDATE_CHECK_INTERVAL = 1
  PUBLISH_STATUS_INTERVAL = 1
  
  drivers_param_folder = ''

  drivers_files = []
  drivers_ordered_list = []
  drivers_active_list = []
  drivers_install_folder = ''
  drivers_install_files = []
  drivers_install_list = []

  selected_driver = "NONE"
  active_paths_list = [] 


  discovery_node_dict = dict()
  discovery_classes_dict = dict()
  failed_class_import_list = []
  discovery_settings_dict = dict()

  node_if = None 
  drvs_dict = dict()
  backup_enabled = False
  retry_enabled = False

  status_msg = MgrDriversStatus()
  status_published = False

  drivers_active_list = []
  drivers_running_dict = dict()

  active_drivers = []

  settings_dict = dict()


  devices_alias_dict = dict()
  devices_alias_dict_session = dict()
  devices_cleared_alias_dict = dict()  # maps recently-cleared alias name -> original device name
  devices_running_name_list = []
  devices_running_type_list = []

  active_nodes = []
  active_topics = []
  active_topic_types = []
  active_services = []

  setting_options_dict = dict()
  setting_options_dict['SERIAL_DEVICES'] = []
  setting_options_dict['IDX_DEVICES'] = []
  setting_options_dict['PTX_DEVICES'] = []
  setting_options_dict['NPX_DEVICES'] = []
  setting_options_dict['LSX_DEVICES'] = []
  setting_options_dict['RBX_DEVICES'] = []

  nepi_config = dict()

  needs_config_save = False
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "drivers_mgr" # Can be overwitten by luanch command
  def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        nepi_sdk.sleep(1)
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################
        # Get for System Folders
        self.msg_if.pub_info("Waiting for system folders")
        system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got system folders: " + str(system_folders))

        self.drivers_param_folder = system_folders['drivers_param']
        self.msg_if.pub_info("Using Drivers Param Folder: " + str(self.drivers_param_folder))

        self.drivers_install_folder = system_folders['drivers_install']
        self.msg_if.pub_info("Using Drivers Install Folder: " + str(self.drivers_install_folder))

        self.user_cfg_folder = system_folders['user_cfg']
        self.msg_if.pub_info("Using User Config Folder: " + str(self.user_cfg_folder))
    
        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders()    
        
        ##############################
        # Wait for System Info
        self.msg_if.pub_info("Waiting for nepi config info")
        self.nepi_config = nepi_system.get_nepi_config(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got NEPI config: " + str(self.nepi_config))

        ##############################
        # Initialize Class Variables
        
        self.initCb(do_updates = False)

        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace
        }



        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'retry_enabled': {
                'namespace': self.node_namespace,
                'factory_val':  False
            },
            'backup_enabled': {
                'namespace': self.node_namespace,
                'factory_val': True
            },
            'drvs_dict': {
                'namespace': self.node_namespace,
                'factory_val': dict()
            },
            'active_drivers': {
                'namespace': self.node_namespace,
                'factory_val': []
            },
            'devices_alias_dict': {
                'namespace': self.node_namespace,
                'factory_val': self.devices_alias_dict
            },
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'driver_status_query': {
                'namespace': self.node_namespace,
                'topic': 'driver_status_query',
                'srv': DriverStatusQuery,
                'req': DriverStatusQueryRequest(),
                'resp': DriverStatusQueryResponse(),
                'callback': self.driverStatusService
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': MgrDriversStatus,
                'qsize': 1,
                'latch': True
            },
            'delete_configs': {
                'namespace': self.base_namespace,
                'topic': 'delete_configs',
                'msg': UpdateString,
                'qsize': 1,
                'latch': True
            },
            'copy_configs': {
                'namespace': self.base_namespace,
                'topic': 'copy_configs',
                'msg': String,
                'qsize': 1,
                'latch': True
            }
        }  

        '''
        nepi_sdk.create_subscriber('~install_driver_pkg', String, self.installDriverPkgCb)
        nepi_sdk.create_subscriber('~backup_on_remeove', Bool, self.enableBackupCb)
        nepi_sdk.create_subscriber('~remove_driver', String, self.removeDriverCb)
        '''


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'refresh_drivers': {
                'namespace': self.node_namespace,
                'topic': 'refresh_drivers',
                'msg': Empty,
                'qsize': 10,
                'callback': self.refreshCb, 
                'callback_args': ()
            },
            'enable_all_drivers': {
                'namespace': self.node_namespace,
                'topic': 'enable_all_drivers',
                'msg': Empty,
                'qsize': 10,
                'callback': self.enableAllCb, 
                'callback_args': ()
            },
            'disable_all_drivers': {
                'namespace': self.node_namespace,
                'topic': 'disable_all_drivers',
                'msg': Empty,
                'qsize': 10,
                'callback': self.disableAllCb, 
                'callback_args': ()
            },
            'update_driver_state': {
                'namespace': self.node_namespace,
                'topic': 'update_driver_state',
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.updateStateCb, 
                'callback_args': ()
            },
            'update_driver_order': {
                'namespace': self.node_namespace,
                'topic': 'update_driver_order',
                'msg': UpdateOrder,
                'qsize': 10,
                'callback': self.updateOrderCb, 
                'callback_args': ()
            },
            'enable_retry': {
                'namespace': self.node_namespace,
                'topic': 'enable_retry',
                'msg': Bool,
                'qsize': 10,
                'callback': self.enableRetryCb, 
                'callback_args': ()
            },
            'update_device_alias': {
                'namespace': self.base_namespace,
                'topic': 'update_device_alias',
                'msg': UpdateString,
                'qsize': None,
                'callback': self.updateDeviceAliasCb, 
                'callback_args': ()
            },
            'clear_device_alias': {
                'namespace': self.base_namespace,
                'topic': 'clear_device_alias',
                'msg': String,
                'qsize': None,
                'callback': self.resetDeviceAliasCb,
                'callback_args': ()
            },
            'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self.systemStatusCb
            }

        }




        # Create Node Class ####################
        self.node_if = NodeClassIF(
                            configs_dict = self.CFGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = self.SRVS_DICT,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                          msg_if = self.msg_if
        )

        nepi_sdk.wait()

        # drvs_dict = self.node_if.get_param("drvs_dict")
        # self.msg_if.pub_warn("Node drvs keys: " + str(drvs_dict.keys()))
        # drivers_active_list = self.getActiveDrivers()
        # self.msg_if.pub_warn("Node active drvs: " + str(drivers_active_list))
        # drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)
        # self.msg_if.pub_warn("Node ordered drvs: " + str(drvs_ordered_list))
  
        # self.msg_if.pub_error("Sleeping")
        # nepi_sdk.sleep(20)

        # drvs_dict = self.node_if.get_param("drvs_dict")
        # self.msg_if.pub_warn("Sleep drvs keys: " + str(drvs_dict.keys()))
        # drivers_active_list = self.getActiveDrivers()
        # self.msg_if.pub_warn("Sleep active drvs: " + str(drivers_active_list))
        # drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)
        # self.msg_if.pub_warn("Sleep ordered drvs: " + str(drvs_ordered_list))

        ###########################
        # Initialize Params

        self.initCb(do_updates = True)

        ###########################
        # Start node processes
        nepi_sdk.start_timer_process(0.5, self.publishStatusCb)
        nepi_sdk.start_timer_process(1.0, self.updaterDriversCb, oneshot=True)
        nepi_sdk.start_timer_process(1.0, self.updaterDevicesCb, oneshot=True)


        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever (until object is detected)
        nepi_sdk.spin()
        #########################################################







  #######################
  ### Mgr Config Functions


  def initCb(self,do_updates = False):
      self.msg_if.pub_warn("Init with do_updates: " + str(do_updates))
      drivers_active_list = []
      if self.node_if is not None:
        self.retry_enabled = self.node_if.get_param("retry_enabled")
        self.backup_enabled = self.node_if.get_param("backup_enabled")
        drvs_dict = self.node_if.get_param("drvs_dict")
        self.msg_if.pub_warn("Init drvs keys: " + str(drvs_dict.keys()))
        active_drivers = self.node_if.get_param("active_drivers")
        self.msg_if.pub_warn("active_drivers: " + str(active_drivers))


      
        drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)
        self.msg_if.pub_warn("Init ordered drvs: " + str(drvs_ordered_list))
        drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_param_folder,drvs_dict)

        for driver_name in active_drivers:
          if driver_name in drvs_dict.keys():
            self.msg_if.pub_warn("Init setting drv : " + str(driver_name) + " to active state: " + str(True))
            drvs_dict[driver_name]['active']=True

        drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)
        self.msg_if.pub_warn("Refresh ordered drvs: " + str(drvs_ordered_list))

        drivers_active_list = self.getActiveDrivers()
        self.msg_if.pub_warn("Init active drvs: " + str(drivers_active_list))

        # print_drv='NPX_MICROSTRAIN_AHAR'
        # if print_drv in drvs_dict.keys():
        #   self.msg_if.pub_warn("Refresh drv dict for drv " + print_drv + " : " + str(drvs_dict[print_drv]))

        self.drvs_dict = drvs_dict

        ###############
        # Update device names
        devices_alias_dict = self.node_if.get_param('devices_alias_dict')
        if devices_alias_dict is not None:
          self.devices_alias_dict = copy.deepcopy(devices_alias_dict)
          self.devices_alias_dict_session = copy.deepcopy(devices_alias_dict)


        if do_updates == True:
          self.refresh()
          nepi_system.set_active_drivers(drivers_active_list, log_name_list = [self.node_name])
          nepi_sdk.set_param('active_drivers', drivers_active_list)
          nepi_system.set_devices_alias_dict(self.devices_alias_dict)
          if self.node_if is not None:
            self.node_if.set_param('devices_alias_dict',self.devices_alias_dict)
        self.publish_status()
        

  def resetCb(self,do_updates = True):
      if self.node_if is not None:
          pass
      if do_updates == True:
          pass
      self.initCb(do_updates = do_updates)


  def factoryResetCb(self,do_updates = True):
      self.aifs_classes_dict = dict()
      self.aif_classes_dict = dict()
      if self.node_if is not None:
          pass
      if do_updates == True:
          pass
      self.initCb(do_updates = do_updates)



  #######################
  # Wait for System and Config Statuses Callbacks
  def systemStatusCb(self,msg):
    self.active_nodes = msg.active_nodes
    self.active_topics = msg.active_topics
    self.active_topic_types = msg.active_topic_types
    self.active_services = msg.active_services

    serial_ports = nepi_serial.get_serial_ports_list()
    avail_serial_ports = []
    for port in serial_ports:
                  if port is not None and self.active_paths_list is not None:
                    if port not in self.active_paths_list:
                      port_name = os.path.basename(port)
                      avail_serial_ports.append(port_name)
    setting_options_dict = dict()
    setting_options_dict['SERIAL_DEVICES'] = avail_serial_ports
    setting_options_dict['IDX_DEVICES'] = []
    setting_options_dict['SVX_DEVICES'] = []
    setting_options_dict['PTX_DEVICES'] = []
    setting_options_dict['NPX_DEVICES'] = []
    setting_options_dict['LSX_DEVICES'] = []
    setting_options_dict['RBX_DEVICES'] = []  
    for topic in msg.active_topics:
      if 'idx/status' in topic:
        setting_options_dict['IDX_DEVICES'].append(topic.replace('/status',''))
      if 'ptx/status' in topic:
        setting_options_dict['PTX_DEVICES'].append(topic.replace('/status',''))
      if 'svx/status' in topic:
        setting_options_dict['SVX_DEVICES'].append(topic.replace('/status',''))
      if 'npx/status' in topic:
        setting_options_dict['NPX_DEVICES'].append(topic.replace('/status',''))
      if 'lsx/status' in topic:
        setting_options_dict['LSX_DEVICES'].append(topic.replace('/status',''))
      if 'rbx/status' in topic:
        setting_options_dict['RBX_DEVICES'].append(topic.replace('/status',''))
    self.setting_options_dict = setting_options_dict


  def configStatusCb(self,msg):
    self.cfg_status = True
  

  def refreshCb(self,msg):
    self.msg_if.pub_warn("Got refresh drivers request")
    self.refresh()

  def refresh(self):
    # refresh drivers dict
    self.drivers_files = nepi_drvs.getDriverFilesList(self.drivers_param_folder)
    self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
    drvs_dict = copy.deepcopy(self.drvs_dict)
    self.msg_if.pub_warn("Refresh start drvs keys: " + str(drvs_dict.keys()))
    drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_param_folder,drvs_dict)
    self.drvs_dict = drvs_dict

    ################################
    # Create Driver Settings
    for driver_name in self.drvs_dict.keys():
      success = self.updateDriverSettingsDict(driver_name)
      self.needs_config_save = True
      self.msg_if.pub_info("Instantiated discovery settings IF class " + driver_name + " for driver " + driver_name)


    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",drvs_dict)

  def getActiveDrivers(self):
      drivers_dict = copy.deepcopy(self.drvs_dict)
      active_drivers = []
      for driver_name in drivers_dict.keys():
        if drivers_dict[driver_name]['active'] == True:
          active_drivers.append(driver_name)
      return active_drivers






  def updaterDriversCb(self,_):
    self.nepi_config = nepi_system.get_nepi_config(log_name_list = [self.node_name])
    #self.msg_if.pub_warn("Got NEPI config: " + str(self.nepi_config))


    drivers_active_list = self.getActiveDrivers()
    #self.msg_if.pub_warn("Starting Update with active drivers: " + str(drivers_active_list))

    ###############################
    ## First update Database
    #self.msg_if.pub_warn("Driver update check checking drivers folder: " + str(self.drivers_param_folder))
    drivers_files = nepi_drvs.getDriverFilesList(self.drivers_param_folder)
    #self.msg_if.pub_warn("Driver update check got driver files: " + str(drivers_files))
    self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
    need_update = self.drivers_files != drivers_files
    if need_update:
      self.msg_if.pub_warn("Need to Update Drv Database")
      self.initCb(do_updates = True)

    drvs_dict = copy.deepcopy(self.drvs_dict)
    drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)

    
    ## Next process active driver processes

    ################################    
    ## Check and purge disabled driver proccess that might be running
    # Get list of available device paths
    available_paths_list = self.getAvailableDevPaths()
    # Get list of active nodes
    warnings.filterwarnings('ignore', '.*unclosed.*', ) 
    node_namespace_list = self.active_nodes
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])
    # First check on running nodes that should not be running
    purge_list=[]
    for driver_name in drvs_ordered_list:
      if driver_name not in drivers_active_list:
        if driver_name in self.discovery_node_dict.keys():
          purge_list.append(driver_name)
        elif driver_name in self.discovery_classes_dict.keys():
          purge_list.append(driver_name)

    if len(purge_list) > 0:
      self.msg_if.pub_warn("Processing driver purge list " + str(purge_list))

    # purge from active discovery dict
    for driver_name in purge_list:
      process = self.drvs_dict[driver_name]['DISCOVERY_DICT']['process']
      if process == "LAUNCH":
        if driver_name in self.discovery_node_dict.keys():
          self.msg_if.pub_warn("Disabling driver: " + str(driver_name))
          node_name = self.discovery_node_dict[driver_name]['node_name']
          if node_name in node_list:
              self.msg_if.pub_warn("Killing driver node " + str(node_name) + " for driver: " + str(driver_name))
              sub_process = self.discovery_node_dict[driver_name]['subprocess']
              success = nepi_sdk.kill_node(node_name,sub_process = sub_process)
              if success:
                self.drvs_dict[driver_name]['running'] = False
                self.drvs_dict[driver_name]['msg'] = "Discovery process stopped"
                self.msg_if.pub_warn("Killed driver node " + str(node_name) + " for driver: " + str(driver_name))
              else:
                self.msg_if.pub_warn("Failed to kill driver node " + str(node_name) + " for driver: " + str(driver_name))
          if driver_name in self.discovery_node_dict.keys():
            del self.discovery_node_dict[driver_name]
            #self.msg_if.pub_warn("Removed driver node dict entry for driver: " + str(driver_name))

      if process == "CALL":
        if driver_name in self.discovery_classes_dict.keys():
          self.msg_if.pub_warn("Disabling driver: " + str(driver_name))
          self.msg_if.pub_warn("Killing all devices for driver: " + str(driver_name) + " active paths list " + str(self.active_paths_list))
          try:
            self.active_paths_list = self.discovery_classes_dict[driver_name].killAllDevices(self.active_paths_list)
            self.msg_if.pub_warn("Got updated active paths list " + str(self.active_paths_list))
          except Exception as e:
            self.msg_if.pub_warn("Failed to call killAllDevices function: " + str(driver_name) + " : " + str(e))
          self.msg_if.pub_warn("Deleting imported classes function for driver: " + str(driver_name))
          del self.discovery_classes_dict[driver_name]
          discovery_file = self.drvs_dict[driver_name]['DISCOVERY_DICT']['file_name']
          discovery_module = discovery_file.split('.')[0]
          self.msg_if.pub_warn("Removing Module : " + discovery_module)
          try:
            del sys.modules[discovery_module]
            if driver_name in self.failed_class_import_list:
              self.msg_if.pub_warn("Removing driver from failed imported list: " + str(driver_name))
              self.failed_class_import_list.remove(driver_name) 
              nepi_sdk.sleep(1)
          except Exception as e:
            self.msg_if.pub_warn("Failed remove driver module: " + str(driver_name) + " : " + str(e))
          
          self.drvs_dict[driver_name]['running'] = False
          self.drvs_dict[driver_name]['msg'] = "Discovery process stopped"

    ################################    
    ## Do Discovery
    #self.msg_if.pub_warn( "Checking on driver discovery for list: " + str(drvs_ordered_list))
    
    retry = self.retry_enabled
    if retry == True:
      self.failed_class_import_list = []
    for driver_name in drvs_ordered_list:
      was_running = False
      if 'running' in self.drvs_dict[driver_name].keys():
        was_running = self.drvs_dict[driver_name]['running']

      drv_dict = self.drvs_dict[driver_name]
      drv_dict['user_cfg_path'] = self.user_cfg_folder
      #self.msg_if.pub_warn( "Checking on driver discovery for: " + driver_name)
      if True: #drv_dict['DISCOVERY_DICT']['file_name'] != "None":
        discovery_path = drv_dict['path']
        discovery_file = drv_dict['DISCOVERY_DICT']['file_name']
        discovery_class_name = drv_dict['DISCOVERY_DICT']['class_name']
        discovery_process = drv_dict['DISCOVERY_DICT']['process']
        discovery_node_name = drv_dict['DISCOVERY_DICT']['node_name']        




      if driver_name in drivers_active_list:
        ############################
        # Check Node processes
        if discovery_process == "LAUNCH":  
            remove_from_dict = False

            # Update drv dict param
            dict_param_name = os.path.join(discovery_node_name, "drv_dict")
            #self.msg_if.pub_warn("Passing param name: " + dict_param_name + " drv_dict: " + str(drv_dict))
            drv_dict['retry_enabled'] =  self.retry_enabled
            nepi_sdk.set_param(dict_param_name,drv_dict)



            if driver_name not in self.discovery_node_dict.keys():
              
              if driver_name not in self.failed_class_import_list: # Check for not retry on non-running nodes

                #Try and launch node
                self.msg_if.pub_info("")
                self.msg_if.pub_info("Launching discovery process: " + discovery_node_name + " with drv_dict " + str(drv_dict))
                [success, msg, sub_process] = nepi_drvs.launchDriverNode(discovery_file, discovery_node_name)
                if success:
                  self.msg_if.pub_info("Discovery node: " + discovery_node_name  + " launched with msg: " + msg)
                  self.discovery_node_dict[driver_name]=dict()
                  self.discovery_node_dict[driver_name]['process'] = "LAUNCH"
                  self.discovery_node_dict[driver_name]['node_name'] = discovery_node_name
                  self.discovery_node_dict[driver_name]['subprocess'] = sub_process
                  self.discovery_node_dict[driver_name]['launch_time'] =  nepi_sdk.get_time()  
                  self.drvs_dict[driver_name]['running'] = False
                  self.drvs_dict[driver_name]['msg'] = "Discovery process started"
                else:
                  self.msg_if.pub_warn("Failed to Launch discovery node: " + discovery_node_name  + " with msg: " + msg)
                  self.drvs_dict[driver_name]['running'] = False
                  self.drvs_dict[driver_name]['msg'] = "Discovery process failed to start: " + msg
                  if retry == False:
                    self.failed_class_import_list.append(discovery_node_name)
                    self.msg_if.pub_warn("Will not retry discovery node launch: " + discovery_node_name )

              if remove_from_dict == True:
                if driver_name in self.discovery_node_dict.keys():
                  del self.discovery_node_dict[driver_name]
              

        ############################
        # Call Call processes 
        if discovery_process == "CALL":
          #self.msg_if.pub_warn( "Checking on driver discovery class for: " + driver_name ) #+ " with drv_dict " + str(drv_dict))
          if driver_name not in self.discovery_classes_dict.keys() and driver_name not in self.failed_class_import_list:
            self.msg_if.pub_info("")
            self.msg_if.pub_info("Importing discovery class " + discovery_class_name + " for driver " + driver_name)
            discovery_module = discovery_file.split('.')[0]
            [success, msg,imported_class] = nepi_drvs.importDriverClass(discovery_file,discovery_path,discovery_module,discovery_class_name)
            if success:
              self.msg_if.pub_info("Instantiating discovery class " + discovery_class_name + " with drv_dict " + str(drv_dict))
              discovery_class = imported_class()
              self.discovery_classes_dict[driver_name] = discovery_class
              self.msg_if.pub_info("Instantiated discovery class " + discovery_class_name + " for driver " + driver_name)
              self.drvs_dict[driver_name]['running'] = False
              self.drvs_dict[driver_name]['msg'] = "Discovery process started"
            else: 
              if retry == False:
                self.failed_class_import_list.append(driver_name)
              self.msg_if.pub_info("Failed to import discovery class " + discovery_class_name + " for driver " + driver_name)
              self.drvs_dict[driver_name]['running'] = False
              self.drvs_dict[driver_name]['msg'] = "Discovery process failed to start"
          elif driver_name not in self.failed_class_import_list:
            #self.msg_if.pub_info("")
            #self.msg_if.pub_warn("Calling discovery function for class: " + discovery_class_name + " for driver " + driver_name)
            discovery_class = self.discovery_classes_dict[driver_name]
            
            try:
              #self.msg_if.pub_info("Start discovery function with active paths: " + str(self.active_paths_list))
              self.active_paths_list = discovery_class.discoveryFunction(available_paths_list, self.active_paths_list, self.base_namespace, drv_dict, retry)
              self.drvs_dict[driver_name]['running'] = True
              self.drvs_dict[driver_name]['msg'] = "Discovery process running"
              #self.msg_if.pub_info("End discovery function with active paths: " + str(self.active_paths_list))
            except Exception as e:
              self.msg_if.pub_info("Failed to call discovery function for driver: " + driver_name + " : " + str(e))
              self.msg_if.pub_info("Disabling driver: " + driver_name)
              if driver_name in self.drvs_dict.keys():
                self.drvs_dict[driver_name]['active'] = False
                self.publish_status()
              self.msg_if.pub_info("Deleting driver discovery class for : " + driver_name)
              del self.discovery_classes_dict[driver_name]
              discovery_module = discovery_file.split('.')[0]
              self.msg_if.pub_info("Removing Module : " + discovery_module)
              del sys.modules[discovery_module]
            # if retry == False:
            #     self.failed_class_import_list.append(driver_name)
            # else:
            #   self.active_paths_list = active_paths_list


        ####################
        ## Check Driver Node Status
        if discovery_process == "LAUNCH": 
          if driver_name in self.discovery_node_dict.keys():
            node_name = self.discovery_node_dict[driver_name]['node_name']
            running = nepi_sdk.check_node_by_name(node_name)  
            self.drvs_dict[driver_name]['running'] = running
            if running == True:
              self.drvs_dict[driver_name]['msg'] = "Application running"
              if was_running == False:
                self.msg_if.pub_warn("Driver Running: " + node_name)
                self.publish_status()
            elif running == False:
              if was_running == True:
                self.drvs_dict[driver_name]['msg'] = "Application stopped running"
                self.publish_status()
          else:
            self.drvs_dict[driver_name]['running'] = False
            self.drvs_dict[driver_name]['msg'] = "Application not running"

          # do_check = False
          # if driver_name in self.discovery_node_dict.keys():
          #   # Check if still running
          #   launch_time = self.discovery_node_dict[driver_name]['launch_time']
          #   cur_time = nepi_sdk.get_time()
          #   do_check = (cur_time - launch_time) > self.NODE_LAUNCH_TIME_SEC

          # if was_running == True and do_check == True:
            
          #   self.msg_if.pub_warn("Driver Stopped Running: " + node_name)
          #   remove_from_dict = True
          #   if retry == False:
          #     self.msg_if.pub_warn("Node not running: " + node_name  + " - Will not restart")
          #     self.failed_class_import_list.append(driver_name)
          #   else:
          #     self.msg_if.pub_warn("Node not running: " + node_name  + " - Will attempt restart")
          #   drvs_dict[driver_name]['msg'] = "Application stopped running"
          #   self.drvs_dict[driver_name]['active'] = False
        
        

    last_drivers_active_list = copy.deepcopy(self.drivers_active_list)
    self.drivers_active_list = drivers_active_list
    if last_drivers_active_list != drivers_active_list:
      nepi_system.set_active_drivers(drivers_active_list, log_name_list = [self.node_name])
      nepi_sdk.set_param('active_drivers', drivers_active_list)
      self.needs_config_save = True
    #self.publish_status()
    # And now that( we are finished, start a timer for the drvt runDiscovery()
    if nepi_sdk.is_shutdown() == False:
      if self.needs_config_save == True:
        self.needs_config_save = False
        self.node_if.set_param("drvs_dict",self.drvs_dict)
        self.node_if.save_config()
      nepi_sdk.start_timer_process(1.0, self.updaterDriversCb, oneshot=True)

  def updateDriverSettingsDict(self,driver_name):
    success = False
    drvs_dict = copy.deepcopy(self.drvs_dict)
    if driver_name not in drvs_dict.keys():
       return False
    
    drv_dict = drvs_dict[driver_name]
    if 'DISCOVERY_DICT' in drv_dict.keys():
      if 'OPTIONS' in drv_dict['DISCOVERY_DICT'].keys():
        settings = drv_dict['DISCOVERY_DICT']['OPTIONS']
        self.msg_if.pub_warn("Creating driver options dict: " + driver_name + " with settings: " + str(settings))
        if driver_name in self.discovery_settings_dict.keys():
          self.msg_if.pub_info("Updating driver discovery settings for : " + driver_name)
          discovery_settings_dict = self.discovery_settings_dict[driver_name]
        else:
          self.msg_if.pub_info("Initializing driver discovery settings for : " + driver_name)
          discovery_settings_dict = dict()
          discovery_settings_dict[driver_name] = discovery_settings_dict
          discovery_settings_dict[driver_name]['settings_if'] = None
          discovery_settings_dict['namespace'] = ''

        settings_dict = self.initDiscoverySettingsDict(driver_name)
        discovery_settings_dict['settings_dict'] = settings_dict

        self.discovery_settings_dict[driver_name] = discovery_settings_dict


        if driver_name in self.discovery_settings_dict.keys():
          settings_dict = self.discovery_settings_dict[driver_name]['settings_dict']
          if settings_dict is not None:
            settings_if = None
            if 'settings_if' in self.discovery_settings_dict[driver_name].keys():
              settings_if = self.discovery_settings_dict[driver_name]['settings_if']
            if settings_if is not None:
              settings_if.init(do_updates = True)
            else:

              settings_namespace = os.path.join(self.node_namespace + '/discovery',driver_name) 
              discovery_settings_dict['namespace'] = settings_namespace  + '/settings'

              settings_if = SettingsIF(namespace = settings_namespace,
                              getSettingsFunction=self.getSettingsFunction, 
                              setSettingFunction=self.setSettingFunction, 
                              callback_arg = driver_name,
                              save_params = False,
                              msg_if = self.msg_if,
                              node_if = self.node_if
                              )
              self.discovery_settings_dict[driver_name]['settings_if'] = settings_if

    return True
          


  ##################################
  #### Driver Settings



  def initDiscoverySettingsDict(self, driver_name):
      # Returns (init_settings_dict, settings) in the string-valued settings form.
      settings_dict = None
      init_settings_dict = dict()
      settings = dict()
      drvs_dict = copy.deepcopy(self.drvs_dict)
      if driver_name not in drvs_dict.keys():
        return settings_dict
      options_dict = None
      try:
        options_dict = drvs_dict[driver_name]['DISCOVERY_DICT']['OPTIONS']
      except:
         self.msg_if.pub_info("Driver does not have discovery options : " + str(driver_name))
      #self.msg_if.pub_info("Got driver discovery options : " + str(options_dict))
      if options_dict is not None:
        for setting_name in options_dict.keys():
          setting = options_dict[setting_name]
          setting_type = setting['type']
          options = setting.get('options',[])
          bounds = [-999,-999]
          if setting_type == 'Discrete':
            setting_type = 'Selection'
            

          if setting_type == 'Int' or setting_type == 'Float':
            try:
              bounds[0] = options[0]
            except:
              pass
            try:
              bounds[1] = options[1]
            except:
              pass
            
          setting_dict = {'name': setting_name,
                        'type': setting_type,
                        'default': setting['default'],
                        'options': options,
                        'bounds': bounds}
          #self.msg_if.pub_info("Adding driver discovery setting : " + str(setting_dict))
          init_settings_dict[setting_name] = setting_dict
        #self.msg_if.pub_info("Calling create driver discovery settings with : " + str(init_settings_dict))
        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        
        if 'value' in options_dict.keys():
          value = options_dict['value']
          settings_dict[driver_name]['set_value'] = value
      #self.msg_if.pub_info("Initialized driver discovery settings dict : " + str(settings_dict))
      return settings_dict

  

   
  def setSettingFunction(self,setting_name, setting_value, args):
      driver_name = args
      success = False
      msg = 'Setting name not in drv options'

      settings_dict = ()
      if driver_name in self.discovery_settings_dict.keys():
        settings_dict = copy.deepcopy(self.discovery_settings_dict[driver_name]['settings_dict'])
        drvs_dict = copy.deepcopy(self.drvs_dict)
        if driver_name in drvs_dict.keys():
          drv_dict = drvs_dict[driver_name]
          if 'DISCOVERY_DICT' in drv_dict.keys():
            if 'OPTIONS' in drv_dict['DISCOVERY_DICT'].keys():
              options_dict = drv_dict['DISCOVERY_DICT']['OPTIONS']
              if setting_name in options_dict.keys():
                self.drvs_dict[driver_name]['DISCOVERY_DICT']['OPTIONS'][setting_name]['value'] = setting_value
                success = True
                msg = 'Success'
                self.needs_config_save = True
                settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, setting_value, check_valid = False)



      return success, msg, settings_dict

  def getSettingsFunction(self,driver_name):
      settings_dict = ()
      drvs_dict = copy.deepcopy(self.drvs_dict)
      if driver_name not in self.discovery_settings_dict.keys() or driver_name not in drvs_dict.keys():
        return settings_dict

      settings_dict = copy.deepcopy(self.discovery_settings_dict[driver_name]['settings_dict'])
      if settings_dict is None:
          return settings_dict

      drv_dict = drvs_dict[driver_name]
      options_dict = None
      try:
        options_dict = drv_dict['DISCOVERY_DICT']['OPTIONS']
      except:
          pass
      if options_dict is None:
        return settings_dict

      if len(options_dict) == 0:
         return settings_dict
      
      #self.msg_if.pub_info("Starting Return driver discovery settings dict : " + str(settings_dict))
      options = []
      value = None

      for option_name in options_dict.keys():
        option_dict = options_dict[option_name]
        options = option_dict['options']
        setting_options_dict = copy.deepcopy(self.setting_options_dict)
        if len(options) == 1:
          options_key = options[0]
          if options_key in setting_options_dict.keys():
            options = setting_options_dict[options_key]

        if 'value' in option_dict.keys():
          value = option_dict['value']

        if value is None:
          value = option_dict['default']

        self.drvs_dict[driver_name]['DISCOVERY_DICT']['OPTIONS'][option_name]['options'] = options
        self.drvs_dict[driver_name]['DISCOVERY_DICT']['OPTIONS'][option_name]['value'] = value


        if option_name in settings_dict.keys():
          #self.msg_if.pub_info("Updating driver discovery settings with : " + str([option_name,value,options]))
          setting_type = settings_dict[option_name]['type']
          if (setting_type == 'Int' or setting_type == 'Float') and len(options) > 1:
              settings_dict = nepi_controls.set_control_bounds(settings_dict, option_name, [options[0], options[1]])
          if setting_type == 'Selection':
              settings_dict = nepi_controls.set_control_options(settings_dict, option_name, options)
          
          settings_dict = nepi_controls.set_control_value(settings_dict, option_name, value, check_valid = False)
      #self.msg_if.pub_warn("Returning driver discovery settings dict : " + str(settings_dict))
      return settings_dict

   
  
  ###################
  ## Drivers Mgr Callbacks
  def enableAllCb(self,msg):
    self.msg_if.pub_info("Got enable all msg: " + str(msg))
    drvs_names = list(self.drvs_dict.keys())
    for driver_name in drvs_names:
      self.drvs_dict[driver_name]['active'] = True
    self.failed_class_import_list = []
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",self.drvs_dict)
      self.node_if.save_config() # Save config on options change

  def disableAllCb(self,msg):
    self.msg_if.pub_info("Got disable all msg: " + str(msg))
    drvs_names = list(self.drvs_dict.keys())
    for driver_name in drvs_names:
      self.drvs_dict[driver_name]['active'] = False
    self.failed_class_import_list = []
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",self.drvs_dict)
      self.node_if.save_config() # Save config on options change


  def updateStateCb(self,msg):
    self.msg_if.pub_info("Got update driver state msg: " + str(msg))
    driver_name = msg.name
    new_enabled = msg.value
    drvs_names = list(self.drvs_dict.keys())
    if driver_name in drvs_names:
      self.drvs_dict[driver_name]['active'] = new_enabled
      if driver_name in self.failed_class_import_list:
        self.failed_class_import_list.remove(driver_name)
      self.msg_if.pub_warn("State Update setting drv : " + str(driver_name) + " to active state: " + str(new_enabled))
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",self.drvs_dict)
      self.node_if.save_config() # Save config on options change



  def updateOrderCb(self,msg):
    self.msg_if.pub_info("Got Update driver order msg: " + str(msg))
    driver_name = msg.name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    drvs_dict = copy.deepcopy(self.drvs_dict)
    if driver_name in drvs_dict.keys() and driver_name != 'None':
      drvs_dict = moveFunction(driver_name,drvs_dict)
      self.drvs_dict = drvs_dict
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param("drvs_dict",self.drvs_dict)
        self.node_if.save_config() # Save config on options change


  def getOrderUpdateFunction(self,move_cmd):
    if move_cmd == 'top':
      updateFunction = nepi_drvs.moveDriverTop
    elif move_cmd == 'bottom':
      updateFunction = nepi_drvs.moveDriverBottom
    elif move_cmd == 'up':
      updateFunction = nepi_drvs.moveDriverUp
    elif move_cmd == 'down':
      updateFunction = nepi_drvs.moveDriverDown
    else:
      updateFunction = self.moveDriverNone
    return updateFunction

  def moveDriverNone(self,driver_name,drvs_dict):
    return drvs_dict


 
  def enableRetryCb(self,msg):
    self.msg_if.pub_info(str(msg))
    self.retry_enabled = msg.data
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("retry_enabled",self.retry_enabled)
      self.node_if.save_config() # Save config on options change
    


  ################################
  ## Device Update Functions


  def updaterDevicesCb(self,_):
    running_service_namespaces = nepi_sdk.find_services_by_name('device_info_query')
    devices_running_name_list = []
    devices_running_type_list = []
    for service_namespace in running_service_namespaces:
        service_base_namespace = os.path.dirname(service_namespace)

        device_type = os.path.basename(service_base_namespace).upper()
        devices_running_type_list.append(device_type)

        device_namespace = os.path.dirname(service_base_namespace)
        device_name = os.path.basename(device_namespace)
        devices_running_name_list.append(device_name)
        

    
    self.devices_running_name_list = devices_running_name_list
    self.devices_running_type_list = devices_running_type_list
    if nepi_sdk.is_shutdown() == False:
      nepi_sdk.start_timer_process(1.0, self.updaterDevicesCb, oneshot=True)




  def updateDeviceAliasCb(self, msg):
      self.msg_if.pub_info("Got Update Device Alias msg: " + str(msg))
      if True: #self.admin_mode_set == True:
          device_name = msg.name
          device_alias = nepi_utils.get_clean_name(msg.value)
          if device_name != '' and device_alias != '':
              is_valid = False
              count = 0
              while is_valid == False:
                  is_valid = True
                  for node_name in self.devices_alias_dict.keys():
                      if device_alias == self.devices_alias_dict[node_name]:
                          is_valid = False
                  if is_valid == False:
                      count = count + 1
                      device_alias = device_alias + '_' + str(count)
              self.msg_if.pub_info("Updating Device " + device_name + " with alias: " + str(device_alias))
              self.devices_alias_dict[device_name] = device_alias
              self.devices_alias_dict_session[device_name] = device_alias
              self.clear_config(device_alias)
              self.copy_config(device_name, device_alias)
          self.publish_status()
          if self.node_if is not None:
              self.node_if.set_param('devices_alias_dict',self.devices_alias_dict)
              self.node_if.save_config()

              #nepi_system.set_devices_alias_dict(self.devices_alias_dict)
          if self.node_if is not None:
              msg = UpdateString()
              msg.name = os.path.join(self.base_namespace,device_name)
              msg.value = os.path.join(self.base_namespace,device_alias)
              self.node_if.publish_pub('copy_configs', msg)


  def resetDeviceAliasCb(self, msg):
      self.msg_if.pub_info("Got Clear Device Alias msg: " + str(msg))
      if True: #self.admin_mode_set == True:
          purge_device = None
          purge_alias = None
          reset_device_name = msg.data
          for device_name in self.devices_alias_dict.keys():
                  if reset_device_name == device_name or reset_device_name == self.devices_alias_dict[device_name]:
                    purge_device = device_name
                    purge_alias = self.devices_alias_dict[device_name]
          if purge_device is not None:
              success = False
              self.msg_if.pub_info("Clearing Alias for Device " + purge_device)
              try:
                
                del self.devices_alias_dict[purge_device]
                ### ONLY DELETE FROM BASE DICT, NOT SESSION DICTIONARY.
                self.devices_alias_dict_session[purge_device] = purge_device
                # The live node keeps running under the old alias name until its next
                # startup. Remember the mapping so publish_status folds that running node
                # back into this device's row instead of showing a duplicate.
                if purge_alias is not None and purge_alias != purge_device:
                    self.devices_cleared_alias_dict[purge_alias] = purge_device
                self.publish_status()
                success = True
              except Exception as e:
                self.msg_if.pub_info("Failed to Clear Device Alias: " + str(e))
              if success == True and self.node_if is not None and purge_alias is not None:
                  msg = UpdateString()
                  msg.name = os.path.join(self.base_namespace,purge_alias)
                  msg.value = os.path.join(self.base_namespace,purge_device)
                  self.node_if.publish_pub('copy_configs', msg)
                  nepi_sdk.sleep(1)
                  delete_msg = UpdateString()
                  delete_msg.name = msg.name
                  self.node_if.publish_pub('delete_configs', delete_msg)
                  
          self.publish_status()
          if self.node_if is not None:

              self.node_if.set_param('devices_alias_dict',self.devices_alias_dict)
              self.node_if.save_config()
              #nepi_system.set_devices_alias_dict(self.devices_alias_dict)



  def clear_config(self, name):
      user_cfg = self.user_cfg_folder
      prefix = name + '-'
      try:
          for fname in os.listdir(user_cfg):
              if fname.endswith('.yaml') and (fname.startswith(prefix) or fname == name + '.yaml'):
                  fpath = os.path.join(user_cfg, fname)
                  try:
                      os.remove(fpath)
                      self.msg_if.pub_info("clear_config: deleted " + fname)
                  except Exception as e:
                      self.msg_if.pub_warn("clear_config: failed to delete " + fname + ": " + str(e))
      except Exception as e:
          self.msg_if.pub_warn("clear_config: failed scanning " + user_cfg + ": " + str(e))

  def copy_config(self, name, new_name):
      user_cfg = self.user_cfg_folder
      prefix = name + '-'
      try:
          for fname in os.listdir(user_cfg):
              if fname.endswith('.yaml') and (fname.startswith(prefix) or fname == name + '.yaml'):
                  src = os.path.join(user_cfg, fname)
                  new_fname = new_name + fname[len(name):]
                  dst = os.path.join(user_cfg, new_fname)
                  try:
                      shutil.copy2(src, dst)
                      self.msg_if.pub_info("copy_config: " + fname + " -> " + new_fname)
                  except Exception as e:
                      self.msg_if.pub_warn("copy_config: failed to copy " + fname + ": " + str(e))
      except Exception as e:
          self.msg_if.pub_warn("copy_config: failed scanning " + user_cfg + ": " + str(e))

  ########################

  def installDriverPkgCb(self,msg):
    self.msg_if.pub_info(str(msg))
    pkg_name = msg.data
    drvs_dict = copy.deepcopy(self.drvs_dict)
    if pkg_name in self.drivers_install_files:
     [success,drvs_dict]  = nepi_drvs.installDriverPkg(pkg_name,drvs_dict,self.drivers_install_folder,self.drivers_param_folder)
    self.drvs_dict = drvs_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",self.drvs_dict)
      self.node_if.save_config()

  def removeDriverCb(self,msg):
    self.msg_if.pub_info(str(msg))
    driver_name = msg.data
    drvs_dict = copy.deepcopy(self.drvs_dict)
    backup_folder = None
    backup_enabled = self.backup_enabled
    if backup_enabled:
      backup_folder = self.drivers_install_folder
    if driver_name in drvs_dict:
      [success,drvs_dict] = nepi_drvs.removeDriver(driver_name,drvs_dict,self.drivers_param_folder,backup_path = backup_folder)
    self.drvs_dict = drvs_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",self.drvs_dict)
      self.node_if.save_config()


  def enableBackupCb(self,msg):
    self.msg_if.pub_info(str(msg))
    self.backup_enabled = msg.data
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("backup_enabled",self.backup_enabled)
      self.node_if.save_config()
    




  #######################
  # Misc Utility Function

  def getAvailableDevPaths(self):
    dev_path_list = glob.glob('/dev/*')
    return dev_path_list
  



  ###############################################




        
  # ln = sys._getframe().f_lineno ; 
  def printND(self):
    drvs_dict = copy.deepcopy(self.drvs_dict)
    self.msg_if.pub_info('')
    self.msg_if.pub_info('*******************')
    self.msg_if.pub_info('Printing Drv Driver Dictionary')
    for driver_name in drvs_dict.keys():
      drv_dict = drvs_dict[driver_name]
      self.msg_if.pub_info('')
      self.msg_if.pub_info()
      self.msg_if.pub_info(str(drv_dict))



  def getDriverStatusMsg(self,driver_name,drvs_dict):

    driver_status_msg = DriverStatus()
    driver_status_msg.driver_name = driver_name
    #self.msg_if.pub_warn("################")
    #self.msg_if.pub_warn("Creating Drv Status Msg for driver: " + str(driver_name))


    #self.msg_if.pub_warn("Checking for entry in settings dict keys: " + str(self.discovery_settings_dict.keys()))
    #self.msg_if.pub_warn("")
    settings_topic = ''
    if driver_name in self.discovery_settings_dict.keys():
      discovery_settings_dict = self.discovery_settings_dict[driver_name]

      #self.msg_if.pub_warn("Creating Drv Status Msg from Dict: " + str(discovery_dict))
      #self.msg_if.pub_warn("")
      if 'namespace' in discovery_settings_dict.keys():
        settings_topic = discovery_settings_dict['namespace']
    driver_status_msg.settings_topic = settings_topic

    if driver_name in drvs_dict.keys() and driver_name != 'NONE':
      drv_dict = drvs_dict[driver_name]
      #self.msg_if.pub_warn("Creating Drv Status Msg from Dict: " + str(drv_dict))
      #self.msg_if.pub_warn("")
      try:
        driver_status_msg.display_name = drv_dict['display_name']
        driver_status_msg.description = drv_dict['description']

        driver_status_msg.type = drv_dict['type']
        driver_status_msg.group_id = drv_dict['group_id']

        driver_status_msg.enabled  = drv_dict['active']
        running = False
        if 'running' in drv_dict.keys():
          running = drv_dict['running']
        driver_status_msg.running = running
        driver_status_msg.order  = drv_dict['order']
        driver_status_msg.msg_str = drv_dict['msg']
      except Exception as e:
        self.msg_if.pub_warn("Failed to create drv status msg for : " + str(drv_dict) + " " + str(e), throttle_s = 10)
    # if self.status_published == False:
    #     self.msg_if.pub_info("Got Driver Status Msg: " + str(driver_status_msg))
    #     self.msg_if.pub_warn("")

    #self.msg_if.pub_warn("Got Driver Status Msg: " + str(driver_status_msg))
    #self.msg_if.pub_warn("")
    #self.msg_if.pub_warn("")
    #self.msg_if.pub_warn("")
    return driver_status_msg


  def driverStatusService(self,request):
    driver_name = request.driver_name
    drvs_dict = copy.deepcopy(self.drvs_dict)
    response = self.getDriverStatusMsg(driver_name,drvs_dict)
    return response

  def publishStatusCb(self,timer):
      self.publish_status()

  def publish_status(self, do_updates = True):
    
    last_status_msg = copy.deepcopy(self.status_msg)
    
    drvs_dict = copy.deepcopy(self.drvs_dict)
    #self.msg_if.pub_warn("Got drivers status start drvs keys: " + str(drvs_dict.keys()), throttle_s = 5.0)
    drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)
    #self.msg_if.pub_warn("MgrDriversStatus Drv List: " + str(drvs_ordered_list), throttle_s = 5.0)
    drivers_active_list = self.getActiveDrivers()
    #self.msg_if.pub_warn("MgrDriversStatus Active List: " + str(drivers_active_list), throttle_s = 5.0)
    status_msg = MgrDriversStatus()

    status_msg.drivers_ordered_list = drvs_ordered_list

    name_list = []
    type_list = []
    group_id_list = []
    status_list = []
    for driver_name in drvs_ordered_list:      
      name_list.append(drvs_dict[driver_name]['display_name'])
      type_list.append(drvs_dict[driver_name]['type'])
      group_id_list.append(drvs_dict[driver_name]['group_id'])
      status_list.append(self.getDriverStatusMsg(driver_name,drvs_dict))
    status_msg.drivers_ordered_name_list = name_list
    status_msg.drivers_ordered_type_list = type_list
    status_msg.drivers_ordered_group_id_list = group_id_list
    status_msg.drivers_ordered_status_list = status_list


    type_list = []
    name_list = []
    for driver_name in drivers_active_list:
      type_list.append(drvs_dict[driver_name]['type'])
      name_list.append(drvs_dict[driver_name]['display_name'])
    status_msg.drivers_active_list = drivers_active_list
    status_msg.drivers_active_type_list = type_list
    status_msg.drivers_active_name_list = name_list


    running_list = []
    for driver_name in drvs_ordered_list:
      if 'running' in drvs_dict[driver_name].keys():
        if drvs_dict[driver_name]['running'] == True:
          running_list.append(driver_name)
    status_msg.drivers_running_list = running_list

    status_msg.retry_enabled = self.retry_enabled


    #########################
    ## Devices Info
    devices_alias_dict = copy.deepcopy(self.devices_alias_dict_session)
    devices_name_list = []
    devices_alias_list = []
    for device_name in devices_alias_dict.keys():
      devices_name_list.append(device_name)
      devices_alias_list.append(devices_alias_dict[device_name])

    # Reconcile recently-cleared aliases. A device aliased to e.g. "USB_CAM" keeps
    # running under that node name until its next startup, so after its alias is cleared
    # the live node would otherwise show up as a brand-new duplicate device. Map the
    # running node name back to the original device name so it folds into that row.
    raw_running_name_list = copy.deepcopy(self.devices_running_name_list)
    # Drop stale mappings whose aliased node is no longer running (restarted/removed).
    for alias_name in list(self.devices_cleared_alias_dict.keys()):
      if alias_name not in raw_running_name_list:
        del self.devices_cleared_alias_dict[alias_name]
    devices_running_name_list = []
    for running_name in raw_running_name_list:
      devices_running_name_list.append(self.devices_cleared_alias_dict.get(running_name, running_name))

    for device_name in devices_running_name_list:
      if device_name not in devices_name_list and device_name not in devices_alias_list:
        devices_name_list.append(device_name)
        devices_alias_list.append(device_name)


    status_msg.devices_name_list = devices_name_list
    status_msg.devices_alias_list = devices_alias_list
    status_msg.devices_running_name_list = devices_running_name_list
    status_msg.devices_running_type_list = self.devices_running_type_list


    self.status_msg = status_msg
    if self.node_if is not None:
      if self.status_published == False:
        self.status_published = True
        #self.msg_if.pub_info("Publishing Status Msg: " + str(self.status_msg))
      self.node_if.publish_pub('status_pub', self.status_msg)
      if do_updates == True:
        if last_status_msg != self.status_msg:
          #self.msg_if.pub_info("Saving Drivers Mgr Config")
          self.node_if.save_config() # Save config after initialization for drvt time

  





  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiDriversMgr()







