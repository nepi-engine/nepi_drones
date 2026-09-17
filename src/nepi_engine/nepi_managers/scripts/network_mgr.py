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

import socket
import subprocess
import collections
import os
from datetime import datetime
import threading
import requests
import time
import copy
import ipaddress

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import MgrNetworkStatus
from nepi_interfaces.msg import MgrSystemStatus
from nepi_interfaces.msg import Reset, NetworkWifiCredentials
from nepi_interfaces.srv import IPAddrQuery, IPAddrQueryRequest, IPAddrQueryResponse
from nepi_interfaces.srv import BandwidthUsageQueryRequest, BandwidthUsageQuery, BandwidthUsageQueryResponse
from nepi_interfaces.srv import WifiQuery, WifiQueryRequest, WifiQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


class NetworkMgr:
    """The Network Manager Node of the NEPI core SDK.

    This node controls network IP settings. Users are not able to override the factory configuration
    but they can add and remove additional IPv4 addresses.
    """
    
    NEPI_ETC_UPDATE_SCRIPTS_PATH = "/opt/nepi/etc/scripts"

    NET_IFACE = "eth0"
    WONDERSHAPER_CALL = "/opt/nepi/nepi_engine/share/wondershaper/wondershaper"

    STATIC_IP_FILE = "/opt/nepi/etc/network/interfaces.d/nepi_static_ip"
    ALIASES_IP_FILE = "/opt/nepi/etc/network/interfaces.d/nepi_user_ip_aliases"
    USER_IP_ALIASES_FILE_PREFACE = "# This file includes all user-added IP address aliases. It is sourced by the top-level static IP addr file.\n\n"

    # Following are to support changing rosmaster IP address
    # SYS_ENV_FILE = "/opt/nepi/sys_env.bash"
    ROS_MASTER_PORT = 11311
    ROSLAUNCH_FILE = "/opt/nepi/nepi_engine/etc/roslaunch.sh"
    REMOTE_ROS_NODE_ENV_LOADER_FILES = ["numurus@num-sb1-zynq:/opt/nepi/nepi_engine/etc/env_loader.sh"]

    # Following support WiFi AP setup
    CREATE_AP_CALL = "/opt/nepi/nepi_engine/share/create_ap/create_ap"
    DEFAULT_WIFI_AP_SSID = "nepi_device_ap"
    DEFAULT_WIFI_AP_PASSPHRASE = "nepi_device_ap"


    STOP_WPA_SUPPLICANT_CMD = ['killall', 'wpa_supplicant']
    
    BANDWIDTH_MONITOR_PERIOD_S = 5.0
    UPDATER_INTERVAL_S = 1.0

    node_if = None

    status_msg = MgrNetworkStatus()
    tx_bw_limit_mbps = -1.0


    
    found_ip_addrs = []
    last_ip_addrs = None

    clock_skewed = False

    last_networks = []
 

    primary_ip_addr = '192.168.179.103/24'
    
    managed_ip_addrs = []
    last_ip_addrs = None
    dhcp_enabled = False # initialize to false -- will be updated in set_dhcp_enabl
    dhcp_connecting = False
    dhcp_enable_state = False
    dhcp_ip_addr = ''

    tx_bw_limit_mbps = -1

    wifi_iface = None

    wifi_enabled = True
    wifi_low_power_enabled = True
    wifi_ready = False
    wifi_client_enabled = False
    wifi_client_retrty = False
    wifi_client_ssid = 'NONE'
    wifi_client_passphrase = ''
    
    wifi_client_connecting = False
    wifi_client_trying = False
    wifi_client_connected = False

    wifi_ap_ready = False

    wifi_ap_enabled = False
    wifi_ap_ssid = 'nepi_device_ap'
    wifi_ap_passphrase = 'nepi_device_ap'

    wifi_ap_connecting = False
    wifi_ap_running = False
    wifi_ap_running_ssid = None


    wifi_scanning = False
    wifi_available_networks = []

    internet_connected = False
    

    network_checked = False
    internet_checked = False
    
    network_lock = threading.Lock()
    wifi_lock = threading.Lock()
    internet_lock = threading.Lock()


    in_container = False

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "network_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        nepi_sdk.sleep(1)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        
        ##############################
        # Wait for System Info
        self.msg_if.pub_info("Waiting for nepi config info")
        self.nepi_config = self.get_nepi_system_config()
            
        try:
            self.in_container = self.nepi_config['NEPI_IN_CONTAINER']
            self.msg_if.pub_warn("Got NEPI In Container: " + str(self.in_container))

            self.manages_network = self.nepi_config['NEPI_MANAGES_NETWORK'] == 1
            self.msg_if.pub_warn("Got NEPI Manages Network: " + str(self.manages_network))

            self.start_dhcp_on_startup=self.nepi_config['NEPI_WIRED_DHCP_ENABLED']
            self.start_wifi_client_on_startup=self.nepi_config['NEPI_WIFI_CLIENT_ENABLED']

            self.start_wifi_ap_on_startup = False

            self.start_wifi_ap_on_startup=self.nepi_config['NEPI_HOTSPOT_ENABLED'] == 1
        except Exception as e:
             self.msg_if.pub_warn("Failed to Get System Config Info " + str(e))     
        


        self.nepi_config_path = self.nepi_config['NEPI_CONFIG']

        self.nepi_system_etc_path = self.nepi_config_path + "/system_cfg/etc"
        self.nepi_system_config_file = self.nepi_system_etc_path + "/nepi_system_config.yaml"

        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders() 

        ##############################
        # Initialize Variables

        self.tx_byte_cnt_deque = collections.deque(maxlen=2)
        self.rx_byte_cnt_deque = collections.deque(maxlen=2)
        

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
        self.PARAMS_DICT = None
        # self.PARAMS_DICT = {
        #     'managed_ip_addrs': {
        #         'namespace': self.node_namespace,
        #         'factory_val': []
        #     },
        #     'tx_bw_limit_mbps': {
        #         'namespace': self.node_namespace,
        #         'factory_val': -1.0
        #     },
        #     'dhcp_enabled': {
        #         'namespace': self.node_namespace,
        #         'factory_val': self.dhcp_enabled
        #     },
        #     'wifi_enabled': {
        #         'namespace': self.node_namespace,
        #         'factory_val': self.wifi_enabled
        #     },
        #     'wifi_low_power_enabled': {
        #         'namespace': self.node_namespace,
        #         'factory_val': self.wifi_low_power_enabled
        #     },
        #     'wifi_client_enabled': {
        #         'namespace': self.node_namespace,
        #         'factory_val': self.wifi_client_enabled
        #     },
        #     'wifi_client_ssid': {
        #         'namespace': self.node_namespace,
        #         'factory_val': self.wifi_client_ssid
        #     },
        #     'wifi_client_passphrase': {
        #         'namespace': self.node_namespace,
        #         'factory_val': self.wifi_client_passphrase
        #     },
        #    'wifi_wifi_ap_enabled': {
        #         'namespace': self.node_namespace,
        #         'factory_val': self.wifi_ap_enabled
        #     },
        #     'wifi_ap_ssid': {
        #         'namespace': self.node_namespace,
        #         'factory_val': self.wifi_ap_ssid
        #     },
        #     'wifi_ap_passphrase': {
        #         'namespace': self.node_namespace,
        #         'factory_val': self.wifi_ap_passphrase
        #     }
        # }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'ip_addr_query': {
                'namespace': self.base_namespace,
                'topic': 'ip_addr_query',
                'srv': IPAddrQuery,
                'req': IPAddrQueryRequest(),
                'resp': IPAddrQueryResponse(),
                'callback': self.handle_ip_addr_query
            },
            'bandwidth_usage_query': {
                'namespace': self.base_namespace,
                'topic': 'bandwidth_usage_query',
                'srv': BandwidthUsageQuery,
                'req': BandwidthUsageQueryRequest(),
                'resp': BandwidthUsageQueryResponse(),
                'callback': self.handle_bandwidth_usage_query
            },
            'wifi_query': {
                'namespace': self.base_namespace,
                'topic': 'wifi_query',
                'srv': WifiQuery,
                'req': WifiQueryRequest(),
                'resp': WifiQueryResponse(),
                'callback': self.handle_wifi_query
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': MgrNetworkStatus,
                'qsize': 1,
                'latch': True
            },
            'saveParamsCb': {
                'namespace': self.base_namespace,
                'topic': 'saveParamsCb',
                'msg': String,
                'qsize': 1,
                'latch': False
            },
            'save_system_config': {
                'namespace': self.base_namespace,
                'topic': 'save_system_config',
                'msg': Empty,
                'qsize': 1,
                'latch': False
            }            
        }  

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'add_ip_addr': {
                'namespace': self.base_namespace,
                'topic': 'add_ip_addr',
                'msg': String,
                'qsize': 10,
                'callback': self.addIpCb, 
                'callback_args': ()
            },
            'remove_ip_addr': {
                'namespace': self.base_namespace,
                'topic': 'remove_ip_addr',
                'msg': String,
                'qsize': 10,
                'callback': self.removeIpCb, 
                'callback_args': ()
            },
            'enable_dhcp': {
                'namespace': self.base_namespace,
                'topic': 'enable_dhcp',
                'msg': Bool,
                'qsize': 10,
                'callback': self.enableDhcpCb, 
                'callback_args': ()
            },
            'limit_mbps': {
                'namespace': self.base_namespace,
                'topic': 'set_tx_bw_limit_mbps',
                'msg': Int32,
                'qsize': 10,
                'callback': self.setUploadBwLimitCb, 
                'callback_args': ()
            },
            'set_rosmaster': {
                'namespace': self.base_namespace,
                'topic': 'set_rosmaster',
                'msg': String,
                'qsize': 10,
                'callback': self.set_rosmaster, 
                'callback_args': ()
            },
            'enable_wifi': {
                'namespace': self.base_namespace,
                'topic': 'enable_wifi',
                'msg': Bool,
                'qsize': 10,
                'callback': self.enableWifICb, 
                'callback_args': ()
            },
            'set_wifi_low_power_mode': {
                'namespace': self.base_namespace,
                'topic': 'set_wifi_low_power_mode',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setWifiLowPowerCb, 
                'callback_args': ()
            },
            'enable_wifi_client': {
                'namespace': self.base_namespace,
                'topic': 'enable_wifi_client',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setWifiClientEnableCb, 
                'callback_args': ()
            },
            'refresh_wifi_networks': {
                'namespace': self.base_namespace,
                'topic': 'refresh_available_wifi_networks',
                'msg': Empty,
                'qsize': 10,
                'callback': self.refreshWifiNetworksCb, 
                'callback_args': ()
            },
            'set_wifi_client_credentials': {
                'namespace': self.base_namespace,
                'topic': 'set_wifi_client_credentials',
                'msg': NetworkWifiCredentials,
                'qsize': 10,
                'callback': self.setWifiClientCredentialsCb, 
                'callback_args': ()
            },      
            'enable_wifi_ap': {
                'namespace': self.base_namespace,
                'topic': 'enable_wifi_access_point',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setWifiApEnableCb, 
                'callback_args': ()
            },
            'set_wifi_ap_credentials': {
                'namespace': self.base_namespace,
                'topic': 'set_wifi_access_point_credentials',
                'msg': NetworkWifiCredentials,
                'qsize': 10,
                'callback': self.setWifiApCredentialsCb, 
                'callback_args': ()
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

        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()

        self.initCb(do_updates = True)


        ###########################
        # Complete Initialization

        nepi_sdk.start_timer_process(self.BANDWIDTH_MONITOR_PERIOD_S, self.bandwidthCheckCb)
        nepi_sdk.start_timer_process(1, self.networkIpCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1, self.internetCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1, self.wifiNetworkCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(2, self.wifiClientCheckCb, oneshot = True)



        nepi_sdk.start_timer_process(1, self.publishStatusCb)
        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        nepi_sdk.spin()
        #########################################################

        


    #######################

    def initCb(self, do_updates = False):
        self.primary_ip_addr = self.get_primary_ip_addr()
        if self.node_if is not None and self.manages_network == True:
            self.nepi_config = self.get_nepi_system_config()

            # Run Some Checks
            self.msg_if.pub_warn("Init Updating IP List")
            ip_addrs = self.update_ip_addr_lists()

            self.msg_if.pub_warn("Init Checking Internet Connection")
            internet_connected = self.internet_check()
            if internet_connected is None:
                internet_connected = False

            self.msg_if.pub_warn("Init Internet Check: " + str(internet_connected))


            # Upated Managed Alias IP Addresses
            managed_ip_addrs=[]
            if self.nepi_config is not None:
                for i in range(10):
                    akey = 'NEPI_ALIAS_IP_' + str(i)
                    if akey in self.nepi_config.keys():
                        if self.is_valid_ip(self.nepi_config[akey]):
                            managed_ip_addrs.append(self.nepi_config[akey])
            self.managed_ip_addrs = managed_ip_addrs
            self.msg_if.pub_warn("Starting Init with Managed addrs: " + str(self.managed_ip_addrs))
            

             # Update TX Bandwidth Limit setting
            # self.tx_bw_limit_mbps = self.node_if.get_param('tx_bw_limit_mbps')
            # self.set_upload_bw_limit(self.tx_bw_limit_mbps)


            # Update DHCP settings
            cdhcp_enabled = self.nepi_config['NEPI_WIRED_DHCP_ENABLED'] == 1
            self.dhcp_enabled = cdhcp_enabled      
            self.msg_if.pub_warn("Starting Init with DHCP Enabled: " + str(self.dhcp_enabled)) 
            # if self.dhcp_enabled == True and internet_connected == False:
            #     self.msg_if.pub_warn("Calling Enable DHCP process")
            #     success = self.enable_dhcp(self.dhcp_enabled)
            

            wifi_iface = self.nepi_config['NEPI_WIFI_INTERFACE']  
            if wifi_iface == 'unknown' or wifi_iface == 'NONE':
                self.wifi_iface = None
            else:
                self.wifi_iface = wifi_iface

            # Update WiFi Enabled settings
            cwifi_enabled = self.nepi_config['NEPI_WIFI_ENABLED'] == 1
            self.wifi_enabled = cwifi_enabled and self.wifi_iface is not None


            # Stop WiFi updates if not enabled
            if self.wifi_enabled == True:

                # Update WiFi Low Power settings
                nlow_power_enabled = False #self.node_if.get_param('wifi_low_power_enabled')
                low_power_enabled = copy.deepcopy(nlow_power_enabled)
                clow_power_enabled = self.nepi_config['NEPI_WIFI_LOW_POWER_ENABLED'] == 1
                self.low_power_enabled = nlow_power_enabled or clow_power_enabled
                # if self.low_power_enabled != nlow_power_enabled:
                #     self.enable_wifi_low_power(self.low_power_enabled)
                #     self.node_if.set_param('low_power_enabled',self.low_power_enabled)    

                # Update WiFi Client settings
                nclient_enabled = False #self.node_if.get_param('wifi_client_enabled')
                client_enabled = copy.deepcopy(nclient_enabled)
                cclient_enabled = self.nepi_config['NEPI_WIFI_CLIENT_ENABLED'] == 1
                self.wifi_client_enabled = nclient_enabled or cclient_enabled


                # if self.wifi_client_enabled != nclient_enabled:
                #     self.node_if.set_param('wifi_client_enabled',self.wifi_client_enabled)    
                # if self.wifi_client_enabled == True: 
                #     self.msg_if.pub_warn("Initializing WiFi Client with NONE")
                #     self.wifi_client_passphrase = 'NONE'
                #     self.wifi_client_ssid = 'NONE'
                #     self.enable_wifi_client(self.wifi_client_enabled)
                
                nwifi_client_ssid = 'NONE' #self.node_if.get_param('wifi_client_ssid')
                cwifi_client_ssid = self.nepi_config['NEPI_WIFI_CLIENT_ID']
                if nwifi_client_ssid == 'NONE' or nwifi_client_ssid == '' or nwifi_client_ssid is None:
                    nwifi_client_ssid=cwifi_client_ssid
                if nwifi_client_ssid == '' or nwifi_client_ssid is None:
                    nwifi_client_ssid='NONE'
                self.wifi_client_ssid = nwifi_client_ssid
                
                nwifi_client_passphrase = 'NONE' #self.node_if.get_param('wifi_client_passphrase')
                cwifi_client_passphrase = self.nepi_config['NEPI_WIFI_CLIENT_PW']
                if nwifi_client_passphrase == 'NONE' or nwifi_client_passphrase == '' or nwifi_client_passphrase is None:
                    nwifi_client_passphrase=cwifi_client_passphrase
                if nwifi_client_passphrase == '' or nwifi_client_passphrase is None:
                    nwifi_client_passphrase='NONE'
                self.wifi_client_passphrase = nwifi_client_passphrase
                    



                
                # Update WiFi Access Point settings
                nap_enabled = False #self.node_if.get_param('wifi_ap_enabled')
                cap_enabled = self.nepi_config['NEPI_WIFI_CLIENT_ENABLED'] == 1
                self.ap_enabled = nap_enabled or cap_enabled

                        
                nwifi_ap_ssid = 'NONE' #self.node_if.get_param('wifi_ap_ssid')
                cwifi_ap_ssid = 'NONE'
                try:
                    cwifi_ap_ssid = self.nepi_config['NEPI_HOTSPOT_ID']
                except:
                    pass
                if nwifi_ap_ssid == 'NONE' or nwifi_ap_ssid == '' or nwifi_ap_ssid is None:
                    nwifi_ap_ssid=cwifi_ap_ssid
                if nwifi_ap_ssid == 'NONE' or nwifi_ap_ssid == '' or nwifi_ap_ssid is None:
                    nwifi_ap_ssid='nepi_device_ap'
                self.wifi_ap_ssid = nwifi_ap_ssid
                
                nwifi_ap_passphrase = 'NONE' # self.node_if.get_param('wifi_ap_passphrase')
                cwifi_ap_passphrase = 'NONE'
                try:
                    cwifi_ap_passphrase = self.nepi_config['NEPI_HOTSPOT_PW']
                except:
                    pass                    
                if nwifi_ap_passphrase == 'NONE' or nwifi_ap_passphrase == '' or nwifi_ap_passphrase is None:
                    nwifi_ap_passphrase=cwifi_ap_passphrase
                if nwifi_ap_passphrase == 'NONE' or nwifi_ap_passphrase == '' or nwifi_ap_passphrase is None:
                    nwifi_ap_passphrase='nepi_device_ap'
                self.wifi_ap_passphrase = nwifi_ap_passphrase

                # if self.ap_enabled != nap_enabled:
                #     self.node_if.set_param('wifi_ap_enabled',self.wifi_ap_enabled)       
                # if self.wifi_ap_enabled == True: 
                #     self.enable_wifi_access_point(self.wifi_ap_enabled)


            self.msg_if.pub_warn("Starting Init with Wifi Iface: " + str(self.wifi_iface))
            self.msg_if.pub_warn("Starting Init with Wifi Enabled: " + str(self.wifi_enabled))

            self.msg_if.pub_warn("Starting Init with Wifi Client Enabled: " + str(self.wifi_client_enabled))
            self.msg_if.pub_warn("Starting Init with Wifi Client ssid: " + str(self.wifi_client_ssid))
            self.msg_if.pub_warn("Starting Init with Wifi Client password: " + str(self.wifi_client_passphrase))

            self.msg_if.pub_warn("Starting Init with Wifi AP Enabled: " + str(self.wifi_ap_enabled))
            self.msg_if.pub_warn("Starting Init with Wifi AP ssid: " + str(self.wifi_ap_ssid))
            self.msg_if.pub_warn("Starting Init with Wifi AP password: " + str(self.wifi_ap_passphrase))
            

            

            if do_updates == True:
                pass
        
            #self.msg_if.pub_warn("Ending Init with Managed addrs: " + str(self.managed_ip_addrs))
            #self.msg_if.pub_warn("Ending Init with Wifi Client ssid: " + str(self.wifi_client_ssid))
            #self.msg_if.pub_warn("Ending Init with Wifi Client password: " + str(self.wifi_client_passphrase))
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

        # nepi_sdk.sleep(1)
        # self.msg_if.pub_warn("Reseting Factory Config")
        # with open(self.ALIASES_IP_FILE, "w") as f:
        #     f.write(self.USER_IP_ALIASES_FILE_PREFACE)

        # # Set the rosmaster back to localhost
        # self.set_rosmaster_impl("localhost")
        # self.msg_if.pub_warn("Factory reset complete -- must reboot device for IP and ROS_MASTER_URI changes to take effect")


    def save_config(self):
        success = True
        if self.node_if is not None:
            self.node_if.save_config()
        return success

    def publishStatusCb(self,timer):
        self.publish_status()


    #######################
    # Update Functions

    def bandwidthCheckCb(self, timer):
        #self.msg_if.pub_warn("Debugging: Checking Network Bandwidth")
        tx_stats = '/sys/class/net/' + self.NET_IFACE + '/statistics/tx_bytes'
        rx_stats = '/sys/class/net/' + self.NET_IFACE + '/statistics/rx_bytes'
        if os.path.exists(tx_stats):
            with open(tx_stats, 'r') as f:
                tx_bytes = int(f.read())
                self.tx_byte_cnt_deque.append(tx_bytes)
        if os.path.exists(rx_stats):
            with open(rx_stats, 'r') as f:
                rx_bytes = int(f.read())
                self.rx_byte_cnt_deque.append(rx_bytes)


    def networkIpCheckCb(self, timer):
        #self.msg_if.pub_warn("Debugging: Entering Network Check process")
        found_ip_addrs = self.update_ip_addr_lists()         
        nepi_sdk.start_timer_process(5, self.networkIpCheckCb, oneshot = True)


    def internetCheckCb(self, timer):
        #self.msg_if.pub_warn("Debugging: Entering Internet Check process")
        cur_year = datetime.now().year
        self.clock_skewed = cur_year < 2024
        connected = self.internet_check()
        nepi_sdk.start_timer_process(1, self.internetCheckCb, oneshot = True)


    def wifiNetworkCheckCb(self,timer):
        #self.msg_if.pub_warn("Debugging: Entering WiFi Check process")
        # Refresh Wifi Networks

        success = self.scan_for_wifi_networks()
   
        nepi_sdk.start_timer_process(20, self.wifiNetworkCheckCb, oneshot=True)
    
    def wifiClientCheckCb(self,timer):
        #self.msg_if.pub_warn("Debugging: Entering WiFi Client Check process")
        # Check for Wifi Connection
        ssid = self.check_wifi_client_connected()
        nepi_sdk.start_timer_process(1, self.wifiClientCheckCb, oneshot=True)


    #######################
    ### Util Functions



    def get_nepi_system_config(self):
        return nepi_system.get_nepi_config(log_name_list = [self.node_name])

    def get_primary_ip_addr(self):
        primary_ip_addr = self.nepi_config['NEPI_STATIC_IP']
        return primary_ip_addr


    def update_ip_addr_lists(self):
        #self.msg_if.pub_warn("Debugging: Scanning For IP Addresses")
        self.primary_ip_addr = self.get_primary_ip_addr()
        last_ip_addrs = copy.deepcopy(self.found_ip_addrs)
        managed_ips = self.managed_ip_addrs
        dhcp_ip_addr = 'uknown'
        rep_ips = [self.primary_ip_addr] + managed_ips
        found_ip_addrs = rep_ips
       
        self.network_checked = True

        self.found_ip_addrs = found_ip_addrs
        if last_ip_addrs != self.found_ip_addrs:
            self.msg_if.pub_warn("Return IPs: " + str(rep_ips))
            self.msg_if.pub_warn("Found IPs: " + str(self.found_ip_addrs))
            self.msg_if.pub_warn("Managed IPs: " + str(managed_ips))
            self.msg_if.pub_warn("DHCP IP: " + str(dhcp_ip_addr))

        # if self.dhcp_ip_addr != '' and self.dhcp_enabled == False and self.internet_connected == True:
        #     [self.dhcp_enabled,self.dhdhcp_enable_state] = [True,True]

        return found_ip_addrs

   
           
    def check_wifi_client_connected(self):

        if self.wifi_enabled == False or self.wifi_iface is None:
            #self.msg_if.pub_warn("Cannot enable WiFi access point - system has no WiFi adapter")
            return 'NONE'

        #self.msg_if.pub_warn("Starting Check wifi connection process with wifi iface: " + str(self.wifi_iface))
        try:
            check_connection_cmd = ['iw', self.wifi_iface, 'link']
            connection_status = subprocess.check_output(check_connection_cmd, text=True)
            #self.msg_if.pub_warn("Got wifi connection status: " + str(connection_status))
        except Exception as e: 
            #self.msg_if.pub_warn("Failed to check on wifi connection: " + str(e))
            return 'NONE'
        if connection_status.startswith('Connected'):        
           self.wifi_client_connected = True    
           self.wifi_client_trying = False
           for line in connection_status.splitlines():
               if line.strip().startswith('SSID'):
                   connected_ssid = line.strip().split()[1]
                   #if connected_ssid != self.wifi_client_ssid:
                   #    self.wifi_client_ssid = connected_ssid
                   return connected_ssid
        
        return 'NONE'

    

    def internet_check(self):
        connected = False
        try:
            connected = nepi_utils.ping_ip("8.8.8.8")
            self.internet_checked = True
        except Exception as e:
            self.msg_if.pub_warn("Internet Check Process Failed: " + str(e))
        self.internet_connected = connected
        return connected




    def is_valid_ip(self,addr):
        valid = False
        if addr is not None:
            addr_split = addr.split('/')
            if len(addr_split) > 1:
                ip = addr_split[0]
                try:
                    ipaddress.ip_address(ip)
                    nm = int(addr_split[1])
                    if nm >= 0 and nm <= 32:
                        valid = True
                except ValueError:
                    pass
        return valid


    def validate_cidr_ip(self, addr):
        # First, validate the input
        if addr is None:
            return False
        else:
            tokens = addr.split('/')
            new_ip = tokens[0]
            new_ip_bits = 0
            try:
                new_ip_bits = socket.inet_aton(new_ip)
            except Exception as e:
                self.msg_if.pub_warn("Rejecting invalid IP address: " + str(addr) + " " + str(e))
                return False
            if (len(tokens) != 2):
                self.msg_if.pub_warn("Rejecting invalid address must be in CIDR notation (x.x.x.x/y). Got " + str(addr))
                return False
            cidr_netmask = (int)(tokens[1])
            if cidr_netmask < 1 or cidr_netmask > 32:
                self.msg_if.pub_warn("Rejecting invalid CIDR netmask (got " + str(addr))
                return False

            # Finally, verify that this isn't the "fixed" address on the device. Don't let anyone sneak past the same
            # address in a different numerical format - compare as a 32-bit val
            cur_ip_addrs = copy.deepcopy(self.found_ip_addrs)
            if len(cur_ip_addrs) > 0:
                fixed_ip_addr = cur_ip_addrs[0].split('/')[0]
                fixed_ip_bits = 0
                try:
                    fixed_ip_bits = socket.inet_aton(fixed_ip_addr)
                except Exception as e:
                    self.msg_if.pub_warn("Cannot validate IP address becaused fixed IP appears invalid "  + str(fixed_ip_addr) + " " + str(e))
                    return False
                if (new_ip_bits == fixed_ip_bits):
                    self.msg_if.pub_warn("IP address invalid because it matches fixed primary IP")
                    return False

                return True
            else:
                return False




    def scan_for_wifi_networks(self):
        #self.msg_if.pub_warn("Checking if can update wifi networks: " + str([self.wifi_enabled == True, self.wifi_client_enabled == True, self.wifi_client_connecting == False]))
        if self.wifi_enabled == True and self.wifi_client_enabled == True and self.wifi_client_connecting == False and self.wifi_scanning == False:
            last_networks = copy.deepcopy(self.wifi_available_networks)
            self.wifi_scanning = True
            #self.msg_if.pub_warn("Debugging: Scanning for available WiFi networks") 
            #self.msg_if.pub_warn("Debugging:" + str(self.wifi_client_enabled) + " " + " " + str(self.wifi_enabled))
            #self.msg_if.pub_info("Updating available WiFi connections on iface: " + str(self.wifi_iface))
            available_networks = []
            network_scan_cmd = ['iw', 'dev', self.wifi_iface, 'scan' ]#, '2>/dev/null']
            scan_result = ""
            try:
                #with self.wifi_lock:
                # timeout needed -- on hardware/environments where wlan0 can't
                # actually scan (e.g. no real WiFi radio), this call blocked
                # forever with no timeout, pegging a full CPU core
                # indefinitely and leaving self.wifi_scanning stuck True so
                # every later scan attempt was silently skipped too.
                scan_result = subprocess.check_output(network_scan_cmd, text=True, timeout=10)
                #self.msg_if.pub_warn("Got WiFi scan results: " + str(scan_result))
            except Exception as e:
                #self.msg_if.pub_warn("Failed to scan for available WiFi networks: " + str(scan_result) + str(e))
                pass
            if scan_result != "":
                for scan_line in scan_result.splitlines():
                    if "SSID:" in scan_line:
                        network = scan_line.split(':')[1].strip()
                        # TODO: Need more checks to ensure this is a connectable network?
                        if network and (network not in available_networks):
                            available_networks.append(network)
                self.wifi_available_networks = list(available_networks)
                #self.msg_if.pub_warn("Got WiFi networks list: " + str(available_networks))
                # if last_networks != self.wifi_available_networks:
                #     self.msg_if.pub_warn("Got Updated WiFi networks list: " + str(available_networks))
            #self.msg_if.pub_warn("Exiting wifi scan process: " + str(available_networks))
            self.wifi_scanning = False




    #######################
    ### ROS System Manager Functions

    def set_rosmaster(self, msg):
        new_master_ip = msg.data
        self.set_rosmaster_impl(new_master_ip)
        success = self.save_config()

    def set_rosmaster_impl(self, master_ip):
        auto_comment = " # Modified by network_mgr " + str(datetime.now()) + "\n"

        # # First, determine if the master is local, either as localhost or one of the configured IP addrs
        # master_is_local = True if (master_ip == "localhost") else False
        # if master_is_local is False:
        #     local_ips = copy.deepcopy(self.found_ip_addrs)
        #     for ip_cidr in local_ips:
        #         ip = ip_cidr.split('/')[0]
        #         if master_ip == ip:
        #             master_is_local = True
        #             break

        # if master_is_local is True:
        #     master_ip = "localhost" # Force 'localhost' whenever the "new" master is a local IP in case that local IP (alias) is later removed
        #     master_ip_for_remote_hosts = self.get_primary_ip_addr().split('/')[0]
        # else:
        #     master_ip_for_remote_hosts = master_ip
        #     # Now ensure we can contact the new rosmaster -- if not, bail out
        #     ret_code = subprocess.call(['nc', '-zvw5', master_ip,  str(self.ROS_MASTER_PORT)])
        #     if (ret_code != 0):
        #         self.msg_if.pub_warn("Failed to detect a remote rosmaster at " + master_ip + ":" + str(self.ROS_MASTER_PORT) + "... refusing to update ROS_MASTER_URI")
        #         return

        # # Edit the sys_env file appropriately
        # rosmaster_line_prefix = "export ROS_MASTER_URI="
        # new_rosmaster_line = rosmaster_line_prefix + "http://" + master_ip + ":" + str(self.ROS_MASTER_PORT) + auto_comment
        # sys_env_output_lines = []
        # with open(self.SYS_ENV_FILE, "r") as f_in:
        #     for line in f_in:
        #         if rosmaster_line_prefix in line:
        #             line = new_rosmaster_line
        #         sys_env_output_lines.append(line)
        # with open(self.SYS_ENV_FILE, "w") as f_out:
        #     f_out.writelines(sys_env_output_lines)

        # # And the roslaunch file (add --wait for remote ros master)
        # roslaunch_line_prefix = "roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}"
        # new_roslaunch_line = roslaunch_line_prefix
        # if master_is_local is False:
        #     new_roslaunch_line += " --wait"
        # new_roslaunch_line += auto_comment
        # roslaunch_output_lines = []
        # with open(self.ROSLAUNCH_FILE, "r") as f_in:
        #     for line in f_in:
        #         if roslaunch_line_prefix in line:
        #             line = new_roslaunch_line
        #         roslaunch_output_lines.append(line)
        # with open(self.ROSLAUNCH_FILE, "w") as f_out:
        #     f_out.writelines(roslaunch_output_lines)

        # # And the env_loader files for remote machines
        # new_rosmaster_line = rosmaster_line_prefix + "http://" + master_ip_for_remote_hosts + ":" + str(self.ROS_MASTER_PORT) + auto_comment
        # tmp_env_loader_file = "./env_loader_file.tmp"
        # for remote_env_loader_file in self.REMOTE_ROS_NODE_ENV_LOADER_FILES:
        #     env_loader_lines = []
        #     ret_code = subprocess.call(['scp', remote_env_loader_file, tmp_env_loader_file])
        #     if (ret_code != 0):
        #         self.msg_if.pub_warn("Failed to get copy of remote file " + remote_env_loader_file + "... not updating ROS_MASTER_URI for that remote host")
        #         continue
        #     with open(tmp_env_loader_file, "r") as f_in:
        #         for line in f_in:
        #             if rosmaster_line_prefix in line:
        #                 line = new_rosmaster_line
        #             env_loader_lines.append(line)
        #     with open(tmp_env_loader_file, "w") as f_out:
        #         f_out.writelines(env_loader_lines)
        #     ret_code = subprocess.call(['scp', tmp_env_loader_file, remote_env_loader_file])
        #     if (ret_code != 0):
        #         self.msg_if.pub_warn("Failed to update remote file " + remote_env_loader_file)
        #     os.remove(tmp_env_loader_file)

        # self.msg_if.pub_warn("Updated ROS_MASTER_URI to " + master_ip + "... requires reboot to complete the switch")
        # success = self.save_config()

    #######################
    ### Wired IP Manager Functions


    def addIpCb(self, addr_msg):
        self.msg_if.pub_warn("Recieved Add IP address: " + str(addr_msg.data))
        addr = addr_msg.data
        self.add_ip(addr)


    def add_ip(self, addr):
        success = False
        if True == self.is_valid_ip(addr):
            if addr not in self.found_ip_addrs:
                if addr != self.primary_ip_addr and addr != '0.0.0.0/24': # and addr != self.dhcp_ip_addr:
                    if addr not in self.managed_ip_addrs:
                        self.managed_ip_addrs = [addr]
                        self.publish_status()
                        # if self.node_if is not None:
                        #     self.node_if.set_param('managed_ip_addrs',self.managed_ip_addrs)
                        # self.save_config()
                    self.found_ip_addrs.append(addr)
                    success = self.publish_status()
                    ### Update ETC Files
                    nepi_system.update_nepi_system_config("NEPI_ALIAS_IPS",addr)
                    etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_wired_aliases.sh"
                    subprocess.call([etc_update_script])
                    nepi_sdk.sleep(1)
                    self.nepi_config = self.get_nepi_system_config()
                    ####################
                    self.msg_if.pub_warn("Added IP address: " + str(addr))

                    success = True
                else:
                    self.msg_if.pub_warn("Unable to add IP address: " + str(addr))
            else:
                self.msg_if.pub_warn("IP address already in system: " + str(addr))
        else:
            self.msg_if.pub_warn("Unable to add invalid/ineligible IP address: " + str(addr))
        return success
        


    def removeIpCb(self, addr_msg):
        self.msg_if.pub_warn("Recieved Remove IP address: " + str(addr_msg.data))
        addr = addr_msg.data
        self.remove_ip(addr)


    def remove_ip(self, addr):
        success = False
        if True == self.is_valid_ip(addr):
            if addr in self.found_ip_addrs:
                if addr in self.managed_ip_addrs:
                    self.managed_ip_addrs.remove(addr)
                    self.publish_status()
                    if addr != self.primary_ip_addr : # and addr != self.dhcp_ip_addr:
                        self.found_ip_addrs.remove(addr)
                        success = self.publish_status()
                        ### Update ETC Files
                        nepi_system.update_nepi_system_config("NEPI_ALIAS_IPS","NONE")
                        etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_wired_aliases.sh"
                        subprocess.call([etc_update_script])
                        nepi_sdk.sleep(1)
                        self.nepi_config = self.get_nepi_system_config()
                        ####################
                        self.msg_if.pub_warn("Removed IP address: " + str(addr))
                        # if self.node_if is not None:
                        #     self.node_if.set_param('managed_ip_addrs',self.managed_ip_addrs)
                        # self.save_config()
                        success = True
            else:
                self.msg_if.pub_warn("Unable to remove invalid/ineligible IP address: " + str(addr))
        else:
            self.msg_if.pub_warn("IP address not found in system: " + str(addr))
        return success


    def setUploadBwLimitCb(self, msg):
        if msg.data >= 0 and msg.data < 1:
            self.msg_if.pub_warn('Cannot set bandwidth limit below 1Mbps')
            return

        self.set_upload_bw_limit(msg.data)



    def set_upload_bw_limit(self,limit):
        # First, update param server
        self.tx_bw_limit_mbps = limit
        self.publish_status()

        self.msg_if.pub_warn("TX bandwidth limits currently not supported in NEPI Containers")
        # else:
        #     # Always clear the current settings
        #     try:
        #         with self.network_lock:
        #             subprocess.call([self.WONDERSHAPER_CALL, '-a', self.NET_IFACE, '-c'])
        #     except Exception as e:
        #         self.msg_if.pub_warn("Unable to clear current bandwidth limits: " + str(e))
        #         return

        #     if self.tx_bw_limit_mbps < 0: #Sentinel values to clear limits
        #         self.msg_if.pub_info("Cleared bandwidth limits")
        #         return

        #     # Now acquire the param from param server and update
        #     bw_limit_kbps = self.tx_bw_limit_mbps* 1000
        #     if self.in_container == True:
        #         pass 
        #     else:
        #         try:
        #             with self.network_lock:
        #                 subprocess.call([self.WONDERSHAPER_CALL, '-a', self.NET_IFACE, '-u', str(bw_limit_kbps)])
        #             self.msg_if.pub_info("Updated TX bandwidth limit to " + str(self.tx_bw_limit_mbps) + " Mbps")
        #             #self.tx_byte_cnt_deque.clear()
        #         except Exception as e:
        #             self.msg_if.pub_warn("Unable to set upload bandwidth limit: " + str(e))
        #     # if self.node_if is not None:
        #     #     self.node_if.set_param('tx_bw_limit_mbps', msg.data)
        #     #     success = self.save_config()





    def enableDhcpCb(self, msg):
        self.msg_if.pub_warn("Got DHCP enable request msg: " + str(msg))
        self.dhcp_enabled = msg.data
        self.msg_if.pub_warn("Updating DHCP enable to: " + str(self.dhcp_enabled))
        if self.dhcp_enabled == True and self.internet_connected == False:
            self.msg_if.pub_warn("Calling Enable DHCP process")
            success = self.enable_dhcp(self.dhcp_enabled)

    def enable_dhcp(self, enabled):
        success = False
        self.dhcp_enabled = enabled
        self.publish_status()
        ### Update ETC Files
        update_val=0
        if enabled == True:
            update_val=1
        #nepi_system.update_nepi_system_config("NEPI_WIRED_DHCP_ENABLED",update_val)
        etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_wired_dhcp.sh"
        subprocess.call([etc_update_script])
        nepi_sdk.sleep(1)
        self.nepi_config = self.get_nepi_system_config()
        ####################
        # if self.node_if is not None:
        #     self.node_if.set_param('dhcp_enabled', self.dhcp_enabled)
        #     success = self.save_config()
        self.dhcp_connecting = False
        return success
       



    #######################
    ### Wifi Manager Functions

    def enableWifICb(self, msg):
        enabled = msg.data
        self.enable_wifi(enabled)

    def enable_wifi(self, enabled):
        success = False
        if self.wifi_iface is not None and self.wifi_client_connecting == False:
            self.wifi_enabled = enabled
            self.publish_status()
            ### Update ETC Files
            update_val=0
            if enabled == True:
                update_val=1
            nepi_system.update_nepi_system_config("NEPI_WIFI_ENABLED",update_val)
            etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_wifi_enable.sh"
            subprocess.call([etc_update_script])
            nepi_sdk.sleep(1)
            self.nepi_config = self.get_nepi_system_config()
            ####################
            # if self.node_if is not None:
            #     self.node_if.set_param('dhcp_enabled', self.dhcp_enabled)
        return success

    def setWifiLowPowerCb(self, msg):
        enabled = msg.data
        self.set_wifi_low_power(enabled)

    def set_wifi_low_power(self, enabled):
        success = False
        if self.wifi_iface is not None and self.wifi_low_power_enabled != enabled and self.wifi_client_connecting == False:
            self.wifi_low_power_enabled = enabled
            self.publish_status()
            ### Update ETC Files
            update_val=0
            if enabled == True:
                update_val=1
            nepi_system.update_nepi_system_config("NEPI_WIFI_LOW_POWER_ENABLED",update_val)
            etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_wifi_low_power.sh"
            subprocess.call([etc_update_script])
            nepi_sdk.sleep(1)
            ####################
            # if self.node_if is not None:
            #     self.node_if.set_param('wifi_low_power_enabled', enabled)
        return success


    def refreshWifiNetworksCb(self, msg):
        self.msg_if.pub_info("Recieved refresh WiFi availble networks")
        success = self.scan_for_wifi_networks()



    #######################
    ### Wifi Client Manager Functions

    def setWifiClientEnableCb(self, msg):
        self.msg_if.pub_info("Recieved Enable WiFi client msg: " + str(msg.data))
        enabled = msg.data
        self.enable_wifi_client(enabled)

    def enable_wifi_client(self,enabled):
        success = False
        if self.wifi_enabled == False:
            self.msg_if.pub_warn("Cannot enabled WiFi access point - system has no WiFi adapter")
            return False
        #self.msg_if.pub_warn("Checking if can enable wifi: " + str([self.wifi_iface is not None, self.wifi_client_enabled != enabled, self.wifi_client_connecting == False]))
        if self.wifi_iface is not None and self.wifi_client_connecting == False:
            self.wifi_client_enabled = enabled
            self.publish_status()
            #Update credentials file
            success = self.set_wifi_client(self.wifi_client_ssid, self.wifi_client_passphrase)
            # self.node_if.set_param('wifi_client_enabled', enabled)
        return success


    def setWifiClientCredentialsCb(self, msg):
        self.msg_if.pub_info("Recieved WiFi client credentials (SSID: " + msg.ssid + ", Passphrase: " + msg.passphrase + ")")

        ssid = msg.ssid
        if ssid == 'None' or ssid == '':
            ssid = 'NONE'          
        passphrase=msg.passphrase
        if passphrase is None or passphrase == 'NONE':
            passphrase = ''

        self.wifi_client_ssid = ssid
        self.wifi_client_passphrase = passphrase
        success = self.set_wifi_client(ssid,passphrase)

        


    def set_wifi_client(self, ssid, passphrase):
        success = False

        if self.wifi_enabled == False:
            self.msg_if.pub_warn("Cannot enable WiFi access point - system has no WiFi adapter")
            return False
        if self.wifi_enabled == False:
            self.msg_if.pub_warn("Cannot enable WiFi access point - WiFi adapter not enabled")
            return False
        
        if self.wifi_client_connecting == False and self.wifi_client_enabled == True: # and self.clock_skewed == False:
            self.msg_if.pub_warn("Checking if Wifi credential have changed")
            # Get current state
            if ssid == 'None' or ssid == '':
                ssid == 'NONE'
            if passphrase == 'None' or passphrase == '':
                passphrase == 'NONE'
            self.msg_if.pub_info("Using WiFi client credentials (SSID: " + ssid + ", Passphrase: " + passphrase + ")")


            self.msg_if.pub_warn("Disabling Wired DHCP") 
            self.dhcp_enabled = False
            self.publish_status()

            self.msg_if.pub_warn("Connecting WiFi client with credentials (SSID: " + ssid + ", Passphrase: " + passphrase + ")")
            self.wifi_client_connecting = True
            self.wifi_client_connected = False

            if ssid != "NONE":
                self.wifi_client_trying = True
            else:
                self.wifi_client_trying = False
        

            ### Update ETC Files
            nepi_system.update_nepi_system_config("NEPI_WIFI_CLIENT_ID",ssid)
            nepi_system.update_nepi_system_config("NEPI_WIFI_CLIENT_PW",passphrase)
            nepi_system.update_nepi_system_config("NEPI_WIFI_CLIENT_ENABLED",1)
            etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_wifi_client.sh"
            try:
                subprocess.call([etc_update_script])
                nepi_sdk.sleep(1)
                self.msg_if.pub_warn("Updated WiFi Client Network")
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Update WiFi Client Network subprocess failed: " + str(e))
                success = False
                     

        # if success == True and self.node_if is not None:
        #     self.node_if.set_param("wifi_client_ssid", ssid)
        #     self.node_if.set_param("wifi_client_passphrase", passphrase)
        #     success = self.save_config()
        self.publish_status()

        #### Clear the connecting flag
        self.msg_if.pub_warn("WiFi Client Update process complete")
        self.wifi_client_connecting = False
        nepi_sdk.sleep(1)
        ssid = self.check_wifi_client_connected()
        return success



    def setWifiApEnableCb(self, msg):
        self.msg_if.pub_warn("Recieved enable wifi access point msg: " + str(msg))
        self.enable_wifi_access_point(msg.data)


    def enable_wifi_access_point(self,enabled):    
        success = True
        is_enabled = copy.deepcopy(self.wifi_ap_enabled)
        self.wifi_ap_enabled = enabled
        self.publish_status()

        if self.wifi_enabled == False:
            self.msg_if.pub_warn("Cannot enable WiFi access point - WiFi adapter not enabled")
            return False
        #if enabled != is_enabled:
        self.wifi_ap_enabled = enabled
        success = self.publish_status()
        self.set_wifi_ap()
        # if self.node_if is not None:
        #     self.node_if.set_param("wifi_wifi_ap_enabled", enabled)
        #     success = self.save_config()
        return True


    def setWifiApCredentialsCb(self, msg):
        success = False
        self.msg_if.pub_warn("Recieved set wifi access point msg: " + str(msg))
        self.wifi_ap_ssid = msg.ssid
        self.wifi_ap_passphrase = msg.passphrase

        if self.wifi_enabled == False:
            self.msg_if.pub_warn("Cannot enable WiFi access point - system has no WiFi adapter")
            return False
        if self.wifi_enabled == False:
            self.msg_if.pub_warn("Cannot enable WiFi access point - WiFi adapter not enabled")
            return False

        if self.wifi_ap_ssid != msg.ssid or self.wifi_ap_passphrase != msg.passphrase:
            self.wifi_ap_ssid = msg.ssid
            self.wifi_ap_passphrase = msg.passphrase
            success = self.publish_status()
            self.set_wifi_ap()
            # if self.node_if is not None:
            #     self.node_if.set_param("wifi_ap_ssid", msg.ssid)
            #     self.node_if.set_param("wifi_ap_passphrase", msg.passphrase)
            #     success = self.save_config()
        return success



    def set_wifi_ap(self):
        self.msg_if.pub_warn("Updating Wifi Access Point with : " + str(self.wifi_ap_ssid) + " " + str(self.wifi_ap_ssid))
        success = False
        if self.wifi_ap_enabled is True and self.wifi_iface is not None:
            if self.wifi_ap_ssid != "":
                ### Update ETC Files
                nepi_system.update_nepi_system_config("NEPI_HOTSPOT_ENABLED",1)
                nepi_system.update_nepi_system_config("NEPI_HOTSPOT_ID",self.wifi_ap_ssid)
                nepi_system.update_nepi_system_config("NEPI_HOTSPOT_PW",self.wifi_ap_passphrase)
                etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_wifi_access_point.sh"
                subprocess.call([etc_update_script])
                nepi_sdk.sleep(1)
                success = True
                ####################            
                    
        return success


    #####################################
    # Manager Service Functions


    def handle_ip_addr_query(self, req):
        return {'ip_addrs':self.found_ip_addrs, 
            'primary_ip_addr':self.primary_ip_addr,
            'managed_ip_addrs':self.managed_ip_addrs,
            'dhcp_enabled': self.dhcp_enabled, 
            'dhcp_ip_addr': self.dhcp_ip_addr,
            'internet_connected':self.internet_connected}

    def handle_bandwidth_usage_query(self, req):
        tx_rate_mbps = 0
        if len(self.tx_byte_cnt_deque) > 1:
            tx_rate_mbps =  8 * (self.tx_byte_cnt_deque[1] - self.tx_byte_cnt_deque[0]) / (self.BANDWIDTH_MONITOR_PERIOD_S * 1000000)

        rx_rate_mbps = 0
        if len(self.rx_byte_cnt_deque) > 1:
            rx_rate_mbps = 8 * (self.rx_byte_cnt_deque[1] - self.rx_byte_cnt_deque[0]) / (self.BANDWIDTH_MONITOR_PERIOD_S * 1000000)



        return {'tx_rate_mbps':tx_rate_mbps, 'rx_rate_mbps':rx_rate_mbps, 'tx_limit_mbps': self.tx_bw_limit_mbps}

    def handle_wifi_query(self, req):
        with self.wifi_lock:
            wifis = self.wifi_available_networks

        with self.internet_lock:
            internet_connected = self.internet_connected
        
        passphrase = self.wifi_client_passphrase
        if passphrase is None or passphrase == 'None' or passphrase == 'NONE':
            passphrase = ''

        return {'has_wifi': (self.wifi_iface is not None), 
                'wifi_ap_enabled': self.wifi_ap_enabled,
                'wifi_ap_ssid': self.wifi_ap_ssid, 
                'wifi_ap_passphrase': self.wifi_ap_passphrase,
                'wifi_ap_running': self.wifi_ap_running,
                'wifi_client_enabled': self.wifi_client_enabled,
                'wifi_client_connecting': self.wifi_client_connecting,
                'wifi_client_connected': self.wifi_client_connected,
                'wifi_client_ssid': self.wifi_client_ssid,
                'wifi_client_passphrase': passphrase,
                'available_networks': wifis,
                'clock_skewed': self.clock_skewed,
                'internet_connected': internet_connected}


    def publish_status(self):
        success = False

        self.status_msg.manages_network = self.manages_network
        # Wired Info
        self.status_msg.ip_addrs = self.found_ip_addrs 
        self.status_msg.primary_ip_addr = self.primary_ip_addr
        self.status_msg.managed_ip_addrs = self.managed_ip_addrs
        self.status_msg.dhcp_enabled =  self.dhcp_enabled 
        
        
        # WiFi Info
        self.status_msg.has_wifi =  (self.wifi_iface is not None) 
        self.status_msg.wifi_ap_enabled =  self.wifi_ap_enabled
        self.status_msg.wifi_ap_ssid =  self.wifi_ap_ssid 
        self.status_msg.wifi_ap_passphrase =  self.wifi_ap_passphrase
        self.status_msg.wifi_ap_running =  self.wifi_ap_running
        self.status_msg.wifi_client_enabled =  self.wifi_client_enabled
        self.status_msg.wifi_client_connecting =  self.wifi_client_trying
        self.status_msg.wifi_client_connected =  self.wifi_client_connected
        self.status_msg.wifi_client_ssid =  self.wifi_client_ssid

        passphrase = self.wifi_client_passphrase
        if passphrase is None or passphrase == 'None'  or passphrase == 'NONE':
            passphrase = ''
        self.status_msg.wifi_client_passphrase =  passphrase

        #with self.wifi_lock:
        wifis = self.wifi_available_networks
        self.status_msg.available_networks =  wifis
        self.status_msg.clock_skewed =  self.clock_skewed

        # Internet Info
        self.status_msg.dhcp_ip_addr =  self.dhcp_ip_addr
        self.status_msg.internet_connected = self.internet_connected

        #self.msg_if.pub_warn("Publishing Network Status Msg: " + str(self.status_msg))
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', self.status_msg)
            #self.msg_if.pub_warn("Published Network Status Msg: " + str(self.status_msg))
            success = True

        return success


if __name__ == "__main__":
    NetworkMgr()
