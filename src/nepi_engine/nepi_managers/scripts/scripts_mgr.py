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
import subprocess
import threading
import traceback
import yaml
import time
import psutil

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import MgrSystemStatus
from nepi_interfaces.srv import (
    GetScriptsQuery,
    GetScriptsQueryRequest,
    GetScriptsQueryResponse,
    GetRunningScriptsQuery,
    GetRunningScriptsQueryRequest,
    GetRunningScriptsQueryResponse,
    LaunchScript,
    LaunchScriptRequest,
    LaunchScriptResponse,
    StopScript,
    StopScriptRequest,
    StopScriptResponse,
    GetSystemStatsQuery,
    GetSystemStatsQueryRequest,
    GetSystemStatsQueryResponse,
)

from nepi_interfaces.msg import AutoStartEnabled

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import StatesIF


class ScriptsManager(object):

    DEFAULT_SCRIPT_STOP_TIMEOUT_S = 10.0

    node_if = None
    
    processes = {}
    scripts = []
    script_counters = {}
    script_configs = {} # Dictionary of dictionaries  

    scripts_folder = '/mnt/nepi_storage/nepi_scripts'
    scripts_log_folder = '/mnt/nepi_storage/logs/nepi_scripts_logs'
  
    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "scripts_manager" # Can be overwitten by luanch command
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
        # Wait for System Manager
        ##############################
        # Get for System Folders
        self.msg_if.pub_info("Waiting for user folders")
        user_folders = nepi_system.get_user_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got user folders: " + str(user_folders))

        if 'nepi_scripts' in user_folders.keys():
            self.scripts_folder = user_folders['nepi_scripts']
        self.msg_if.pub_info("Using Scipts Folder: " + str(self.scripts_folder))

        if 'logs/nepi_scripts_logs' in user_folders.keys():
            self.scripts_log_folder = user_folders['logs/nepi_scripts_logs']
        self.msg_if.pub_info("Using Scripts Log Folder: " + str(self.scripts_log_folder))
        
        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders()
          

        ##############################
        # Initialize Class Variables
        self.script_stop_timeout_s = self.DEFAULT_SCRIPT_STOP_TIMEOUT_S
        self.running_scripts = set()

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
            'script_configs': {
                'namespace': self.node_namespace,
                'factory_val': self.script_configs
            },
            'script_stop_timeout_s': {
                'namespace': self.node_namespace,
                'factory_val': self.DEFAULT_SCRIPT_STOP_TIMEOUT_S
            }       
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'get_scripts': {
                'namespace': self.base_namespace,
                'topic': 'get_scripts',
                'srv': GetScriptsQuery,
                'req': GetScriptsQueryRequest(),
                'resp': GetScriptsQueryResponse(),
                'callback': self.handle_get_scripts
            },
            'get_running_scripts': {
                'namespace': self.base_namespace,
                'topic': 'get_running_scripts',
                'srv': GetRunningScriptsQuery,
                'req': GetRunningScriptsQueryRequest(),
                'resp': GetRunningScriptsQueryResponse(),
                'callback': self.handle_get_running_scripts
            },
            'launch_script': {
                'namespace': self.base_namespace,
                'topic': 'launch_script',
                'srv': LaunchScript,
                'req': LaunchScriptRequest(),
                'resp': LaunchScriptResponse(),
                'callback': self.handle_launch_script
            },
            'stop_script': {
                'namespace': self.base_namespace,
                'topic': 'stop_script',
                'srv': StopScript,
                'req': StopScriptRequest(),
                'resp': StopScriptResponse(),
                'callback': self.handle_stop_script
            },'get_system_stats': {
                'namespace': self.base_namespace,
                'topic': 'get_system_stats',
                'srv': GetSystemStatsQuery,
                'req': GetSystemStatsQueryRequest(),
                'resp': GetSystemStatsQueryResponse(),
                'callback': self.handle_get_system_stats
            }
        }


        # Publishers Config Dict ####################
        self.PUBS_DICT = None

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'script_autostart': {
                'namespace': self.base_namespace,
                'topic': 'enable_script_autostart',
                'msg': AutoStartEnabled,
                'qsize': None,
                'callback': self.AutoStartEnabled_cb, 
                'callback_args': ()
            },
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


        ###########################
        # Complete Initialization
        self.script_stop_timeout_s = self.node_if.get_param('script_stop_timeout_s')
        self.scripts = self.get_scripts()
        self.file_sizes = self.get_file_sizes()
        for script in self.scripts:
            #TODO: These should be gathered from a stats file on disk to remain cumulative for all time (clearable on ROS command)
            self.script_counters[script] = {'started': 0, 'completed': 0, 'stopped_manually': 0, 'errored_out': 0, 'cumulative_run_time': 0.0}

        self.setupScriptConfigs()

        # Overlay the persisted configs (auto_start, cmd_line_args) saved to disk / param server
        # onto the freshly-built defaults. The config init callback runs while node_if is still
        # None during NodeClassIF construction, so this must be done explicitly here -- otherwise
        # script_configs only ever holds the defaults and auto_start is never honored on restart.
        self.update_script_configs()

        self.monitor_thread = threading.Thread(target=self.monitor_scripts)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

        self.watch_thread = threading.Thread(target=self.watch_directory, args=(self.scripts_folder, self.on_file_change))
        self.watch_thread.daemon = True
        self.watch_thread.start()

        # Autolaunch any scripts that are so-configured
        for script_name in self.script_configs:
            script_config = self.script_configs[script_name]
            if script_config['auto_start'] is True:
                self.msg_if.pub_info("Auto-starting " + script_name)
                req = LaunchScriptRequest(script_name)
                self.handle_launch_script(req)



        ###########################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_sdk.spin()
        #########################################################

    


    #######################
    ### Mgr Config Functions


    def setupScriptConfigs(self, single_script=None):
        script_list = []
        if single_script is None:
            script_list = self.scripts
        else:
            script_list = [single_script]

        # Ensure all known scripts have a script_config
        for script_name in script_list:
            if script_name not in self.script_configs:
                #self.msg_if.pub_warn("Initializing config for " + script_name)
                self.script_configs[script_name] = {'auto_start': False, 'cmd_line_args': ''}

                # Good place to dos2unix it
                subprocess.call(['dos2unix', os.path.join(self.scripts_folder, script_name)])

        # And make sure any unknown scripts don't
        configs_to_delete = [] # Can't delete configs while iterating over them -- runtime error, so just capture them here and then delete in a fresh loop
        for script_name in self.script_configs:
            if script_name not in self.scripts:
                self.msg_if.pub_info("Purging unknown script " + script_name + "from script_configs")
                configs_to_delete.append(script_name)
        for script_name in configs_to_delete:
            del self.script_configs[script_name]

    def update_script_configs(self):
        #self.msg_if.pub_warn("Ready to run update_script_configs!!!")
        script_configs = {} # Dictionary of dictionaries

        try:
            if self.node_if is not None:
                script_configs = self.node_if.get_param('script_configs')
        except KeyError:
            #self.msg_if.pub_warn("Parameter ~script_configs does not exist")
            pass

        for script_name in script_configs:
            if script_name not in self.scripts:
                self.msg_if.pub_warn("Config file includes configuration for unknown script " + script_name  + "... skipping this config")
                continue

            script_config = script_configs[script_name]
            self.msg_if.pub_info(script_name + " configuration from param server: " + str(script_config))

            if 'auto_start' not in script_config or 'cmd_line_args' not in script_config:
                self.msg_if.pub_warn("Invalid config. file settings for script " + script_name + "... skipping this config")
                continue

            self.script_configs[script_name]['auto_start'] = script_config['auto_start']
            self.script_configs[script_name]['cmd_line_args'] = script_config['cmd_line_args']
        
    def AutoStartEnabled_cb(self, msg):
        if msg.script not in self.script_configs:
            self.msg_if.pub_warn("Cannot configure autostart for unknown script " + msg.script)
            return
        
        self.msg_if.pub_info("Script AUTOSTART : " + msg.script + " - " + str(msg.enabled))
        
        self.script_configs[msg.script]['auto_start'] = msg.enabled

        # This is an unusual parameter in that it triggers an automatic save of the config file.
        # save_config() persists the param server's values to disk, so we must push the updated
        # script_configs to the param server FIRST -- otherwise the stale/empty value on the
        # param server is what gets written, and the auto_start setting is lost across restarts.
        if self.node_if is not None:
            self.node_if.set_param('script_configs', self.script_configs)
        self.node_if.save_config()

    def initCb(self, do_updates = False):
        if self.node_if is not None:
    
            if do_updates == True:
                # Read the script_configs parameter from the ROS parameter server
                self.update_script_configs()
            #self.publish_status()
        
        

        
    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            self.node_if.reset_params()
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def factoryResetCb(self,do_updates = True):
        if self.node_if is not None:
            self.node_if.factory_reset_params()
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)
        
    def get_scripts(self):
        """
        Detect and report scripts that exist in a particular directory in the filesystem.
        """
        #self.scripts = self.get_scripts()
        scripts = []
        for filename in os.listdir(self.scripts_folder):
            filepath = os.path.join(self.scripts_folder, filename)
            if not os.path.isdir(filepath):
                scripts.append(filename)
        return scripts

    def watch_directory(self, directory, callback):
        files_mtime = {}

        while True:
            for file in os.listdir(directory):
                file_path = os.path.join(directory, file)
                if os.path.isfile(file_path):
                    current_mtime = os.path.getmtime(file_path)
                    if file not in files_mtime or files_mtime[file] != current_mtime:
                        files_mtime[file] = current_mtime
                        callback(file_path, file_deleted=False)
            # And check for deleted files, too
            deleted_files = []
            for file in files_mtime.keys():
                if file not in os.listdir(directory):
                    deleted_files.append(file)
                    callback(os.path.join(directory, file), file_deleted=True)
            for file in deleted_files:
                del files_mtime[file]
            time.sleep(1)

    def on_file_change(self, file_path, file_deleted):
        script_name = os.path.basename(file_path)
        
        #Update the scripts list
        self.scripts = self.get_scripts()
        if file_deleted is False:
            # Update the script config here to set up the new/modified script
            self.setupScriptConfigs(single_script=script_name)

            # Update the file size in case it changed
            if os.path.exists(file_path):
                try:
                    f_size = os.path.getsize(file_path)
                except:
                    f_size = 0
            else:
                f_size = 0
            self.file_sizes[script_name] = f_size

            # Reset the script counters entry for this file... consider this a new script
            self.script_counters[script_name] = {'started': 0, 'completed': 0, 'stopped_manually': 0, 'errored_out': 0, 'cumulative_run_time': 0.0}
        else:
            # Just delete this script from all relevant dictionaries
            self.script_configs.pop(script_name, None)
            self.file_sizes.pop(script_name, None)
            self.script_counters.pop(script_name, None)

    def get_file_sizes(self):
        """
        Get the file sizes of the scripts in the specified directory.
        """
        file_sizes = {}
        for filename in self.scripts:
            filepath = os.path.join(self.scripts_folder, filename)
            file_size = os.path.getsize(filepath)
            file_sizes[filename] = file_size
        return file_sizes

    def handle_get_scripts(self, req):
        return GetScriptsQueryResponse(sorted(self.scripts))
    
    def handle_get_running_scripts(self, req):
        """
        Handle a request to get a list of currently running scripts.
        """
        running_scripts = sorted(list(self.running_scripts))

        return GetRunningScriptsQueryResponse(running_scripts)

    def handle_launch_script(self, req):
        """
        Handle a request to launch anscript.
        """            
        if req.script in self.scripts:
            if req.script not in self.running_scripts:
                try:
                    # Ensure the script is executable (and readable/writable -- why not)
                    script_full_path = os.path.join(self.scripts_folder, req.script)
                    os.chmod(script_full_path, 0o774)

                    # Set up logfile. Because we pipe stdout from the script into the file, we must pay attention to buffering at
                    # multiple levels. open() and Popen() are set for line buffering, and we launch the script in a PYTHONUNBUFFERED environment
                    # to ensure that the script print() calls don't get buffered... without that last one, everything is buffered at 8KB no matter
                    # what open() and Popen() are set to.
                    process_cmdline = [script_full_path] + self.script_configs[req.script]['cmd_line_args'].split()
                    script_logfilename = os.path.join(self.scripts_log_folder, req.script + '.log')
                    script_logfile = open(script_logfilename, 'wt', buffering=1) # buffering=1 ==> Line buffering
                    curr_env = os.environ.copy()
                    curr_env['PYTHONUNBUFFERED'] = 'on'
                    
                    process = subprocess.Popen(process_cmdline, stdout=script_logfile, stderr=subprocess.STDOUT, bufsize=1, env=curr_env) # bufsize=1 ==> Line buffering
                    self.processes[req.script] = {'process': process, 'pid': process.pid, 'start_time': psutil.Process(process.pid).create_time(), 'logfile': script_logfile}
                    self.processes[req.script] = {'process': process, 'pid': process.pid, 'logfile': script_logfile}
                    self.running_scripts.add(req.script)  # Update the running_scripts set
                    self.msg_if.pub_info("running: " + str(req.script))
                    self.script_counters[req.script]['started'] += 1  # update the counter
                    return LaunchScriptResponse(True)
                except Exception as e:
                    return LaunchScriptResponse(False)
            else:
                self.msg_if.pub_warn("is already running... will not start another instance " +  str(req.script))
                return LaunchScriptResponse(False)
        else:
            self.msg_if.pub_info("not found: " + str(req.script))
            return LaunchScriptResponse(False)

    def handle_stop_script(self, req):
        """
        Handle a request to stop anscript.
        """
        if req.script in self.processes:
            process = self.processes[req.script]['process']
            retval = False
            try:
                process.terminate()
                process.wait(timeout=self.script_stop_timeout_s)
                self.script_counters[req.script]['cumulative_run_time'] += 1 #(nepi_sdk.get_msg_stamp() - nepi_sdk.msg_stamp_from_sec(self.processes[req.script]['start_time'])).to_sec()
                self.processes[req.script]['logfile'].close()
                del self.processes[req.script]
                self.running_scripts.remove(req.script)  # Update the running_scripts set
                self.msg_if.pub_info("stopped: " + str(req.script))
                self.script_counters[req.script]['stopped_manually'] += 1  # update the counter
                retval = True
            except Exception as e:
                process.kill()
                try:
                    process.wait(timeout=self.script_stop_timeout_s)
                    retval = True
                except Exception as e2:
                    self.msg_if.pub_warn("Failed to kill process" + str(e2))
                    retval = False
            
            return retval
        else:
            self.msg_if.pub_warn("Script not running " + str(req.script))
            return False
    
    def monitor_scripts(self):
        """
        Monitor the status of allscripts.
        """
        while not nepi_sdk.is_shutdown():
            for script in self.scripts:
                if script in self.processes:
                    process = self.processes[script]
                    if process['process'].poll() is not None:
                        del self.processes[script]
                        self.running_scripts.remove(script)  # Update the running_scripts set

                        process['logfile'].close()
                        if process['process'].returncode == 0:
                            self.script_counters[script]['completed'] += 1
                            self.msg_if.pub_info("completed: " + str(script))
                        else:
                            self.script_counters[script]['errored_out'] += 1

                        # Update the cumulative run time whether exited on success or error
                        self.script_counters[script]['cumulative_run_time'] += 1 #(nepi_sdk.get_msg_stamp() - nepi_sdk.msg_stamp_from_sec(process['start_time'])).to_sec()
                        process['logfile'].close()
            # Pace the loop -- previously only slept when a script had just
            # completed, so with zero scripts configured (or none completing)
            # this spun unbounded, pegging a full CPU core permanently.
            nepi_sdk.wait()


    def handle_get_system_stats(self, req):
        script_name = req.script
        response = GetSystemStatsQueryResponse(cpu_percent=None, memory_percent=None, run_time_s=None,
                                               cumulative_run_time_s=None, file_size_bytes=None, log_size_bytes=None, started_runs=None,
                                               completed_runs=None, error_runs=None, stopped_manually=None, auto_start_enabled=None)

        if (script_name not in self.file_sizes) or (script_name not in self.script_counters) or (script_name not in self.script_configs):
            self.msg_if.log_msg_warn("Requested script not found: " + script_name, throttle_s = 10)
            return response # Blank response

        # Get file size for the script_name
        response.file_size_bytes = self.file_sizes[script_name]

        # Get the counter values for the script_name
        response.started_runs = self.script_counters[script_name]['started']
        response.completed_runs = self.script_counters[script_name]['completed']
        response.error_runs = self.script_counters[script_name]['errored_out']
        response.stopped_manually = self.script_counters[script_name]['stopped_manually']
        response.cumulative_run_time_s = self.script_counters[script_name]['cumulative_run_time']
        
        # And config info we want to feed back... TODO: maybe these should be part of a totally separate request
        response.auto_start_enabled = self.script_configs[script_name]['auto_start']

        # Check if the script_name is in the running list
        if (script_name not in self.running_scripts):
            try:
                response.log_size_bytes = os.path.getsize(os.path.join(self.scripts_log_folder, script_name + ".log"))
            except:
                pass
            return response  # Only includes the 'static' info
        
        pid = self.processes[script_name]['pid']
        response.log_size_bytes = os.fstat(self.processes[script_name]['logfile'].fileno()).st_size
        try:
            # Get resource usage for the specific PID
            #usage = resource.getrusage(resource.RUSAGE_CHILDREN)
            process = psutil.Process(pid)

            # Get CPU usage
            #self.cpu_percent = (usage.ru_utime + usage.ru_stime) / os.sysconf("SC_CLK_TCK")
            response.cpu_percent = process.cpu_percent(0.1)

            # Get memory usage
            #self.memory_usage = usage.ru_maxrss
            response.memory_percent = 100 * float(process.memory_full_info().uss) / float(psutil.virtual_memory().total)
                        
            # Get creation/start-up time
            response.run_time_s = (nepi_sdk.get_msg_stamp() - nepi_sdk.msg_stamp_from_sec(process.create_time())).to_sec()
            # The script_counters cumulative run time only gets updated on script termination, so to keep this value moving in the response,
            # increment it here.
            response.cumulative_run_time_s += response.run_time_s
            #self.msg_if.pub_info("CPU Percent: %.5f%%, Memory Usage: %.5f%%, Run Time: %.2f" % (response.cpu_percent, response.memory_percent, response.run_time_s))

        except Exception as e:
            self.msg_if.pub_warn("Error gathering running stats: " + str(e))
            return response  # Add new None values for the counters
        # Return the system stats as a GetSystemStatsQuery response object
        return response


if __name__ == '__main__':
    ScriptsManager()
