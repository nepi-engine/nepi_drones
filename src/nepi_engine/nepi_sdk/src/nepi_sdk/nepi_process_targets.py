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

import copy
import math

import numpy as np

from nepi_sdk import nepi_utils
from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_process
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data
from nepi_sdk import nepi_img

from nepi_interfaces.msg import ProcessResultsTargets
from nepi_interfaces.msg import NavPose

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_process_targets"
logger = Logger(log_name = log_name)


########################
## REQUIRED Process IF Utilities


DEFAULT_PROCESS = 'targets_1'

RESULTS_PUB_MSG = ProcessResultsTargets
RESULTS_PUB_TYPE = 'nepi_interfaces/ProcessResultsTargets'
RESULTS_PUB_DICT = nepi_sdk.convert_msg2dict(RESULTS_PUB_MSG())
RESULTS_PUB_TOPIC = 'targets'




def convert_results_pub_dict2msg( msg, msg_type, results_pub_dict):
    msg = RESULTS_PUB_MSG
    msg_type = RESULTS_PUB_TYPE
    results_msg = None
    if msg is not None and msg_type is not None and results_pub_dict is not None:
            results_dict = nepi_sdk.convert_msg2dict(msg())

            for result_name in results_dict.keys():
                if result_name in results_pub_dict.keys():
                    results_dict[result_name] = results_pub_dict[result_name]
            results_msg = nepi_sdk.convert_dict2msg(msg_type, results_dict)
    return results_msg


def convert_results_pub_msg2dict(self, results_msg):
    results_dict = nepi_sdk.convert_msg2dict(results_msg)
    return results_dict




def process_results_image(cv2_img, status_dict, controls_dict, results_dict):
        ##################
        # Get Image Data
        try:
            cv2_img_results = copy.deepcopy(cv2_img)

        except:
            return cv2_img

        if status_dict is None:
            status_dict = dict()
        width_deg = status_dict.get('width_deg', 100)
        height_deg = status_dict.get('height_deg', 70)

        ##################
        # Get Controls Data
        if controls_dict is None:
            controls_dict = dict()
        overlay_color = controls_dict.get('overlay_color',(0,0,127))
        overlay_font = controls_dict.get('overlay_color',nepi_img.OVERLAY_FONT)
        overlay_font_color = controls_dict.get('overlay_color',nepi_img.OVERLAY_FONT_COLOR)
        overlay_line_type = controls_dict.get('overlay_color',nepi_img.OVERLAY_LINE_TYPE)
        overlay_line_color = controls_dict.get('overlay_color',nepi_img.OVERLAY_LINE_COLOR)
        overlay_labels = controls_dict.get('overlay_labels',True)
        overlay_range_bearing = controls_dict.get('overlay_range_bearing',True)

        ##################
        # Get Results Data
        if results_dict is None:
            results_dict = dict()       
        targets_list = results_dict.get('targets', [])

        ##################
        # Process Results Image


        for i, target_dict in enumerate(targets_list):
            try:
                cv2_shape = cv2_img.shape
                img_width = cv2_shape[1] 
                img_height = cv2_shape[0] 


                ###### Apply Image Overlays and Publish Image ROS Message
                # Overlay adjusted detection boxes on image 
                class_name = target_dict['name']
                xmin = target_dict['xmin_pixel']
                ymin = target_dict['ymin_pixel']
                xmax = target_dict['xmax_pixel']
                ymax = target_dict['ymax_pixel']

                if xmin <= 0:
                    xmin = 5
                if ymin <= 0:
                    ymin = 5
                if xmax >= img_width:
                    xmax = img_width - 5
                if ymax >= img_height:
                    ymax = img_height - 5


                bot_left_px = (xmin, ymin)
                top_right_px = (xmax, ymax)


                class_color = overlay_color
            
                #logger.log_warn("Got Class Color: " + str(class_color) + ' type: ' + str(type(class_color)) + " type: " + str(type(class_color[0])) )
                line_thickness = max(1, math.ceil(max([img_height, img_width])/2000))
                

                success = False
                try:
                    cv2_img_results = nepi_img.overlay_bounding_box(cv2_img_results,bot_left_px, top_right_px, line_color=class_color, line_thickness=line_thickness)
                    success = True
                except Exception as e:
                    logger.log_warn("Failed to create bounding box rectangle: " + str(e))

                # Overlay text data on OpenCV image
                if success == True:

                    overlay_text = ""

                    if overlay_labels:
                        overlay_text = overlay_text + class_name + " "
                        
                    if overlay_range_bearing:
                        rb_text = ''
                        if target_dict['range_m'] != -999 and target_dict['range_m'] != '':
                            rb_text = rb_text + str(round(target_dict['range_m'],1)) + 'm :'
                        if target_dict['azimuth_deg'] != -999 and target_dict['elevation_deg'] != -999:
                            rb_text = rb_text + str(round(target_dict['azimuth_deg'],1)) + 'deg '
                            rb_text = rb_text + str(round(target_dict['elevation_deg'],1)) + 'deg '
                        if len(rb_text) > 0:
                            overlay_text = overlay_text + rb_text


                    if len(overlay_text) > 0:

                        text_size = nepi_img.optimal_text_size
                        #logger.log_warn("Text Size: " + str(text_size))
                        line_height = text_size[0][1]
                        line_width = text_size[0][0]
                        x_padding = int(line_height*0.4)
                        y_padding = int(line_height*0.4)
                        
                        center = bot_left_box[0] + int(( top_right_box[0] - bot_left_box[0]) / 2 )
                        #bot_left_text = (xmin + (line_thickness * 2) + x_padding , ymin + line_height + (line_thickness * 2) + y_padding)
                        bot_left_text = (center + x_padding , ymin - (line_thickness * 2) - y_padding)
                        # Create Text Background Box
                        #bot_left_box =  (bot_left_text[0] - x_padding , bot_left_text[1] + y_padding)
                        bot_left_box =  ( center - x_padding, bot_left_text[1] + y_padding)
                        top_right_box = (center + line_width + x_padding, bot_left_text[1] - line_height - y_padding )

                        cv2_img_results = overlay_text(cv2_img_results, overlay_text, x_px = 10 , y_px = 10, color_rgb = class_color, scale = None, thickness = None, background_rgb = None, apply_shadow = True)

            except:
                pass

        return cv2_img_results
    



########################
## Process Utility Functions

def update_results(results_dict, targets_dict):
    results_pub_dict = None
    if targets_dict is not None:
        results_dict = nepi_data.set_data_values(results_dict, targets_dict)
    
        results_pub_dict = copy.deepcopy(RESULTS_PUB_DICT)
        #print([results_dict,results_pub_dict])
        for key in targets_dict.keys():
            if key in results_pub_dict.keys():
                results_pub_dict[key] = targets_dict[key]

    timestamp = nepi_data.get_datum_value(results_dict, 'timestamp')
    if timestamp == -999:
        age_sec = -999
    else:
        age_sec =  nepi_utils.get_time() - timestamp
    results_dict = nepi_data.set_datum_value(results_dict, 'age_sec', age_sec)
    return results_dict, results_pub_dict


########################
## Process Functions   
#######################
processes_dict = dict()
functions_dict = dict()



########################
## Process 1   



targets_1_dict = {

  
    'data_dict': dict(
        targets_dict_list = [], 
        navpose_dict = nepi_sdk.convert_msg2dict(NavPose()),
        last_targets_time = 0,
        last_targets_dict = None
    ),


    'controls_dict': dict(

        class_filters = {"type":"Selections", "default":[], "options":[], 
                   # OPTIONAL
                   'display_name':'Select Classes', 'description':'Set Class Filters', 'hidden':False}, 

        size_min_filter = {
            'type': 'FloatSlider', 'default': 0.001, 'bounds': [0.0, 1.0], 'round_value': 3,
            'display_name': 'Max Range (m)',
            'description': 'Ignore targets with pixel areas less than min.', 'hidden': False},

        size_max_filter = {
            'type': 'FloatSlider', 'default': 0.99, 'bounds': [0.0, 1.0], 'round_value': 3,
            'display_name': 'Max Range (m)',
            'description': 'Ignore targets with pixel areas larger than max.', 'hidden': False},

        threshold_filter = {
            'type': 'FloatSlider', 'default': 0.3, 'bounds': [0.0, 1.0], 'round_value': 1,
            'display_name': 'Max Range (m)',
            'description': 'Ignore targets with confidance lower than threshold.', 'hidden': False},

        best_filter = {"type":"Selection", "default":['LARGEST'], "options":BEST_FILTER_OPTIONS, 
                   # OPTIONAL
                   'display_name':'Best Filter', 'description':'Set Best Filte', 'hidden':False}, 

    ),


    'results_dict': dict(

        timestamp = {"type":"Float", "value":-999,
                    # OPTIONAL
                    'display_name':'Timestamp', 'description':'Timestamp', 'hidden':True},

        age_sec = {"type":"Float", "value":-999, 'round_value': 3,
                    # OPTIONAL
                    'display_name':'Age (Sec)', 'description':'Age in seconds', 'hidden':False, 'round_display': 3,},

        azimuth_deg = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Azimuth (Deg)', 'description':'Degrees in horizontal axis to targetsed target', 'hidden':False, 'round_display': 1,},

        elevation_deg = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Elevation (Deg)', 'description':'Degrees in vertical axis to targetsed target', 'hidden':False, 'round_display': 1,},

        range_m = {"type":"Float", "value":2.0, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Range (M)', 'description':'Range in meters to targetsed target', 'hidden':False, 'round_display': 1,},
    ),

}


def targets_1_process(data_dict, controls_dict, results_dict):
    start_time = nepi_utils.get_time()
    last_data_dict = copy.deepcopy(data_dict)
    last_results_dict = copy.deepcopy(results_dict)
    controls_values_dict = nepi_controls.get_controls_values_dict(controls_dict)
    #logger.log_warn("Got Data: " + str(data_dict), throttle_s = 5)
    #logger.log_warn("Got Controls: " + str(controls_values_dict), throttle_s = 10)


    #logger.log_warn("Got Data and Controls: " + str([data_dict, controls_dict]), throttle_s = 5)
    results_pub_dict = None
    targets_dict = None

    results_pub_msg = None

    return data_dict, controls_dict, results_dict, results_pub_msg


processes_dict = nepi_process.update_processes_dict(processes_dict, process_name = 'targets_1', process_dict = targets_1_dict)
#logger.log_warn("Updated processes dict: " + str(processes_dict))
functions_dict['targets_1'] = targets_1_process





########################
## Process 2  



targets_2_dict = {
   
    'data_dict': dict(
        targets_dict_list = [], 
        navpose_dict = nepi_sdk.convert_msg2dict(NavPose()),
        last_targets_time = 0,
        last_targets_dict = None
    ),


    'controls_dict': dict(

        class_filters = {"type":"Selections", "default":[], "options":[], 
                   # OPTIONAL
                   'display_name':'Select Classes', 'description':'Set Class Filters', 'hidden':False}, 


        threshold_filter = {
            'type': 'FloatSlider', 'default': 0.3, 'bounds': [0.0, 1.0], 'round_value': 1,
            'display_name': 'Max Range (m)',
            'description': 'Ignore targets with confidance lower than threshold.', 'hidden': False},


    ),


    'results_dict': dict(

        timestamp = {"type":"Float", "value":-999,
                    # OPTIONAL
                    'display_name':'Timestamp', 'description':'Timestamp', 'hidden':True},

        age_sec = {"type":"Float", "value":-999, 'round_value': 3,
                    # OPTIONAL
                    'display_name':'Age (Sec)', 'description':'Age in seconds', 'hidden':False, 'round_display': 3,},

    ),

}


def targets_2_process(data_dict, controls_dict, results_dict):
    start_time = nepi_utils.get_time()
    last_data_dict = copy.deepcopy(data_dict)
    last_results_dict = copy.deepcopy(results_dict)
    controls_values_dict = nepi_controls.get_controls_values_dict(controls_dict)
    #logger.log_warn("Got Data: " + str(data_dict), throttle_s = 5)
    #logger.log_warn("Got Controls: " + str(controls_values_dict), throttle_s = 10)


    #logger.log_warn("Got Data and Controls: " + str([data_dict, controls_dict]), throttle_s = 5)
    results_pub_dict = None
    targets_dict = None

    results_pub_msg = None

    return data_dict, controls_dict, results_dict, results_pub_msg


processes_dict = nepi_process.update_processes_dict(processes_dict, process_name = 'targets_2', process_dict = targets_2_dict)
#logger.log_warn("Updated processes dict: " + str(processes_dict))
functions_dict['targets_2'] = targets_2_process

########################
## Processes Init Dict  
PROCESSES_DICT = copy.deepcopy(processes_dict)
FUNCTIONS_DICT = functions_dict