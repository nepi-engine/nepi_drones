#!/usr/bin/env python

##
## Copyright (c) 2024 Numurus <https://www.numurus.com>.
##
## This file is part of nepi setup tools (nepi_setup) repo
## (see https://github.com/nepi-engine/nepi_setup)
##
## License: nepi setup tools are licensed under the "Numurus Software License", 
## which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
##
## Redistributions in source code must retain this top-level comment block.
## Plagiarizing this software to sidestep the license obligations is illegal.
##
## Contact Information:
## ====================
## - mailto:nepi@numurus.com
##


# This script loads and exports key values from file


import os
import sys
import yaml

# for arg in sys.argv:
#     print_string=("arg=" + str(arg))
#     print_list.append(print_string)

print_list=[]
def read_yaml_2_dict(file_path):
    dict_from_file = dict()
    dict_return=dict()
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                dict_from_file = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
           pass
        if dict_from_file is None:
           print_list.append("success=-2")
        else:
            for key in dict_from_file.keys():
                
                if isinstance(dict_from_file[key], dict) == False:
                    dict_return[key] = dict_from_file[key]
                else:
                    dict_level2=dict_from_file[key]
                    for subkey2 in dict_level2.keys():
                        if isinstance(dict_level2[subkey2], dict) == False:
                            dict_return[subkey2] = dict_level2[subkey2]
                        else:
                            # A 3rd-level dict (e.g. a top-level key whose
                            # value is itself a dict of dicts, such as a
                            # robot-configs-style table) is left un-flattened
                            # rather than descended into. The line this
                            # replaced -- "for subkey3 in
                            # dict_level2.keys.subkey2():" -- is not valid
                            # Python (calling .subkey2() on the .keys bound
                            # method) and raised AttributeError on any file
                            # with this shape, silently emptying the whole
                            # result (the caller's yaml.load is wrapped in
                            # try/except, but this loop was not) -- losing
                            # even the already-flattened level-1/level-2
                            # fields from earlier keys, not just this one.
                            # No caller of this script has ever needed a
                            # level-3 value promoted to the flat namespace,
                            # so skipping it is strictly safer than the
                            # broken attempt to recurse further.
                            pass
    else:
       print_list.append("success=-3")
    return dict_return

if len(sys.argv) > 1:
    YAML_FILE = sys.argv[1]
    # print_string=("yfile=" + str(YAML_FILE))
    # print_list.append(print_string)
    CONFIG_DICT = read_yaml_2_dict(YAML_FILE)
    if CONFIG_DICT is not None:
        if len(CONFIG_DICT.keys()) > 0:
            for key in CONFIG_DICT.keys():
                print_string=(str(key) + "=" + str(CONFIG_DICT[key]))
                print_list.append(print_string)
            print_list.append("success=1")
        else:
            print_list.append("success=-4")
    
else:
    print_list.append("success=-1")

print_string="\'"
for entry in print_list:
    print_string += entry + " "
print_string += "\'"
print(print_string)
