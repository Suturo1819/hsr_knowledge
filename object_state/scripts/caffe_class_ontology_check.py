#!/usr/bin/env python

import rospy
import os
import yaml
import rospkg
from json_prolog import json_prolog

rospack = rospkg.RosPack()
prolog = json_prolog.Prolog()


def transform_caffe_config_yaml(package='object_state', path='/example_perception.yaml'):
    rospy.loginfo("Waiting for json_prolog.")
    rospy.wait_for_service('/json_prolog/query')
    file_path = rospack.get_path(package) + path
    caffe_splits_map = []

    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            content = file.read().replace('%YAML:1.0', '')
            caffe_splits_map = yaml.load(content)
    else:
        rospy.logerr('File ' + path + ' in package ' + package + 'does not exist.')

    output_yaml = dict()
    output_yaml['classes'] = []

    for c in caffe_splits_map['classes']:
        query_string = "owl_description(hsr_objects:'" + c.capitalize().replace('_', '') + "',_)"
        solution = prolog.all_solutions(query_string)
        if not solution:
            print(c.capitalize() + ' is missing')
        else:
            output_yaml['classes'].append(c)


if __name__ == '__main__':
    rospy.init_node('caffe_config', anonymous=True)
    transform_caffe_config_yaml()
