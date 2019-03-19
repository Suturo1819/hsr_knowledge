#!/usr/bin/env python

import rospy
import os
import yaml
from rospkg import RosPack
from json_prolog import json_prolog

rospack = RosPack()
prolog = json_prolog.Prolog()


def check_caffe_config_yaml(package='object_state', path='/example_perception.yaml'):
    rospy.loginfo("Waiting for json_prolog.")
    rospy.wait_for_service('/json_prolog/query')
    file_path = rospack.get_path(package) + path
    caffe_splits_names = []

    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            content = file.read().replace('%YAML:1.0', '')
            caffe_splits_names = yaml.load(content)
    else:
        rospy.logerr('File ' + path + ' in package ' + package + 'does not exist.')

    unresolved_classes = []

    for c in caffe_splits_names['classes']:
        owlised_name = c.capitalize().replace('_', '')
        query_string = "owl_description(hsr_objects:'" + owlised_name + "',_)"
        solution = prolog.all_solutions(query_string)
        if not solution:
            unresolved_classes.append([c, owlised_name])

    if unresolved_classes:
        rospy.logwarn("Couldn't find classes:")
        for cls in unresolved_classes:
            rospy.logwarn(cls[0] + " translated to owl-class " + cls[1])
    else:
        rospy.loginfo("All classes found in ontology.")


if __name__ == '__main__':
    rospy.init_node('caffe_config', anonymous=True)
    check_caffe_config_yaml()
