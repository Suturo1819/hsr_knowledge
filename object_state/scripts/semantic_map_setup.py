#!/usr/bin/env python
# rosrun object_state semantic_map_setup.py

import rospy
from yaml import load
from rosparam import upload_params
import rospkg

rospack = rospkg.RosPack()


def load_rs_yaml_to_parameters():
    with open(rospack.get_path('suturo_perception_msgs') + "/config/semantic_map.yaml", "r") as file:
        content = file.read().replace('!!opencv-matrix', '').replace('%YAML:1.0', '')
        yamlfile = load(content)
        upload_params("/semantic_map", yamlfile)


if __name__ == '__main__':
    load_rs_yaml_to_parameters()
