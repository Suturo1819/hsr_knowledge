#!/usr/bin/env python
# rosrun object_state semantic_map_setup.py

import rospy
import os
import yaml
import tf
import rospkg
from tf.transformations import euler_matrix, quaternion_matrix
import functools

rospack = rospkg.RosPack()


def write_region_filter_yaml_from_manual():
    listener = tf.TransformListener()

    sem_map_manual_yaml_path = rospack.get_path("suturo_perception_msgs") + "/config/semantic_map_manual_data.yaml"
    sem_map_yaml = dict()
    if os.path.exists(sem_map_manual_yaml_path):
        with open(sem_map_manual_yaml_path, "r") as file:
            sem_map_yaml = yaml.load(file.read())
    else:
        rospy.logwarn("The file 'config/semantic_map_manual_data.yaml' "
                      "in package 'suturo_perception_msgs' does not exist.")

    opencv_data = dict(names=[])
    region_bottom_offset = sem_map_yaml['surface_bottom_offset']
    region_top_offset = sem_map_yaml['surface_top_offset']
    region_height = region_bottom_offset + region_top_offset

    now = rospy.Time(0)
    (trans, rot) = listener.lookupTransform("/map", "/environment/table_surface_center", now)
    pose = tf_to_region_matrix(trans, rot, region_bottom_offset, region_top_offset)
    opencv_data['robocup_table'] = to_opencv_dict(sem_map_yaml['robocup_table'], pose, region_height)
    opencv_data['names'].append('robocup_table')

    shelf_region_frame_map = [['robocup_shelf_0', "/environment/shelf_floor_0_piece"],
                              ['robocup_shelf_1', "/environment/shelf_floor_1_piece"],
                              ['robocup_shelf_2', "/environment/shelf_floor_2_piece"],
                              ['robocup_shelf_3', "/environment/shelf_floor_3_piece"],
                              ['robocup_shelf_4', "/environment/shelf_floor_4_piece"]]

    for (region, frame) in shelf_region_frame_map:
        opencv_data['names'].append(region)
        (trans, rot) = listener.lookupTransform("/map", frame, now)
        pose = tf_to_region_matrix(trans, rot, region_bottom_offset, region_top_offset)
        opencv_data[region] = to_opencv_dict(sem_map_yaml['robocup_shelf'], pose, region_height)

    write_dict_to_ocv_yaml(opencv_data, 'semantic_map.yaml')


def tf_to_region_matrix (trans, rot, bottom_offset, top_offset):
    pose = quaternion_matrix(rot[0], rot[1], rot[2], rot[3])
    pose[0][3] = trans[0]
    pose[1][3] = trans[1]
    pose[2][3] = trans[2] + (top_offset - bottom_offset)/2
    return pose


def to_opencv_dict(input_dict, pose, region_height):
    ocv_dict = dict()
    ocv_dict['type'] = input_dict['type']
    ocv_dict['width'] = input_dict['width']
    ocv_dict['height'] = region_height
    ocv_dict['depth'] = input_dict['depth']
    ocv_dict_t = dict()
    ocv_dict_t['rows'] = 4
    ocv_dict_t['cols'] = 4
    ocv_dict_t['dt'] = 'd'
    ocv_dict_t['data'] = str(functools.reduce(lambda a, b: a + b, pose.tolist()))
    ocv_dict['transform'] = ocv_dict_t
    return ocv_dict


def write_dict_to_ocv_yaml (dictionary, filename):
    with open(filename, 'w') as outfile:
        yaml.dump(dictionary, outfile, default_flow_style=False)

    opencv_yaml_flavor = "%YAML:1.0\n"
    with open(filename, 'r') as outfile:
        opencv_yaml_flavor += outfile.read().replace('\'', '').replace("transform:", "transform: !!opencv-matrix")

    with open(filename, 'w') as outfile:
        outfile.write(opencv_yaml_flavor)


if __name__ == '__main__':
    rospy.init_node('semantic_map_setup', anonymous=True)
    write_region_filter_yaml_from_manual()
