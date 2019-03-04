#!/usr/bin/env python

import rospy
from json_prolog import json_prolog
# from suturo_perception_msgs.msg import ObjectDetectionData
from suturo_perception_msgs.msg import PerceiveActionResult

prolog = json_prolog.Prolog()


def map_class_name(cls_name):
    if cls_name in ["Tomato", "Potato", "BellPepper"]:
        cls_name += "-Foodstuff"
    elif cls_name in ["FoodCan", "FoodJar"]:
        cls_name = "FoodVessel"
    elif cls_name == "Vegetable":
        cls_name += "-Food"
    return cls_name


def callback(perceived_object_list):
    for perceivedObject in perceived_object_list.result.detectionData:
        rospy.wait_for_service('/json_prolog/query')
        rospy.loginfo(perceivedObject.name)

        obj_class = str(perceivedObject.obj_class)
        if obj_class:
            splits = obj_class.split('_')  # snakecase to upper camelcase
            obj_class = map_class_name(''.join(x.title() for x in splits))
        else:
            rospy.loginfo("The given class name" + obj_class + "is empty. Setting to CUP.")
            obj_class = "Cup"

        confidence = str(perceivedObject.confidence)
        shape = str(perceivedObject.shape)
        source_frame = "map"  # str(perceivedObject.pose.header.frame_id)

        x = str(perceivedObject.pose.pose.position.x)
        y = str(perceivedObject.pose.pose.position.y)
        z = str(perceivedObject.pose.pose.position.z)

        qx = str(perceivedObject.pose.pose.orientation.x)
        qy = str(perceivedObject.pose.pose.orientation.y)
        qz = str(perceivedObject.pose.pose.orientation.z)
        qw = str(perceivedObject.pose.pose.orientation.w)

        threshold = "0.05"

        query_string = ("create_object_at(knowrob:'" + obj_class + "',"
                                                                   "['" + source_frame + "', _, [" + x + "," + y + "," + z + "],"
                                                                                                                             "[" + qx + "," + qy + "," + qz + "," + qw + "]],"
                        + threshold + ", ObjectInstance).")

        rospy.loginfo('Send query: \n' + query_string)

        solutions = prolog.all_solutions(query_string)

        rospy.loginfo(solutions)


def listener():
    rospy.init_node('perception_listener', anonymous=True)
    rospy.Subscriber("hsr_perception/result", PerceiveActionResult, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
