#!/usr/bin/env python
#import roslib; roslib.load_manifest('json_prolog')

import rospy
from json_prolog import json_prolog
from suturo_perception_msgs.msg import ObjectDetectionData
import time

prolog = json_prolog.Prolog()

def callback(perceivedObject):
    rospy.wait_for_service('/json_prolog/query')
    rospy.loginfo(perceivedObject.name)

    obj_class = str(perceivedObject.obj_class)
    if obj_class:
        obj_class = obj_class[0].upper() + obj_class[1:]
    else:
        obj_class = "Cup"
        
    confidence = str(perceivedObject.confidence)
    shape = str(perceivedObject.shape)
    source_frame = "map" # str(perceivedObject.pose.header.frame_id)

    x = str(perceivedObject.pose.pose.position.x)
    y = str(perceivedObject.pose.pose.position.y)
    z = str(perceivedObject.pose.pose.position.z)

    qx = "0" #str(perceivedObject.pose.pose.orientation.x)
    qy = "0" #str(perceivedObject.pose.pose.orientation.y)
    qz = "0" #str(perceivedObject.pose.pose.orientation.z)
    qw = "1" #str(perceivedObject.pose.pose.orientation.w)

    threshold = "0.15"

    query_string = ("object_at(knowrob:'" + obj_class + "',"
                    "['" + source_frame + "', _, [" + x + "," + y + "," + z + "]," 
                    "[" + qx + "," + qy + "," + qz + "," + qw + "]]," 
                    + threshold + ", ObjectInstance).")

    solutions = prolog.all_solutions(query_string)
    rospy.loginfo('Send query: \n' + query_string)
    rospy.loginfo(solutions)

    
def listener():
    rospy.init_node('perception_listener', anonymous=True)
    rospy.Subscriber("suturo_perception/object_detection", ObjectDetectionData, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
