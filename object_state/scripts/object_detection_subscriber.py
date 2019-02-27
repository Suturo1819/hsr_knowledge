#!/usr/bin/env python
#import roslib; roslib.load_manifest('json_prolog')

import rospy
from json_prolog import json_prolog
from suturo_perception_msgs.msg import ObjectDetectionData
from suturo_perception_msgs.msg import PerceiveActionResult
import time

prolog = json_prolog.Prolog()

def callback(perceivedObjectList):

    for perceivedObject in perceivedObjectList.result.detectionData:
        rospy.wait_for_service('/json_prolog/query')
        rospy.loginfo(perceivedObject.name)

        obj_class = str(perceivedObject.obj_class)
        if obj_class:
            splits = obj_class.split('_') #snakecase to upper camelcase
            obj_class = ''.join(x.title() for x in splits)
            if obj_class in ["Tomato", "Potato", "BellPepper"]:
                obj_class += "-Foodstuff"
        else:
            obj_class = "Cup"
            
        confidence = str(perceivedObject.confidence)
        shape = str(perceivedObject.shape)
        source_frame = "map" # str(perceivedObject.pose.header.frame_id)

        x = str(perceivedObject.pose.pose.position.x)
        y = str(perceivedObject.pose.pose.position.y)
        z = str(perceivedObject.pose.pose.position.z)

        qx = str(perceivedObject.pose.pose.orientation.x)
        qy = str(perceivedObject.pose.pose.orientation.y)
        qz = str(perceivedObject.pose.pose.orientation.z)
        qw = str(perceivedObject.pose.pose.orientation.w)

        threshold = "0.15"

        query_string = ("create_object_at(knowrob:'" + obj_class + "',"
                        "['" + source_frame + "', _, [" + x + "," + y + "," + z + "]," 
                        "[" + qx + "," + qy + "," + qz + "," + qw + "]]," 
                        + threshold + ", ObjectInstance).")

        solutions = prolog.all_solutions(query_string)
        rospy.loginfo('Send query: \n' + query_string)
        rospy.loginfo(solutions)

    
def listener():
    rospy.init_node('perception_listener', anonymous=True)
    rospy.Subscriber("hsr_perception/result", PerceiveActionResult, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
