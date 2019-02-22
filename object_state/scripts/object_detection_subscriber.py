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

    # if ((not currObj.name in object_map) 
    #     or (is_near_list(currObj, object_map[currObj.name]) == -1) 
    #     or (last_update_map[currObj.name].to_nsec() < rospy.Time.now().to_nsec() - TIMEOUT)):
    #         before = time.time()
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
    #query.finish()

    # for solution in query.solutions():
    #     rospy.loginfo('Found solution.')
    # query.finish()
    #sleep(2)
            # rospy.loginfo('Took: ' + str(time.time() - before))
            # if (not currObj.name in object_map):
            #     object_map[currObj.name] = [currObj]
            # elif (index_of_is_near == -1):
            #     object_map[currObj.name].append(currObj)
            # else:
            #     object_map[index_of_is_near] = currObj
            # last_update_map[currObj.name] = rospy.Time.now()



    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('perception_listener', anonymous=True)
    #rospy.wait_for_service('json_prolog/simple_query', timeout=10)

    
    rospy.Subscriber("suturo_perception/object_detection", ObjectDetectionData, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
