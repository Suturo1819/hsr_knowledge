#!/usr/bin/env python

import rospy
from suturo_knowledge_msgs.srv import *
from json_prolog import json_prolog
import tf

prolog = json_prolog.Prolog()


def handle_object_location_in_shelf(request):
    rospy.loginfo("Receiving location request for object '%s'." % request.object_id)

    # Hackiddy hacky pose from TF. Just prototype dummy.
    (trans, rot) = safe_lookup_transform("/environment/shelf_floor_2_piece")
    if not trans:
        return ObjectLocationInShelfResponse("tf_error")
    trans[2] += 0.15

    query = "create_object_at(knowrob:'Goal', ['map', _, %s, %s], 0.1, Instance)" % (str(trans), str(rot))
    solution = prolog.all_solutions(query)
    if solution:
        goal_frame = str(solution[0]['Instance']).split('#')[1].replace('\'', '')
        rospy.loginfo("new goal: %s" % goal_frame)
        return ObjectLocationInShelfResponse(goal_frame)
    else:
        return ObjectLocationInShelfResponse("no_solution_error")


def safe_lookup_transform(source_frame, duration=rospy.Duration(3)):
    listener = tf.TransformListener()
    now = rospy.Time()
    try:
        listener.waitForTransform("/map", source_frame, now, duration)
        return listener.lookupTransform("/map", source_frame, now)
    except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException):
        rospy.logerr("No transform between 'map' and '%s'" % source_frame)
        return [None, None]


def object_location_in_shelf():
    rospy.init_node('shelf_location_service_node')
    rospy.loginfo("Waiting for json_prolog.")
    rospy.wait_for_service('json_prolog/query')
    rospy.loginfo("Starting 'object_location_in_shelf' service.")
    rospy.Service('object_location_in_shelf', ObjectLocationInShelf, handle_object_location_in_shelf)
    rospy.spin()


if __name__ == '__main__':
    object_location_in_shelf()