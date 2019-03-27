#! /usr/bin/env python

import rospy
import sys
import suturo_perception_msgs.msg
import actionlib


def perceive_client(server_name):
    # Initialise action client
    action = None
    goal = None
    if server_name == 'hsr_perception_table':
        action = suturo_perception_msgs.msg.PerceiveTableAction
        goal = suturo_perception_msgs.msg.PerceiveTableGoal(visualisation=False)
    elif server_name == 'hsr_perception_shelf':
        action = suturo_perception_msgs.msg.PerceiveShelfAction
        goal = suturo_perception_msgs.msg.PerceiveShelfGoal(visualisation=False)
    else:
        rospy.logerr("Server name %s can not be resolved to action." % server_name)

    client = actionlib.SimpleActionClient(str(server_name), action)

    rospy.loginfo("Waiting for server %s..." % server_name)
    client.wait_for_server()
    rospy.loginfo("Server is up!")

    rospy.loginfo("Sending goal.")
    client.send_goal(goal)

    rospy.loginfo("Waiting for results.")
    client.wait_for_result(rospy.Duration(60))
    if client.simple_state == actionlib.SimpleGoalState.DONE:
        return client.get_result()
    else:
        rospy.logerr("No result received after 1 minute.")
    return None


if __name__ == '__main__':
    try:
        rospy.init_node('perceive_py')
        if len(sys.argv) > 1:
            rospy.loginfo("Start server call to action %s" % sys.argv[1])
            result = perceive_client(sys.argv[1])
        else:
            rospy.logerr("No action server specified.")
            rospy.logerr("Use either 'hsr_perception_shelf' or 'hsr_perception_table' as argument.")
        print("Result: ", ", ".join([str(n.obj_class) for n in result.detectionData]))
        print("Result: %s" % str(result.detectionData))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")