#! /usr/bin/env python

import rospy
import sys
import suturo_perception_msgs.msg
import actionlib


def perceive_client(regions):
    # Initialise action client
    action = suturo_perception_msgs.msg.ExtractObjectInfoAction
    regions = [str(region) for region in regions]

    goal = suturo_perception_msgs.msg.ExtractObjectInfoGoal()
    goal.visualize = True
    goal.regions = regions
    # if server_name == 'hsr_perception_table':
    #     action = suturo_perception_msgs.msg.PerceiveTableAction
    #     goal = suturo_perception_msgs.msg.PerceiveTableGoal(visualisation=False)
    # elif server_name == 'hsr_perception_shelf':
    #     action = suturo_perception_msgs.msg.PerceiveShelfAction
    #     goal = suturo_perception_msgs.msg.PerceiveShelfGoal(visualisation=False)
    # else:
    #     rospy.logerr("Server name %s can not be resolved to action." % server_name)

    client = actionlib.SimpleActionClient("extract_object_infos", action)

    rospy.loginfo("Waiting for server %s..." % "extract_object_infos")
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
            rospy.loginfo("Start perception call to regions %s" % str(sys.argv[1:]))
            result = perceive_client(sys.argv[1:])
        else:
            rospy.logerr("No regions specified.")
            rospy.logerr("Use 'robocup_table' oder 'robocup_shelf_0', 'robocup_shelf_1', etc.")
        print("Result: ", ", ".join([str(n.obj_class) for n in result.detectionData]))
        print("Result: %s" % str(result.detectionData))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")