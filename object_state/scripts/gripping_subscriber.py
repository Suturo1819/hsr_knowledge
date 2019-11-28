#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import rosprolog_client
import tmc_msgs.msg
import actionlib
import tf2_ros

prolog = rosprolog_client.Prolog()
sec_in_nanos = 1000000000


class GripperSubscriber(rospy.Subscriber):
    def __init__(self, topic, message_type):
        self.timestamp = rospy.Time()
        self.gripper_closed = False
        self.pub = rospy.Publisher("gripping_value", Float64, queue_size=10)
        self.talker = actionlib.SimpleActionClient("talk_request_action", tmc_msgs.msg.TalkRequestAction)
        self.talker_goal = tmc_msgs.msg.TalkRequestGoal()
        self.talker_goal.data = tmc_msgs.msg.Voice(None, None, 1, "")
        super(GripperSubscriber, self).__init__(topic, message_type, self.callback)

    def callback(self, data):
        """Scan gripper joint every second."""
        if data.header.stamp.to_nsec() - self.timestamp.to_nsec() > sec_in_nanos:
            self.timestamp = data.header.stamp
            joint_index = data.name.index("hand_motor_joint")
            gripper_open_width_data = data.position[joint_index]
            if gripper_open_width_data > 0.7 and self.gripper_closed:
                self.release_object_from_gripper()
            elif gripper_open_width_data < 0.4 and not self.gripper_closed:
                self.attach_object_to_gripper()

    def release_object_from_gripper(self):
        """Call beliefstate to release the object from the gripper."""
        self.gripper_closed = False
        held_object_query = "gripper(Gripper), objects_on_surface(Objs, Gripper), member(Obj, Objs), object_frame_name(Obj, Name)."
        object_in_gripper_raw = prolog.all_solutions(held_object_query)
        if object_in_gripper_raw:
            object_in_gripper = str(object_in_gripper_raw[0]['Name']).split('_')[0]
            print(str(object_in_gripper_raw[0]['Name']))
            superclass_query = "object_frame_name(_Instance, {}), " \
                               "rdfs_type_of(_Instance, _Class)," \
                               "owl_direct_subclass_of(_Class, _Super)," \
                               "rdf_split_url(_, SuperclassName, _Super)".format(str(object_in_gripper_raw[0]['Name']))
            superclass_result_raw = prolog.all_solutions(superclass_query)
            output_object = object_in_gripper
            if superclass_result_raw:
                superclass = str(superclass_result_raw[0]['SuperclassName']).replace('\'', '')
                output_object = superclass
            release_object_query = "release_object_from_gripper."
            solutions = prolog.all_solutions(release_object_query)
            if solutions:
                self.talker_goal.data.sentence = "Putting down: " + output_object
                self.talker.send_goal(self.talker_goal)
                rospy.loginfo("Putting down: " + output_object)
                # print("RELEASE " + ("successful." if len(solutions) > 0 else "failed."))
        else:
            rospy.loginfo("No object in gripper to release.")

    def attach_object_to_gripper(self):
        """From all objects in the beliefstate, choose the one closest to the gripper and attach it."""
        transform_msg = safe_lookup_transform("map", "hand_palm_link")
        self.gripper_closed = True
        if transform_msg:
            trans = transform_msg.transform.translation
            surface_query = "select_surface([%s], Surface)" % ", ".join([str(c) for c in [trans.x, trans.y, trans.z]])
            objects_on_surface_query = "objects_on_surface(Objects, Surface), member(Obj, Objects), " \
                                       "object_frame_name(Obj, Frame)"
            objects_nearby_gripper_raw = prolog.all_solutions(surface_query + "," + objects_on_surface_query + ".")
            if objects_nearby_gripper_raw:
                objects_nearby = [str(solution['Frame']).replace('\'', '') for solution in objects_nearby_gripper_raw]
                closest_object = ("", 1)
                for frame in objects_nearby:
                    trans = safe_lookup_transform("hand_palm_link", frame).transform.translation
                    dist = np.linalg.norm(np.array([trans.x, trans.y, trans.z]))
                    if dist < 0.15 and dist < closest_object[1]:
                        closest_object = (frame, dist)
                    print (closest_object)
                if closest_object[0]:
                    superclass_query = "object_frame_name(_Instance, '{}'), " \
                                       "rdfs_type_of(_Instance, _Class)," \
                                       "owl_direct_subclass_of(_Class, _Super)," \
                                       "rdf_split_url(_, SuperclassName, _Super)".format(closest_object[0])
                    superclass_result_raw = prolog.all_solutions(superclass_query)
                    if superclass_result_raw:
                        rospy.loginfo(str(superclass_result_raw))
                        superclass = str(superclass_result_raw[0]['SuperclassName']).replace('\'', '')
                        rospy.loginfo("Grasping the " + superclass)
                        self.talker_goal.data.sentence = "Grasping the " + superclass
                    else:
                        rospy.loginfo("Grasping the " + closest_object[0].split('_')[0])
                        self.talker_goal.data.sentence = "Grasping the " + closest_object[0].split('_')[0]
                    self.talker.send_goal(self.talker_goal)
                    attach_to_gripper_query = "object_frame_name(Object, '"+closest_object[0]+"'), attach_object_to_gripper(Object)."
                    solution = prolog.all_solutions(attach_to_gripper_query)
                else:
                    rospy.loginfo("No object nearby the gripper to attach.")
            else:
                rospy.loginfo("No object nearby the gripper to attach.")


def safe_lookup_transform(source_frame, target_frame, duration=rospy.Duration(3)):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    now = rospy.Time()
    try:
        return tf_buffer.lookup_transform(source_frame, target_frame, now, duration)
    except (tf2_ros.ConnectivityException, tf2_ros.LookupException, tf2_ros.ExtrapolationException):
        print("source: " + source_frame + "\ntarget: " + target_frame)
        rospy.logerr("No transform between "+source_frame+" and "+target_frame)


def listener():
    rospy.init_node('gripper_attachments_by_hacky_knowledge', anonymous=True)
    GripperSubscriber("hsrb/robot_state/joint_states", JointState)
    rospy.spin()


if __name__ == '__main__':
    listener()
